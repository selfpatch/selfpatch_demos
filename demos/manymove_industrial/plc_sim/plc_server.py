"""OPC UA server that emulates an industrial PLC for the manymove_industrial demo.

The control logic, tag namespace and alarm conditions are designed to be
swappable with a real OpenPLC v3 + ST program once the IEC 61131-3 build
pipeline is in place. From a bridge-client perspective the OPC UA surface
is identical: standard AlarmConditionType events on namespace 2.

Three alarm conditions are exposed:
 - photoeye_flicker  : WARN  - intermittent pick photoeye, precursor fault
 - conveyor_overspeed: ERROR - motor speed beyond commissioning envelope
 - estop_engaged     : CRITICAL - line e-stop pressed

External tags can be poked via a small admin endpoint so the demo
orchestrator (record_full.sh) can trigger faults at known timestamps
without going through the OPC UA write path.
"""

from __future__ import annotations

import asyncio
import logging
import os
import signal
from typing import Optional

from aiohttp import web
from asyncua import Server, ua
from asyncua.common.events import Event

LOGGER = logging.getLogger("plc_sim")

ENDPOINT = os.environ.get("PLC_OPCUA_ENDPOINT", "opc.tcp://0.0.0.0:4840/manymove_plc/server")
ADMIN_PORT = int(os.environ.get("PLC_ADMIN_PORT", "8500"))
APP_URI = "urn:selfpatch:manymove_industrial:plc_sim"


class PLCSimulator:
    """Holds the OPC UA server, tags, alarm conditions and admin endpoint."""

    def __init__(self) -> None:
        self.server = Server()
        self.alarm_nodes: dict[str, "AlarmHandle"] = {}

    async def setup(self) -> None:
        await self.server.init()
        self.server.set_endpoint(ENDPOINT)
        self.server.set_application_uri(APP_URI)
        self.server.set_server_name("manymove_industrial PLC sim")

        # Namespaces: 0 = OPC standard, 1 = local URI, 2 = our PLC tags.
        self.ns_plc = await self.server.register_namespace(APP_URI)
        LOGGER.info("registered PLC namespace idx=%s", self.ns_plc)

        objects = self.server.nodes.objects
        plc_folder = await objects.add_folder(self.ns_plc, "PLC")

        # Process tags (analog + digital, similar to ST %IX/%QX/%MW).
        self.tag_photoeye_pick = await plc_folder.add_variable(
            self.ns_plc, "photoeye_pick", False, ua.VariantType.Boolean,
        )
        self.tag_photoeye_drop = await plc_folder.add_variable(
            self.ns_plc, "photoeye_drop", False, ua.VariantType.Boolean,
        )
        self.tag_conveyor_speed_rpm = await plc_folder.add_variable(
            self.ns_plc, "conveyor_speed_rpm", 0.0, ua.VariantType.Double,
        )
        self.tag_conveyor_motor = await plc_folder.add_variable(
            self.ns_plc, "conveyor_motor", True, ua.VariantType.Boolean,
        )
        self.tag_estop = await plc_folder.add_variable(
            self.ns_plc, "estop", False, ua.VariantType.Boolean,
        )
        for tag in (
            self.tag_photoeye_pick,
            self.tag_photoeye_drop,
            self.tag_conveyor_speed_rpm,
            self.tag_conveyor_motor,
            self.tag_estop,
        ):
            await tag.set_writable()

        # Alarm conditions (OPC UA Part 9 AlarmConditionType). Source MUST
        # be an Object node (supports EventNotifier attribute), not a
        # Variable. Per OPC UA spec, the Server object is the canonical
        # event emitter when no per-equipment Object exists.
        server_node = self.server.get_node(ua.ObjectIds.Server)
        self.alarm_nodes["photoeye_flicker"] = AlarmHandle(
            server=self.server,
            source=server_node,
            event_type=ua.ObjectIds.AlarmConditionType,
            severity=300,  # 1-1000, mid range
            condition_name="PhotoeyeFlicker",
            message="Pick photoeye toggling above acceptance rate",
        )
        self.alarm_nodes["conveyor_overspeed"] = AlarmHandle(
            server=self.server,
            source=server_node,
            event_type=ua.ObjectIds.AlarmConditionType,
            severity=600,
            condition_name="ConveyorOverspeed",
            message="Conveyor speed beyond commissioning envelope",
        )
        self.alarm_nodes["estop_engaged"] = AlarmHandle(
            server=self.server,
            source=server_node,
            event_type=ua.ObjectIds.AlarmConditionType,
            severity=900,
            condition_name="EstopEngaged",
            message="Line e-stop pressed",
        )
        for handle in self.alarm_nodes.values():
            await handle.setup()

    async def run(self) -> None:
        async with self.server:
            LOGGER.info("OPC UA server up at %s", ENDPOINT)
            admin_runner = await self._start_admin_endpoint()
            try:
                await asyncio.Event().wait()
            finally:
                await admin_runner.cleanup()

    async def _start_admin_endpoint(self) -> web.AppRunner:
        """Tiny HTTP control surface so external orchestrators (record_full.sh,
        smoke tests, container_scripts) can trigger alarms without speaking
        OPC UA themselves."""
        app = web.Application()
        app.router.add_post("/alarm/{name}/raise", self._handle_raise)
        app.router.add_post("/alarm/{name}/clear", self._handle_clear)
        app.router.add_get("/health", lambda _: web.json_response({"ok": True}))
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", ADMIN_PORT)
        await site.start()
        LOGGER.info("admin endpoint up at http://0.0.0.0:%s", ADMIN_PORT)
        return runner

    async def _handle_raise(self, request: web.Request) -> web.Response:
        name = request.match_info["name"]
        handle = self.alarm_nodes.get(name)
        if handle is None:
            return web.json_response({"error": f"unknown alarm {name}"}, status=404)
        body = await request.json() if request.body_exists else {}
        message = body.get("message")
        await handle.raise_alarm(message=message)
        LOGGER.info("admin raised alarm: %s", name)
        return web.json_response({"ok": True, "alarm": name, "state": "active"})

    async def _handle_clear(self, request: web.Request) -> web.Response:
        name = request.match_info["name"]
        handle = self.alarm_nodes.get(name)
        if handle is None:
            return web.json_response({"error": f"unknown alarm {name}"}, status=404)
        await handle.clear_alarm()
        LOGGER.info("admin cleared alarm: %s", name)
        return web.json_response({"ok": True, "alarm": name, "state": "cleared"})


class AlarmHandle:
    """Manages one AlarmConditionType lifecycle (active / cleared)."""

    def __init__(
        self,
        server: Server,
        source,
        event_type: int,
        severity: int,
        condition_name: str,
        message: str,
    ) -> None:
        self.server = server
        self.source = source
        self.event_type = event_type
        self.severity = severity
        self.condition_name = condition_name
        self.default_message = message
        self.generator: Optional[Event] = None
        self.active = False

    async def setup(self) -> None:
        self.generator = await self.server.get_event_generator(self.event_type, self.source)
        self.generator.event.SourceName = self.condition_name
        self.generator.event.Severity = self.severity
        self.generator.event.Message = ua.LocalizedText(self.default_message)

    async def raise_alarm(self, message: Optional[str] = None) -> None:
        if self.generator is None:
            raise RuntimeError("alarm not initialised")
        if message:
            self.generator.event.Message = ua.LocalizedText(message)
        else:
            self.generator.event.Message = ua.LocalizedText(self.default_message)
        await self.generator.trigger()
        self.active = True

    async def clear_alarm(self) -> None:
        if self.generator is None:
            return
        self.generator.event.Message = ua.LocalizedText(
            f"{self.default_message} cleared",
        )
        # Severity 0 signals cleared/healed in our mapping convention.
        prev_severity = self.generator.event.Severity
        self.generator.event.Severity = 0
        await self.generator.trigger()
        self.generator.event.Severity = prev_severity
        self.active = False


async def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")
    sim = PLCSimulator()
    await sim.setup()

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    server_task = asyncio.create_task(sim.run())
    await stop.wait()
    server_task.cancel()
    try:
        await server_task
    except asyncio.CancelledError:
        pass


if __name__ == "__main__":
    asyncio.run(main())
