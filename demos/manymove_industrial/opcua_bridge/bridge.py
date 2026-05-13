"""OPC UA -> ros2_medkit fault bridge.

ROS 2 node that subscribes to AlarmConditionType events on the PLC OPC UA
server and forwards them to the medkit FaultManager via the
`/fault_manager/report_fault` ROS service. Each PLC alarm maps to a
canonical MANYMOVE_PLC_* fault code so the same medkit dashboard that
aggregates manymove BT-side faults also receives the PLC-side faults
with a distinct `source_id`.

The mapping table is deliberately explicit (not data-driven from the
OPC UA discovery) so the demo SOVD manifest, the documentation and the
bridge stay in lockstep.
"""

from __future__ import annotations

import asyncio
import logging
import os
import signal
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.srv import ReportFault

from asyncua import Client, ua

LOGGER = logging.getLogger("opcua_bridge")

OPCUA_ENDPOINT = os.environ.get(
    "OPCUA_ENDPOINT",
    "opc.tcp://plc-sim:4840/manymove_plc/server",
)
SOURCE_ID = os.environ.get("BRIDGE_SOURCE_ID", "/plc/sensor_io")
REPORT_FAULT_SERVICE = os.environ.get(
    "REPORT_FAULT_SERVICE", "/fault_manager/report_fault",
)


@dataclass
class AlarmMapping:
    """One PLC alarm -> medkit fault binding."""

    condition_name: str          # OPC UA SourceName / ConditionName
    fault_code: str              # medkit MANYMOVE_PLC_* code
    severity: int                # medkit 0=INFO 1=WARN 2=ERROR 3=CRITICAL
    description: str             # short operator-facing description


# Canonical mapping. Keep in sync with:
#  - docs/PLC_FAULT_CODES.md
#  - demos/manymove_industrial/config/manymove_industrial_manifest.yaml
#  - demos/manymove_industrial/plc_sim/plc_server.py
ALARM_MAP: dict[str, AlarmMapping] = {
    "PhotoeyeFlicker": AlarmMapping(
        condition_name="PhotoeyeFlicker",
        fault_code="MANYMOVE_PLC_PHOTOEYE_FLICKER",
        severity=1,
        description="Pick photoeye toggling above commissioning rate",
    ),
    "ConveyorOverspeed": AlarmMapping(
        condition_name="ConveyorOverspeed",
        fault_code="MANYMOVE_PLC_CONVEYOR_OVERSPEED",
        severity=2,
        description="Conveyor speed beyond commissioning envelope",
    ),
    "EstopEngaged": AlarmMapping(
        condition_name="EstopEngaged",
        fault_code="MANYMOVE_PLC_ESTOP_ENGAGED",
        severity=3,
        description="Line e-stop engaged",
    ),
}


class BridgeNode(Node):
    """ROS 2 node hosting the fault reporting client.

    The async OPC UA work runs in a separate thread with its own event
    loop; on each alarm event, it dispatches a fault report via
    rclpy's executor by submitting a job onto the ROS thread.
    """

    def __init__(self) -> None:
        super().__init__("opcua_bridge")
        self.client = self.create_client(ReportFault, REPORT_FAULT_SERVICE)
        self.source_id = SOURCE_ID
        self._await_service()

    def _await_service(self) -> None:
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                f"waiting for service {REPORT_FAULT_SERVICE} ...",
            )

    def report_fault(
        self, mapping: AlarmMapping, message: str, event_type: int,
    ) -> None:
        request = ReportFault.Request()
        request.fault_code = mapping.fault_code
        request.event_type = event_type  # 0=FAILED, 1=PASSED
        request.severity = mapping.severity if event_type == 0 else 0
        request.description = message
        request.source_id = self.source_id

        future = self.client.call_async(request)

        def _done(fut) -> None:
            try:
                response = fut.result()
                self.get_logger().info(
                    f"reported {mapping.fault_code} event={event_type} "
                    f"accepted={getattr(response, 'accepted', '?')}",
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f"report_fault call failed: {exc}",
                )

        future.add_done_callback(_done)


class AlarmEventHandler:
    """asyncua SubHandler. Translates AlarmConditionType events into
    BridgeNode.report_fault dispatches. Loopback prevention: if the
    SourceName begins with our own source_id, drop the event."""

    def __init__(self, node: BridgeNode) -> None:
        self.node = node

    async def event_notification(self, event) -> None:
        try:
            source_name = getattr(event, "SourceName", None)
            severity = getattr(event, "Severity", None)
        except Exception:  # noqa: BLE001
            LOGGER.warning("event missing standard fields: %s", event)
            return

        if source_name is None:
            return
        if source_name.startswith(self.node.source_id):
            LOGGER.debug("ignoring loopback event from %s", source_name)
            return

        mapping = ALARM_MAP.get(source_name)
        if mapping is None:
            LOGGER.debug("no mapping for source_name=%s", source_name)
            return

        # Pull message text (LocalizedText.Text).
        message_text = getattr(event, "Message", None)
        message_str = mapping.description
        if message_text is not None:
            text = getattr(message_text, "Text", None)
            if text:
                message_str = text

        # Severity 0 signals cleared/healed in our PLC convention.
        event_type = 1 if severity == 0 else 0
        if event_type == 1:
            message_str = f"{mapping.description} cleared"
        self.node.report_fault(mapping, message_str, event_type)


async def opcua_loop(node: BridgeNode, stop_event: asyncio.Event) -> None:
    handler = AlarmEventHandler(node)
    # One-shot DNS diagnostic so future "name resolution failure" loops are
    # debuggable without docker exec into the running container.
    try:
        import socket
        from urllib.parse import urlparse

        parsed = urlparse(OPCUA_ENDPOINT)
        host = parsed.hostname or OPCUA_ENDPOINT
        infos = socket.getaddrinfo(host, parsed.port or 4840)
        LOGGER.info(
            "DNS check: %s resolves to %s",
            host,
            ", ".join(sorted({i[4][0] for i in infos})),
        )
    except Exception as dns_exc:  # noqa: BLE001
        LOGGER.warning("DNS check FAILED: %s", dns_exc)

    while not stop_event.is_set():
        try:
            async with Client(OPCUA_ENDPOINT) as client:
                LOGGER.info("connected to %s", OPCUA_ENDPOINT)
                sub = await client.create_subscription(500, handler)
                event_type = client.get_node(ua.ObjectIds.AlarmConditionType)
                await sub.subscribe_events(client.nodes.server, event_type)
                LOGGER.info("subscribed to AlarmConditionType events")
                while not stop_event.is_set():
                    await asyncio.sleep(0.5)
        except (OSError, ua.UaError, asyncio.TimeoutError) as exc:
            LOGGER.warning("OPC UA session error: %s, retrying in 2s", exc)
            await asyncio.sleep(2.0)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )

    rclpy.init()
    node = BridgeNode()

    stop = asyncio.Event()

    def _signal_handler() -> None:
        stop.set()
        rclpy.shutdown()

    def _opcua_thread() -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(opcua_loop(node, stop))
        finally:
            loop.close()

    thread = threading.Thread(target=_opcua_thread, name="opcua-loop", daemon=True)
    thread.start()

    # rclpy.spin in main thread so service futures resolve.
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: _signal_handler())
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=3.0)


if __name__ == "__main__":
    main()
