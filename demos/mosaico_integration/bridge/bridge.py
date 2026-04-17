#!/usr/bin/env python3
"""Bridge: subscribe to the medkit /faults/stream SSE, ingest each fault
snapshot bag into mosaicod via the mosaicolabs Python SDK.

Architecture:
  1. medkit gateway  --[SSE: fault event]-->   bridge
  2. bridge          --[REST: GET snapshot]-->  medkit gateway
  3. medkit gateway  --[MCAP bag response]-->   bridge
  4. bridge          --[Arrow Flight]-->        mosaicod

License-safe: this is a separate process talking Arrow Flight to an
unmodified mosaicod docker image, per Mosaico's recommended pattern.
"""

from __future__ import annotations

import json
import logging
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterator, Optional

import httpx

LOG = logging.getLogger("mosaico_bridge")

# --- Config (env-driven) ---------------------------------------------------

MEDKIT_URL = os.environ.get("MEDKIT_URL", "http://localhost:8080")
MOSAICO_HOST = os.environ.get("MOSAICO_HOST", "localhost")
MOSAICO_PORT = int(os.environ.get("MOSAICO_PORT", "6726"))
LOG_LEVEL = os.environ.get("LOG_LEVEL", "INFO").upper()
ROBOT_ID = os.environ.get("ROBOT_ID", "sensor_demo_001")
SOURCE_DEMO = os.environ.get("SOURCE_DEMO", "sensor_diagnostics")
DOWNLOAD_DIR = Path(os.environ.get("DOWNLOAD_DIR", "/tmp/mosaico_bridge"))

# Optional whitelist of fault codes to ingest into Mosaico. Comma-separated.
# Empty string (default) means "accept all fault codes". Set to e.g.
# "LIDAR_SIM" to ingest only LIDAR snapshots and skip everything else
# (IMU drifting, NaN injection, etc.) that medkit still captures on its
# own side.
_allowlist_raw = os.environ.get("FAULT_CODE_ALLOWLIST", "").strip()
FAULT_CODE_ALLOWLIST: set[str] = (
    {code.strip() for code in _allowlist_raw.split(",") if code.strip()}
    if _allowlist_raw
    else set()
)

# Medkit fault severity mapping (matches ros2_medkit_msgs/Fault SEVERITY_*)
SEVERITY_NAMES = {
    0: "INFO",
    1: "WARNING",
    2: "ERROR",
    3: "CRITICAL",
}


# --- Data model ------------------------------------------------------------


@dataclass
class FaultEvent:
    """Subset of medkit FaultEvent we care about."""

    event_id: int
    event_type: str
    fault_code: str
    severity: int
    severity_name: str
    description: str
    status: str
    reporting_sources: list[str]
    timestamp_sec: float
    raw: dict

    @classmethod
    def from_sse_data(cls, event_id: int, data_json: dict) -> "FaultEvent":
        fault = data_json.get("fault", {}) or {}
        sev = int(fault.get("severity", 0))
        return cls(
            event_id=event_id,
            event_type=data_json.get("event_type", ""),
            fault_code=fault.get("fault_code", ""),
            severity=sev,
            severity_name=SEVERITY_NAMES.get(sev, f"UNKNOWN_{sev}"),
            description=fault.get("description", ""),
            status=fault.get("status", ""),
            reporting_sources=list(fault.get("reporting_sources", []) or []),
            timestamp_sec=float(data_json.get("timestamp", 0.0) or 0.0),
            raw=data_json,
        )


# --- SSE stream parser -----------------------------------------------------


def stream_fault_events(
    base_url: str,
    last_event_id: Optional[int] = None,
) -> Iterator[FaultEvent]:
    """Yield fault events from medkit /api/v1/faults/stream.

    Reconnects forever; on each reconnect resumes from the most recent
    event id we yielded via the SSE `Last-Event-ID` header (medkit
    replays buffered events per the SSE spec).
    """
    url = f"{base_url}/api/v1/faults/stream"
    # Mutable one-slot box so the `set_last_id` closure passed into
    # `_parse_sse` can update the value the outer loop reads on reconnect.
    resume = [last_event_id]

    def _update_last(eid: int) -> None:
        resume[0] = eid

    while True:
        headers = {"Accept": "text/event-stream"}
        if resume[0] is not None:
            headers["Last-Event-ID"] = str(resume[0])

        try:
            LOG.info(
                "Connecting SSE stream %s (last_event_id=%s)", url, resume[0]
            )
            # No request timeout for SSE; medkit emits keepalives.
            with httpx.stream("GET", url, headers=headers, timeout=None) as resp:
                resp.raise_for_status()
                LOG.info("SSE stream connected (status=%s)", resp.status_code)
                yield from _parse_sse(resp.iter_lines(), set_last_id=_update_last)
        except (httpx.HTTPError, httpx.StreamError) as exc:
            LOG.warning(
                "SSE stream error: %s; reconnecting in 2s (resume from id=%s)",
                exc,
                resume[0],
            )
            time.sleep(2.0)
        except KeyboardInterrupt:
            LOG.info("Interrupted, exiting")
            return


def _parse_sse(lines: Iterator[str], set_last_id) -> Iterator[FaultEvent]:
    """Parse SSE protocol from a line iterator. Yields FaultEvent objects.

    Expected medkit frame format:
        id: <N>
        event: <event_type>
        data: {<json>}
        <empty line>
    """
    cur_id: Optional[int] = None
    cur_data: list[str] = []

    for raw in lines:
        line = raw.rstrip("\r")
        if line == "":
            # Dispatch the buffered event
            if cur_data:
                payload = "\n".join(cur_data)
                try:
                    data_json = json.loads(payload)
                except json.JSONDecodeError as exc:
                    # Not a transient issue - the gateway broke the SSE
                    # protocol contract. Log loudly and keep going.
                    LOG.error(
                        "SSE data not JSON (protocol break, dropping 1 frame): %s; raw=%r",
                        exc,
                        payload[:500],
                    )
                else:
                    eid = cur_id if cur_id is not None else 0
                    yield FaultEvent.from_sse_data(eid, data_json)
                    set_last_id(eid)
            cur_id = None
            cur_data = []
            continue

        if line.startswith(":"):
            # SSE comment / keepalive
            continue
        if line.startswith("id:"):
            try:
                cur_id = int(line[3:].strip())
            except ValueError:
                cur_id = None
        # SSE `event:` field is intentionally not parsed here; the
        # semantic event type is carried inside the JSON body and ends up
        # in `FaultEvent.event_type`.
        elif line.startswith("data:"):
            cur_data.append(line[5:].lstrip())


# --- Entity resolution -----------------------------------------------------


def resolve_entity_for_download(
    base_url: str, fault: FaultEvent
) -> Optional[tuple[str, str]]:
    """Walk medkit entity types to find one that owns this fault's bulk-data.

    Returns (entity_type, entity_id) or None.

    Notes:
    - reporting_sources contains ROS node names like "/bridge/diagnostic_bridge",
      NOT SOVD entity FQNs. We can't derive the entity ID from them directly.
    - The fault snapshot bag is registered against the diagnostic-bridge APP
      (kebab-case), discoverable via /apps/{id}/bulk-data/rosbags/{fault_code}.
    - We discover the right entity by listing all apps + components and
      asking each for the bulk-data/rosbags/{fault_code} until one returns 200.
      The list is small (~10 entities) so this is cheap.
    - This HEAD-probe discovery is a pragmatic workaround. The gateway-wide
      SSE fault stream does not surface SOVD entity context today; tracked
      in selfpatch/ros2_medkit#380 (either an x-medkit extension in the SSE
      payload or per-entity fault subscriptions). Once that lands, this
      function collapses to a single direct fetch.
    """
    # Build candidate list: enumerate apps and components.
    candidates: list[tuple[str, str]] = []
    for etype in ("apps", "components"):
        try:
            r = httpx.get(f"{base_url}/api/v1/{etype}", timeout=5.0)
            r.raise_for_status()
            items = r.json().get("items", []) or []
            for item in items:
                eid = item.get("id")
                if eid:
                    candidates.append((etype, eid))
        except httpx.HTTPError as exc:
            LOG.warning(
                "Failed to list /api/v1/%s while resolving fault %s: %s",
                etype,
                fault.fault_code,
                exc,
            )

    if not candidates:
        LOG.warning(
            "No apps/components discovered for fault %s; gateway may be unreachable or still coming up",
            fault.fault_code,
        )
        return None

    tried = 0
    for etype, eid in candidates:
        tried += 1
        url = f"{base_url}/api/v1/{etype}/{eid}/bulk-data/rosbags/{fault.fault_code}"
        try:
            r = httpx.head(url, timeout=5.0, follow_redirects=True)
        except httpx.HTTPError as exc:
            LOG.debug("HEAD probe failed for %s: %s", url, exc)
            continue
        if r.status_code == 200:
            LOG.info(
                "Resolved fault %s to %s/%s", fault.fault_code, etype, eid
            )
            return (etype, eid)
        LOG.debug("HEAD %s -> %d", url, r.status_code)
    LOG.warning(
        "No entity owns bulk-data for fault %s (probed %d candidates); "
        "bag may not be registered yet (post-fault timer still running)",
        fault.fault_code,
        tried,
    )
    return None


# --- Bag download ----------------------------------------------------------


def download_bag(base_url: str, fault: FaultEvent) -> Optional[Path]:
    """Download the .mcap snapshot bag for a confirmed fault.

    The medkit gateway resolves the rosbag2 directory and streams the inner
    .mcap back to the caller.
    """
    resolved = resolve_entity_for_download(base_url, fault)
    if resolved is None:
        LOG.error(
            "Could not resolve entity for fault_code=%s sources=%s",
            fault.fault_code,
            fault.reporting_sources,
        )
        return None
    etype, eid = resolved
    url = f"{base_url}/api/v1/{etype}/{eid}/bulk-data/rosbags/{fault.fault_code}"
    DOWNLOAD_DIR.mkdir(parents=True, exist_ok=True)
    out = DOWNLOAD_DIR / f"{fault.fault_code}_{int(fault.timestamp_sec)}.mcap"
    LOG.info("Downloading bag: %s -> %s", url, out)
    try:
        with httpx.stream("GET", url, timeout=60.0) as resp:
            resp.raise_for_status()
            with out.open("wb") as f:
                for chunk in resp.iter_bytes(chunk_size=64 * 1024):
                    if chunk:
                        f.write(chunk)
    except httpx.HTTPError as exc:
        LOG.error("Bag download failed: %s", exc)
        return None
    size = out.stat().st_size
    LOG.info("Downloaded %.1f KB to %s", size / 1024.0, out)
    if size == 0:
        LOG.error("Downloaded bag is empty: %s", out)
        return None
    # Sanity-check the MCAP magic header. If the bag download raced the
    # rosbag2 finalizer we'd otherwise hand a truncated/corrupt file to
    # RosbagInjector and lose the root cause inside its exception path.
    # MCAP magic: 0x89 "MCAP" 0x30 "\r\n" (8 bytes).
    with out.open("rb") as fh:
        magic = fh.read(8)
    if magic != b"\x89MCAP0\r\n":
        LOG.error(
            "Downloaded bag has invalid MCAP magic header (got %r, size=%d): %s",
            magic,
            size,
            out,
        )
        return None
    return out


# --- Mosaico ingest --------------------------------------------------------


def ingest_to_mosaico(bag_path: Path, fault: FaultEvent) -> Optional[str]:
    """Ingest bag into mosaicod via mosaicolabs SDK over Arrow Flight.

    Returns sequence_name on success, None on failure.
    """
    # Lazy import to keep bridge importable for unit tests without SDK.
    from mosaicolabs.ros_bridge import ROSInjectionConfig, RosbagInjector
    from pyarrow.flight import FlightError

    sequence_name = (
        f"{SOURCE_DEMO}_{ROBOT_ID}_{fault.fault_code}"
        f"_{int(fault.timestamp_sec)}_{fault.event_id}"
    )
    metadata = {
        "robot_id": ROBOT_ID,
        "fault_code": fault.fault_code,
        "fault_severity": fault.severity_name,
        "fault_severity_int": fault.severity,
        "fault_description": fault.description or "",
        "fault_status": fault.status,
        "reporting_sources": ",".join(fault.reporting_sources),
        "captured_at_iso": datetime.fromtimestamp(
            fault.timestamp_sec, tz=timezone.utc
        ).isoformat(),
        "event_id": fault.event_id,
        "source_demo": SOURCE_DEMO,
        "source_pipeline": "ros2_medkit_fault_snapshot_v0",
    }
    cfg = ROSInjectionConfig(
        file_path=bag_path,
        sequence_name=sequence_name,
        metadata=metadata,
        host=MOSAICO_HOST,
        port=MOSAICO_PORT,
    )
    LOG.info(
        "Ingesting %s into mosaicod (sequence=%s, host=%s:%d)",
        bag_path,
        sequence_name,
        MOSAICO_HOST,
        MOSAICO_PORT,
    )
    try:
        RosbagInjector(cfg).run()
    except (FlightError, OSError, ConnectionError, TimeoutError) as exc:
        # Transport/flight failures are the only things we want to swallow
        # into a retry-friendly None here. Programmer errors
        # (AttributeError, KeyError, TypeError) from an SDK API rename
        # intentionally propagate so the drift surfaces loudly instead of
        # being logged as "failed ingest" forever.
        LOG.error("RosbagInjector failed (transport/flight): %s", exc)
        return None
    LOG.info("Ingest complete: sequence=%s", sequence_name)
    return sequence_name


def verify_sequence_in_mosaico(sequence_name: str) -> bool:
    """Connect to mosaicod and assert our new sequence is listed."""
    from mosaicolabs import MosaicoClient
    from pyarrow.flight import FlightError

    try:
        with MosaicoClient.connect(host=MOSAICO_HOST, port=MOSAICO_PORT) as client:
            seqs = list(client.list_sequences())
    except (FlightError, OSError, ConnectionError, TimeoutError) as exc:
        # Same narrowing as ingest_to_mosaico: only transport failures
        # degrade verification to "skip". API renames crash loudly.
        LOG.error("Verification list_sequences failed (transport/flight): %s", exc)
        return False
    if sequence_name in seqs:
        LOG.info("VERIFIED: sequence %s present in mosaicod", sequence_name)
        return True
    LOG.warning(
        "Sequence %s NOT in mosaicod list (found %d sequences)",
        sequence_name,
        len(seqs),
    )
    return False


# --- Main loop -------------------------------------------------------------


def handle_fault_event(fault: FaultEvent) -> None:
    LOG.info(
        "Fault event id=%s type=%s code=%s severity=%s sources=%s",
        fault.event_id,
        fault.event_type,
        fault.fault_code,
        fault.severity_name,
        fault.reporting_sources,
    )
    if fault.event_type != "fault_confirmed":
        LOG.debug("Ignoring non-confirmed event: %s", fault.event_type)
        return
    if not fault.fault_code:
        LOG.warning("Confirmed fault has empty fault_code, skipping")
        return
    if FAULT_CODE_ALLOWLIST and fault.fault_code not in FAULT_CODE_ALLOWLIST:
        LOG.info(
            "Skipping fault_code=%s (not in FAULT_CODE_ALLOWLIST=%s)",
            fault.fault_code,
            sorted(FAULT_CODE_ALLOWLIST),
        )
        return

    # Medkit's post-fault timer keeps the bag writer open for
    # `duration_after_sec` (10s in this demo's medkit_params.yaml). Wait
    # for finalization before trying to download.
    post_fault_wait = float(os.environ.get("POST_FAULT_WAIT_SEC", "12"))
    LOG.info("Waiting %.0fs for post-fault recording to finalize...", post_fault_wait)
    time.sleep(post_fault_wait)

    bag = download_bag(MEDKIT_URL, fault)
    if bag is None:
        return
    sequence_name = ingest_to_mosaico(bag, fault)
    if sequence_name is None:
        return
    verify_sequence_in_mosaico(sequence_name)


def main() -> int:
    logging.basicConfig(
        level=getattr(logging, LOG_LEVEL, logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    LOG.info(
        "Bridge starting: medkit=%s mosaicod=%s:%d robot_id=%s",
        MEDKIT_URL,
        MOSAICO_HOST,
        MOSAICO_PORT,
        ROBOT_ID,
    )
    LOG.info("Download dir: %s", DOWNLOAD_DIR)

    # Wait until medkit is reachable so we don't crash on initial startup
    # before sensor-demo healthchecks pass.
    for attempt in range(60):
        try:
            r = httpx.get(f"{MEDKIT_URL}/api/v1/health", timeout=2.0)
            if r.status_code == 200:
                LOG.info("Medkit gateway healthy")
                break
        except (httpx.InvalidURL, httpx.UnsupportedProtocol) as exc:
            # Config problem, not a transient network issue. Fail fast so
            # the operator fixes MEDKIT_URL instead of waiting 120s.
            LOG.error("MEDKIT_URL %r is unusable: %s", MEDKIT_URL, exc)
            return 1
        except httpx.HTTPError:
            pass
        if attempt == 0:
            LOG.info("Waiting for medkit gateway to come up at %s...", MEDKIT_URL)
        elif attempt % 10 == 0:
            LOG.info(
                "Still waiting for medkit gateway at %s (attempt %d/60)...",
                MEDKIT_URL,
                attempt,
            )
        time.sleep(2.0)
    else:
        LOG.error("Medkit gateway never became healthy after 120s")
        return 1

    try:
        for fault in stream_fault_events(MEDKIT_URL):
            try:
                handle_fault_event(fault)
            except Exception as exc:  # noqa: BLE001
                LOG.exception("Error handling fault event: %s", exc)
    except KeyboardInterrupt:
        LOG.info("Bridge stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
