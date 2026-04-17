# Mosaico integration demo

End-to-end pipeline that ingests medkit fault snapshots into Mosaico as queryable, structured forensic data. No robot hardware, no 24/7 recording.

A fault fires on the simulated LiDAR, medkit confirms it and flushes a 15 s pre-fault + 10 s post-fault ring buffer to an `.mcap` file. A small Python bridge picks up the SSE event, downloads the bag from the gateway REST API, and ingests it into mosaicod over Apache Arrow Flight. From `docker compose up` to a queryable `Sequence` takes about a minute.

## Two stacks

**Single-robot** (`docker-compose.yml`): one sensor-demo + one bridge. Best for stepping through the pipeline the first time and for the notebook.

**Fleet** (`docker-compose.fleet.yml`): three sensor-demos (warehouse-A, warehouse-B, outdoor-yard) each with its own bridge, all feeding one mosaicod. All three fire `LIDAR_SIM`, and robot-02 is rotating (IMU `drift_rate = 0.3 rad/s`). The metadata filter (Step 1 in the notebook) catches 3 of 3; the compound IMU `.Q` stationarity query (Step 2) excludes the rotating one. 2 of 3 remain for content-level drill-down (noise vs drift).

## Quick start

```bash
git clone https://github.com/selfpatch/selfpatch_demos.git
cd selfpatch_demos/demos/mosaico_integration

# Single-robot
docker compose up -d
./scripts/trigger-fault.sh                       # end-to-end ~15-20 s
docker compose logs -f bridge

# Fleet
docker compose -f docker-compose.fleet.yml up -d
./scripts/trigger-fleet-faults.sh                # end-to-end ~20-30 s

# Open the notebook against mosaicod on localhost:16726
jupyter notebook notebooks/mosaico_demo.ipynb

# Cleanup
docker compose -f docker-compose.fleet.yml down -v
```

## Flow

```
medkit gateway  --[SSE: fault event]-->   bridge
bridge          --[REST: GET snapshot]--> medkit gateway
medkit gateway  --[MCAP bag response]-->  bridge
bridge          --[Arrow Flight]-->       mosaicod
```

License-safe: mosaicod runs as the unmodified upstream Docker image. The bridge is a separate Python process that talks the public Apache Arrow Flight protocol via Mosaico's own SDK. We never link or modify mosaicod or its Rust crates.

## What lands in Mosaico

| ROS topic | ROS message type | Mosaico ontology | Status |
|---|---|---|---|
| `/sensors/scan` | `sensor_msgs/LaserScan` | `LaserScan` (`futures.laser`) | ✅ via [PR #368][pr368] |
| `/sensors/imu` | `sensor_msgs/Imu` | `IMU` | ✅ shipped adapter |
| `/sensors/fix` | `sensor_msgs/NavSatFix` | `GPS` | ✅ shipped adapter |
| `/sensors/image_raw` | `sensor_msgs/Image` | `Image` | ⚠️ adapter ships, excluded from this demo's snapshot |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | (none) | ⚠️ silently dropped - no adapter yet |

`/diagnostics` is the only adapter gap. `/sensors/image_raw` is a config choice - raw 30 Hz camera would dominate the bag at ~27 MB/s; swap in a CompressedImage topic when vision forensics matter.

[pr368]: https://github.com/mosaico-labs/mosaico/pull/368

## Storage

- Per snapshot: **~2 MB** MCAP (LaserScan 10 Hz, IMU 100 Hz, GPS 1 Hz, /diagnostics, 25 s window).
- 24/7 recording of the same four topics: ~6 GB/robot/day.
- At ~5 confirmed faults/robot/day the catalog holds ~10 MB/robot/day.

## Mosaico SDK pin

PR #368 (ROS adapters for the `futures` ontology) merged on 2026-04-13 as commit `b3867be`. We validated end-to-end against the PR while it was in draft; post-merge no demo-side change was required. The subsequent `mosaicolabs==0.3.2` PyPI wheel is missing the `futures` subpackage from the distributed artifact, so the bridge `Dockerfile` installs directly from the repo at `b3867be` until a packaging-fixed release ships. The notebook still needs `import mosaicolabs.models.futures.laser` at the top to register the ontology on the read side.

## Integration notes

1. **Entity resolution from the fault event**: the gateway-wide `/faults/stream` event carries ROS node names in `reporting_sources` (the shape SOVD asks for), while bulk-data endpoints are keyed by SOVD entity ID. The bridge enumerates apps + components and HEAD-probes to find the entity that owns each bag - cheap for ~10 entities, and a natural candidate for an `x-medkit-*` SSE extension or per-entity fault streams at fleet scale. See [ros2_medkit#380][medkit380] for the roadmap discussion.
2. **Ontology registration on the read side**: the write side (bridge) resolves Mosaico adapters by ROS msg type, but a read-side consumer has to `import mosaicolabs.models.futures.laser` to register the `laser_scan` ontology tag in the global registry. The notebook does this in its first cell.

[medkit380]: https://github.com/selfpatch/ros2_medkit/issues/380

## Troubleshooting

- `docker compose build bridge` fails the LaserScan sanity import → the pinned Mosaico commit is no longer fetchable or the source layout has moved. Update `MOSAICO_PIN` in `bridge/Dockerfile` to a current `main` commit that still contains `mosaicolabs/models/futures/laser.py`.
- Bridge logs `No entity owns bulk-data for fault X (probed N candidates)` → the post-fault timer has not fired yet. Wait a few seconds and re-trigger.
- Notebook raises `No ontology registered with tag 'laser_scan'` → the ontology import cell was skipped; re-run it.
- `docker compose pull mosaicod` is slow on first run → the upstream image is ~110 MB, distroless.
