# Connecting the demos to the selfpatch platform

Any demo in this repository can show up in the selfpatch platform as a robot. The connection goes through the selfpatch agent: a small container next to the demo that opens an outbound connection to the platform and lets the platform read the demo's ros2_medkit gateway. Nothing on the demo side listens for the platform, so the same steps work when the platform is in the cloud and the demo is behind NAT.

This page is also written for AI coding assistants. Paste it into the assistant's context before asking it to run a demo against the platform or to build platform UI on top of a demo.

## What you need

- A running demo (`./run-demo.sh` in its directory).
- An account on the platform you want to use.
- Docker. The scripts pull the public agent image `ghcr.io/selfpatch/agent:0.1` (amd64 and arm64). `SELFPATCH_AGENT_IMAGE` picks another one.

## Connect a demo

1. In the platform: Devices > Add device > "Through the selfpatch agent". Copy the pairing code. It is valid for 10 minutes.
2. In the demo directory:

   ```bash
   ./connect-platform.sh
   ```

   Paste the code when asked, or pass it as `PAIRING_CODE=XXXX-XXXX-XXXX-XXXX`.

3. The device shows as online in the platform within a few seconds.

The agent runs as the container `selfpatch-agent-<gateway container>` and keeps its credential in a volume of the same name. It restarts with Docker and reconnects on its own. `./connect-platform.sh --disconnect` removes it. Remove the device in the platform as well.

Available in `sensor_diagnostics`, `ota_nav2_sensor_fix` and `turtlebot3_integration`. For any other demo run `lib/connect-platform.sh <gateway container>`.

| Variable | Default | Use |
|---|---|---|
| `PLATFORM_URL` | `https://platform.selfpatch.ai` | The platform to pair with, for example a staging deployment |
| `PAIRING_CODE` | asked for | The code from the platform |
| `SELFPATCH_AGENT_IMAGE` | `ghcr.io/selfpatch/agent:0.1` | Agent image |
| `PLATFORM_INSECURE_HTTP` | `0` | `1` allows a plain http platform, for a local dev platform |
| `GATEWAY_CONTAINER` | the demo's gateway container | Which demo instance to connect |

## A platform running on your machine

The agent runs in a container, so `localhost` is the container itself. Point it at the host:

```bash
PLATFORM_URL=http://host.docker.internal:<platform port> PLATFORM_INSECURE_HTTP=1 ./connect-platform.sh
```

On Linux, `host.docker.internal` needs `--add-host=host.docker.internal:host-gateway` or the host's LAN address instead.

## Several robots

Every demo instance is its own device in the platform, with its own agent. The sensor demo can run several times side by side:

```bash
cd demos/sensor_diagnostics
COMPOSE_PROJECT_NAME=robot2 DEMO_CONTAINER=robot2 GATEWAY_PORT=8082 \
  WEB_UI_CONTAINER=robot2_web_ui WEB_UI_PORT=3002 ROS_DOMAIN_ID=41 \
  docker compose up -d --build
GATEWAY_CONTAINER=robot2 ./connect-platform.sh
```

Give each instance its own `ROS_DOMAIN_ID` so the robots do not see each other's ROS graph. The fault scripts take `GATEWAY_URL`, for example `GATEWAY_URL=http://localhost:8082 ./inject-noise.sh`.

## Making something happen

| Demo | Faults | Back to normal |
|---|---|---|
| `sensor_diagnostics` | `./inject-noise.sh`, `./inject-drift.sh`, `./inject-nan.sh`, `./inject-failure.sh`, `./inject-fault-scenario.sh` (all sensors at once) | `./restore-normal.sh` |
| `ota_nav2_sensor_fix` | `./send-goal.sh`: the robot drives into the phantom obstacle of the broken lidar, Nav2 aborts | `./publish-fix.sh`, `./apply-fix.sh`, `./send-goal.sh` again |
| `turtlebot3_integration` | `./inject-nav-failure.sh`, `./inject-localization-failure.sh` | `./restore-normal.sh` |

## What the gateway exposes

The platform shows what the demo's gateway returns. The full list of routes is in the gateway's own OpenAPI document: `curl http://localhost:8080/api/v1/docs`. The parts a platform view usually needs:

- Entity tree: `/areas`, `/components`, `/apps`, `/functions`, each with `/{id}` and relations such as `/components/{id}/hosts`.
- Faults: `/faults` (all), `/faults/stream` (server-sent events), `/{entity}/faults`, `/{entity}/faults/{fault_code}` with freeze frames and the recording reference.
- Recordings: `/{entity}/bulk-data` and `/{entity}/bulk-data/rosbags/{id}` (MCAP).
- Live values: `/{entity}/data` and `/{entity}/data/{data_id}`.
- Logs: `/{entity}/logs`.
- Health and version: `/health`, `/version-info`.

Through the platform the agent forwards read requests (GET) only. Operations, scripts, configuration writes and fault clearing work on the gateway directly (`http://localhost:8080`) but not through the platform.

## Building platform UI on a demo

Build against what the gateway actually returns, not against mocked data. Before adding a view, call the endpoint on a running demo and use that response. If a view needs data the gateway does not expose, that is a finding, not something to mock: open an issue in [ros2_medkit](https://github.com/selfpatch/ros2_medkit/issues) with the view you want, the endpoint you tried and its response. Some gaps are by design and some are missing features, and the issue is where that gets decided.
