# OpenPLC Tank Demo

PLC diagnostics demo using OpenPLC v4 + ros2_medkit OPC-UA plugin.

## Architecture

```
OpenPLC v4 Runtime (Docker)           ros2_medkit Gateway
  [Tank ST program]                     [opcua_plugin.so]
  OPC-UA Server :4840  <-- sub -->   open62541pp client
                                        |
                                      OpcuaPoller
                                        |
                                +---------+---------+
                                |         |         |
                          IntrospectionProvider   REST   FaultManager
                          (PLC entity tree)      routes  (alarm->fault)
```

## SOVD Entity Tree

```
Area: plc_systems
  Component: openplc_runtime
    App: tank_process     (level, temperature, pressure + alarms)
    App: fill_pump        (speed control)
    App: drain_valve      (position control)
```

## Quick Start

```bash
./scripts/run-demo.sh

# Wait ~30s for startup, then:
curl http://localhost:8080/api/v1/health
curl http://localhost:8080/api/v1/apps | jq '.items[].id'
curl http://localhost:8080/api/v1/apps/tank_process/x-plc-data | jq .
```

## Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/v1/areas` | List areas (plc_systems) |
| GET | `/api/v1/components` | List components (openplc_runtime) |
| GET | `/api/v1/apps` | List apps (tank_process, fill_pump, drain_valve) |
| GET | `/apps/{id}/x-plc-data` | All OPC-UA values for entity |
| GET | `/apps/{id}/x-plc-data/{node}` | Single value |
| POST | `/apps/{id}/x-plc-operations/{op}` | Write value `{"value": 75.0}` |
| GET | `/components/{id}/x-plc-status` | Connection state, stats |
| GET | `/apps/{id}/faults` | SOVD faults (from PLC alarms) |

## Fault Injection

```bash
# Inject high temperature (>80C threshold)
./scripts/inject-fault.sh high_temp

# Inject low level (<100mm threshold)
./scripts/inject-fault.sh low_level

# Inject overpressure (>5bar threshold)
./scripts/inject-fault.sh overpressure

# Restore normal operation
./scripts/restore-normal.sh
```

## Control PLC via REST

```bash
# Set pump speed to 75%
curl -X POST http://localhost:8080/api/v1/apps/fill_pump/x-plc-operations/set_pump_speed \
  -H "Content-Type: application/json" -d '{"value": 75.0}'

# Set valve position to 50%
curl -X POST http://localhost:8080/api/v1/apps/drain_valve/x-plc-operations/set_valve_position \
  -H "Content-Type: application/json" -d '{"value": 50.0}'
```

## Integration Tests

```bash
./scripts/run_integration_tests.sh
```

Tests cover: entity discovery, live data, control writes, fault lifecycle, connection resilience, error responses.

## Ports

| Service | Port | Protocol |
|---------|------|----------|
| OpenPLC OPC-UA | 4840 | OPC-UA TCP |
| Gateway REST | 8080 | HTTP |
| Web UI | 3000 | HTTP |

## Stopping

```bash
./scripts/stop-demo.sh          # stop containers
./scripts/stop-demo.sh --volumes # stop + remove volumes
```
