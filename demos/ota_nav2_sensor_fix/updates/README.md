# Registering an update by hand

`GET /updates` at boot lists only the bad update (`broken_lidar_3_0_0`). A new
update is not in the catalog until someone publishes it, and you publish it by
POSTing its descriptor - the JSON you see in `fixed_lidar_3_0_1.json`:

```bash
curl -X POST http://localhost:8080/api/v1/updates \
     -H 'Content-Type: application/json' \
     -d @updates/fixed_lidar_3_0_1.json
```

`./publish-fix.sh` is just that one call. Edit the JSON (or write your own) to
register any other update. The fields:

| Field | What it is |
|-------|------------|
| `id` | The update id used in every later call (`/updates/<id>/prepare`, `/execute`). |
| `update_name` | Human-readable name shown in the Updates panel. |
| `notes` | Free-text release note. |
| `x_medkit_version` | The build version (a vendor extension, so it is `x_medkit_*`). |
| `updated_components` | The SOVD component this update changes. `added_components` / `removed_components` instead would make it an install / uninstall. The kind is derived from which of the three you set. |
| `x_medkit_target_package` | The ROS 2 package the artifact installs. |
| `x_medkit_executable` | The binary the swapped-in node runs. |
| `x_medkit_replaces_executable` | The binary it replaces (so the plugin knows which process to kill on execute). |
| `x_medkit_artifact_url` | Where the update server serves the tarball. **This file must exist on the update server** (the demo ships `fixed_lidar-3.0.1.tar.gz`); `apply-fix.sh` fetches it during `prepare`. |
| `automated`, `origins`, `duration`, `size` | Informational metadata surfaced in the panel. |

After the POST, `GET /updates` includes your new id, and you apply it with
`./apply-fix.sh` (or `PUT /updates/<id>/prepare` then `/execute`).
