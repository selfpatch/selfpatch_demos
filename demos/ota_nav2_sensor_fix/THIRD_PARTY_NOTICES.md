# Third-party notices

These assets are **fetched at Docker build time** from their upstream repositories at
the pinned commits below (see `Dockerfile.gateway`), not vendored in this repo - only
our RB-Theron wrapper (`ros2_packages/ota_nav2_sensor_fix_demo/urdf/warehouse_rbtheron.urdf.xacro`),
our gz-port world (`ros2_packages/ota_nav2_sensor_fix_demo/models/aws_small_warehouse/worlds/warehouse.sdf`),
and the build-time `file://models/` -> `model://` rewrite are ours. The pinned commits
are the source of truth; the Dockerfile passes them as the `ROBOTNIK_DESC_REF` /
`ROBOTNIK_SENS_REF` / `AWS_WAREHOUSE_REF` build args. Each upstream's license travels
with its clone (a `LICENSE` file at the repo root).

| Component | Source | License | Commit |
| --- | --- | --- | --- |
| Robotnik RB-Theron description | https://github.com/RobotnikAutomation/robotnik_description (jazzy-devel) | BSD-3-Clause | 751059edd6af3a9c083018cfaee59e4496d46580 |
| Robotnik sensors | https://github.com/RobotnikAutomation/robotnik_sensors (jazzy-devel) | BSD-3-Clause | e5186c343910b86a924201edb256f79eb0f73295 |
| AWS small warehouse world | https://github.com/aws-robotics/aws-robomaker-small-warehouse-world (ros2) | MIT-0 | ee0af733315e78432408c3cd98d378ecee5f767c |
