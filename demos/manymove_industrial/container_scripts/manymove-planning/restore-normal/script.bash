#!/bin/bash
set -eu

source /opt/ros/jazzy/setup.bash

for code in MANYMOVE_PLANNER_COLLISION_DETECTED MANYMOVE_PLANNER_RETRY_ATTEMPT \
            MANYMOVE_PLANNER_RETRIES_EXHAUSTED MANYMOVE_OBJECT_WAIT_TIMEOUT; do
  ros2 service call /fault_manager/report_fault \
    ros2_medkit_msgs/srv/ReportFault \
    "{fault_code: '${code}', \
      event_type: 1, \
      severity: 0, \
      description: 'restore-normal: synthetic clear', \
      source_id: '/bt_client_xarm7'}" \
    || true
done
