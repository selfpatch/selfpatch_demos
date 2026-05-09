#!/bin/bash
set -eu

source /opt/ros/jazzy/setup.bash

ros2 service call /fault_manager/report_fault \
  ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: 'MANYMOVE_PLANNER_COLLISION_DETECTED', \
    event_type: 0, \
    severity: 2, \
    description: 'inject-collision: synthetic collision on bt_client_xarm7', \
    source_id: '/bt_client_xarm7'}"
