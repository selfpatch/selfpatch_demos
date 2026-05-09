#!/bin/bash
set -eu

source /opt/ros/jazzy/setup.bash

CODE="MANYMOVE_SELFTEST"

ros2 service call /fault_manager/report_fault \
  ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: '${CODE}', event_type: 0, severity: 0, \
    description: 'arm-self-test: pipeline OK', source_id: '/bt_client_xarm7'}"

sleep 1

ros2 service call /fault_manager/report_fault \
  ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: '${CODE}', event_type: 1, severity: 0, \
    description: 'arm-self-test: cleared', source_id: '/bt_client_xarm7'}"
