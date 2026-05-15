#!/bin/bash
set -e

source /home/project26-group3/connect-four/devel/setup.bash
exec rostopic pub -1 /robot/gripper/command control_msgs/GripperCommand '{position: 0.0, max_effort: 0.0}'
