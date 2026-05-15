#!/bin/bash
set -e

source /home/project26-group3/connect-four/devel/setup.bash
exec python3 /home/project26-group3/connect-four/src/move/src/panel_motion.py column 1
