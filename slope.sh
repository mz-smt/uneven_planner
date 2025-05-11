#!/bin/bash
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]:-${(%):-%x}}" )" >/dev/null 2>&1 && pwd )"
source ${CURRENT_DIR}/devel/setup.bash

roslaunch plan_manager run_slope_20deg.launch > output.log 2>&1 & sleep 1;
wait;
