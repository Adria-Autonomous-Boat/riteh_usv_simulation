#!/bin/bash
LOG_DIR=${COLREG_LOG_DIR:-~/ros2_ws/logs/colreg_logs}
mkdir -p "$LOG_DIR"
LOG=$LOG_DIR/colreg_$(date +%Y%m%d_%H%M%S).log
echo "Logging to $LOG"
ros2 launch riteh_usv_sim colreg.launch.py "$@" 2>&1 | tee "$LOG"
