#!/usr/bin/env bash
# Start or stop headless PX4 SITL x500 quadrotors in one Gazebo Harmonic world (WSL2 Ubuntu).
#
#   scripts/px4/sitl.sh start "x0,y0" ["x1,y1" ...]   spawn poses in the Gazebo world frame (ENU, metres)
#   scripts/px4/sitl.sh stop
#
# Instance i gets MAV_SYS_ID i+1 and sends its offboard MAVLink stream to UDP 14540+i
# (PX4 multi-vehicle defaults). Instance 0 starts the Gazebo server; the others attach to it.
# GUI=1 also opens the Gazebo client. See docs/integration/PX4_SETUP.md.
set -euo pipefail
PX4_DIR=${PX4_DIR:-$HOME/px4-v1.17.0}
LOG_DIR=${LOG_DIR:-/tmp/dtc-px4-sitl}
mkdir -p "$LOG_DIR"

stop() {
  for f in "$LOG_DIR"/px4-*.pid; do
    [ -e "$f" ] || continue
    kill "$(cat "$f")" 2>/dev/null || true
    rm -f "$f"
  done
  # The Gazebo server is a child of instance 0's startup script, not one of our pids.
  pkill -f "gz sim .*-s" 2>/dev/null || true
  pkill -f "gz sim -g" 2>/dev/null || true
  sleep 2
}

case "${1:-}" in
  stop) stop ;;
  start)
    shift
    [ $# -ge 1 ] || { echo "usage: $0 start \"x,y\" [\"x,y\" ...]" >&2; exit 2; }
    [ $# -le 3 ] || { echo "at most 3 vehicles on this machine (CPU budget)" >&2; exit 2; }
    stop
    cd "$PX4_DIR"
    i=0
    for pose in "$@"; do
      # PX4 SITL keeps parameters set at runtime in rootfs/<i>; start every run from the defaults.
      rm -f "build/px4_sitl_default/rootfs/$i/parameters.bson" "build/px4_sitl_default/rootfs/$i/parameters_backup.bson"
      if [ "${GUI:-0}" = 1 ]; then headless=""; else headless=1; fi
      HEADLESS=$headless PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="$pose" \
        nohup ./build/px4_sitl_default/bin/px4 -i "$i" -d > "$LOG_DIR/px4-$i.log" 2>&1 &
      echo $! > "$LOG_DIR/px4-$i.pid"
      if [ "$i" -eq 0 ]; then
        # Wait for the Gazebo world before attaching more vehicles.
        for _ in $(seq 60); do grep -q "Gazebo world is ready" "$LOG_DIR/px4-0.log" && break; sleep 1; done
        grep -q "Gazebo world is ready" "$LOG_DIR/px4-0.log" || { echo "Gazebo did not start, see $LOG_DIR/px4-0.log" >&2; exit 1; }
      fi
      i=$((i + 1))
    done
    echo "started $i vehicle(s); logs in $LOG_DIR"
    ;;
  *) echo "usage: $0 start \"x,y\" [...] | stop" >&2; exit 2 ;;
esac
