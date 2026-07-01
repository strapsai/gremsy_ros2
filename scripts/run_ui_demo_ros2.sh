#!/bin/bash
# Launch the ui_demo_ros2 GUI so its window appears on YOUR machine, while the
# process runs inside the dtc-drivers_ros2 container on nx-03 (pure ROS2 backend).
#
# Run it from your laptop through X11 forwarding:
#
#     ssh -X nx-03 bash /home/airlab/airlab_ws/spirit_drivers_ws/src/gremsy_ros2/scripts/run_ui_demo_ros2.sh
#
# The '-X' is what forwards the display; this script copies that session's X
# cookie into the container so GTK can authenticate. Pass a drone name as $1 to
# target a different spirit (default: $ROBOT_NAME or spiritnx3).
set -e

CONTAINER=dtc-drivers_ros2
WS=/home/airlab/airlab_ws/spirit_drivers_ws
DRONE="${1:-${ROBOT_NAME:-spiritnx3}}"

if [ -z "$DISPLAY" ]; then
  echo "ERROR: \$DISPLAY is empty. Run this via 'ssh -X nx-03 bash <this script>'." >&2
  exit 1
fi

NUM="${DISPLAY##*:}"; NUM="${NUM%.*}"
COOKIE=$(xauth list 2>/dev/null | awk -v n=":${NUM}\$" '$1 ~ n {print $3; exit}')

# If we have a TTY (i.e. launched via `ssh -Xt`), allocate one for docker exec so
# the GUI is killed when you disconnect instead of lingering in the container.
TT=""; [ -t 0 ] && TT="-t"

docker exec $TT -e DISPLAY="localhost:${NUM}.0" -e XCOOKIE="$COOKIE" -e XNUM="$NUM" -e DRONE="$DRONE" \
  "$CONTAINER" bash -ic '
    source '"$WS"'/install/setup.bash >/dev/null 2>&1
    export XAUTHORITY=/tmp/gremsy_ui_xauth; : > "$XAUTHORITY"
    for nm in localhost:$XNUM 127.0.0.1:$XNUM $(hostname):$XNUM $(hostname)/unix:$XNUM ; do
      xauth add "$nm" . "$XCOOKIE" 2>/dev/null
    done
    ros2 launch gremsy_ros2 ui_demo_ros2.launch.py drone:=$DRONE
  '
