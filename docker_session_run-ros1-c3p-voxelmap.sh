#!/bin/bash

IMAGE_NAME='c3p-voxelmap_noetic'
TMUX_SESSION='ros1_session'

DATASET_CONTAINER_PATH='/ros_ws/dataset/recorded-c3p-voxelmap.bag'
BAG_OUTPUT_CONTAINER='/ros_ws/recordings'

RECORDED_BAG_NAME="recorded-c3p-voxelmap.bag"
HDMAPPING_OUT_NAME="output_hdmapping"

usage() {
  echo "Usage:"
  echo "  $0 <input.bag> <output_dir>"
  echo
  echo "If no arguments are provided, a GUI file selector will be used."
  exit 1
}

echo "=== c3p-voxelmap rosbag pipeline ==="

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
fi

if [[ $# -eq 2 ]]; then
  DATASET_HOST_PATH="$1"
  BAG_OUTPUT_HOST="$2"
elif [[ $# -eq 0 ]]; then
  command -v zenity >/dev/null || {
    echo "Error: zenity is not available"
    exit 1
  }
  DATASET_HOST_PATH=$(zenity --file-selection --title="Select BAG file")
  BAG_OUTPUT_HOST=$(zenity --file-selection --directory --title="Select output directory")
else
  usage
fi

if [[ -z "$DATASET_HOST_PATH" || -z "$BAG_OUTPUT_HOST" ]]; then
  echo "Error: no file or directory selected"
  exit 1
fi

if [[ ! -f "$DATASET_HOST_PATH" ]]; then
  echo "Error: BAG file does not exist: $DATASET_HOST_PATH"
  exit 1
fi

mkdir -p "$BAG_OUTPUT_HOST"

DATASET_HOST_PATH=$(realpath "$DATASET_HOST_PATH")
BAG_OUTPUT_HOST=$(realpath "$BAG_OUTPUT_HOST")

echo "Input bag : $DATASET_HOST_PATH"
echo "Output dir: $BAG_OUTPUT_HOST"

xhost +local:docker >/dev/null

docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_PATH":ro \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c '

    tmux new-session -d -s '"$TMUX_SESSION"'

    # ---------- PANEL 1: ROS launch ----------
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 5
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
roslaunch c3p_voxelmap mapping_velodyne.launch
'\'' C-m

    # ---------- PANEL 2: rosbag record ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 2
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[record] start"
rosbag record /cloud_registered /aft_mapped_to_init /clock -O '"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME"'
echo "[record] exit"
'\'' C-m

    # ---------- PANEL 3: rosbag play ----------
    tmux split-window -h -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 5
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[play] start"
rosbag play '"$DATASET_CONTAINER_PATH"' --clock; tmux wait-for -S BAG_DONE;
echo "[play] done"
'\'' C-m

    # ---------- PANEL 4: controller ----------
    tmux new-window -t '"$TMUX_SESSION"' -n control '\''
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[control] waiting for play end"
tmux wait-for BAG_DONE
echo "[control] stop record"
tmux send-keys -t '"$TMUX_SESSION"'.1 C-c

sleep 5
rosnode kill -a

sleep 7
pkill -f ros || true
'\''

    tmux attach -t '"$TMUX_SESSION"'
  '

docker run -it --rm \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c "
    set -e
    source /opt/ros/noetic/setup.bash
    source /ros_ws/devel/setup.bash
    rosrun c3p-voxelmap-to-hdmapping listener  \
      \"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME\" \
      \"$BAG_OUTPUT_CONTAINER/$HDMAPPING_OUT_NAME-c3p-voxelmap\"
  "

echo "=== DONE ==="
