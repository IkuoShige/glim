#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -


docker run \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  --pid=host \
  --gpus all \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --env="DISPLAY=${DISPLAY}" \
  --env="XAUTHORITY=${XAUTH}" \
  -e=ROS_DOMAIN_ID \
  -v ./glim/config:/glim/config \
  glim_ros2:humble_cuda12.2 \
  bash
  #ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim/config
