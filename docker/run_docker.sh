#!/bin/bash

IMAGE_NAME="ros2-humble"
CONTAINER_NAME="ros2_event_camera"

xhost +
docker run -it --gpus all --rm --privileged \
    --name $CONTAINER_NAME \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -e DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    -v /dev/bus/usb:/deb/bus/usb \
    -e TZ=Europe/Luxembourg \
    -v $(pwd):/workspace \
    -w /workspace \
    $IMAGE_NAME
