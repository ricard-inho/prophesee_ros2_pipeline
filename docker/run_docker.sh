#!/bin/bash

IMAGE_NAME="ros2-humble"

docker run -it --rm --privileged \
    -e DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    -v /dev/bus/usb:/deb/bus/usb \
    -v $(pwd):/workspace \
    -w /workspace \
    $IMAGE_NAME
