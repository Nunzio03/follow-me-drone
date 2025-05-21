#!/bin/bash
IMAGE=tello:orientamento
echo $IMAGE

xhost +
docker run --rm -it --privileged --ipc host --net host \
    -e QT_X11_NO_MITSHM=1 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority \
    -e XAUTHORITY=/root/.Xauthority \
    -v .:/root/tello_ws\
    --name tello_tracking \
    $IMAGE bash
