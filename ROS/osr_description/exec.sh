#!/bin/bash

xhost +local:docker
docker exec -it \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           --env="XAUTHORITY=$XAUTH" \
           osr_urdf \
           /bin/bash
