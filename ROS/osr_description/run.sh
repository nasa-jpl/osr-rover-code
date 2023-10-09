#!/bin/bash

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

xhost +local:root
docker run -it --name osr_urdf --rm \
	       --privileged \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
           --env="XAUTHORITY=$XAUTH" \
           --volume="$XAUTH:$XAUTH" \
           --mount type=bind,src=$(pwd),dst=/osr_ws/src/osr_description \
           --net host \
           osr_urdf_img \
           /bin/bash

