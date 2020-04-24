xhost +si:localuser:root

docker build -t rosrviz .

XAUTH=/tmp/.docker.xauth
xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
if [ ! -f /tmp/.docker.xauth ]
then
  export XAUTH=/tmp/.docker.xauth
  xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi
DISPLAY="${DISPLAY:-:0}"

docker run -it --net=host \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -e "TERM=xterm-256color" \
    -v "/tmp/.docker.xauth:/tmp/.docker.xauth" \
    --runtime=nvidia \
    rosrviz 
