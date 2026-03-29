docker build -t guiniverse:dev .

xhost +local:docker

docker run \
    -it --rm --network host \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/dri:/dev/dri \
    -v guiniverse_config:/guiniverse_config \
    -e ROS_DOMAIN_ID=187 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    guiniverse:dev