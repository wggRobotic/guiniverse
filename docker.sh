docker build -t guiniverse:dev .

xhost +local:docker
VIDEO_TARGET_IP=$(ip -4 addr show wlo1 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')

docker run \
    -it --rm --network host \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/dri:/dev/dri \
    -v guiniverse_config:/guiniverse_config \
    -e ROS_DOMAIN_ID=187 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    guiniverse:dev \
    bash -c ". /opt/ros/humble/setup.bash && \
             . /ros2_ws/install/setup.bash && \
             cd /guiniverse_config && \
             ros2 run guiniverse main --ros-args -p host_ip:=$VIDEO_TARGET_IP"