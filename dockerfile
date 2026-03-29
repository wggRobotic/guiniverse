FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt update
RUN apt install -y ros-humble-cv-bridge ros-humble-image-transport ros-humble-compressed-image-transport
RUN apt install -y ros-humble-rmw-cyclonedds-cpp

RUN apt install -y \
  libopencv-dev \
  libglew-dev \
  libglfw3-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  pkg-config

RUN apt install -y \
  build-essential \
  cmake \
  git

RUN apt install -y \
  libx11-dev \
  libxrandr-dev \
  libxinerama-dev \
  libxcursor-dev \
  libxi-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev

RUN apt install -y \
  libwayland-dev \
  libwayland-client0 \
  libwayland-cursor0 \
  libwayland-egl1-mesa \
  libxkbcommon-dev

WORKDIR /ros2_ws
COPY ./guiniverse /ros2_ws/src/guiniverse
RUN . /opt/ros/humble/setup.bash && colcon build

CMD . /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /guiniverse_config && ros2 run guiniverse main
