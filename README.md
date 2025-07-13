# guiniverse (a GUI with ImGui)

## Overview

This project is a graphical user interface (GUI) for the robots we are working on currently.
The GUI provides various tools and visualizations for controlling and monitoring the selected robots operations.

## Step-by-Step Installation

#### Step 1: Set up ROS2 Workspace

If you don't have a ROS2 workspace yet, create one:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

#### Step 2: Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/wggRobotic/guiniverse
cd guiniverse
```

#### Step 3: Initialize Submodules

This project uses several submodules (GLFW, ImGui, quirc). Initialize them:

```bash
git submodule update --init --remote --recursive
```

#### Step 4: Install ROS2 Dependencies

Navigate back to your workspace root and install dependencies:

```bash
cd ~/ros2_ws

# Initialize rosdep (only needed once on your system)
sudo rosdep init  # Skip if already initialized

# Update rosdep database
rosdep update

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

#### Step 5: Build the Package

```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash  # Adjust for your ROS2 distribution

# Build the package
colcon build --cmake-force-configure --packages-select guiniverse \
             --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

#### Step 6: Source the Workspace

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Add to your .bashrc for automatic sourcing (optional)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Running the Application

After successful installation, you can run the GUI:

```bash
# Make sure you've sourced your workspace
source ~/ros2_ws/install/setup.bash

# Run the application
ros2 run guiniverse guiniverse
```


### For Development

If you're developing or debugging, use the build script:

```bash
cd ~/ros2_ws/src/guiniverse
./scripts/build.bash
```

## Supported Robots

This GUI supports multiple robot platforms. Each robot has its own control interface and features.

### N10 (Mars Rover)
- **Status**: ✅ Implemented
- **Features**: Full control interface with joystick input
- **Topics**: Publishes to `cmd_vel` for robot control

### Idefix (Robo Dog)
- **Status**: ✅ Implemented  
- **Features**: 
  - Virtual joystick control
  - Gas button for safety
  - IMU data visualization
  - Image streaming via GStreamer
  - Data capture system with QR code detection
- **Topics**: 
  - Publishes to `cmd_vel` (real robot)
  - Publishes to `/turtle1/cmd_vel` (TurtleSim simulation)
  - Subscribes to `imu` for orientation data

### NoName (4-Wheel Explorer)
- **Status**: 🚧 Coming soon
- **Features**: Planned implementation for 4-wheel drive robot

## Launch Files

You can also use the provided launch files:

```bash
# Launch the rover application
ros2 launch guiniverse rover.launch.py
```