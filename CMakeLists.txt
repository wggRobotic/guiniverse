cmake_minimum_required(VERSION 3.10)
project(guiniverse)

# Set policies and compiler flags
cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)

# Compiler specific flags for GNU or Clang compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
pkg_check_modules(GLEW REQUIRED glew)

# Add GLFW
add_subdirectory(lib/glfw)

# Add ImGui
add_library(imgui STATIC
  lib/imgui/imgui.cpp
  lib/imgui/imgui_draw.cpp
  lib/imgui/imgui_tables.cpp
  lib/imgui/imgui_widgets.cpp
  lib/imgui/imgui_demo.cpp
  lib/imgui/backends/imgui_impl_opengl3.cpp
  lib/imgui/backends/imgui_impl_glfw.cpp
)

# Include directories for ImGui
target_include_directories(imgui PUBLIC lib/imgui)

# Link OpenGL and GLFW libraries with ImGui
target_link_libraries(imgui
  OpenGL::GL
  ${GLEW_LIBRARIES}
  glfw
)

# Add main executable
file(GLOB_RECURSE src src/*.cpp)
add_executable(main ${src})

# Include directories for main executable
target_include_directories(main PRIVATE include)

# ROS2 dependencies
ament_target_dependencies(main
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  OpenCV
  cv_bridge
  image_transport
  compressed_image_transport
  std_srvs
)

# Link libraries to the main executable
target_link_libraries(main
  glfw
  OpenGL::GL
  ${GLEW_LIBRARIES}
  imgui
)

# Install target
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
