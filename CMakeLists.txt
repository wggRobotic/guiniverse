cmake_minimum_required(VERSION 3.10)
project(guiniverse)

cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)

find_package(edu_robot REQUIRED)

find_package(OpenGL REQUIRED) 
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLEW REQUIRED glew)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
#

include_directories(
  ${GST_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

  lib/quirc/lib
)


add_subdirectory(lib/glfw)

add_library(imgui STATIC
  lib/imgui/imgui.cpp
  lib/imgui/imgui_draw.cpp
  lib/imgui/imgui_tables.cpp
  lib/imgui/imgui_widgets.cpp
  lib/imgui/imgui_demo.cpp
  lib/imgui/backends/imgui_impl_opengl3.cpp
  lib/imgui/backends/imgui_impl_glfw.cpp
)
target_include_directories(imgui PUBLIC lib/imgui)

target_link_libraries(imgui
  OpenGL::GL
  ${GLEW_LIBRARIES}
  glfw
)

#quirc 
add_library(quirc STATIC
  lib/quirc/lib/decode.c
  lib/quirc/lib/identify.c
  lib/quirc/lib/quirc.c
  lib/quirc/lib/version_db.c
)
target_include_directories(imgui PUBLIC lib/imgui)
#


file(GLOB_RECURSE src src/*.cpp)
add_executable(main ${src})

target_include_directories(main PRIVATE include)

ament_target_dependencies(main
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  OpenCV
  cv_bridge
  image_transport
  compressed_image_transport
  edu_robot
  std_srvs
)

target_link_libraries(main
  glfw
  OpenGL::GL
  ${GLEW_LIBRARIES}
  imgui
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  quirc
  ${OpenCV_LIBS}
)

# Install target
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
