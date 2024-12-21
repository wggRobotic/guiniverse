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
target_include_directories(imgui PUBLIC
  lib/imgui
  lib/imgui/backends
  ${CMAKE_SOURCE_DIR}/lib/glfw/include
)

# Link OpenGL and GLFW libraries with ImGui
target_link_libraries(imgui
  OpenGL::GL
  glfw
  ${GLEW_LIBRARIES}
)

# Add main executable
add_executable(main src/main.cpp)

# Include directories for main executable
target_include_directories(main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Include directories for ROS2 integration
target_include_directories(main PUBLIC
  lib/imgui
  ${CMAKE_SOURCE_DIR}/lib/glfw/include
  ${CMAKE_SOURCE_DIR}/include
)

# ROS2 dependencies
ament_target_dependencies(main
  rclcpp
)

# Link libraries to the main executable
target_link_libraries(main
  imgui
  glfw
  OpenGL::GL
  ${GLEW_LIBRARIES}
)

# Install target
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
