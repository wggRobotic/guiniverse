cmake_minimum_required(VERSION 3.10)
project(guiniverse)

cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLEW REQUIRED glew)

# GLFW hinzufügen
add_subdirectory(lib/glfw)

# ImGui hinzufügen
add_library(imgui STATIC
  lib/imgui/imgui.cpp
  lib/imgui/imgui_draw.cpp
  lib/imgui/imgui_tables.cpp
  lib/imgui/imgui_widgets.cpp
  lib/imgui/backends/imgui_impl_opengl3.cpp
  lib/imgui/backends/imgui_impl_glfw.cpp
)

# Include-Verzeichnisse
target_include_directories(imgui PUBLIC
  lib/imgui
  lib/imgui/backends
  ${CMAKE_SOURCE_DIR}/lib/glfw/include
)

# Hauptprogramm
add_executable(main src/main.cpp)

# Abhängigkeiten verlinken
target_link_libraries(main
  imgui
  glfw
  OpenGL::GL
  ${GLEW_LIBRARIES}
)

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
