#pragma once

#include <rclcpp/rclcpp.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <signal.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

extern std::atomic<bool> running;
extern std::mutex data_mutex;
extern std::string shared_data;

void ros2_main_thread();
void imgui_thread();