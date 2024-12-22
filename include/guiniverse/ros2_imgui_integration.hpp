#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>

extern std::atomic<bool> running;
extern std::mutex data_mutex;
extern std::string shared_data;

void ros2_main_thread();
void imgui_thread();
