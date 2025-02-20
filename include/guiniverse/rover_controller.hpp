#pragma once

#include <guiniverse/input.hpp>
#include <vector>
#include <imgui.h>
#include <GLFW/glfw3.h>

struct rover_controller_wheel
{
    ImVec2 position = ImVec2(0, 0);

    float last_angle = 0;
    float last_speed = 0;

};

struct rover_controller
{
public:

    rover_controller(GLFWwindow* w);

    void add_wheel(ImVec2 position);

    void ImGui_panel_control(char* label);
    void ImGui_panel_visualisation(char* label);
    void get_input(input_state& input);
    void process();
    void exchange_ros_data();

    ImVec2 get_linear_velocity() {return linear_velocity;};
    float get_angular_velocity() {return angular_velocity;};

private:

    rover_controller() = delete;

    GLFWwindow* window;

    std::vector<rover_controller_wheel> wheels;

    ImVec2 main_axes = ImVec2(0.f, 0.f);
    float scalar = 0.5f;
    float joystick_scalar = 0.5f;

    bool gas_button = false;

    bool enable_button = false;
    bool disable_button = false;
    bool physical_enable_button = false;
    bool physical_disable_button = false;

    bool gripper_mode_button = false;
    bool drive_mode_button = false;

    bool dog_walk_button = false;

    bool gripper_mode = false;

    ImVec2 linear_velocity = ImVec2(0.f, 0.f);
    float angular_velocity = 0.f;

    bool should_set_motor_status = false;
    bool motor_status = false;
};