#pragma once

#include <GLFW/glfw3.h>
#include <imgui.h>

enum input_profile 
{
    INPUT_PROFILE_LOGITECH_JOYSTICK,
    INPUT_PROFILE_XBOX_CONTROLLER,
    INPUT_PROFILE_MAX_ENUM
};

static const char* known_input_device_names[] = {
    "Logitech Logitech Extreme 3D",
    "XBOX Controller"
};

struct input_state 
{
public:
    input_state(GLFWwindow* w) : window(w) {};

    void update();
    void imgui_panel();

    ImVec2 get_main_axes() {return scaled_main_axes;};

private:
    
    input_state() = delete;
    
    GLFWwindow* window;
    
    bool input_devices_connected[GLFW_JOYSTICK_LAST + 1];
    char input_device_names[GLFW_JOYSTICK_LAST + 1][128];
    char input_device_profiles[GLFW_JOYSTICK_LAST + 1];
    int input_device_selected_index = -1;

    bool refresh_button = true;

    ImVec2 main_axes = ImVec2(0.f, 0.f);
    float scalar = 1.f;

    ImVec2 scaled_main_axes = ImVec2(0.f, 0.f);
};