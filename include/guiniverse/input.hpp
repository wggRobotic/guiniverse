#pragma once

#include <GLFW/glfw3.h>
#include <imgui.h>

enum InputButton
{
    INPUT_BUTTON_GAS,
    INPUT_BUTTON_ENABLE,
    INPUT_BUTTON_DISABLE,
    INPUT_BUTTON_MAX_ENUM
};

enum InputAxis
{
    INPUT_AXIS_MAIN_X,
    INPUT_AXIS_MAIN_Y,
    INPUT_AXIS_SCALAR,
    INPUT_AXIS_MAX_ENUM,
};

enum InputProfile 
{
    INPUT_PROFILE_LOGITECH_JOYSTICK,
    INPUT_PROFILE_XBOX_CONTROLLER,
    INPUT_PROFILE_MAX_ENUM
};

static const char* known_input_device_names[] = {
    "Logitech Logitech Extreme 3D",
    "XBOX Controller"
};

static const int input_button_mappings[INPUT_PROFILE_MAX_ENUM][INPUT_BUTTON_MAX_ENUM] = {
    {0},
};

static const int input_axes_mappings[INPUT_PROFILE_MAX_ENUM][INPUT_AXIS_MAX_ENUM] = {
    {0, 1, 3},
};

struct input_state 
{
public:
    input_state(GLFWwindow* w) : window(w) {};

    void update();
    void imgui_panel();

    bool get_button(int i) 
    {
        if(i < INPUT_BUTTON_MAX_ENUM) 
            return buttons[i];
        return false;
    };

    float get_axis(int i) 
    {
        if(i < INPUT_AXIS_MAX_ENUM) 
            return axes[i];
        return 0.f;
    };

private:
    
    input_state() = delete;
    
    GLFWwindow* window;
    
    bool input_devices_connected[GLFW_JOYSTICK_LAST + 1];
    char input_device_names[GLFW_JOYSTICK_LAST + 1][128];
    char input_device_profiles[GLFW_JOYSTICK_LAST + 1];
    int input_device_selected_index = -1;

    bool refresh_button = true;

    bool buttons[INPUT_BUTTON_MAX_ENUM] = {false};
    float axes[INPUT_AXIS_MAX_ENUM] = {0};
};