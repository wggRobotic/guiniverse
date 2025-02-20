#pragma once

#include <GLFW/glfw3.h>

enum InputProfile 
{
    INPUT_PROFILE_UNKNOWN = -1,
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

    void update();
    void ImGui_panel(char* label);

    bool get_button(unsigned int i) 
    {
        if (i < 32 && input_device_selected_index != -1)
            if (input_device_profiles[input_device_selected_index] != INPUT_PROFILE_UNKNOWN) 
                return buttons[i];
        return false;
    };

    float get_axis(unsigned int i) 
    {
        if (i < 16 && input_device_selected_index != -1)
            if (input_device_profiles[input_device_selected_index] != INPUT_PROFILE_UNKNOWN) 
                return axes[i] * (invert_axes[input_device_selected_index][i] ? -1.f : 1.f);
        return 0.f;
    };

    int get_device_profile() { 
        if (input_device_selected_index != -1) 
            return input_device_profiles[input_device_selected_index];
        return INPUT_PROFILE_UNKNOWN;
    };

private:
    
    bool input_devices_connected[GLFW_JOYSTICK_LAST + 1];
    char input_device_names[GLFW_JOYSTICK_LAST + 1][128];
    char input_device_profiles[GLFW_JOYSTICK_LAST + 1];
    int input_device_selected_index = 0;

    bool refresh_button = true;

    bool buttons[32] = {false};
    float axes[16] = {0};
    bool invert_axes[GLFW_JOYSTICK_LAST + 1][16] = {{true, true, false, true}};
};