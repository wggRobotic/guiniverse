#pragma once

#include <GLFW/glfw3.h>
#include <imgui.h>

enum input_mode 
{
    INPUT_KEYBOARD,
    INPUT_JOYSTICK_LOGITECH,
    INPUT_JOYSTICK_XBOX,
};

struct input_state 
{
public:
    input_state(GLFWwindow* w) : window(w) {};

    void control_panal_imgui();

    ImVec2 get_main_axes() {return main_axes;};
private:
    
    input_state() = delete;
    
    GLFWwindow* window;
    
    bool joystick_connected = true;
    int input_mode = INPUT_JOYSTICK_LOGITECH;

    ImVec2 main_axes = ImVec2(0.f, 0.f);
};