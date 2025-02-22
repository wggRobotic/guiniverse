#pragma once

#include <GLFW/glfw3.h>

enum InputProfile 
{
    INPUT_PROFILE_UNKNOWN = -1,
    INPUT_PROFILE_LOGITECH_JOYSTICK,
    INPUT_PROFILE_XBOX_CONTROLLER,
    INPUT_PROFILE_MAX_ENUM
};

static const char* known_m_InputDeviceNames[] = 
{
    "Logitech Logitech Extreme 3D",
    "XBOX Controller"
};

class JoystickInput 
{
public:

    void update();
    void ImGuiPanel(char* label);

    bool getButton(unsigned int i);
    float getAxis(unsigned int i);
    int getDeviceProfile();

private:
    
    bool m_InputDevicesConnected[GLFW_JOYSTICK_LAST + 1];
    char m_InputDeviceNames[GLFW_JOYSTICK_LAST + 1][128];
    char m_InputDeviceProfiles[GLFW_JOYSTICK_LAST + 1];
    int m_InputDeviceSelected = 0;

    bool m_RefreshButton = true;

    bool m_Buttons[32] = {false};
    float m_Axes[16] = {0};
    bool m_InvertAxes[GLFW_JOYSTICK_LAST + 1][16] = {{true, true, false, true}};
};