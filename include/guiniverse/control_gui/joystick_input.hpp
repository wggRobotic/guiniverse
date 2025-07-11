#pragma once

#include <GLFW/glfw3.h>

class JoystickInput
{
public:
    void ImGuiPanel(const char* label);

    bool GetButton(unsigned i);
    float GetAxis(unsigned i);

private:
    int m_SelectedJoystick = -1;
};
