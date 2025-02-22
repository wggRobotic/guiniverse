#include <guiniverse/ControlGui/JoystickInput.hpp>

#include <stdio.h>
#include <string.h>
#include <imgui.h>

void JoystickInput::update() 
{

    int joystick_axes_count;
    float* joystick_axes;
    int joystick_button_count;
    unsigned char* joystick_buttons;

    if (m_RefreshButton)
    {
        m_RefreshButton = false;
        for (int i = 0; i <= GLFW_JOYSTICK_LAST; i++) 
        {
            m_InputDevicesConnected[i] = false;
            if ((m_InputDevicesConnected[i] = glfwJoystickPresent(GLFW_JOYSTICK_1 + i)) != 0) 
            {
                const char* name = glfwGetJoystickName(GLFW_JOYSTICK_1 + i);
                if (name) snprintf(m_InputDeviceNames[i], sizeof(m_InputDeviceNames[i]), "%s", name);
                else snprintf(m_InputDeviceNames[i], sizeof(m_InputDeviceNames[i]), "Couldn't read device name");
                
                m_InputDeviceProfiles[i] = INPUT_PROFILE_UNKNOWN;

                for (int j = 0; j < INPUT_PROFILE_MAX_ENUM; j++)
                {
                    if (strcmp(known_m_InputDeviceNames[j], m_InputDeviceNames[i]) == 0)
                    {
                        m_InputDeviceProfiles[i] = j;
                        break;
                    }
                }

            }
            else if (i == m_InputDeviceSelected) m_InputDeviceSelected = -1;
        }
    }    

    if (m_InputDeviceSelected != -1) 
    {
        joystick_axes = (float*) glfwGetJoystickAxes(m_InputDeviceSelected, &joystick_axes_count);
        joystick_buttons = (unsigned char*) glfwGetJoystickButtons(m_InputDeviceSelected, &joystick_button_count);

        if (joystick_buttons == NULL || joystick_axes == NULL) 
        {
            m_InputDevicesConnected[m_InputDeviceSelected] = false;
            m_InputDeviceSelected = -1;
        }
    }

    for (int i = 0; i < 32; i++) m_Buttons[i] = false;
    for (int i = 0; i < 16; i++) m_Axes[i] = false;

    if (m_InputDeviceSelected != -1) if (m_InputDeviceProfiles[m_InputDeviceSelected] != INPUT_PROFILE_UNKNOWN) 
    {

        for (int i = 0; i < joystick_button_count; i++) 
            m_Buttons[i] = joystick_buttons[i];
        for (int i = 0; i < joystick_axes_count; i++) 
            m_Axes[i] = joystick_axes[i];
    }

}

void JoystickInput::ImGuiPanel(char* label) {

    if (ImGui::Begin(label))
    {
        if (ImGui::BeginCombo("Select input device", m_InputDeviceSelected == -1 ? "No device selected" : m_InputDeviceNames[m_InputDeviceSelected]))
        {
            if (ImGui::Selectable("No device selected", m_InputDeviceSelected == -1))
                m_InputDeviceSelected = -1;

            for (int i = 0; i <= GLFW_JOYSTICK_LAST; i++) if (m_InputDevicesConnected[i] == true) 
            {
                if (ImGui::Selectable(m_InputDeviceNames[i], m_InputDeviceSelected == i)) 
                    m_InputDeviceSelected = i;
            }
            
            ImGui::EndCombo();
        }

        if (ImGui::Button("Refresh"))
            m_RefreshButton = true;

        if (m_InputDeviceSelected != -1) 
        {
            if (m_InputDeviceProfiles[m_InputDeviceSelected] == INPUT_PROFILE_UNKNOWN) ImGui::TextColored(ImVec4(1.0, 0.0, 0.0, 1.0), "Couldn't find input profile for this device");
            else 
            {
                ImGui::Text("Invert axes:");
                char invert_text[128];

                for (int i = 0; i < 16; i++) 
                {
                    snprintf(invert_text, sizeof(invert_text), "INPUT_AXIS_%d", i);
                    ImGui::Checkbox(invert_text, &m_InvertAxes[m_InputDeviceSelected][i]);
                }
            }
        }

        ImGui::End();
    }

}

bool JoystickInput::getButton(unsigned int i) 
{
    if (i < 32 && m_InputDeviceSelected != -1)
        if (m_InputDeviceProfiles[m_InputDeviceSelected] != INPUT_PROFILE_UNKNOWN) 
            return m_Buttons[i];
    return false;
};

float JoystickInput::getAxis(unsigned int i) 
{
    if (i < 16 && m_InputDeviceSelected != -1)
        if (m_InputDeviceProfiles[m_InputDeviceSelected] != INPUT_PROFILE_UNKNOWN) 
            return m_Axes[i] * (m_InvertAxes[m_InputDeviceSelected][i] ? -1.f : 1.f);
    return 0.f;
};

int JoystickInput::getDeviceProfile() { 
    if (m_InputDeviceSelected != -1) 
        return m_InputDeviceProfiles[m_InputDeviceSelected];
    return INPUT_PROFILE_UNKNOWN;
};