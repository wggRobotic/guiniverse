#include <GLFW/glfw3.h>
#include <guiniverse/control_gui/joystick_input.hpp>
#include <imgui.h>

void JoystickInput::ImGuiPanel(const char* label)
{
    if (!ImGui::Begin(label))
    {
        ImGui::End();
        return;
    }

    if (ImGui::BeginCombo("select joystick", glfwGetJoystickName(m_SelectedJoystick)))
    {
        if (ImGui::Selectable("no joystick selected", m_SelectedJoystick == -1))
        {
            m_SelectedJoystick = -1;
        }

        for (int jid = 0; jid <= GLFW_JOYSTICK_LAST; jid++)
        {
            ImGui::PushID(jid);
            if (glfwJoystickIsGamepad(jid) && ImGui::Selectable(glfwGetJoystickName(jid), m_SelectedJoystick == jid))
            {
                m_SelectedJoystick = jid;
            }
            ImGui::PopID();
        }

        ImGui::EndCombo();
    }

    ImGui::End();
}

bool JoystickInput::GetButton(const unsigned i)
{
    GLFWgamepadstate state;
    if (!glfwGetGamepadState(m_SelectedJoystick, &state))
        return 0.f;
    return state.buttons[i];
};

float JoystickInput::GetAxis(const unsigned i)
{
    GLFWgamepadstate state;
    if (!glfwGetGamepadState(m_SelectedJoystick, &state))
        return 0.f;
    return state.axes[i];
};
