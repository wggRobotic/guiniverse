#include <guiniverse/imgui_util.hpp>
#include <guiniverse/no_name/no_name.hpp>

void NoName::OnGuiStartup()
{
    m_ImageSystem->OnGuiStartup();
}

void NoName::OnGuiShutdown()
{
    m_ImageSystem->OnGuiShutdown();
}

void NoName::OnGuiFrame(GLFWwindow* window, JoystickInput& input)
{
    (void) input;

    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        m_Input.EnableButtonPhysical = false;
        m_Input.DisableButtonPhysical = false;

        m_Input.MainAxes = ImVec2(0.f, 0.f);

        m_Input.GasButton = false;

        // if (device_name == "Logitech Logitech Extreme 3D")
        // {
        //     m_Input.MainAxes = ImVec2(input.GetAxis(0), input.GetAxis(1));
        //     float new_joystick_scalar = input.GetAxis(3) / 2.f + 0.5f;
        //     if (new_joystick_scalar != m_Input.joystick_scalar)
        //     {
        //         m_Input.joystick_scalar = new_joystick_scalar;
        //         m_Input.scalar = new_joystick_scalar;
        //     }
        //     m_Input.GasButton |= input.GetButton(0);
        //     m_Input.EnableButtonPhysical |= input.GetButton(10);
        //     m_Input.DisableButtonPhysical |= input.GetButton(11);
        // }

        if (glfwGetKey(window, GLFW_KEY_A))
            m_Input.MainAxes.x = 1.f;
        else if (glfwGetKey(window, GLFW_KEY_D))
            m_Input.MainAxes.x = -1.f;
        if (glfwGetKey(window, GLFW_KEY_W))
            m_Input.MainAxes.y = 1.f;
        else if (glfwGetKey(window, GLFW_KEY_S))
            m_Input.MainAxes.y = -1.f;

        float length = sqrtf(
            m_Input.MainAxes.x * m_Input.MainAxes.x
            + m_Input.MainAxes.y * m_Input.MainAxes.y);
        if (length > 1.0f)
            m_Input.MainAxes = ImVec2(
                m_Input.MainAxes.x / length,
                m_Input.MainAxes.y / length);

        m_Input.GasButton |= glfwGetKey(window, GLFW_KEY_SPACE);

        m_Input.EnableButtonPhysical |= glfwGetKey(window, GLFW_KEY_F);
        m_Input.DisableButtonPhysical |= glfwGetKey(window, GLFW_KEY_R);
    }

    if (ImGui::Begin("Control"))
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        ImVec2 pos = ImGui::GetCursorScreenPos();

        m_Input.MainAxes = imgui_joystick(
            "virtual joystick",
            200.f,
            ImVec2(0.2f, 0.2f),
            (m_Input.MainAxes.x == 0.f && m_Input.MainAxes.y == 0.f) ? 0
                                                                     : &m_Input.MainAxes,
            (m_Input.GasButton ? IM_COL32(150, 150, 150, 255) : IM_COL32(80, 80, 80, 255)));

        ImGui::Text("Joystick %f %f", m_Input.MainAxes.x, m_Input.MainAxes.y);

        ImGui::Button("Enable");
        if (ImGui::IsItemActive())
            m_Input.EnableButtonPhysical = true;
        ImGui::Button("Disable");
        if (ImGui::IsItemActive())
            m_Input.DisableButtonPhysical = true;

        ImGui::SetCursorScreenPos(
            ImVec2(pos.x + 220.f, pos.y + ImGui::GetStyle().ItemSpacing.y));

        ImGui::VSliderFloat("##vslider", ImVec2(20, 200), &m_Input.Scalar, 0.0f, 1.0f, "%.2f");
    }
    ImGui::End();

    if (ImGui::Begin("WheelVisualization"))
    {
        std::lock_guard<std::mutex> lock(m_WheelsMutex);

        for (unsigned i = 0; i < m_Wheels.size(); i++)
        {
            imgui_arrow(
                ImVec2(-m_Wheels.at(i).Y * 500.f + 150.f, -m_Wheels.at(i).X * 500.f + 200.f),
                0.f,
                m_Wheels.at(i).TargetRPM * (m_Wheels.at(i).Invert ? -1.f : 1.f),
                IM_COL32(60, 60, 60, 255),
                3.f,
                9.f,
                false);

            imgui_arrow(
                ImVec2(-m_Wheels.at(i).Y * 500.f + 150.f, -m_Wheels.at(i).X * 500.f + 200.f),
                0.f,
                m_Wheels.at(i).LastRPM * (m_Wheels.at(i).Invert ? -1.f : 1.f),
                IM_COL32(255, 255, 255, 255),
                5.f,
                10.f,
                false);
        }
    }
    ImGui::End();

    m_ImageSystem->OnGuiFrame();
    m_DataCaptureSystem->ImGuiPanels();
}
