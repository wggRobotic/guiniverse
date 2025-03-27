#include <guiniverse/NoName/NoName.hpp>

#include <guiniverse/imgui_utils.hpp>

void NoName::onGuiStartup()
{
    m_ImageSystem->onGuiStartup();
}

void NoName::onGuiShutdown()
{
    m_ImageSystem->onGuiShutdown();
}

void NoName::onGuiFrame(GLFWwindow* window, JoystickInput& input)
{

    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        m_Input.enable_button_physical = false;
        m_Input.disable_button_physical = false;

        m_Input.main_axes = ImVec2(0.f, 0.f);

        m_Input.gas_button = false;

        auto device_name = input.getDeviceName();

        if (device_name == "Logitech Logitech Extreme 3D")
        {
            m_Input.main_axes = ImVec2(input.getAxis(0), input.getAxis(1));

            float new_joystick_scalar = input.getAxis(3) / 2.f + 0.5f;
            if (new_joystick_scalar != m_Input.joystick_scalar) {
                m_Input.joystick_scalar = new_joystick_scalar;
                m_Input.scalar = new_joystick_scalar;
            }

            m_Input.gas_button |= input.getButton(0);

            m_Input.enable_button_physical |= input.getButton(10);
            m_Input.disable_button_physical |= input.getButton(11);
        }

        if (glfwGetKey(window, GLFW_KEY_A)) m_Input.main_axes.x = 1.f;
        else if (glfwGetKey(window, GLFW_KEY_D)) m_Input.main_axes.x = -1.f;
        if (glfwGetKey(window, GLFW_KEY_W)) m_Input.main_axes.y = 1.f;
        else if (glfwGetKey(window, GLFW_KEY_S)) m_Input.main_axes.y = -1.f;

        float length = sqrtf(m_Input.main_axes.x * m_Input.main_axes.x + m_Input.main_axes.y * m_Input.main_axes.y);
        if (length > 1.0f) m_Input.main_axes = ImVec2(m_Input.main_axes.x / length, m_Input.main_axes.y / length);

        m_Input.gas_button |= glfwGetKey(window, GLFW_KEY_SPACE);
    
        m_Input.enable_button_physical |= glfwGetKey(window, GLFW_KEY_F);
        m_Input.disable_button_physical |= glfwGetKey(window, GLFW_KEY_R);
    }

    if (ImGui::Begin("Control"))
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        ImVec2 pos = ImGui::GetCursorScreenPos();

        m_Input.main_axes = imgui_joystick("virtual joystick", 200.f, ImVec2(0.2f, 0.2f), (m_Input.main_axes.x == 0.f && m_Input.main_axes.y == 0.f) ? 0 : &m_Input.main_axes, (m_Input.gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        ImGui::Text("Joystick %f %f", m_Input.main_axes.x, m_Input.main_axes.y);

        ImGui::Button("Enable");
        if (ImGui::IsItemActive()) m_Input.enable_button_physical = true;
        ImGui::Button("Disable");
        if (ImGui::IsItemActive()) m_Input.disable_button_physical = true;

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 220.f, pos.y + ImGui::GetStyle().ItemSpacing.y));

        ImGui::VSliderFloat("##vslider", ImVec2(20, 200), &m_Input.scalar, 0.0f, 1.0f, "%.2f");

    }
    ImGui::End();

    if (ImGui::Begin("WheelVisualization"))
    {
        std::lock_guard<std::mutex> lock(m_WheelsMutex);

        for(int i = 0; i < m_Wheels.size(); i++)
        {
            imgui_arrow(ImVec2(-m_Wheels.at(i).y * 500.f + 150.f, -m_Wheels.at(i).x * 500.f + 200.f), 0.f, m_Wheels.at(i).target_rpm * (m_Wheels.at(i).invert ? -1.f : 1.f), IM_COL32(60, 60, 60, 255), 3.f, 9.f, false);
            
            imgui_arrow(ImVec2(-m_Wheels.at(i).y * 500.f + 150.f, -m_Wheels.at(i).x * 500.f + 200.f), 0.f, m_Wheels.at(i).last_rpm * (m_Wheels.at(i).invert ? -1.f : 1.f), IM_COL32(255, 255, 255, 255), 5.f, 10.f, false);
        }
    }
    ImGui::End();

    m_ImageSystem->onGuiFrame();
    m_DataCaptureSystem->ImGuiPanels();
}