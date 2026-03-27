#include "imgui.h"
#include <guiniverse/Quac/Quac.hpp>

#include <guiniverse/imgui_utils.hpp>

void Quac::onGuiStartup()
{
    m_ImageSystem->onGuiStartup();
    m_Arm.target_pose = ImVec2(0.f, 0.f);
}

void Quac::onGuiShutdown()
{
    m_ImageSystem->onGuiShutdown();
}

void Quac::onGuiFrame(GLFWwindow* window, JoystickInput& input)
{
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

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
        }

        if (glfwGetKey(window, GLFW_KEY_A)) m_Input.main_axes.x = 1.f;
        else if (glfwGetKey(window, GLFW_KEY_D)) m_Input.main_axes.x = -1.f;
        if (glfwGetKey(window, GLFW_KEY_W)) m_Input.main_axes.y = 1.f;
        else if (glfwGetKey(window, GLFW_KEY_S)) m_Input.main_axes.y = -1.f;

        float length = sqrtf(m_Input.main_axes.x * m_Input.main_axes.x + m_Input.main_axes.y * m_Input.main_axes.y);
        if (length > 1.0f) m_Input.main_axes = ImVec2(m_Input.main_axes.x / length, m_Input.main_axes.y / length);

        m_Input.gas_button |= glfwGetKey(window, GLFW_KEY_SPACE);
    }

    if (ImGui::Begin("Control"))
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        ImGui::Checkbox("Publish cmd", &m_Input.publish_cmd);

        ImVec2 pos = ImGui::GetCursorScreenPos();

        m_Input.main_axes = imgui_joystick("virtual joystick", 200.f, ImVec2(0.2f, 0.2f), (m_Input.main_axes.x == 0.f && m_Input.main_axes.y == 0.f) ? 0 : &m_Input.main_axes, (m_Input.gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 220.f, pos.y + ImGui::GetStyle().ItemSpacing.y));

        ImGui::VSliderFloat("##vslider", ImVec2(20, 200), &m_Input.scalar, 0.0f, 1.0f, "%.2f");

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 260.f, pos.y + ImGui::GetStyle().ItemSpacing.y));

    }
    ImGui::End();

    if (ImGui::Begin("Robotic arm"))
    {
        std::lock_guard<std::mutex> lock(m_Arm.mutex);

        if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) m_Arm.target_pose.y += 0.001f;
        if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) m_Arm.target_pose.y -= 0.001f;
        if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) m_Arm.target_pose.x -= 0.001f;
        if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) m_Arm.target_pose.x += 0.001f;

        m_Arm.publish_pose = false;
        if (glfwGetKey(window, GLFW_KEY_P)) m_Arm.publish_pose = true;

        ImGui::Text("target x: %fm   target y: %fm ", m_Arm.target_pose.x, m_Arm.target_pose.y);

        float scalar = 800.f;
        ImVec2 offset = ImVec2(300.f, 250.f);

        struct {
            float base_front = 0.01f;
            float base_back = -0.07f;
            float base_y = -0.015f;
            float diaginal_down_y = 0.01f;
            float diagonal_x = 0.065;
            float diagonal_y = 0.08f;
        } chassis;

#define chasis_line(x_0, y_0, x_1, y_1) imgui_line(ImVec2(offset.x + (x_0) * scalar, offset.y - (y_0) * scalar), ImVec2(offset.x + (x_1) * scalar, offset.y - (y_1) * scalar), IM_COL32(150, 150, 150, 255), 3.f)

        chasis_line(chassis.base_back, chassis.base_y, chassis.base_front, chassis.base_y);
        chasis_line(chassis.base_back, chassis.base_y, chassis.base_back, chassis.base_y + chassis.diaginal_down_y);
        chasis_line(chassis.base_back, chassis.base_y + chassis.diaginal_down_y, chassis.base_back - chassis.diagonal_x, chassis.base_y + chassis.diaginal_down_y + chassis.diagonal_y);

        float angles[3];
        angles[0] = M_PIf / 2.f - m_Arm.joints[0].value;
        angles[1] = - M_PIf / 2;
        angles[2] = - m_Arm.joints[1].value;

        float segment_lengths[3] = {0.105f, 0.035f, 0.05f};
        bool received[3] = {m_Arm.joints[0].index != -1, m_Arm.joints[0].index != -1, m_Arm.joints[1].index != -1};

        ImVec2 joint = offset;

        float angle = 0;

        for ( int i = 0; i < 3; i++ )
        {
            angle += angles[i];
            ImVec2 next_joint = ImVec2(cos(angle) * segment_lengths[i] * scalar + joint.x, -sin(angle) * segment_lengths[i] * scalar + joint.y);

            imgui_line(joint, next_joint, IM_COL32(200, 200, 200, 255), 4.f);
    
            joint = next_joint;
        }

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 panel_pos = ImGui::GetWindowPos();
    
        draw_list->AddCircleFilled(ImVec2(
                offset.x + m_Arm.target_pose.x * scalar + panel_pos.x, 
                offset.y - m_Arm.target_pose.y * scalar + panel_pos.y
            ), 4.f, IM_COL32(255, 0, 0, 255));
    
    }
    ImGui::End();

    m_ImageSystem->onGuiFrame();
    m_DataCaptureSystem->ImGuiPanels();
}