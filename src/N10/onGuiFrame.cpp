#include <guiniverse/N10/N10.hpp>

#include <guiniverse/imgui_utils.hpp>

void N10::onGuiStart() 
{
    
}

void N10::onGuiShutdown() 
{
    m_ImageSystem->onGuiShutdown();
}

void N10::onGuiFrame(GLFWwindow* window, JoystickInput& input)
{
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        m_Input.drive.main_axis_x = 0.f;
        m_Input.drive.main_axis_y = 0.f;

        m_Input.drive.gas_button = false;
        m_Input.drive.dog_walk_button = false;
        
        m_Input.drive.enable_button_physical = false;
        m_Input.drive.disable_button_physical = false;

        m_Input.gripper.forward_axis = 0.f;
        m_Input.gripper.up_axis = 0.f;
        m_Input.gripper.ground_angle_axis = 0.f;

        auto device_name = input.getDeviceName();

        if (device_name == "Logitech Logitech Extreme 3D")
        {

            if (input.getButton(8)) m_GripperMode.store(false);
            else if (input.getButton(9)) m_GripperMode.store(true);

            if (!m_GripperMode.load())
            {
                m_Input.drive.main_axis_x = input.getAxis(0);
                m_Input.drive.main_axis_y = input.getAxis(1);

                float new_joystick_scalar = input.getAxis(3) / 2.f + 0.5f;
                if (new_joystick_scalar != m_Input.drive.scalar_axis_joystick) {
                    m_Input.drive.scalar_axis_joystick = new_joystick_scalar;
                    m_Input.drive.scalar_axis = new_joystick_scalar;
                }

                m_Input.drive.gas_button |= input.getButton(0);
                m_Input.drive.dog_walk_button |= input.getButton(1);
            }
            else
            {
                m_Input.gripper.forward_axis = input.getAxis(1);
                m_Input.gripper.up_axis = input.getAxis(5);
                m_Input.gripper.ground_angle_axis = (input.getButton(7) == GLFW_PRESS ? 1.f : (input.getButton(8) == GLFW_PRESS ? -1.f : 0.f));

                float new_gripper_state_joystick = input.getAxis(3) / 2.f + 0.5f;
                if (new_gripper_state_joystick != m_Input.gripper.gripper_state_joystick) {
                    m_Input.gripper.gripper_state_joystick = new_gripper_state_joystick;
                    m_Input.gripper.gripper_state = new_gripper_state_joystick;
                }
            }
            
            m_Input.drive.enable_button_physical |= input.getButton(10);
            m_Input.drive.disable_button_physical |= input.getButton(11);

            m_Input.gripper.pos0_button = input.getButton(6);
            m_Input.gripper.pos0_button = input.getButton(7);
        }

        bool key_w = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS);
        bool key_a = (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS);
        bool key_s = (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS); 
        bool key_d = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS); 

        if (key_w || key_a || key_s || key_d)
        {
            m_Input.drive.main_axis_x = 0.f;
            m_Input.drive.main_axis_y = 0.f;

            if (key_w) m_Input.drive.main_axis_y = 1.f;
            if (key_a) m_Input.drive.main_axis_x = 1.f;
            if (key_s) m_Input.drive.main_axis_y = -1.f;
            if (key_d) m_Input.drive.main_axis_x = -1.f;
        }

        if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) m_Input.gripper.forward_axis = -1.f;
        if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS) m_Input.gripper.forward_axis = 1.f;
        if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) m_Input.gripper.up_axis = -1.f;
        if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) m_Input.gripper.up_axis = 1.f;
        if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) m_Input.gripper.ground_angle_axis = -1.f;
        if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS) m_Input.gripper.ground_angle_axis = 1.f;

        float length = sqrtf(m_Input.drive.main_axis_x * m_Input.drive.main_axis_x + m_Input.drive.main_axis_y * m_Input.drive.main_axis_y);
        if (length > 1.0f) 
        {
            m_Input.drive.main_axis_x = m_Input.drive.main_axis_x / length;
            m_Input.drive.main_axis_y = m_Input.drive.main_axis_y / length;
        }
        
        m_Input.drive.gas_button |= glfwGetKey(window, GLFW_KEY_SPACE);
        m_Input.drive.dog_walk_button |= glfwGetKey(window, GLFW_KEY_M);

        if (glfwGetKey(window, GLFW_KEY_T)) m_GripperMode.store(false);
        if (glfwGetKey(window, GLFW_KEY_G)) m_GripperMode.store(true);

        m_Input.drive.enable_button_physical |= glfwGetKey(window, GLFW_KEY_F);
        m_Input.drive.disable_button_physical |= glfwGetKey(window, GLFW_KEY_R);

    }

    if (ImGui::Begin("Control"))
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        if (ImGui::RadioButton("Drive", !m_GripperMode.load())) m_GripperMode.store(false);
        if (ImGui::RadioButton("Gripper", m_GripperMode.load())) m_GripperMode.store(true);

        ImVec2 pos = ImGui::GetCursorScreenPos();

        ImVec2 drive_joystick_axes = ImVec2(m_Input.drive.main_axis_x, m_Input.drive.main_axis_y);

        drive_joystick_axes = imgui_joystick("virtual joystick", 200.f, ImVec2(0.2f, 0.2f), (drive_joystick_axes.x == 0.f && drive_joystick_axes.y == 0.f) ? 0 : &drive_joystick_axes, (m_Input.drive.gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        m_Input.drive.main_axis_x = drive_joystick_axes.x;
        m_Input.drive.main_axis_y = drive_joystick_axes.y;

        ImGui::Text("Joystick %f %f", drive_joystick_axes.x, drive_joystick_axes.y);

        ImGui::Button("Enable");
        if (ImGui::IsItemActive()) m_Input.drive.enable_button_physical = true;
        ImGui::Button("Disable");
        if (ImGui::IsItemActive()) m_Input.drive.disable_button_physical = true;

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 220.f, pos.y + ImGui::GetStyle().ItemSpacing.y));
        ImGui::VSliderFloat("##drive_slider_scalar", ImVec2(20, 200), &m_Input.drive.scalar_axis, 0.0f, 1.0f, "%.2f");

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 300.f, pos.y + ImGui::GetStyle().ItemSpacing.y));
        m_Input.gripper.forward_axis = imgui_joyslider("gripper_slider_forward", 200.f, 0.05f, (m_Input.gripper.forward_axis == 0.f ? 0 : &m_Input.gripper.forward_axis ));

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 350.f, pos.y + ImGui::GetStyle().ItemSpacing.y));
        m_Input.gripper.up_axis = imgui_joyslider("gripper_slider_up", 200.f, 0.05f, (m_Input.gripper.up_axis == 0.f ? 0 : &m_Input.gripper.up_axis ));

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 400.f, pos.y + ImGui::GetStyle().ItemSpacing.y));
        m_Input.gripper.ground_angle_axis = imgui_joyslider("gripper_slider_ground_angle", 200.f, 0.05f, (m_Input.gripper.ground_angle_axis == 0.f ? 0 : &m_Input.gripper.ground_angle_axis ));

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 450.f, pos.y + ImGui::GetStyle().ItemSpacing.y));
        ImGui::VSliderFloat("##gripper_slider_gripper", ImVec2(20, 200), &m_Input.gripper.gripper_state, 0.0f, 1.0f, "%.2f");

    }
    ImGui::End();

    if (ImGui::Begin("Visualization"))
    {
        {
            std::lock_guard<std::mutex> lock(m_WheelsMutex);

            for(int i = 0; i < m_Wheels.size(); i++)
            {
                imgui_arrow(ImVec2(-m_Wheels.at(i).y * 600.f + 160.f, -m_Wheels.at(i).x * 600.f + 250.f), m_Wheels.at(i).target_angle, m_Wheels.at(i).target_rpm * (m_Wheels.at(i).invert ? -1.f : 1.f), IM_COL32(60, 60, 60, 255), 3.f, 9.f, false);
                
                imgui_arrow(ImVec2(-m_Wheels.at(i).y * 600.f + 160.f, -m_Wheels.at(i).x * 600.f + 250.f), m_Wheels.at(i).last_angle, m_Wheels.at(i).last_rpm * (m_Wheels.at(i).invert ? -1.f : 1.f), IM_COL32(255, 255, 255, 255), 5.f, 10.f, true);
            }
            
            
        }
        
    }
    ImGui::End();
    
    m_ImageSystem->ImGuiPanels();
    m_DataCaptureSystem->ImGuiPanels();
}