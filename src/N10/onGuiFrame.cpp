#include <guiniverse/N10/N10.hpp>

#include <guiniverse/imgui_utils.hpp>

#include <cmath>

void N10::onGuiStart() 
{
    
}

void N10::onGuiShutdown() 
{
    m_GSTImageSystem.onGuiShutdown();
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

        float length = sqrtf(m_Input.drive.main_axis_x * m_Input.drive.main_axis_x + m_Input.drive.main_axis_y * m_Input.drive.main_axis_y);
        if (length > 1.0f) 
        {
            m_Input.drive.main_axis_x = m_Input.drive.main_axis_x / length;
            m_Input.drive.main_axis_y = m_Input.drive.main_axis_y / length;
        }

        if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) m_Input.gripper.forward_axis = -1.f;
        if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS) m_Input.gripper.forward_axis = 1.f;
        if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) m_Input.gripper.up_axis = -1.f;
        if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) m_Input.gripper.up_axis = 1.f;
        if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) m_Input.gripper.ground_angle_axis = -1.f;
        if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS) m_Input.gripper.ground_angle_axis = 1.f;
        
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

        ImVec2 pos = ImGui::GetCursorScreenPos();
        pos.y += ImGui::GetStyle().ItemSpacing.y;

        if (ImGui::RadioButton("Drive", !m_GripperMode.load())) m_GripperMode.store(false);
        if (ImGui::RadioButton("Gripper", m_GripperMode.load())) m_GripperMode.store(true);

        ImGui::Button("Enable");
        if (ImGui::IsItemActive()) m_Input.drive.enable_button_physical = true;
        ImGui::Button("Disable");
        if (ImGui::IsItemActive()) m_Input.drive.disable_button_physical = true;

        ImGui::Checkbox("Gripperangles", &m_Input.gripper.send_angles);

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 100.f, pos.y));

        ImVec2 drive_joystick_axes = ImVec2(m_Input.drive.main_axis_x, m_Input.drive.main_axis_y);

        drive_joystick_axes = imgui_joystick("virtual joystick", 200.f, ImVec2(0.2f, 0.2f), (drive_joystick_axes.x == 0.f && drive_joystick_axes.y == 0.f) ? 0 : &drive_joystick_axes, (m_Input.drive.gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        m_Input.drive.main_axis_x = drive_joystick_axes.x;
        m_Input.drive.main_axis_y = drive_joystick_axes.y;

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 320.f, pos.y));

        ImGui::VSliderFloat("##drive_slider_scalar", ImVec2(20, 200), &m_Input.drive.scalar_axis, 0.0f, 1.0f, "%.2f");

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 400.f, pos.y));
        m_Input.gripper.forward_axis = imgui_joyslider("gripper_slider_forward", 200.f, 0.05f, (m_Input.gripper.forward_axis == 0.f ? 0 : &m_Input.gripper.forward_axis ));

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 450.f, pos.y));
        m_Input.gripper.up_axis = imgui_joyslider("gripper_slider_up", 200.f, 0.05f, (m_Input.gripper.up_axis == 0.f ? 0 : &m_Input.gripper.up_axis ));

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 500.f, pos.y));
        m_Input.gripper.ground_angle_axis = imgui_joyslider("gripper_slider_ground_angle", 200.f, 0.05f, (m_Input.gripper.ground_angle_axis == 0.f ? 0 : &m_Input.gripper.ground_angle_axis ));

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 550.f, pos.y));
        ImGui::VSliderFloat("##gripper_slider_gripper", ImVec2(20, 200), &m_Input.gripper.gripper_state, 0.0f, 1.0f, "%.2f");

    }
    ImGui::End();

    if (ImGui::Begin("WheelVisualization"))
    {
        std::lock_guard<std::mutex> lock(m_WheelsMutex);

        for(int i = 0; i < m_Wheels.size(); i++)
        {
            imgui_arrow(ImVec2(-m_Wheels.at(i).y * 500.f + 150.f, -m_Wheels.at(i).x * 500.f + 200.f), m_Wheels.at(i).target_angle, m_Wheels.at(i).target_rpm * (m_Wheels.at(i).invert ? -1.f : 1.f), IM_COL32(60, 60, 60, 255), 3.f, 9.f, false);
            
            imgui_arrow(ImVec2(-m_Wheels.at(i).y * 500.f + 150.f, -m_Wheels.at(i).x * 500.f + 200.f), m_Wheels.at(i).last_angle, m_Wheels.at(i).last_rpm * (m_Wheels.at(i).invert ? -1.f : 1.f), IM_COL32(255, 255, 255, 255), 5.f, 10.f, false);
        }
    }
    ImGui::End();

    if (ImGui::Begin("GripperVisualization"))
    {
        std::lock_guard<std::mutex> lock(m_GripperMutex);

        float sensor_distance = m_GripperDistanceSensorDistance.load();

        ImGui::Text("target x: %fm   target y: %fm   distance_sensor: %fm", m_Gripper.target_x, m_Gripper.target_y, sensor_distance);

        float scalar = 800.f;
        ImVec2 offset = ImVec2(50.f, 250.f);

        struct {
            float bottom = 0.32f;
            float top = 0.265f;
            float height = 0.135f;
            float gripper_top = 0.0475f;
            float gripper_bottom = 0.04f;
            float gripper_x = 0.018f;
            float gripper_y = 0.05f;
        } chassis;

#define chasis_line(x_0, y_0, x_1, y_1) imgui_line(ImVec2(offset.x + (x_0) * scalar, offset.y - (y_0) * scalar), ImVec2(offset.x + (x_1) * scalar, offset.y - (y_1) * scalar), IM_COL32(150, 150, 150, 255), 3.f)

        chasis_line(0.f, 0.f, 0.f, chassis.height);
        chasis_line(0.f, 0.f, chassis.bottom, 0.f);
        chasis_line(0.f, chassis.height, chassis.top, chassis.height);
        chasis_line(chassis.bottom, 0.f, chassis.bottom, chassis.gripper_bottom);
        chasis_line(chassis.top, chassis.height, chassis.top, chassis.height - chassis.gripper_top);
        chasis_line(chassis.bottom, chassis.gripper_bottom, chassis.top, chassis.height - chassis.gripper_top);

        ImVec2 joint = ImVec2(offset.x + (chassis.bottom - chassis.gripper_x) * scalar, offset.y - chassis.gripper_y * scalar);
        float angle = 0.f;

        for ( int i = 0; i < 3; i++ )
        {
            angle += m_Gripper.feedback_angles[i];
            ImVec2 next_joint = ImVec2(cos(angle) * m_Gripper.segments[i] * scalar + joint.x, -sin(angle) * m_Gripper.segments[i] * scalar + joint.y);

            imgui_line(joint, next_joint, IM_COL32(200, 200, 200, 255), 4.f);
    
            joint = next_joint;
        }

        ImVec2 next_joint = ImVec2(cos(angle) * sensor_distance * scalar + joint.x, -sin(angle) * sensor_distance * scalar + joint.y);
        imgui_line(joint, next_joint, IM_COL32(255, 100, 100, 255), 2.f);


        imgui_arrow(
            ImVec2(
                (m_Gripper.target_x + chassis.bottom - chassis.gripper_x) * scalar + offset.x, 
                offset.y - (m_Gripper.target_y + chassis.gripper_y) * scalar
            ), 
            m_Gripper.target_ground_angle - M_PIf / 2.f, 20.f, 
            (m_Gripper.inrange ? IM_COL32(255, 255, 255, 255) : IM_COL32(180, 50, 50, 255)), 
            3.f, 
            10.f, 
            false
        );
    
    }
    ImGui::End();
    
    m_GSTImageSystem.ImGuiPanels();
    m_DataCaptureSystem->ImGuiPanels();
}