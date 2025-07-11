#include "imgui.h"

#include <cmath>
#include <guiniverse/imgui_util.hpp>
#include <guiniverse/n10/n10.hpp>

void N10::OnGuiStartup()
{
    m_ImageSystem->OnGuiStartup();
}

void N10::OnGuiShutdown()
{
    m_ImageSystem->OnGuiShutdown();
}

void N10::OnGuiFrame(GLFWwindow* window, JoystickInput& input)
{
    (void) input;

    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        m_Input.Drive.MainAxisX = 0.f;
        m_Input.Drive.MainAxisY = 0.f;

        m_Input.Drive.GasButton = false;
        m_Input.Drive.DogWalkButton = false;

        m_Input.Drive.EnableButtonPhysical = false;
        m_Input.Drive.DisableButtonPhysical = false;

        m_Input.Gripper.ForwardAxis = 0.f;
        m_Input.Gripper.UpAxis = 0.f;
        m_Input.Gripper.GroundAngleAxis = 0.f;

        // if (device_name == "Logitech Logitech Extreme 3D")
        // {
        //     if (input.GetButton(8))
        //         m_GripperMode.store(false);
        //     else if (input.GetButton(9))
        //         m_GripperMode.store(true);
        //     m_Input.Drive.MainAxisX = input.GetAxis(0);
        //     m_Input.Drive.MainAxisY = input.GetAxis(1);
        //     float new_joystick_scalar = input.GetAxis(3) / 2.f + 0.5f;
        //     if (new_joystick_scalar != m_Input.Drive.ScalarAxis_joystick)
        //     {
        //         m_Input.Drive.ScalarAxis_joystick = new_joystick_scalar;
        //         m_Input.Drive.ScalarAxis = new_joystick_scalar;
        //     }
        //     m_Input.Drive.GasButton |= input.GetButton(0);
        //     m_Input.Drive.DogWalkButton |= input.GetButton(1);
        //     m_Input.Drive.EnableButtonPhysical |= input.GetButton(10);
        //     m_Input.Drive.DisableButtonPhysical |= input.GetButton(11);
        // }

        bool key_w = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS);
        bool key_a = (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS);
        bool key_s = (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS);
        bool key_d = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);

        if (key_w || key_a || key_s || key_d)
        {
            m_Input.Drive.MainAxisX = 0.f;
            m_Input.Drive.MainAxisY = 0.f;

            if (key_w)
                m_Input.Drive.MainAxisY = 1.f;
            if (key_a)
                m_Input.Drive.MainAxisX = 1.f;
            if (key_s)
                m_Input.Drive.MainAxisY = -1.f;
            if (key_d)
                m_Input.Drive.MainAxisX = -1.f;
        }

        float length = sqrtf(
            m_Input.Drive.MainAxisX * m_Input.Drive.MainAxisX
            + m_Input.Drive.MainAxisY * m_Input.Drive.MainAxisY);
        if (length > 1.0f)
        {
            m_Input.Drive.MainAxisX = m_Input.Drive.MainAxisX / length;
            m_Input.Drive.MainAxisY = m_Input.Drive.MainAxisY / length;
        }

        if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
            m_Input.Gripper.ForwardAxis = -1.f;
        if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS)
            m_Input.Gripper.ForwardAxis = 1.f;
        if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS)
            m_Input.Gripper.UpAxis = -1.f;
        if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS)
            m_Input.Gripper.UpAxis = 1.f;
        if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
            m_Input.Gripper.GroundAngleAxis = -1.f;
        if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS)
            m_Input.Gripper.GroundAngleAxis = 1.f;

        m_Input.Drive.GasButton |= glfwGetKey(window, GLFW_KEY_SPACE);
        m_Input.Drive.DogWalkButton |= glfwGetKey(window, GLFW_KEY_M);

        if (glfwGetKey(window, GLFW_KEY_T))
            m_GripperMode.store(false);
        if (glfwGetKey(window, GLFW_KEY_G))
            m_GripperMode.store(true);

        m_Input.Drive.EnableButtonPhysical |= glfwGetKey(window, GLFW_KEY_F);
        m_Input.Drive.DisableButtonPhysical |= glfwGetKey(window, GLFW_KEY_R);
    }

    if (ImGui::Begin("Control"))
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        ImVec2 pos = ImGui::GetCursorScreenPos();
        pos.y += ImGui::GetStyle().ItemSpacing.y;

        if (ImGui::RadioButton("Drive", !m_GripperMode.load()))
            m_GripperMode.store(false);
        if (ImGui::RadioButton("Gripper", m_GripperMode.load()))
            m_GripperMode.store(true);

        ImGui::Button("Enable");
        if (ImGui::IsItemActive())
            m_Input.Drive.EnableButtonPhysical = true;
        ImGui::Button("Disable");
        if (ImGui::IsItemActive())
            m_Input.Drive.DisableButtonPhysical = true;

        ImGui::Checkbox("Gripperangles", &m_Input.Gripper.SendAngles);

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 100.f, pos.y));

        ImVec2 drive_joystick_axes = ImVec2(
            m_Input.Drive.MainAxisX,
            m_Input.Drive.MainAxisY);

        drive_joystick_axes = imgui_joystick(
            "virtual joystick",
            200.f,
            ImVec2(0.2f, 0.2f),
            (drive_joystick_axes.x == 0.f && drive_joystick_axes.y == 0.f) ? 0 : &drive_joystick_axes,
            (m_Input.Drive.GasButton ? IM_COL32(150, 150, 150, 255) : IM_COL32(80, 80, 80, 255)));

        m_Input.Drive.MainAxisX = drive_joystick_axes.x;
        m_Input.Drive.MainAxisY = drive_joystick_axes.y;

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 320.f, pos.y));

        ImGui::VSliderFloat(
            "##drive_slider_scalar",
            ImVec2(20, 200),
            &m_Input.Drive.ScalarAxis,
            0.0f,
            1.0f,
            "%.2f");

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 400.f, pos.y));
        m_Input.Gripper.ForwardAxis = imgui_joyslider(
            "gripper_slider_forward",
            200.f,
            0.05f,
            (m_Input.Gripper.ForwardAxis == 0.f ? 0 : &m_Input.Gripper.ForwardAxis));

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 450.f, pos.y));
        m_Input.Gripper.UpAxis = imgui_joyslider(
            "gripper_slider_up",
            200.f,
            0.05f,
            (m_Input.Gripper.UpAxis == 0.f ? 0 : &m_Input.Gripper.UpAxis));

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 500.f, pos.y));
        m_Input.Gripper.GroundAngleAxis = imgui_joyslider(
            "gripper_slider_ground_angle",
            200.f,
            0.05f,
            (m_Input.Gripper.GroundAngleAxis == 0.f ? 0 : &m_Input.Gripper.GroundAngleAxis));

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 550.f, pos.y));
        ImGui::VSliderFloat(
            "##gripper_slider_gripper",
            ImVec2(20, 200),
            &m_Input.Gripper.GripperState,
            0.0f,
            1.0f,
            "%.2f");
    }
    ImGui::End();

    if (ImGui::Begin("WheelVisualization"))
    {
        std::lock_guard<std::mutex> lock(m_WheelsMutex);

        ImGui::Text("VoltagePowerManagement %f", m_VoltagePowerManagement.load());
        ImGui::Text("VoltageAdpter %f", m_VoltageAdapter.load());

        for (unsigned i = 0; i < m_Wheels.size(); i++)
        {
            imgui_arrow(
                ImVec2(-m_Wheels.at(i).Y * 500.f + 150.f, -m_Wheels.at(i).X * 500.f + 200.f),
                m_Wheels.at(i).TargetAngle,
                m_Wheels.at(i).TargetRPM * (m_Wheels.at(i).Invert ? -1.f : 1.f),
                IM_COL32(60, 60, 60, 255),
                3.f,
                9.f,
                false);

            imgui_arrow(
                ImVec2(-m_Wheels.at(i).Y * 500.f + 150.f, -m_Wheels.at(i).X * 500.f + 200.f),
                m_Wheels.at(i).LastAngle,
                m_Wheels.at(i).LastRPM * (m_Wheels.at(i).Invert ? -1.f : 1.f),
                IM_COL32(255, 255, 255, 255),
                5.f,
                10.f,
                false);
        }
    }
    ImGui::End();

    if (ImGui::Begin("GripperVisualization"))
    {
        std::lock_guard<std::mutex> lock(m_GripperMutex);

        float sensor_distance = m_GripperDistanceSensorDistance.load();

        ImGui::Text(
            "target x: %fm   target y: %fm   distance_sensor: %fm",
            m_Gripper.TargetX,
            m_Gripper.TargetY,
            sensor_distance);

        float scalar = 800.f;
        ImVec2 offset = ImVec2(50.f, 250.f);

        struct
        {
            float bottom = 0.32f;
            float top = 0.265f;
            float height = 0.135f;
            float gripper_top = 0.0475f;
            float gripper_bottom = 0.04f;
            float gripper_x = 0.018f;
            float gripper_y = 0.05f;
        } chassis;

#define chasis_line(x_0, y_0, x_1, y_1) \
    imgui_line(ImVec2(offset.x + (x_0) * scalar, offset.y - (y_0) * scalar), ImVec2(offset.x + (x_1) * scalar, offset.y - (y_1) * scalar), IM_COL32(150, 150, 150, 255), 3.f)

        chasis_line(0.f, 0.f, 0.f, chassis.height);
        chasis_line(0.f, 0.f, chassis.bottom, 0.f);
        chasis_line(0.f, chassis.height, chassis.top, chassis.height);
        chasis_line(chassis.bottom, 0.f, chassis.bottom, chassis.gripper_bottom);
        chasis_line(
            chassis.top,
            chassis.height,
            chassis.top,
            chassis.height - chassis.gripper_top);
        chasis_line(
            chassis.bottom,
            chassis.gripper_bottom,
            chassis.top,
            chassis.height - chassis.gripper_top);

        ImVec2 joint = ImVec2(offset.x + (chassis.bottom - chassis.gripper_x) * scalar, offset.y - chassis.gripper_y * scalar);
        float angle = 0.f;

        for (int i = 0; i < 3; i++)
        {
            angle += m_Gripper.FeedbackAngles[i];
            ImVec2 next_joint = ImVec2(
                cos(angle) * m_Gripper.Segments[i] * scalar + joint.x,
                -sin(angle) * m_Gripper.Segments[i] * scalar + joint.y);

            imgui_line(joint, next_joint, IM_COL32(200, 200, 200, 255), 4.f);

            joint = next_joint;
        }

        ImVec2 next_joint = ImVec2(
            cos(angle) * sensor_distance * scalar + joint.x,
            -sin(angle) * sensor_distance * scalar + joint.y);
        imgui_line(joint, next_joint, IM_COL32(255, 100, 100, 255), 2.f);

        imgui_arrow(
            ImVec2(
                (m_Gripper.TargetX + chassis.bottom - chassis.gripper_x) * scalar
                    + offset.x,
                offset.y - (m_Gripper.TargetY + chassis.gripper_y) * scalar),
            m_Gripper.TargetGroundAngle - M_PIf / 2.f,
            20.f,
            (m_Gripper.InRange ? IM_COL32(255, 255, 255, 255) : IM_COL32(180, 50, 50, 255)),
            3.f,
            10.f,
            false);
    }
    ImGui::End();

    m_ImageSystem->OnGuiFrame();
    m_DataCaptureSystem->ImGuiPanels();
}
