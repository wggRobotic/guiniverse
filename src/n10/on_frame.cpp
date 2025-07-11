#include <guiniverse/n10/n10.hpp>

void N10::OnFrame()
{
    rclcpp::spin_some(GetNode());

    m_ImageSystemBackendGST->OnFrame();

    struct
    {
        float lin_x = 0.f;
        float lin_y = 0.f;
        float ang = 0.f;
        bool gas = false;
        bool enable_service = false;
        bool enable_service_enable;
    } drive_input;

    struct
    {
        float up = 0.f;
        float forward = 0.f;
        float ground_angle = 0.f;
        float state = 0.f;
        bool send_angles = false;
    } gripper_input;

    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        drive_input.gas = m_Input.Drive.GasButton;

        if (m_Input.Drive.DogWalkButton)
        {
            drive_input.lin_x = m_Input.Drive.MainAxisY * m_Input.Drive.ScalarAxis * 0.8f;
            drive_input.lin_y = m_Input.Drive.MainAxisX * m_Input.Drive.ScalarAxis * 0.8f;
            drive_input.ang = 0.f;
        }
        else
        {
            drive_input.lin_x = m_Input.Drive.MainAxisY * m_Input.Drive.ScalarAxis * 0.8f;
            drive_input.lin_y = 0.f;
            drive_input.ang = m_Input.Drive.MainAxisX * m_Input.Drive.ScalarAxis * 2.f
                            * (m_Input.Drive.MainAxisY * m_Input.Drive.ScalarAxis < 0 ? -1.f : 1.f);
        }

        if (m_Input.Drive.EnableButtonPhysical)
        {
            if (!m_Input.Drive.EnableButton)
            {
                drive_input.enable_service = true;
                drive_input.enable_service_enable = true;
                m_Input.Drive.EnableButton = true;
            }
        }
        else
            m_Input.Drive.EnableButton = false;

        if (m_Input.Drive.DisableButtonPhysical)
        {
            if (!m_Input.Drive.DisableButton)
            {
                drive_input.enable_service = true;
                drive_input.enable_service_enable = false;
                m_Input.Drive.DisableButton = true;
            }
        }
        else
            m_Input.Drive.DisableButton = false;

        gripper_input.up = m_Input.Gripper.UpAxis;
        gripper_input.forward = m_Input.Gripper.ForwardAxis;
        gripper_input.ground_angle = m_Input.Gripper.GroundAngleAxis;
        gripper_input.state = m_Input.Gripper.GripperState;
        gripper_input.send_angles = m_Input.Gripper.SendAngles;
    }

    {
        std::lock_guard<std::mutex> lock(m_WheelsMutex);

        m_WheelsRPMMessage.data.resize(m_Wheels.size());
        m_WheelsAngleMessage.data.resize(m_Wheels.size());

        for (size_t i = 0; i < m_Wheels.size(); i++)
        {
            float comp_x = drive_input.lin_x - drive_input.ang * m_Wheels.at(i).Y;
            float comp_y = drive_input.lin_y + drive_input.ang * m_Wheels.at(i).X;

            float target_rpm, target_angle;

            if (drive_input.gas)
                target_rpm = 60.f * sqrtf(comp_x * comp_x + comp_y * comp_y)
                           / (2 * m_Wheels.at(i).Radius * M_PI)
                           * ((comp_x < 0.f) ^ m_Wheels.at(i).Invert ? -1.f : 1.f);
            else
                target_rpm = 0.f;

            target_angle = (comp_y == 0.f ? 0.f : (comp_x == 0.f ? M_PI / 2 * (comp_y > 0.f ? 1.f : -1.f) : atanf(comp_y / comp_x)));

            m_WheelsRPMMessage.data.at(i) = target_rpm;
            m_Wheels.at(i).TargetRPM = target_rpm;

            m_WheelsAngleMessage.data.at(i) = target_angle;
            m_Wheels.at(i).TargetAngle = target_angle;
        }
    }
    m_WheelsRPMPublisher->publish(m_WheelsRPMMessage);
    m_WheelsAnglePublisher->publish(m_WheelsAngleMessage);

    if (!m_EnableMotorClientWaiting && drive_input.enable_service)
    {
        if (!m_EnableMotorClient->service_is_ready())
            RCLCPP_WARN(GetNode()->get_logger(), "Enable service is not available.");

        else
        {
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = drive_input.enable_service_enable;

            if (drive_input.enable_service_enable)
                RCLCPP_INFO(GetNode()->get_logger(), "Enabeling ...");
            else
                RCLCPP_INFO(GetNode()->get_logger(), "Disabling ...");

            m_EnableMotorClient->async_send_request(request, std::bind(&N10::EnableMotorClientCallback, this, std::placeholders::_1));

            m_EnableMotorClientWaiting = true;
            m_EnableMotorClientTimeSent = GetNode()->now().seconds();
        }
    }
    else if (
        m_EnableMotorClientWaiting
        && m_EnableMotorClientTimeSent + 5 < GetNode()->now().seconds())
    {
        RCLCPP_INFO(GetNode()->get_logger(), "Enable Service didn't respond in 5 seconds");
        m_EnableMotorClientWaiting = false;
    }

    {
        std::lock_guard<std::mutex> lock(m_GripperMutex);

        float speed_target = 0.001f;

        m_Gripper.TargetX += gripper_input.forward * speed_target;
        m_Gripper.TargetY += gripper_input.up * speed_target;
        m_Gripper.TargetGroundAngle += gripper_input.ground_angle * 0.01f;

        float angles[3];

        m_Gripper.InRange = true;
        if (!CalculateGripperAngles(
                m_Gripper.TargetX,
                m_Gripper.TargetY,
                m_Gripper.TargetGroundAngle,
                angles))
            m_Gripper.InRange = false;

        if (!m_GripperMode.load())
        {
            m_Gripper.Ready = false;

            for (int i = 0; i < 4; i++)
            {
                float angular_step = 0.005f;

                float diff = m_Gripper.DrivePositionAngles[i]
                           - m_GripperAngleMessage.data[i];

                m_GripperAngleMessage.data[i] += (fabs(diff) < angular_step ? diff : (diff > 0 ? angular_step : -angular_step));
            }
        }
        else if (m_Gripper.InRange)
        {
            if (!m_Gripper.Ready)
            {
                m_Gripper.CurrentX = m_Gripper.TargetX;
                m_Gripper.CurrentY = m_Gripper.TargetY;
                m_Gripper.CurrentGroundAngle = m_Gripper.TargetGroundAngle;

                m_Gripper.Ready = true;

                for (int i = 0; i < 4; i++)
                {
                    float angular_step = 0.005f;

                    float diff = angles[i] - m_GripperAngleMessage.data[i];
                    if (fabs(diff) > 0.0001)
                        m_Gripper.Ready = false;

                    m_GripperAngleMessage.data[i] += (fabs(diff) < angular_step ? diff : (diff > 0 ? angular_step : -angular_step));
                }
            }

            if (m_Gripper.Ready)
            {
                float speed_gripper = 0.001f;

                float length = sqrtf(
                    (m_Gripper.TargetX - m_Gripper.CurrentX)
                        * (m_Gripper.TargetX - m_Gripper.CurrentX)
                    + (m_Gripper.TargetY - m_Gripper.CurrentY)
                          * (m_Gripper.TargetY - m_Gripper.CurrentY));

                float new_x = m_Gripper.CurrentX
                            + (m_Gripper.TargetX - m_Gripper.CurrentX) * (speed_gripper > length ? 1.f : speed_gripper / length);
                float new_y = m_Gripper.CurrentY
                            + (m_Gripper.TargetY - m_Gripper.CurrentY) * (speed_gripper > length ? 1.f : speed_gripper / length);

                float ground_angle_diff = m_Gripper.TargetGroundAngle
                                        - m_Gripper.CurrentGroundAngle;
                float new_ground_angle = m_Gripper.CurrentGroundAngle + (0.01f > fabs(ground_angle_diff) ? ground_angle_diff : (ground_angle_diff > 0.f ? 0.01f : -0.01f));

                CalculateGripperAngles(new_x, new_y, new_ground_angle, angles);

                m_GripperAngleMessage.data[0] = angles[0];
                m_GripperAngleMessage.data[1] = angles[1];
                m_GripperAngleMessage.data[2] = angles[2];
                m_GripperAngleMessage.data[3] = gripper_input.state * M_PIf / 2.f;

                m_Gripper.CurrentX = new_x;
                m_Gripper.CurrentY = new_y;
                m_Gripper.CurrentGroundAngle = new_ground_angle;
            }
        }
    }
    if (gripper_input.send_angles)
        m_GripperAnglePublisher->publish(m_GripperAngleMessage);

    rclcpp::spin_some(GetNode());
}

bool N10::CalculateGripperAngles(float x, float y, float ground_angle, float* result_angles)
{
    bool in_range = true;

    float theta = atan2f(y, x);

    float cos_phi = (x * x + y * y - m_Gripper.Segments[0] * m_Gripper.Segments[0]
                     - m_Gripper.Segments[1] * m_Gripper.Segments[1])
                  / (-2.f * m_Gripper.Segments[0] * m_Gripper.Segments[1]);

    float cos_sigma = (m_Gripper.Segments[1] * m_Gripper.Segments[1] - x * x - y * y
                       - m_Gripper.Segments[0] * m_Gripper.Segments[0])
                    / (-2.f * m_Gripper.Segments[0] * sqrtf(x * x + y * y));

    if (1.f < cos_phi || -1.f > cos_phi || 1.f < cos_sigma || -1.f > cos_sigma)
    {
        in_range = false;

        cos_phi = (1.f < cos_phi ? 1.f : -1.f);
        cos_sigma = (1.f < cos_sigma ? 1.f : -1.f);
    }

    result_angles[0] = theta + acosf(cos_sigma);
    if (result_angles[0] > 1.9f)
    {
        result_angles[0] = 1.9f;
        in_range = false;
    }
    if (result_angles[0] < -0.1f)
    {
        result_angles[0] = -0.1f;
        in_range = false;
    }

    result_angles[1] = acosf(cos_phi) - M_PIf;
    if (result_angles[1] > 2.2f)
    {
        result_angles[1] = 2.2f;
        in_range = false;
    }
    if (result_angles[1] < -2.2f)
    {
        result_angles[1] = -2.2f;
        in_range = false;
    }

    result_angles[2] = ground_angle - (result_angles[0] + result_angles[1]);
    if (result_angles[2] > 2.2)
    {
        result_angles[2] = 2.2f;
        in_range = false;
    }
    if (result_angles[2] < -2.2f)
    {
        result_angles[2] = -2.2f;
        in_range = false;
    }

    return in_range;
}
