#include <guiniverse/N10/N10.hpp>

void N10::onFrame()
{
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
    } gripper_input;

    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        drive_input.gas = m_Input.drive.gas_button;

        if (m_Input.drive.dog_walk_button) {
            drive_input.lin_x = m_Input.drive.main_axis_y * m_Input.drive.scalar_axis;
            drive_input.lin_y = m_Input.drive.main_axis_x * m_Input.drive.scalar_axis;
            drive_input.ang = 0.f;
        }
        else {
            drive_input.lin_x = m_Input.drive.main_axis_y * m_Input.drive.scalar_axis * 0.4f;
            drive_input.lin_y = 0.f;
            drive_input.ang = m_Input.drive.main_axis_x * m_Input.drive.scalar_axis * (m_Input.drive.main_axis_y * m_Input.drive.scalar_axis < 0 ? -1.f : 1.f);
        }

        if (m_Input.drive.enable_button_physical)
        {
            if (!m_Input.drive.enable_button)
            {
                drive_input.enable_service = true;
                drive_input.enable_service_enable = true;
                m_Input.drive.enable_button = true;
            }
        }
        else m_Input.drive.enable_button = false;

        if (m_Input.drive.disable_button_physical)
        {
            if (!m_Input.drive.disable_button)
            {
                drive_input.enable_service = true;
                drive_input.enable_service_enable = false;
                m_Input.drive.disable_button = true;
            }
        }
        else m_Input.drive.disable_button = false;


        gripper_input.up = m_Input.gripper.up_axis;
        gripper_input.forward = m_Input.gripper.forward_axis;
        gripper_input.ground_angle = m_Input.gripper.ground_angle_axis;
        gripper_input.state = m_Input.gripper.gripper_state;
    }

    {
        m_TurtleTwistMessage.linear.x = (drive_input.gas ? drive_input.lin_x : 0.f);
        m_TurtleTwistMessage.linear.y = (drive_input.gas ? drive_input.lin_y : 0.f);
        m_TurtleTwistMessage.angular.z = (drive_input.gas ? drive_input.ang : 0.f);
    }
    m_TurtleTwistPublisher->publish(m_TurtleTwistMessage);

    {
        std::lock_guard<std::mutex> lock(m_WheelsMutex);
    
        m_WheelsRPMMessage.data.resize(m_Wheels.size());
        m_WheelsAngleMessage.data.resize(m_Wheels.size());

        for(int i = 0; i < m_Wheels.size(); i++)
        {
            float comp_x = drive_input.lin_x - drive_input.ang * m_Wheels.at(i).y;
            float comp_y = drive_input.lin_y + drive_input.ang * m_Wheels.at(i).x;

            float target_rpm, target_angle;

            if (drive_input.gas) 
                target_rpm = 60.f * sqrtf(comp_x*comp_x + comp_y*comp_y) / (2 * m_Wheels.at(i).radius * M_PI) * ((comp_x < 0.f) ^ m_Wheels.at(i).invert ? -1.f : 1.f);
            else 
                target_rpm = 0.f;

            target_angle = (comp_y == 0.f ? 
                0.f : 
                (comp_x == 0.f ? 
                    M_PI / 2 * (comp_y > 0.f ? 1.f : -1.f) : 
                    atanf(comp_y / comp_x)
                )
            );
                
            m_WheelsRPMMessage.data.at(i) = target_rpm;
            m_Wheels.at(i).target_rpm = target_rpm;

            m_WheelsAngleMessage.data.at(i) = target_angle;
            m_Wheels.at(i).target_angle = target_angle;
        }
    }
    m_WheelsRPMPublisher->publish(m_WheelsRPMMessage);
    m_WheelsAnglePublisher->publish(m_WheelsAngleMessage);

    if (!m_EnableMotorClientWaiting && drive_input.enable_service)
    {
        if (!m_EnableMotorClient->service_is_ready()) 
            RCLCPP_WARN(node->get_logger(), "Enable service is not available.");

        else {
        
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = drive_input.enable_service_enable;

            if (drive_input.enable_service_enable)
                RCLCPP_INFO(node->get_logger(), "Enabeling ...");
            else
                RCLCPP_INFO(node->get_logger(), "Disabling ...");

            m_EnableMotorClient->async_send_request(request, std::bind(&N10::EnableMotorClientCallback, this, std::placeholders::_1));

            m_EnableMotorClientWaiting = true;
            m_EnableMotorClientTimeSent = node->now().seconds();
        }
    }
    else if (m_EnableMotorClientWaiting && m_EnableMotorClientTimeSent + 5 < node->now().seconds()) {
        RCLCPP_INFO(node->get_logger(), "Enable Service didn't respond in 5 seconds");
        m_EnableMotorClientWaiting = false; 
    }

    {
        std::lock_guard<std::mutex> lock(m_GripperMutex);

        float speed_target = 0.001f;

        m_Gripper.target_x += gripper_input.forward * speed_target;
        m_Gripper.target_y += gripper_input.up * speed_target;
        m_Gripper.target_ground_angle += gripper_input.ground_angle * 0.01f;

        float angles[3];

        m_Gripper.inrange = true;
        if (!calculateGripperAngles(m_Gripper.target_x, m_Gripper.target_y, m_Gripper.target_ground_angle, angles))
            m_Gripper.inrange = false;

        if (!m_GripperMode.load())
        {
            m_Gripper.ready = false;

            for (int i = 0; i < 4; i++)
            {
                float angular_step = 0.005f;

                float diff = m_Gripper.drive_position_angles[i] - m_GripperAngleMessage.data[i];

                m_GripperAngleMessage.data[i] += (fabs(diff) < angular_step ? diff : (diff > 0 ? angular_step : -angular_step));
            }
        }
        else if (m_Gripper.inrange)
        {
            
            if (!m_Gripper.ready)
            {

                m_Gripper.current_x = m_Gripper.target_x;
                m_Gripper.current_y = m_Gripper.target_y;
                m_Gripper.current_ground_angle = m_Gripper.target_ground_angle;

                m_Gripper.ready = true;

                for (int i = 0; i < 4; i++)
                {
                    float angular_step = 0.005f;

                    float diff = angles[i] - m_GripperAngleMessage.data[i];
                    if (fabs(diff) < 0.0001) m_Gripper.ready = false;

                    m_GripperAngleMessage.data[i] += (fabs(diff) < angular_step ? diff : (diff > 0 ? angular_step : -angular_step));
                }
            }

            if (m_Gripper.ready)
            {
                float speed_gripper = 0.001f;

                float length = sqrtf(
                    (m_Gripper.target_x - m_Gripper.current_x) * (m_Gripper.target_x - m_Gripper.current_x) + 
                    (m_Gripper.target_y - m_Gripper.current_y) * (m_Gripper.target_y - m_Gripper.current_y)
                );

                float new_x = m_Gripper.current_x + (m_Gripper.target_x - m_Gripper.current_x) * (speed_gripper > length ? 1.f : speed_gripper / length);
                float new_y = m_Gripper.current_y + (m_Gripper.target_y - m_Gripper.current_y) * (speed_gripper > length ? 1.f : speed_gripper / length);

                float ground_angle_diff = m_Gripper.target_ground_angle - m_Gripper.current_ground_angle;
                float new_ground_angle = m_Gripper.current_ground_angle + 
                    (0.01f > fabs(ground_angle_diff) ? ground_angle_diff : (ground_angle_diff > 0.f ? 0.01f : -0.01f));

                calculateGripperAngles(new_x, new_y, new_ground_angle, angles);
                
                m_GripperAngleMessage.data[0] = angles[0];
                m_GripperAngleMessage.data[1] = angles[1];
                m_GripperAngleMessage.data[2] = angles[2];
                m_GripperAngleMessage.data[3] = gripper_input.state * M_PIf / 2.f;

                m_Gripper.current_x = new_x;
                m_Gripper.current_y = new_y;
                m_Gripper.current_ground_angle = new_ground_angle;
            }

        }

    }
    m_GripperAnglePublisher->publish(m_GripperAngleMessage);

}

bool N10::calculateGripperAngles(float x, float y, float ground_angle, float* result_angles)
{
    bool inrange = true;

    float theta = atan2f(y, x);

    float cos_phi = 
        (x * x + y * y - m_Gripper.segments[0] * m_Gripper.segments[0] - m_Gripper.segments[1] * m_Gripper.segments[1]) / 
        (-2.f * m_Gripper.segments[0] * m_Gripper.segments[1]);

    float cos_sigma = 
        (m_Gripper.segments[1] * m_Gripper.segments[1] - x * x - y * y - m_Gripper.segments[0] * m_Gripper.segments[0]) / 
        (-2.f * m_Gripper.segments[0] * sqrtf(x * x + y * y));

    if (1.f < cos_phi || -1.f > cos_phi || 1.f < cos_sigma || -1.f > cos_sigma)
    {
        inrange = false;

        cos_phi = (1.f < cos_phi ? 1.f : -1.f);
        cos_sigma = (1.f < cos_sigma ? 1.f : -1.f);
    }

    result_angles[0] = theta + acosf(cos_sigma);
    if (result_angles[0] > 1.9f) {result_angles[0] = 1.9f; inrange = false;}
    if (result_angles[0] < -0.1f) {result_angles[0] = -0.1f; inrange = false;}

    result_angles[1] = acosf(cos_phi) - M_PIf;
    if (result_angles[1] > 2.2f) {result_angles[1] = 2.2f; inrange = false;}
    if (result_angles[1] < -2.2f) {result_angles[1] = -2.2f; inrange = false;}

    result_angles[2] = ground_angle - (result_angles[0] + result_angles[1]);
    if (result_angles[2] > 2.2) {result_angles[2] = 2.2f; inrange = false;}
    if (result_angles[2] < -2.2f) {result_angles[2] = -2.2f; inrange = false;}

    return inrange;
}