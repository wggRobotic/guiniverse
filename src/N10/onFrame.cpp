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

        float gripper_x = 0.f;
        float gripper_y = 0.f;
        float gripper_ground_angle = 0.f;
        float gripper_state = 0.f;
    }
    drive_input;

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
            drive_input.ang = m_Input.drive.main_axis_x * m_Input.drive.scalar_axis * (m_Input.drive.main_axis_y * m_Input.drive.scalar_axis < 0 ? -1.f : 1.f) * 2.5f;
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
        
    }

}