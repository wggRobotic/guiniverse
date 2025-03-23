#include <guiniverse/NoName/NoName.hpp>

void NoName::onFrame()
{
    rclcpp::spin_some(node);

    m_ImageSystemBackendGST->onFrame();

    float lin_x = 0.f;
    float ang = 0.f;
    bool gas = false;

    bool set_mode_service = false;
    bool set_mode_service_enable;

    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        gas = m_Input.gas_button;

        lin_x = m_Input.main_axes.y * m_Input.scalar;
        ang = m_Input.main_axes.x * m_Input.scalar * (m_Input.main_axes.y * m_Input.scalar < 0 ? -1.f : 1.f) * 2.f;

        if (m_Input.enable_button_physical)
        {
            if (!m_Input.enable_button)
            {
                set_mode_service = true;
                set_mode_service_enable = true;
                m_Input.enable_button = true;
            }
        }
        else m_Input.enable_button = false;

        if (m_Input.disable_button_physical)
        {
            if (!m_Input.disable_button)
            {
                set_mode_service = true;
                set_mode_service_enable = false;
                m_Input.disable_button = true;
            }
        }
        else m_Input.disable_button = false;
    }

    {
        m_TurtleTwistMessage.linear.x = (gas ? lin_x : 0.f);
        m_TurtleTwistMessage.linear.y = 0.f;
        m_TurtleTwistMessage.angular.z = (gas ? ang : 0.f);
    }
    m_TurtleTwistPublisher->publish(m_TurtleTwistMessage);

    {
        std::lock_guard<std::mutex> lock(m_WheelsMutex);
    
        m_RPMMessage.data.resize(m_Wheels.size());

        for(int i = 0; i < m_Wheels.size(); i++)
        {
            m_Wheels[i].target_rpm = 60.f * (lin_x - m_Wheels[i].y * ang) / (2.f * m_Wheels[i].radius * M_PI) * (m_Wheels[i].invert ? -1.f : 1.f);
            m_RPMMessage.data[i] = m_Wheels[i].target_rpm;
        }
    }
    m_RPMPublisher->publish(m_RPMMessage);

    if (!m_SetModeClientWaiting && set_mode_service)
    {
        if (!m_SetModeClient->service_is_ready()) 
            RCLCPP_WARN(node->get_logger(), "SetMode service is not available.");

        else {
        
            auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
            request->mode.mode = (set_mode_service_enable ? edu_robot::msg::Mode::REMOTE_CONTROLLED : edu_robot::msg::Mode::INACTIVE );

            if (set_mode_service_enable)
                RCLCPP_INFO(node->get_logger(), "Enabeling ...");
            else
                RCLCPP_INFO(node->get_logger(), "Disabling ...");

            m_SetModeClient->async_send_request(request, std::bind(&NoName::SetModeClientCallback, this, std::placeholders::_1));

            m_SetModeClientWaiting = true;
            m_SetModeClientTimeSent = node->now().seconds();
        }
    }
    else if (m_SetModeClientWaiting && m_SetModeClientTimeSent + 5 < node->now().seconds()) {
        RCLCPP_INFO(node->get_logger(), "SetMode Service didn't respond in 5 seconds");
        m_SetModeClientWaiting = false; 
    }

    rclcpp::spin_some(node);
}