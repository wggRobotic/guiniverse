#include <guiniverse/no_name/no_name.hpp>

void NoName::OnFrame()
{
    rclcpp::spin_some(GetNode());

    m_ImageSystemBackendGST->OnFrame();

    float lin_x = 0.f;
    float ang = 0.f;
    bool gas = false;

    bool set_mode_service = false;
    bool set_mode_service_enable;

    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        gas = m_Input.GasButton;

        lin_x = m_Input.MainAxes.y * m_Input.Scalar;
        ang = m_Input.MainAxes.x * m_Input.Scalar
            * (m_Input.MainAxes.y * m_Input.Scalar < 0 ? -1.f : 1.f) * 2.f;

        if (m_Input.EnableButtonPhysical)
        {
            if (!m_Input.EnableButton)
            {
                set_mode_service = true;
                set_mode_service_enable = true;
                m_Input.EnableButton = true;
            }
        }
        else
            m_Input.EnableButton = false;

        if (m_Input.DisableButtonPhysical)
        {
            if (!m_Input.DisableButton)
            {
                set_mode_service = true;
                set_mode_service_enable = false;
                m_Input.DisableButton = true;
            }
        }
        else
            m_Input.DisableButton = false;
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

        for (unsigned i = 0; i < m_Wheels.size(); i++)
        {
            m_Wheels[i].TargetRPM = 60.f * (lin_x - m_Wheels[i].Y * ang)
                                  / (2.f * m_Wheels[i].Radius * M_PI)
                                  * (m_Wheels[i].Invert ? -1.f : 1.f);
            m_RPMMessage.data[i] = m_Wheels[i].TargetRPM;
        }
    }
    m_RPMPublisher->publish(m_RPMMessage);

    if (!m_SetModeClientWaiting && set_mode_service)
    {
        if (!m_SetModeClient->service_is_ready())
            RCLCPP_WARN(GetNode()->get_logger(), "SetMode service is not available.");

        else
        {
            auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
            request->mode.mode = (set_mode_service_enable ? edu_robot::msg::Mode::REMOTE_CONTROLLED : edu_robot::msg::Mode::INACTIVE);

            if (set_mode_service_enable)
                RCLCPP_INFO(GetNode()->get_logger(), "Enabeling ...");
            else
                RCLCPP_INFO(GetNode()->get_logger(), "Disabling ...");

            m_SetModeClient->async_send_request(request, std::bind(&NoName::SetModeClientCallback, this, std::placeholders::_1));

            m_SetModeClientWaiting = true;
            m_SetModeClientTimeSent = GetNode()->now().seconds();
        }
    }
    else if (
        m_SetModeClientWaiting && m_SetModeClientTimeSent + 5 < GetNode()->now().seconds())
    {
        RCLCPP_INFO(GetNode()->get_logger(), "SetMode Service didn't respond in 5 seconds");
        m_SetModeClientWaiting = false;
    }

    rclcpp::spin_some(GetNode());
}
