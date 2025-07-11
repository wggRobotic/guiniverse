#include <guiniverse/idefix/idefix.hpp>

void Idefix::OnFrame()
{
    rclcpp::spin_some(GetNode());

    m_ImageSystemBackendGST->OnFrame();

    float lin_x = 0.f;
    float lin_y = 0.f;
    float ang_z = 0.f;
    bool gas = false;

    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        gas = m_Input.gas_button;

        lin_x = m_Input.main_axes.y * m_Input.scalar;
        lin_y = 0.f;
        ang_z = m_Input.main_axes.x * m_Input.scalar
              * (m_Input.main_axes.y * m_Input.scalar < 0 ? -1.f : 1.f) * 2.f;
    }

    m_TwistMessage.linear.x = (gas ? lin_x : 0.f);
    m_TwistMessage.linear.y = (gas ? lin_y : 0.f);
    m_TwistMessage.angular.z = (gas ? ang_z : 0.f);
    m_TwistPublisher->publish(m_TwistMessage);

    rclcpp::spin_some(GetNode());
}
