#include <guiniverse/Idefix/Idefix.hpp>

void Idefix::onStartup()
{

}

void Idefix::onShutdown()
{

}

void Idefix::onFrame()
{
    float lin_x = 0.f;
    float lin_y = 0.f;
    float ang = 0.f;
    bool gas = false;

    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        gas = m_Input.gas_button;

        lin_x = m_Input.main_axes.y * m_Input.scalar;
        lin_y = 0.f;
        ang = m_Input.main_axes.x * m_Input.scalar * (m_Input.main_axes.y * m_Input.scalar < 0 ? -1.f : 1.f) * 2.f;
    }

    {
        m_TurtleTwistMessage.linear.x = (gas ? lin_x : 0.f);
        m_TurtleTwistMessage.linear.y = (gas ? lin_y : 0.f);
        m_TurtleTwistMessage.angular.z = (gas ? ang : 0.f);
    }
    m_TurtleTwistPublisher->publish(m_TurtleTwistMessage);

    {
        m_TwistMessage.linear.x = (gas ? lin_x : 0.f);
        m_TwistMessage.linear.y = (gas ? lin_y : 0.f);
        m_TwistMessage.angular.z = (gas ? ang : 0.f);
    }
    m_TwistPublisher->publish(m_TwistMessage);
}