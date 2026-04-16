#include <guiniverse/Quac/Quac.hpp>

void Quac::onFrame()
{
    m_ImageSystemBackendGST->onFrame();

    rclcpp::spin_some(node);
    // cmd_vel
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        if (m_Input.publish_cmd)
        {
            m_TwistMessage.linear.x = (m_Input.gas_button ? 1.f : 0.f) * m_Input.main_axes.y * m_Input.scalar;
            m_TwistMessage.angular.z = (m_Input.gas_button ? 1.f : 0.f) *  m_Input.main_axes.x * m_Input.scalar * (m_Input.main_axes.y * m_Input.scalar < 0 ? -1.f : 1.f) * 2.f;
            m_TwistPublisher->publish(m_TwistMessage);
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_Arm.mutex);

        if (m_Arm.publish_pose)
        {
            m_ArmPoseMessage.position.x = m_Arm.target_pose.x;
            m_ArmPoseMessage.position.z = m_Arm.target_pose.y;

            m_ArmPosePublisher->publish(m_ArmPoseMessage);
        }
    }

    {
        rclcpp::Time current_time = node->now();

        rclcpp::Duration elapsed = current_time - m_LastIPTime;

        if (elapsed.seconds() >= 1.0)
        {
            m_IPPublisher->publish(m_IPMessage);
            m_LastIPTime = current_time;
        }

    }
    
}