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
            m_TwistStampedMessage.header.stamp = node->now();
            m_TwistStampedMessage.header.frame_id = "base_link";
            m_TwistStampedMessage.twist.linear.x = (m_Input.gas_button ? 1.f : 0.f) * m_Input.main_axes.y * m_Input.scalar;
            m_TwistStampedMessage.twist.angular.z = (m_Input.gas_button ? 1.f : 0.f) *  m_Input.main_axes.x * m_Input.scalar * (m_Input.main_axes.y * m_Input.scalar < 0 ? -1.f : 1.f) * 2.f;
            m_TwistStampedPublisher->publish(m_TwistStampedMessage);
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_Arm.mutex);

        if (m_Arm.publish_pose)
        {
            m_ArmPoseMessage.position.x = m_Arm.target_pose.x;
            m_ArmPoseMessage.position.y = m_Arm.target_pose.y;

            m_ArmPosePublisher->publish(m_ArmPoseMessage);
        }
    }
    
}