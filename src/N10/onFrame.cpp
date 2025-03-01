#include <guiniverse/N10/N10.hpp>

void N10::onFrame()
{
    float lin_x = 0.f;
    float lin_y = 0.f;
    float ang = 0.f;
    bool gas = false;
    bool enable_service = false;
    bool enable_service_enable;

    if (!m_GripperMode.load()) {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        gas = m_Input.gas_button;

        if (m_Input.dog_walk_button) {
            lin_x = m_Input.main_axes.y * m_Input.scalar;
            lin_y = m_Input.main_axes.x * m_Input.scalar;
            ang = 0.f;
        }
        else {
            lin_x = m_Input.main_axes.y * m_Input.scalar;
            lin_y = 0.f;
            ang = m_Input.main_axes.x * m_Input.scalar * (m_Input.main_axes.y * m_Input.scalar < 0 ? -1.f : 1.f) * 2.f;
        }

        if (m_Input.enable_button)
        {
            enable_service = true;
            enable_service_enable = true;
        }
        if (m_Input.disable_button)
        {
            enable_service = true;
            enable_service_enable = false;
        }
    }

    {
        m_TurtleTwistMessage.linear.x = (gas ? lin_x : 0.f);
        m_TurtleTwistMessage.linear.y = (gas ? lin_y : 0.f);
        m_TurtleTwistMessage.angular.z = (gas ? ang : 0.f);
       
        m_TurtleTwistPublisher->publish(m_TurtleTwistMessage);
    }

    {
        std::lock_guard<std::mutex> lock(m_WheelsMutex);
    
        m_WheelsRPMMessage.data.resize(m_Wheels.size());
        m_WheelsServoMessage.data.resize(m_Wheels.size());

        for(int i = 0; i < m_Wheels.size(); i++)
        {
            float comp_x = lin_x - ang * m_Wheels.at(i).y;
            float comp_y = lin_y + ang * m_Wheels.at(i).x;

            float target_rpm, target_angle;

            if (gas) 
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

            m_WheelsServoMessage.data.at(i) = target_angle;
            m_Wheels.at(i).target_angle = target_angle;
        }

        m_WheelsRPMPublisher->publish(m_WheelsRPMMessage);
        m_WheelsServoPublisher->publish(m_WheelsServoMessage);
    }

}