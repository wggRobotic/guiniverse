#include <guiniverse/Quac/Quac.hpp>
#include <string>

Quac::Quac() : RobotController("Quac", "quac")
{   
}

Quac::~Quac()
{
}

void Quac::onStartup()
{
    m_Input.gas_button = false;
    m_Input.publish_cmd = true;

    m_TwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("/quac/cmd_vel_pilot", 10);

    m_DataCaptureSystem = std::make_shared<DataCaptureSystem>(node);
    m_DataCaptureSystem->addSection("QRCodes", "qrcode");

    m_ImageSystem = std::make_shared<ImageSystem>(node);

    m_ImageSystemBackendGST = std::make_shared<ImageSystemBackendGST>(m_ImageSystem);
    m_ImageSystemBackendGST->addSink(5000);
    m_ImageSystemBackendGST->addSink(5001);

    m_Arm.publish_pose = false;
    for (int i = 0; i < 3; i++) { m_Arm.joints[i].index = -1; m_Arm.joints[i].value = 0;}
    
    m_JointStatesSubscriber = node->create_subscription<sensor_msgs::msg::JointState>("/quac/joint_states", 10, std::bind(&Quac::jointStateCallback, this, std::placeholders::_1));
    m_ArmPosePublisher = node->create_publisher<geometry_msgs::msg::Pose>("/quac/ee_pos", 10);
}

void Quac::onShutdown()
{
    m_TwistPublisher.reset();
    m_DataCaptureSystem.reset();
    m_ImageSystemBackendGST.reset();
    m_ImageSystem.reset();

    m_JointStatesSubscriber.reset();
    m_ArmPosePublisher.reset();
}

void Quac::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::string joint_names[3] = {"arm_servo_0_joint", "arm_servo_1_joint", "arm_servo_0_joint"};

    std::lock_guard<std::mutex> lock(m_Arm.mutex);

    for (int i = 0; i < 3; i++)
    {
        if (m_Arm.joints[i].index != -1)
        {
            if (m_Arm.joints[i].index < msg->name.size())
                if (msg->name[m_Arm.joints[i].index] == joint_names[i])
                {
                    m_Arm.joints[i].value = msg->position[m_Arm.joints[i].index];
                    continue;
                }

            m_Arm.joints[i].index = -1;
        }
        
        if (m_Arm.joints[i].index == -1)
        {
            for (int j = 0; j < msg->name.size(); j++)
            {
                if (msg->name[j] == joint_names[i])
                {
                    m_Arm.joints[i].index = j;
                    m_Arm.joints[i].value = msg->position[j];
                }
            }
        }
    }
    
}