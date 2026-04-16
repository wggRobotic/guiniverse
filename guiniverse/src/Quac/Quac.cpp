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

    m_TwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_pilot", 10);
    m_IPPublisher = node->create_publisher<std_msgs::msg::String>("video_target_ip", 10);
    m_LastIPTime = node->now();
    
    node->declare_parameter<std::string>("host_ip", "127.0.0.1");
    m_IPMessage.data = node->get_parameter("host_ip").as_string();

    m_ImuSubscriber = node->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&Quac::ImuCallback, this, std::placeholders::_1));
    m_MagneticFieldSubscriber = node->create_subscription<sensor_msgs::msg::MagneticField>("magnetic_field", 10, std::bind(&Quac::MagneticFieldCallback, this, std::placeholders::_1));;

    m_DataCaptureSystem = std::make_shared<DataCaptureSystem>(node);
    m_DataCaptureSystem->addSection("QRCodes", "qrcode");

    m_ImageSystem = std::make_shared<ImageSystem>(node);

    m_ImageSystemBackendGST = std::make_shared<ImageSystemBackendGST>(m_ImageSystem);
    m_ImageSystemBackendGST->addSink(5000);
    m_ImageSystemBackendGST->addSink(5001);

    m_ImageSystemBackendROS = std::make_shared<ImageSystemBackendROS>(m_ImageSystem, node);
    m_ImageSystemBackendROS->addSubscriber("thermal_image");

    m_Arm.publish_pose = false;
    for (int i = 0; i < 3; i++) { m_Arm.joints[i].index = -1; m_Arm.joints[i].value = 0;}
    
    m_JointStatesSubscriber = node->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&Quac::jointStateCallback, this, std::placeholders::_1));
    m_ArmPosePublisher = node->create_publisher<geometry_msgs::msg::Pose>("ee_pos", 10);
}

void Quac::onShutdown()
{
    m_TwistPublisher.reset();
    m_IPPublisher.reset();

    m_ImuSubscriber.reset();
    m_MagneticFieldSubscriber.reset();
    m_DataCaptureSystem.reset();
    m_ImageSystemBackendGST.reset();
    m_ImageSystemBackendROS.reset();
    m_ImageSystem.reset();

    m_JointStatesSubscriber.reset();
    m_ArmPosePublisher.reset();
}

void Quac::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_SensorData.mutex);

    m_SensorData.acceleration.x = msg->linear_acceleration.x;
    m_SensorData.acceleration.y = msg->linear_acceleration.y;
    m_SensorData.acceleration.z = msg->linear_acceleration.z;

    m_SensorData.angular_velocity.x = msg->angular_velocity.x;
    m_SensorData.angular_velocity.y = msg->angular_velocity.y;
    m_SensorData.angular_velocity.z = msg->angular_velocity.z;
}

void Quac::MagneticFieldCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_SensorData.mutex);

    m_SensorData.magnetic_field.x = msg->magnetic_field.x;
    m_SensorData.magnetic_field.y = msg->magnetic_field.y;
    m_SensorData.magnetic_field.z = msg->magnetic_field.z;
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