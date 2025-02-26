#include <guiniverse/N10/N10.hpp>

#include <guiniverse/imgui_utils.hpp>

N10::N10() : RobotController("N10", "n10")
{
    
    m_BarcodeSubscriber = create_subscription<std_msgs::msg::String>("barcode", 10, std::bind(&N10::BarcodeCallback, this, std::placeholders::_1));

    m_WheelsRPMFeedbackSubscriber = create_subscription<std_msgs::msg::Float32MultiArray>("wheels/rpms/feedback", 10, std::bind(&N10::WheelsRPMFeedbackCallback, this, std::placeholders::_1));
    m_WheelsServoFeedbackSubscriber = create_subscription<std_msgs::msg::Float32MultiArray>("wheels/servo/feedback", 10, std::bind(&N10::WheelsServoFeedbackCallback, this, std::placeholders::_1));
    m_GripperServoFeedbackSubscriber = create_subscription<std_msgs::msg::Float32MultiArray>("gripper/servo/feedback", 10, std::bind(&N10::GripperServoFeedbackCallback, this, std::placeholders::_1));

    m_WheelsRPMPublisher = create_publisher<std_msgs::msg::Float32MultiArray>("wheels/rpms/input", 10);
    m_WheelsServoPublisher = create_publisher<std_msgs::msg::Float32MultiArray>("wheels/servo/input", 10);
    m_GripperServoPublisher = create_publisher<std_msgs::msg::Float32MultiArray>("gripper/servo/input", 10);

    m_TurtleTwistPublisher = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    m_EnableMotorClient = create_client<std_srvs::srv::SetBool>("wheels/enable");

    m_Wheels.reserve(6);
}

N10::~N10()
{
}

void N10::initImageSystem()
{
    m_ImageSystem = std::make_shared<ImageSystem>(shared_from_this());

    m_ImageSystem->addTopic("front/color");
    m_ImageSystem->addTopic("rear/color");
    m_ImageSystem->addTopic("gripper/color");

}

void N10::addWheel(float x, float y, float radius, bool invert) 
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);

    RoverWheel wheel = {x, y, radius, invert, 0.f, 0.f, 0.f, 0.f};

    m_Wheels.push_back(wheel);
}

void N10::ImGuiPanels(GLFWwindow* window, JoystickInput& input)
{
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        m_Input.main_axes = ImVec2(0.f, 0.f);

        m_Input.gas_button = false;
        m_Input.dog_walk_button = false;
        m_Input.enable_button = false;
        m_Input.disable_button = false;

        switch (input.getDeviceProfile())
        {
            case INPUT_PROFILE_LOGITECH_JOYSTICK: {
                m_Input.main_axes = ImVec2(input.getAxis(0), input.getAxis(1));

                float new_joystick_scalar = input.getAxis(3) / 2.f + 0.5f;
                if (new_joystick_scalar != m_Input.joystick_scalar) {
                    m_Input.joystick_scalar = new_joystick_scalar;
                    m_Input.scalar = new_joystick_scalar;
                }

                m_Input.gas_button |= input.getButton(0);
                m_Input.dog_walk_button |= input.getButton(1);

                if (input.getButton(8)) m_GripperMode.store(false);
                else if (input.getButton(9)) m_GripperMode.store(true);
                
                m_Input.enable_button |= input.getButton(10);
                m_Input.disable_button |= input.getButton(11);
                
            } break;

        }

        if (glfwGetKey(window, GLFW_KEY_A)) m_Input.main_axes.x = 1.f;
        else if (glfwGetKey(window, GLFW_KEY_D)) m_Input.main_axes.x = -1.f;
        if (glfwGetKey(window, GLFW_KEY_W)) m_Input.main_axes.y = 1.f;
        else if (glfwGetKey(window, GLFW_KEY_S)) m_Input.main_axes.y = -1.f;

        float length = sqrtf(m_Input.main_axes.x * m_Input.main_axes.x + m_Input.main_axes.y * m_Input.main_axes.y);
        if (length > 1.0f) m_Input.main_axes = ImVec2(m_Input.main_axes.x / length, m_Input.main_axes.y / length);

        m_Input.gas_button |= glfwGetKey(window, GLFW_KEY_SPACE);
        m_Input.dog_walk_button |= glfwGetKey(window, GLFW_KEY_M);

        if (glfwGetKey(window, GLFW_KEY_T)) m_GripperMode.store(false);
        if (glfwGetKey(window, GLFW_KEY_G)) m_GripperMode.store(true);

        m_Input.enable_button |= glfwGetKey(window, GLFW_KEY_F);
        m_Input.disable_button |= glfwGetKey(window, GLFW_KEY_R);

    }

    if (ImGui::Begin("Control"))
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        if (ImGui::RadioButton("Drive", !m_GripperMode.load())) m_GripperMode.store(false);
        if (ImGui::RadioButton("Gripper", m_GripperMode.load())) m_GripperMode.store(true);

        ImVec2 pos = ImGui::GetCursorScreenPos();

        m_Input.main_axes = imgui_joystick("virtual joystick", 200.f, ImVec2(0.1f, 0.1f), (m_Input.main_axes.x == 0.f && m_Input.main_axes.y == 0.f) ? 0 : &m_Input.main_axes, (m_Input.gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        ImGui::Text("Joystick %f %f", m_Input.main_axes.x, m_Input.main_axes.y);

        if (ImGui::Button("Enable")) m_Input.enable_button = true;
        if (ImGui::Button("Disable")) m_Input.disable_button = true;

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 220.f, pos.y + ImGui::GetStyle().ItemSpacing.y));

        ImGui::VSliderFloat("##vslider", ImVec2(20, 200), &m_Input.scalar, 0.0f, 1.0f, "%.2f");

    }
    ImGui::End();

    if (ImGui::Begin("Visualization"))
    {
        {
            std::lock_guard<std::mutex> lock(m_WheelsMutex);

            for(int i = 0; i < m_Wheels.size(); i++)
            {
                imgui_arrow(ImVec2(-m_Wheels.at(i).y * 1000.f + 200.f, -m_Wheels.at(i).x * 1000.f + 200.f), m_Wheels.at(i).target_angle, m_Wheels.at(i).target_rpm * (m_Wheels.at(i).invert ? -1.f : 1.f), IM_COL32(60, 60, 60, 255), 3.f, 9.f, false);
                
                imgui_arrow(ImVec2(-m_Wheels.at(i).y * 1000.f + 200.f, -m_Wheels.at(i).x * 1000.f + 200.f), m_Wheels.at(i).last_angle, m_Wheels.at(i).last_rpm * (m_Wheels.at(i).invert ? -1.f : 1.f), IM_COL32(255, 255, 255, 255), 5.f, 10.f, true);
            }
            
            
        }
        
    }
    ImGui::End();
    
    m_ImageSystem->ImGuiPanels();

}

void N10::onFrame()
{

    rclcpp::spin_some(shared_from_this());

    {
        std::lock_guard<std::mutex> lock(m_WheelsMutex);
    
        m_WheelsRPMMessage.data.resize(m_Wheels.size());
        m_WheelsServoMessage.data.resize(m_Wheels.size());
        m_GripperServoMessage.data.resize(m_Wheels.size());
    }

    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        ImVec2 main_axes_scaled = ImVec2(m_Input.main_axes.x * m_Input.scalar * (m_Input.gas_button ? 1.f : 0.f), m_Input.main_axes.y * m_Input.scalar * (m_Input.gas_button ? 1.f : 0.f));

        if (m_Input.dog_walk_button) {
            m_TurtleTwistMessage.linear.x = main_axes_scaled.y;
            m_TurtleTwistMessage.linear.y = main_axes_scaled.x;
            m_TurtleTwistMessage.angular.z = 0.f;
        }
        else {
            m_TurtleTwistMessage.linear.x = main_axes_scaled.y;
            m_TurtleTwistMessage.linear.y = 0.f;
            m_TurtleTwistMessage.angular.z = main_axes_scaled.x * (main_axes_scaled.y < 0 ? -1.f : 1.f);
        }

    }

    m_TurtleTwistPublisher->publish(m_TurtleTwistMessage);

    {
        float lin_x = 0.f;
        float lin_y = 0.f;
        float ang = 0.f;
        bool gas = false;

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
                ang = m_Input.main_axes.x * m_Input.scalar * (m_Input.main_axes.y * m_Input.scalar < 0 ? -1.f : 1.f) * 4.f;
            }
        }

        std::lock_guard<std::mutex> lock(m_WheelsMutex);

        for(int i = 0; i < m_Wheels.size(); i++)
        {
            float comp_x = lin_x - ang * m_Wheels.at(i).y;
            float comp_y = lin_y + ang * m_Wheels.at(i).x;

            if (gas) m_Wheels.at(i).target_rpm = 60.f * sqrtf(comp_x*comp_x + comp_y*comp_y) / (2 * m_Wheels.at(i).radius * M_PI) * (comp_x >= 0.f ? 1.f : -1.f) * (m_Wheels.at(i).invert ? -1.f : 1.f);
            else m_Wheels.at(i).target_rpm = 0.f;

            m_Wheels.at(i).target_angle = (comp_y == 0.f ? 
                0.f : 
                (comp_x == 0.f ? 
                    M_PI / 2 * (comp_y > 0.f ? 1.f : -1.f) : 
                    atanf(comp_y / comp_x)
                )
            );
        }
    }

    

    rclcpp::spin_some(shared_from_this());

}

void N10::BarcodeCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // std::lock_guard<std::mutex> lock(barcode_mutex);
    // shared_barcodes[msg->data]++;
}

void N10::WheelsRPMFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);

    if (msg->data.size() == m_Wheels.size()) for (int i = 0; i < m_Wheels.size(); i++)
        m_Wheels.at(i).last_rpm = msg->data.at(i);
    
}

void N10::WheelsServoFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);

    if (msg->data.size() == m_Wheels.size()) for (int i = 0; i < m_Wheels.size(); i++)
        m_Wheels.at(i).last_angle = msg->data.at(i);
}

void N10::GripperServoFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_GripperMutex);
}

void N10::EnableMotorClientCallback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response) 
{
    RCLCPP_INFO(this->get_logger(), "Response: success=%s, message='%s'",
                response.get()->success ? "true" : "false",
                response.get()->message.c_str());

    m_EnableMotorClientWaiting = false;
}