#include <guiniverse/N10/N10.hpp>

#include <guiniverse/imgui_utils.hpp>

N10::N10() : RobotController("N10", "n10"), m_Wheels(6)
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

}

N10::~N10()
{
}

void N10::initImageSystem()
{
    m_ImageSystem = std::make_shared<ImageSystem>(shared_from_this());
}

void N10::addWheel(float x, float y, float radius, bool invert) 
{
    RoverWheel wheel = {x, y, radius, invert, 0.f, 0.f, 0.f, 0.f};

    m_Wheels.push_back(wheel);
}

void N10::ImGuiPanels(GLFWwindow* window, JoystickInput& input)
{
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        main_axes = ImVec2(0.f, 0.f);

        gas_button = false;
        dog_walk_button = false;
        enable_button = false;
        disable_button = false;

        switch (input.getDeviceProfile())
        {
            case INPUT_PROFILE_LOGITECH_JOYSTICK: {
                main_axes = ImVec2(input.getAxis(0), input.getAxis(1));

                float new_joystick_scalar = input.getAxis(3) / 2.f + 0.5f;
                if (new_joystick_scalar != joystick_scalar) {
                    joystick_scalar = new_joystick_scalar;
                    scalar = new_joystick_scalar;
                }

                gas_button |= input.getButton(0);
                dog_walk_button |= input.getButton(1);

                if (input.getButton(8)) gripper_mode = false;
                else if (input.getButton(9)) gripper_mode = true;
                
                enable_button |= input.getButton(10);
                disable_button |= input.getButton(11);
                
            } break;

        }

        if (glfwGetKey(window, GLFW_KEY_A)) main_axes.x = 1.f;
        else if (glfwGetKey(window, GLFW_KEY_D)) main_axes.x = -1.f;
        if (glfwGetKey(window, GLFW_KEY_W)) main_axes.y = 1.f;
        else if (glfwGetKey(window, GLFW_KEY_S)) main_axes.y = -1.f;

        float length = sqrtf(main_axes.x * main_axes.x + main_axes.y * main_axes.y);
        if (length > 1.0f) main_axes = ImVec2(main_axes.x / length, main_axes.y / length);

        gas_button |= glfwGetKey(window, GLFW_KEY_SPACE);
        dog_walk_button |= glfwGetKey(window, GLFW_KEY_M);

        if (glfwGetKey(window, GLFW_KEY_T)) gripper_mode = false;
        if (glfwGetKey(window, GLFW_KEY_G)) gripper_mode = true;

        enable_button |= glfwGetKey(window, GLFW_KEY_F);
        disable_button |= glfwGetKey(window, GLFW_KEY_R);

    }

    if (ImGui::Begin("Control"))
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        if (ImGui::RadioButton("Drive", !gripper_mode)) gripper_mode = false;
        if (ImGui::RadioButton("Gripper", gripper_mode)) gripper_mode = true;

        ImVec2 pos = ImGui::GetCursorScreenPos();

        main_axes = imgui_joystick("virtual joystick", 200.f, ImVec2(0.1f, 0.1f), (main_axes.x == 0.f && main_axes.y == 0.f) ? 0 : &main_axes, (gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        ImGui::Text("Joystick %f %f", main_axes.x, main_axes.y);

        if (ImGui::Button("Enable")) enable_button = true;
        if (ImGui::Button("Disable")) disable_button = true;

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 220.f, pos.y + ImGui::GetStyle().ItemSpacing.y));

        ImGui::VSliderFloat("##vslider", ImVec2(20, 200), &scalar, 0.0f, 1.0f, "%.2f");

        ImGui::End();
    }

    if (ImGui::Begin("Visualization"))
    {

        {
            std::lock_guard<std::mutex> lock(m_WheelsMutex);
        }
        
        ImGui::End();
    }
    
}

void N10::onFrame()
{

    rclcpp::spin_some(shared_from_this());

    m_WheelsRPMMessage.data.resize(m_Wheels.size());
    m_WheelsServoMessage.data.resize(m_Wheels.size());
    m_GripperServoMessage.data.resize(m_Wheels.size());

    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        ImVec2 main_axes_scaled = ImVec2(main_axes.x * scalar * (gas_button ? 1.f : 0.f), main_axes.y * scalar * (gas_button ? 1.f : 0.f));

        if (dog_walk_button) {
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
        float lin_x;
        float lin_y;
        float ang;
        bool gas;

        {
            std::lock_guard<std::mutex> lock(m_InputMutex);

            gas = gas_button;

            if (dog_walk_button) {
                lin_x = main_axes.y * scalar;
                lin_y = main_axes.x * scalar;
                ang = 0.f;
            }
            else {
                lin_x = main_axes.x * scalar;
                lin_y = 0.f;
                ang = main_axes.x * scalar * (main_axes.y * scalar < 0 ? -1.f : 1.f);
            }
        }

        std::lock_guard<std::mutex> lock(m_WheelsMutex);

        for(int i = 0; i < m_Wheels.size(); i++)
        {
            float comp_x = lin_x + ang * m_Wheels.at(i).y;
            float comp_y = lin_y + ang * m_Wheels.at(i).x;

            m_Wheels.at(i).target_rpm = 60.f * sqrtf(comp_x*comp_x + comp_y*comp_y) / (2 * m_Wheels.at(i).radius * M_PI) * (comp_x >= 0.f ? 1.f : -1.f) * (m_Wheels.at(i).invert ? -1.f : 1.f);

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
}

void N10::WheelsServoFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);
}

void N10::GripperServoFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);
}

void N10::EnableMotorClientCallback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response) 
{
    RCLCPP_INFO(this->get_logger(), "Response: success=%s, message='%s'",
                response.get()->success ? "true" : "false",
                response.get()->message.c_str());

    m_EnableMotorClientWaiting = false;
}