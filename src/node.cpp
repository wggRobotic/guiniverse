#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <mutex>
#include <map>
#include <vector>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <guiniverse/shared_data.hpp>
#include <guiniverse/node.hpp>

using namespace std::chrono_literals;

#define MAKE_CALLBACK(INDEX) [this, INDEX](const sensor_msgs::msg::Image::ConstSharedPtr &msg) { SetImage(INDEX, msg->data, msg->width, msg->height, msg->step, msg->encoding); }


std::mutex twist_mutex;
geometry_msgs::msg::Twist shared_twist;

std::mutex gripper_mutex;
std_msgs::msg::Float32MultiArray shared_gripper;

std::mutex barcode_mutex;
std::map<std::string, size_t> shared_barcodes;

std::mutex shared_data_mutex;
std::string shared_data;

std::mutex image_topics_mutex;
std::vector<std::string> shared_image_topics = {"n10/front/color", "n10/rear/color", "n10/gripper/color"};

std::mutex image_mutex;
std::vector<cv::Mat> shared_image_data;

GuiniverseNode::GuiniverseNode()
    : Node("guiniverse"), count_(0), running_(true)
{
    m_Timer = this->create_wall_timer(500ms, std::bind(&GuiniverseNode::TimerCallback, this));
    m_TwistPublisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    m_GripperPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("gripper", 10);

    m_BarcodeSubscriber = this->create_subscription<std_msgs::msg::String>(
        "barcode", 10, std::bind(&GuiniverseNode::BarcodeCallback, this, std::placeholders::_1));

    m_EnableMotorClient = this->create_client<std_srvs::srv::SetBool>("enable_motor");
}
void GuiniverseNode::SetImage(size_t index, const std::vector<uint8_t> &data, uint32_t width, uint32_t height, uint32_t step, const std::string &encoding)
{
    std::lock_guard<std::mutex> data_lock(image_mutex);

    if (index >= shared_image_data.size())
    {
        std::cerr << "Index out of range: " << index << std::endl;
        return;
    }

    int cv_type;
    if (encoding == "mono8")
    {
        cv_type = CV_8UC1;
    }
    else if (encoding == "bgr8")
    {
        cv_type = CV_8UC3;
    }
    else if (encoding == "rgba8")
    {
        cv_type = CV_8UC4;
    }
    else
    {
        std::cerr << "Unsupported encoding: " << encoding << std::endl;
        return;
    }

    cv::Mat image(height, width, cv_type, const_cast<uint8_t *>(data.data()), step);
    shared_image_data[index] = image.clone();
}


void GuiniverseNode::SetupWithImageTransport(image_transport::ImageTransport &it)
{
    {
        std::lock_guard<std::mutex> lock_image_topics(image_topics_mutex);
        image_subscribers.resize(shared_image_topics.size());
    }

    {
        std::lock_guard<std::mutex> lock_image_data(image_mutex);
        shared_image_data.resize(shared_image_topics.size());
    }

    for (size_t i = 0; i < shared_image_topics.size(); i++)
    {
        image_subscribers[i] = it.subscribe(
            shared_image_topics.at(i), 10, MAKE_CALLBACK(i));
    }
}


void GuiniverseNode::run()
{
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && running_)
    {
        {
            std::lock_guard<std::mutex> lock(shared_data_mutex);
            shared_data = "ROS2 is running: " + std::to_string(this->now().seconds());
        }

        if (!running)
        {
            running_ = running;
        }

        rclcpp::spin_some(shared_from_this());
        rate.sleep();
    }
}

void GuiniverseNode::BarcodeCallback(const StringConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(barcode_mutex);
    shared_barcodes[msg->data]++;
}


void GuiniverseNode::TimerCallback()
{
    {
        std::lock_guard<std::mutex> lock(twist_mutex);
        shared_twist = m_TwistMessage;
    }

    {
        std::lock_guard<std::mutex> lock(gripper_mutex);
        shared_gripper = m_GripperMessage;
    }
}


void GuiniverseNode::SetMotorStatus(bool status)
{
    std::lock_guard<std::mutex> lock(shared_data_mutex);
    shared_data = status ? "Motors Enabled" : "Motors Disabled";
}