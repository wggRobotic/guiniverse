#include <guiniverse/image_system/image_system_backend_ros.hpp>

ImageSystemBackendROS::ImageSystemBackendROS(std::shared_ptr<ImageSystem> image_system, std::shared_ptr<rclcpp::Node> node)
    : ImageSystemBackend(image_system), m_Node(node), m_ImageTransport(std::make_shared<image_transport::ImageTransport>(node))
{
}

ImageSystemBackendROS::~ImageSystemBackendROS()
{
}

void ImageSystemBackendROS::AddSubscriber(const std::string& topic_name)
{
    int size = m_Processors.size();
    m_Processors.resize(size + 1);

    m_Processors[size].TopicName = topic_name;

    auto callback = [&](const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        std::vector buffer = msg->data;
        cv::Mat image(
            static_cast<int>(msg->width),
            static_cast<int>(msg->height),
            CV_8UC3,
            buffer.data());
        m_ImageSystem->ImageCallback(m_Processors[size].Index, image);
    };

    image_transport::TransportHints transport_hints(m_Node.get(), "compressed");
    m_Processors[size].Subscriber = m_ImageTransport->subscribe(topic_name, 10, callback, nullptr, &transport_hints);
    m_Processors[size].Index = m_ImageSystem->AddImageProcessor("Ros2 topic " + topic_name);
}
