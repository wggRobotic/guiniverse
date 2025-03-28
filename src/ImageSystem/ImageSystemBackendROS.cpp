#include <guiniverse/ImageSystem/ImageSystemBackendROS.hpp>

ImageSystemBackendROS::ImageSystemBackendROS(std::shared_ptr<ImageSystem> image_system, std::shared_ptr<rclcpp::Node> node) 
    : ImageSystemBackend(image_system), m_Node(node), m_ImageTransport(std::make_shared<image_transport::ImageTransport>(node))
{
    
}

ImageSystemBackendROS::~ImageSystemBackendROS()
{

}

#define MAKE_CALLBACK(INDEX) [this, INDEX](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {\
    cv::Mat image_mat((int)msg->width, (int)msg->height, CV_8UC3, (void*)msg->data.data());\
    m_ImageSystem->ImageCallback(m_Processors[INDEX].index, image_mat);\
}\


void ImageSystemBackendROS::addSubscriber(const std::string& topic_name, int addons)
{
    int size = m_Processors.size();
    m_Processors.resize(size + 1);

    m_Processors[size].topic_name = topic_name;

    image_transport::TransportHints transport_hints(m_Node.get(), "compressed");
    m_Processors[size].subscriber = m_ImageTransport->subscribe(topic_name, 10, MAKE_CALLBACK(size), nullptr, &transport_hints);
    m_Processors[size].index = m_ImageSystem->addImageProcessor(addons, "Ros2 topic " + topic_name);
}