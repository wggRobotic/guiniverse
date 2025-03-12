#include <guiniverse/ImageSystem/ImageSystemBackendROS.hpp>

ImageSystemBackendROS::ImageSystemBackendROS(std::shared_ptr<ImageSystem> image_system, std::shared_ptr<rclcpp::Node> node) 
    : ImageSystemBackend(image_system), m_Node(node), m_ImageTransport(std::make_shared<image_transport::ImageTransport>(node))
{
    
}

ImageSystemBackendROS::~ImageSystemBackendROS()
{

}

#define MAKE_CALLBACK(INDEX) [this, INDEX](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {\
\
    int image_layout = GL_RGB;\
\
    if (msg->encoding == "mono8") image_layout = GL_R;\
    else if (msg->encoding == "bgr8") image_layout = GL_BGR;\
    else if (msg->encoding == "rgba8") image_layout = GL_RGBA;\
\
    m_ImageSystem->ImageCallback(m_Processors[INDEX].index, image_layout, msg->width, msg->height, (unsigned char*) msg->data.data());\
}\


void ImageSystemBackendROS::addSubscriber(const std::string& topic_name, int extras)
{
    int size = m_Processors.size();
    m_Processors.resize(size + 1);

    m_Processors[size].topic_name = topic_name;

    image_transport::TransportHints transport_hints(m_Node.get(), "compressed");
    m_Processors[size].subscriber = m_ImageTransport->subscribe(topic_name, 10, MAKE_CALLBACK(size), nullptr, &transport_hints);
    m_Processors[size].index = m_ImageSystem->addImageProcessor(extras, "Ros2 topic " + topic_name);
}