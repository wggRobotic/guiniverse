#include <GL/glew.h>

#include <guiniverse/ImageSystem.hpp>

#include <imgui.h>

#define GLCALL(call) \
    do { \
        call; \
        GLenum err = glGetError(); \
        if (err != GL_NO_ERROR) { \
            fprintf(stderr, "OpenGL error in %s at %s:%d: %d\n", #call, __FILE__, __LINE__, err); \
        } \
    } while (0)

void Image::SetupMat(cv::Mat &mat)
{
    int cv_type;
    if (Encoding == "mono8") cv_type = CV_8UC1;
    else if (Encoding == "bgr8") cv_type = CV_8UC3;
    else if (Encoding == "rgba8") cv_type = CV_8UC4;
    else
    {
        std::cerr << "Unsupported Encoding: " << Encoding << std::endl;
        return;
    }

    mat = cv::Mat(Height, Width, cv_type, Data.data(), Step);
}


#define MAKE_CALLBACK(INDEX) [this, INDEX](const sensor_msgs::msg::Image::ConstSharedPtr &msg) { setImage(INDEX, msg->data, msg->width, msg->height, msg->step, msg->encoding); }

ImageSystem::ImageSystem(std::shared_ptr<rclcpp::Node> Node) : image_transport::ImageTransport(Node), m_Node(Node)
{
    m_ImageProcessors.reserve(5);
}

void ImageSystem::addTopic(const std::string& TopicName) {

    std::lock_guard<std::mutex> lock(m_ImageProcessorMutex);

    int current_size = m_ImageProcessors.size();

    m_ImageProcessors.resize(current_size + 1);

    m_ImageProcessors.at(current_size).topic_name = TopicName;
    m_ImageProcessors.at(current_size).subscriber = subscribe(TopicName, 10, MAKE_CALLBACK(current_size));
}

void ImageSystem::setImage(size_t index, const std::vector<uint8_t> &data, uint32_t width, uint32_t height, uint32_t step, const std::string &encoding)
{
    std::lock_guard<std::mutex> lock(m_ImageProcessorMutex);

    if (index >= m_ImageProcessors.size())
    {
        std::cerr << "Index out of range: " << index << std::endl;
        return;
    }

    m_ImageProcessors.at(index).image = {width, height, step, encoding, data};
    m_ImageProcessors.at(index).dirty = true;
    m_ImageProcessors.at(index).holds_image = true;
}

void ImageSystem::ImGuiPanels() {

    std::lock_guard<std::mutex> lock(m_ImageProcessorMutex);

    for (int i = 0; i < m_ImageProcessors.size(); i++) 
    {
        if (ImGui::Begin(m_ImageProcessors.at(i).topic_name.c_str()))
        {
            if (m_ImageProcessors.at(i).holds_image)
            {
                if (m_ImageProcessors.at(i).init == false)
                {
                    GLCALL(glGenTextures(1, &m_ImageProcessors.at(i).texture));
                    GLCALL(glBindTexture(GL_TEXTURE_2D, m_ImageProcessors.at(i).texture));

                    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
                    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
                    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
                    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

                    GLCALL(glBindTexture(GL_TEXTURE_2D, 0));

                    m_ImageProcessors.at(i).init = true;
                }

                if (m_ImageProcessors.at(i).dirty)
                {
                    GLCALL(glBindTexture(GL_TEXTURE_2D, m_ImageProcessors.at(i).texture));

                    if (m_ImageProcessors.at(i).texture_width != m_ImageProcessors.at(i).image.Width || m_ImageProcessors.at(i).texture_height != m_ImageProcessors.at(i).image.Height)
                    {
                        m_ImageProcessors.at(i).texture_width = m_ImageProcessors.at(i).image.Width;
                        m_ImageProcessors.at(i).texture_height = m_ImageProcessors.at(i).image.Height;

                        GLCALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, m_ImageProcessors.at(i).texture_width, m_ImageProcessors.at(i).texture_height, 0, GL_BGR, GL_UNSIGNED_BYTE, NULL));                
                    }
                    
                    GLCALL(glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_ImageProcessors.at(i).image.Width, m_ImageProcessors.at(i).image.Height, GL_BGR, GL_UNSIGNED_BYTE, m_ImageProcessors.at(i).image.Data.data()));

                    GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
                }

                m_ImageProcessors.at(i).dirty = false;
            
                ImVec2 widget_size = ImGui::GetContentRegionAvail();

                float image_aspect = (float)m_ImageProcessors.at(i).texture_width / (float)m_ImageProcessors.at(i).texture_height;
                float widget_aspect = widget_size.x / widget_size.y;

                ImVec2 display_size;

                if (widget_aspect > image_aspect)
                    display_size = ImVec2(widget_size.y * image_aspect, widget_size.y);
                else 
                    display_size = ImVec2(widget_size.x, widget_size.x / image_aspect);
                
                ImGui::Image((ImTextureID)m_ImageProcessors.at(i).texture, display_size);
            }
            else 
                ImGui::Text("No image yet");
        }
        ImGui::End();
    }

}