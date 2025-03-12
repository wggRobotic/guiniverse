#include <guiniverse/ImageSystem/ImageSystem.hpp>

#include <imgui.h>

#define GLCALL(call) \
    do { \
        call; \
        GLenum err = glGetError(); \
        if (err != GL_NO_ERROR) { \
            fprintf(stderr, "OpenGL error in %s at %s:%d: %d\n", #call, __FILE__, __LINE__, err); \
        } \
    } while (0)

int ImageSystemImageLayoutPixelSize(int image_layout)
{
    switch (image_layout)
    {
    case GL_RGB:
        return 3;
    case GL_BGR:
        return 3;
    case GL_R:
        return 1;
    case GL_RGBA:
        return 4;

    default:
        return 1;
    }
}

ImageSystem::ImageSystem(rclcpp::Node::SharedPtr node) : m_Node(node) 
{
    m_QRCodePublisher = m_Node->create_publisher<std_msgs::msg::String>("qrcode", 10);
}

int ImageSystem::addImageProcessor(int extras, const std::string& imgui_panel_name)
{
    int size = m_ImageProcessors.size();
    m_ImageProcessors.resize(size + 1);
    m_ImageProcessors[size].imgui_panel_name = imgui_panel_name;
    m_ImageProcessors[size].extras = extras;

    return size;
}

void ImageSystem::ImageCallback(int index, int image_layout, int width, int height, unsigned char* data)
{
    std::lock_guard<std::mutex> lock(*m_ImageProcessors[index].mutex);

    m_ImageProcessors[index].image.dirty = true;
    m_ImageProcessors[index].image.holds = true;
    m_ImageProcessors[index].image.width = width;
    m_ImageProcessors[index].image.height = height;
    m_ImageProcessors[index].image.image_layout = image_layout;
    m_ImageProcessors[index].image.data.assign(data, data + ImageSystemImageLayoutPixelSize(image_layout) * width * height);

    if ((m_ImageProcessors[index].extras & ImageSystemExtra_QRCodeDecoder) && m_QRCodeDecoder.isReady())
    {
        std::vector<std::string> results;
        m_QRCodeDecoder.getLastResult(results);

        for (int i = 0; i < results.size(); i++)
        {
            m_QRCodeMessae.data = results[i];
            m_QRCodePublisher->publish(m_QRCodeMessae);
        }

        m_QRCodeDecoder.startDecoding(width, height, data, image_layout);
    }
}

void ImageSystem::onGuiStartup()
{
    m_QRCodeDecoder.start();

    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        std::lock_guard<std::mutex> lock(*m_ImageProcessors[i].mutex);

        GLCALL(glGenTextures(1, &m_ImageProcessors[i].texture.gl_texture));
        GLCALL(glBindTexture(GL_TEXTURE_2D, m_ImageProcessors[i].texture.gl_texture));

        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

        GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
    }
}

void ImageSystem::onGuiShutdown()
{
    m_QRCodeDecoder.stop();

    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        std::lock_guard<std::mutex> lock(*m_ImageProcessors[i].mutex);

        GLCALL(glDeleteTextures(1, &m_ImageProcessors[i].texture.gl_texture));
    }
}

void ImageSystem::onGuiFrame()
{
    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        std::lock_guard<std::mutex> lock(*m_ImageProcessors[i].mutex);

        if (m_ImageProcessors[i].image.holds)
        {
            GLCALL(glBindTexture(GL_TEXTURE_2D, m_ImageProcessors[i].texture.gl_texture));

            if (
                m_ImageProcessors[i].texture.width != m_ImageProcessors[i].image.width || 
                m_ImageProcessors[i].texture.height != m_ImageProcessors[i].image.height
            )
            {
                m_ImageProcessors[i].texture.width = m_ImageProcessors[i].image.width;
                m_ImageProcessors[i].texture.height = m_ImageProcessors[i].image.height;

                GLCALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, m_ImageProcessors[i].texture.width, m_ImageProcessors[i].texture.height, 0, m_ImageProcessors[i].image.image_layout, GL_UNSIGNED_BYTE, m_ImageProcessors[i].image.data.data()));                
            }
            else
            {
                GLCALL(glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_ImageProcessors[i].texture.width, m_ImageProcessors[i].texture.height, m_ImageProcessors[i].image.image_layout, GL_UNSIGNED_BYTE, m_ImageProcessors[i].image.data.data()));
            }
        
            GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
        }

        if (ImGui::Begin(m_ImageProcessors[i].imgui_panel_name.c_str()))
        {
            if (m_ImageProcessors[i].image.holds)
            {

                ImGui::Checkbox("Flip vertically", &m_ImageProcessors[i].flip_vertically);
                ImGui::Checkbox("Flip horizontally", &m_ImageProcessors[i].flip_horizontally);

                ImVec2 widget_size = ImGui::GetContentRegionAvail();

                float image_aspect = (float)m_ImageProcessors[i].texture.width / (float)m_ImageProcessors[i].texture.height;
                float widget_aspect = widget_size.x / widget_size.y;

                ImVec2 display_size;

                if (widget_aspect > image_aspect)
                    display_size = ImVec2(widget_size.y * image_aspect, widget_size.y);
                else 
                    display_size = ImVec2(widget_size.x, widget_size.x / image_aspect);
                
                ImVec2 uv0 = ImVec2(m_ImageProcessors[i].flip_horizontally ? 1.f : 0.f, m_ImageProcessors[i].flip_vertically ? 1.f : 0.f);
                ImVec2 uv1 = ImVec2(m_ImageProcessors[i].flip_horizontally ? 0.f : 1.f, m_ImageProcessors[i].flip_vertically ? 0.f : 1.f);
                ImGui::Image((ImTextureID)m_ImageProcessors[i].texture.gl_texture, display_size, uv0, uv1);
            }
            else 
                ImGui::Text("No image yet");
        }
        ImGui::End();
    }
}