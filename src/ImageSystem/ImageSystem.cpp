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

void ImageSystemImage::clear()
{
    image.release();
}

void ImageSystemImage::create_texture()
{
    GLCALL(glGenTextures(1, &gl_texture));
    GLCALL(glBindTexture(GL_TEXTURE_2D, gl_texture));

    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

    GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
}

void ImageSystemImage::destroy_texture()
{
    GLCALL(glDeleteTextures(1, &gl_texture));
    texture_width = 0;
    texture_height = 0;
}

void ImageSystemImage::imgui_image(bool flip_vertically, bool flip_horizontally)
{
    bool empty;

    {
        std::lock_guard<std::mutex> lock(image_mutex);

        empty = image.empty();

        if (!empty)
        {
            GLCALL(glBindTexture(GL_TEXTURE_2D, gl_texture));
            if (
                image.cols != texture_width || 
                image.rows != texture_height
            )
            {
                texture_width = image.cols;
                texture_height = image.rows;
                GLCALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, texture_width, texture_height, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data));                
            }
            else
            {
                GLCALL(glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture_width, texture_height, GL_RGB, GL_UNSIGNED_BYTE, image.data));
            }
            GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
        }   
    }

    if (!empty)
    {
        ImVec2 widget_size = ImGui::GetContentRegionAvail();

        float image_aspect = (float)texture_width/ (float)texture_height;
        float widget_aspect = widget_size.x / widget_size.y;

        ImVec2 display_size;

        if (widget_aspect > image_aspect)
            display_size = ImVec2(widget_size.y * image_aspect, widget_size.y);
        else 
            display_size = ImVec2(widget_size.x, widget_size.x / image_aspect);
        
        ImGui::Image(
            (ImTextureID)gl_texture, 
            display_size, 
            ImVec2(flip_horizontally ? 1.f : 0.f, flip_vertically ? 1.f : 0.f), 
            ImVec2(flip_horizontally ? 0.f : 1.f, flip_vertically ? 0.f : 1.f)
        );
    }
    else
    {
        ImGui::Text("No image yet");
    }
}
    
void ImageSystemImage::sub_image_transfer_ownership(cv::Mat& mat)
{
    std::lock_guard<std::mutex> lock(image_mutex);

    dirty = true;
    image = mat;
}

void ImageSystemImage::copy_image(cv::Mat& mat)
{
    std::lock_guard<std::mutex> lock(image_mutex);

    image.copyTo(mat);
}

ImageSystem::ImageSystem(rclcpp::Node::SharedPtr node) : m_Node(node) 
{
    m_ImageProcessors.reserve(3);
}

ImageSystem::~ImageSystem()
{
    for (int i = 0; i < m_ImageProcessors.size(); i++) 
    {
        m_ImageProcessors[i]->addons.thread_should_close.store(true);
        if (m_ImageProcessors[i]->addons.thread.joinable())
            m_ImageProcessors[i]->addons.thread.join();

        if (m_ImageProcessors[i]->addons.flags & ImageSystemAddOn_QRCode)
        {
            quirc_destroy(m_ImageProcessors[i]->addons.qrcode.quirc_instance);
        }
        
    }
}

int ImageSystem::addImageProcessor(int addon_flags, const std::string& imgui_panel_name)
{
    int size = m_ImageProcessors.size();
    m_ImageProcessors.resize(size + 1);
    m_ImageProcessors[size] = std::make_shared<ImageSystemImageProcessor>();

    m_ImageProcessors[size]->imgui_panel_name = imgui_panel_name;
    
    m_ImageProcessors[size]->addons.flags = addon_flags;

    if (m_ImageProcessors[size]->addons.flags & ImageSystemAddOn_QRCode) 
    {
        m_ImageProcessors[size]->addons.qrcode.quirc_instance = quirc_new();
        m_ImageProcessors[size]->addons.qrcode.publisher = m_Node->create_publisher<std_msgs::msg::String>("qrcode", 10);
    }

    m_ImageProcessors[size]->addons.thread = std::thread(&ImageSystem::AddOnThreadFunction, this, size);

    return size;
}

void ImageSystem::AddOnThreadFunction(int index)
{

    while (!m_ImageProcessors[index]->addons.thread_should_close.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        cv::Mat image;
        m_ImageProcessors[index]->image.copy_image(image);

        //qrcode
        if (m_ImageProcessors[index]->addons.flags & ImageSystemAddOn_QRCode && !image.empty()) 
        {
            cv::Mat grayscale;
            cv::cvtColor(image, grayscale, cv::COLOR_RGB2GRAY);

            quirc_resize(m_ImageProcessors[index]->addons.qrcode.quirc_instance, image.cols, image.rows);

            int width, height;
            uint8_t *qr_image = quirc_begin(m_ImageProcessors[index]->addons.qrcode.quirc_instance, &width, &height);
            memcpy(qr_image, grayscale.data, width * height);
            quirc_end(m_ImageProcessors[index]->addons.qrcode.quirc_instance);

            int count = quirc_count(m_ImageProcessors[index]->addons.qrcode.quirc_instance);

            for (int i = 0; i < count; i++) {
                struct quirc_code code;
                struct quirc_data data;

                quirc_extract(m_ImageProcessors[index]->addons.qrcode.quirc_instance, i, &code);

                if (quirc_decode(&code, &data) == QUIRC_SUCCESS) {
                    std_msgs::msg::String msg;
                    msg.data = std::string((char*)data.payload);
                    m_ImageProcessors[index]->addons.qrcode.publisher->publish(msg);
                }
            }
        }

    }
}

void ImageSystem::ImageCallback(int index, cv::Mat& image)
{

    cv::Mat old_image;
    m_ImageProcessors[index]->image.copy_image(old_image);
    m_ImageProcessors[index]->image.sub_image_transfer_ownership(image);

    if (old_image.cols == image.cols && old_image.rows == image.rows)
    {
        cv::Mat diff_image;
        cv::absdiff(image, old_image, diff_image);
        
        m_ImageProcessors[index]->addons.diff.image.sub_image_transfer_ownership(diff_image);
    }
}

void ImageSystem::onGuiStartup()
{
    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        m_ImageProcessors[i]->image.create_texture();

        if (m_ImageProcessors[i]->addons.flags & ImageSystemAddOn_Diff)
        {
            m_ImageProcessors[i]->addons.diff.image.create_texture();
        }
    }
}

void ImageSystem::onGuiShutdown()
{

    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        m_ImageProcessors[i]->image.destroy_texture();

        if (m_ImageProcessors[i]->addons.flags & ImageSystemAddOn_Diff)
        {
            m_ImageProcessors[i]->addons.diff.image.destroy_texture();
        }
    }
}

void ImageSystem::onGuiFrame()
{
    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {

        if (ImGui::Begin(m_ImageProcessors[i]->imgui_panel_name.c_str()))
        {
            ImGui::Checkbox("Flip vertically", &m_ImageProcessors[i]->flip_vertically);
            ImGui::Checkbox("Flip horizontally", &m_ImageProcessors[i]->flip_horizontally);

            m_ImageProcessors[i]->image.imgui_image(m_ImageProcessors[i]->flip_vertically, m_ImageProcessors[i]->flip_horizontally);
        }
        ImGui::End();

        

        if (m_ImageProcessors[i]->addons.flags & ImageSystemAddOn_Diff)
        {
            if (ImGui::Begin((m_ImageProcessors[i]->imgui_panel_name + " - diff").c_str()))
            {
                float diff_intensity = m_ImageProcessors[i]->addons.diff.diff_intensity.load();
                ImGui::SliderFloat(("##" + m_ImageProcessors[i]->imgui_panel_name + " diff slider").c_str(), &diff_intensity, 1.f, 10.f);
                m_ImageProcessors[i]->addons.diff.diff_intensity.store(diff_intensity);

                m_ImageProcessors[i]->addons.diff.image.imgui_image(m_ImageProcessors[i]->flip_vertically, m_ImageProcessors[i]->flip_horizontally);
            }
            ImGui::End();
        }
    }
}