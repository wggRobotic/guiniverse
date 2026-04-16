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

void ImageSystemImage::imgui_image(bool flip_vertically, bool flip_horizontally, bool rotate, unsigned int default_texture)
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
                GLCALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, texture_width, texture_height, 0, GL_BGR, GL_UNSIGNED_BYTE, image.data));                
            }
            else
            {
                GLCALL(glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture_width, texture_height, GL_BGR, GL_UNSIGNED_BYTE, image.data));
            }
            GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
        }   
    }

    
    {
        ImVec2 widget_size = ImGui::GetContentRegionAvail();

        float image_aspect = (empty ? 1.f :(float)texture_width/ (float)texture_height * par);
        if (rotate) image_aspect = 1/image_aspect;
        float widget_aspect = widget_size.x / widget_size.y;

        ImVec2 display_size;

        if (widget_aspect > image_aspect)
            display_size = ImVec2(widget_size.y * image_aspect, widget_size.y);
        else 
            display_size = ImVec2(widget_size.x, widget_size.x / image_aspect);
    
        ImVec2 uv[4] =
        {
            ImVec2(0.f, rotate ? 1.f : 0.f), 
            ImVec2(rotate ? 0.f : 1.f, 0.f), 
            ImVec2(1.f, rotate ? 0.f : 1.f),
            ImVec2(rotate ? 1.f : 0.f, 1.f), 
        };

        if (flip_horizontally)
        {
            ImVec2 temp = uv[0];
            uv[0] = uv[1];
            uv[1] = temp;

            temp = uv[2];
            uv[2] = uv[3];
            uv[3] = temp;
        }

        if (flip_vertically)
        {
            ImVec2 temp = uv[0];
            uv[0] = uv[3];
            uv[3] = temp;

            temp = uv[1];
            uv[1] = uv[2];
            uv[2] = temp;
        }

        ImVec2 cursor = ImGui::GetCursorScreenPos();

        ImGui::GetWindowDrawList()->AddImageQuad(
            (ImTextureID)(empty ? default_texture : gl_texture),
            cursor,
            ImVec2(cursor.x + display_size.x, cursor.y),
            ImVec2(cursor.x + display_size.x, cursor.y + display_size.y),
            ImVec2(cursor.x, cursor.y + display_size.y),
            uv[0], uv[1], uv[2], uv[3],
            IM_COL32_WHITE
        );
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

        quirc_destroy(m_ImageProcessors[i]->addons.qrcode.quirc_instance);
    }
}

int ImageSystem::addImageProcessor(const std::string& imgui_panel_name)
{
    int size = m_ImageProcessors.size();
    m_ImageProcessors.resize(size + 1);
    m_ImageProcessors[size] = std::make_shared<ImageSystemImageProcessor>();

    m_ImageProcessors[size]->imgui_panel_name = imgui_panel_name;

    m_ImageProcessors[size]->addons.qrcode.quirc_instance = quirc_new();
    m_ImageProcessors[size]->addons.qrcode.publisher = m_Node->create_publisher<std_msgs::msg::String>("qrcode", 10);

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
        if (m_ImageProcessors[index]->addons.flags.load() & ImageSystemAddOn_QRCode && !image.empty()) 
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

void ImageSystem::ImageCallback(int index, cv::Mat& image, float par)
{

    int addon_flags = m_ImageProcessors[index]->addons.flags.load();

    cv::Mat old_image;
    if (addon_flags & ImageSystemAddOn_Diff) m_ImageProcessors[index]->image.copy_image(old_image);

    m_ImageProcessors[index]->image.sub_image_transfer_ownership(image);
    m_ImageProcessors[index]->image.par = par;

    if (old_image.cols == image.cols && old_image.rows == image.rows && addon_flags & ImageSystemAddOn_Diff)
    {
        cv::Mat diff_image;
        cv::absdiff(image, old_image, diff_image);
        
        m_ImageProcessors[index]->addons.diff.image.sub_image_transfer_ownership(diff_image);
        m_ImageProcessors[index]->addons.diff.image.par = par;
    }

    if (addon_flags & ImageSystemAddon_GrayScale)
    {
        cv::Mat grayscale;
        cv::Mat grayscale_rgb;

        cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);
        cv::cvtColor(grayscale, grayscale_rgb, cv::COLOR_GRAY2BGR);

        m_ImageProcessors[index]->addons.grayscale.image.sub_image_transfer_ownership(grayscale_rgb);
        m_ImageProcessors[index]->addons.grayscale.image.par = par;
    }
}

void ImageSystem::onGuiStartup()
{
    GLCALL(glGenTextures(1, &default_texture));
    GLCALL(glBindTexture(GL_TEXTURE_2D, default_texture));

    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

    unsigned int data[] = {0xff0000ff, 0xff00ff00, 0xffff0000, 0xff00ffff};
    GLCALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, data));                

    GLCALL(glBindTexture(GL_TEXTURE_2D, 0));

    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        m_ImageProcessors[i]->image.create_texture();
        m_ImageProcessors[i]->addons.diff.image.create_texture();
        m_ImageProcessors[i]->addons.grayscale.image.create_texture();
    }
}

void ImageSystem::onGuiShutdown()
{

    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        m_ImageProcessors[i]->image.destroy_texture();
        m_ImageProcessors[i]->addons.diff.image.destroy_texture();
        m_ImageProcessors[i]->addons.grayscale.image.destroy_texture();
    }

    GLCALL(glDeleteTextures(1, &default_texture));
}

void ImageSystem::onGuiFrame()
{
    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        int addon_flags = m_ImageProcessors[i]->addons.flags.load();
        bool qrcode_addon = (addon_flags & ImageSystemAddOn_QRCode ? true : false);
        bool diff_addon = (addon_flags & ImageSystemAddOn_Diff ? true : false);
        bool grayscale_addon = (addon_flags & ImageSystemAddon_GrayScale ? true : false);

        if (ImGui::Begin(m_ImageProcessors[i]->imgui_panel_name.c_str()))
        {
            ImGui::Checkbox("Flip vertically", &m_ImageProcessors[i]->flip_vertically);
            ImGui::SameLine(0.0f, 10.0f);
            ImGui::Checkbox("Flip horizontally", &m_ImageProcessors[i]->flip_horizontally);
            ImGui::SameLine(0.0f, 10.0f);
            ImGui::Checkbox("rotate", &m_ImageProcessors[i]->rotate);
            ImGui::SameLine(0.0f, 10.0f);
            ImGui::Checkbox("QRCode scanning", &qrcode_addon);
            ImGui::SameLine(0.0f, 10.0f);
            ImGui::Checkbox("Image diff", &diff_addon);
            ImGui::SameLine(0.0f, 10.0f);
            ImGui::Checkbox("Image grayscale", &grayscale_addon);

            m_ImageProcessors[i]->image.imgui_image(m_ImageProcessors[i]->flip_vertically, m_ImageProcessors[i]->flip_horizontally, m_ImageProcessors[i]->rotate, default_texture);
        }
        ImGui::End();

        if (addon_flags & ImageSystemAddOn_Diff)
        {
            if (ImGui::Begin((m_ImageProcessors[i]->imgui_panel_name + " - diff").c_str()))
            {
                m_ImageProcessors[i]->addons.diff.image.imgui_image(m_ImageProcessors[i]->flip_vertically, m_ImageProcessors[i]->flip_horizontally, m_ImageProcessors[i]->rotate, default_texture);
            }
            ImGui::End();
        }

        if (addon_flags & ImageSystemAddon_GrayScale)
        {
            if (ImGui::Begin((m_ImageProcessors[i]->imgui_panel_name + " - grayscale").c_str()))
            {
                m_ImageProcessors[i]->addons.grayscale.image.imgui_image(m_ImageProcessors[i]->flip_vertically, m_ImageProcessors[i]->flip_horizontally, m_ImageProcessors[i]->rotate, default_texture);
            }
            ImGui::End();
        }

        m_ImageProcessors[i]->addons.flags.store((qrcode_addon ? ImageSystemAddOn_QRCode : 0) | (diff_addon ? ImageSystemAddOn_Diff : 0) | (grayscale_addon ? ImageSystemAddon_GrayScale : 0));
    }
}