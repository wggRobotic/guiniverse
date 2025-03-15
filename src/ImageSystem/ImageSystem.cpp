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
    m_ImageProcessors.reserve(3);
}

ImageSystem::~ImageSystem()
{
    for (int i = 0; i < m_ImageProcessors.size(); i++) 
    {
        m_ImageProcessors[i]->thread_should_close.store(true);
        if (m_ImageProcessors[i]->addon_thread.joinable())
            m_ImageProcessors[i]->addon_thread.join();

        if (m_ImageProcessors[i]->addons & ImageSystemAddOn_QRCode)
        {
            quirc_destroy(m_ImageProcessors[i]->qrcode_addon.quirc_instance);
        }
        
        if (m_ImageProcessors[i]->addons & ImageSystemAddOn_HazardSigns);
    }
}

int ImageSystem::addImageProcessor(int addons, const std::string& imgui_panel_name)
{
    int size = m_ImageProcessors.size();
    m_ImageProcessors.resize(size + 1);
    m_ImageProcessors[size] = std::make_shared<ImageSystemImageProcessor>();

    m_ImageProcessors[size]->imgui_panel_name = imgui_panel_name;
    m_ImageProcessors[size]->addons = addons;

    if (m_ImageProcessors[size]->addons & ImageSystemAddOn_QRCode) 
    {
        m_ImageProcessors[size]->qrcode_addon.quirc_instance = quirc_new();
        m_ImageProcessors[size]->qrcode_addon.publisher = m_Node->create_publisher<std_msgs::msg::String>("qrcode", 10);
    }

    if (m_ImageProcessors[size]->addons & ImageSystemAddOn_HazardSigns);

    m_ImageProcessors[size]->addon_thread = std::thread(&ImageSystem::AddOnThreadFunction, this, size);

    return size;
}

void ImageSystem::AddOnThreadFunction(int index)
{
    while (!m_ImageProcessors[index]->thread_should_close.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));


        //qrcode
        int width, height;
        {
            std::lock_guard<std::mutex> lock(m_ImageProcessors[index]->image_mutex);

            width = m_ImageProcessors[index]->image.width;
            height = m_ImageProcessors[index]->image.height;
            const unsigned char* data = m_ImageProcessors[index]->image.data.data();

            m_ImageProcessors[index]->qrcode_addon.gray_scale.reserve(width * height);

            switch (m_ImageProcessors[index]->image.image_layout)
            {
            case GL_RGB: for (int i = 0, offset = 0; i < width * height; i++, offset+=3) {
                m_ImageProcessors[index]->qrcode_addon.gray_scale[i] = (unsigned char)(
                    0.299f * (float)data[offset + 0] + 
                    0.587f * (float)data[offset + 1] + 
                    0.114f * (float)data[offset + 2]
                );
            } break;

            case GL_BGR: for (int i = 0, offset = 0; i < width * height; i++, offset+=3) {
                m_ImageProcessors[index]->qrcode_addon.gray_scale[i] = (unsigned char)(
                    0.299f * (float)data[offset + 2] + 
                    0.587f * (float)data[offset + 1] + 
                    0.114f * (float)data[offset + 0]
                );
            } break;

            case GL_R: for (int i = 0, offset = 0; i < width * height; i++, offset+=1) {
                m_ImageProcessors[index]->qrcode_addon.gray_scale[i] = (unsigned char)(
                    0.299f * (float)data[offset + 0]
                );
            } break;

            case GL_RGBA: for (int i = 0, offset = 0; i < width * height; i++, offset+=4) {
                m_ImageProcessors[index]->qrcode_addon.gray_scale[i] = (unsigned char)(
                    0.299f * (float)data[offset + 0] + 
                    0.587f * (float)data[offset + 1] + 
                    0.114f * (float)data[offset + 2]
                );
            } break;

            default: for (int i = 0, offset = 0; i < width * height; i++, offset+=1) {
                m_ImageProcessors[index]->qrcode_addon.gray_scale[i] = (unsigned char)(
                    0.299f * (float)data[offset + 0]
                );
            } break;

            }
        }

        quirc_resize(m_ImageProcessors[index]->qrcode_addon.quirc_instance, width, height);

        uint8_t *qr_image = quirc_begin(m_ImageProcessors[index]->qrcode_addon.quirc_instance, &width, &height);
        memcpy(qr_image,  m_ImageProcessors[index]->qrcode_addon.gray_scale.data(), width * height);
        quirc_end(m_ImageProcessors[index]->qrcode_addon.quirc_instance);

        int count = quirc_count(m_ImageProcessors[index]->qrcode_addon.quirc_instance);

        for (int i = 0; i < count; i++) {
            struct quirc_code code;
            struct quirc_data data;

            quirc_extract(m_ImageProcessors[index]->qrcode_addon.quirc_instance, i, &code);

            if (quirc_decode(&code, &data) == QUIRC_SUCCESS) {
                std_msgs::msg::String msg;
                msg.data = std::string((char*)data.payload);
                m_ImageProcessors[index]->qrcode_addon.publisher->publish(msg);
            }
        }

    }
}

void ImageSystem::ImageCallback(int index, int image_layout, int width, int height, unsigned char* data)
{
    std::lock_guard<std::mutex> lock(m_ImageProcessors[index]->image_mutex);

    m_ImageProcessors[index]->image.dirty = true;
    m_ImageProcessors[index]->image.holds = true;
    m_ImageProcessors[index]->image.width = width;
    m_ImageProcessors[index]->image.height = height;
    m_ImageProcessors[index]->image.image_layout = image_layout;
    m_ImageProcessors[index]->image.data.assign(data, data + ImageSystemImageLayoutPixelSize(image_layout) * width * height);
}

void ImageSystem::onGuiStartup()
{
    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        std::lock_guard<std::mutex> lock(m_ImageProcessors[i]->image_mutex);

        GLCALL(glGenTextures(1, &m_ImageProcessors[i]->texture.gl_texture));
        GLCALL(glBindTexture(GL_TEXTURE_2D, m_ImageProcessors[i]->texture.gl_texture));

        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

        GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
    }
}

void ImageSystem::onGuiShutdown()
{

    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        std::lock_guard<std::mutex> lock(m_ImageProcessors[i]->image_mutex);

        GLCALL(glDeleteTextures(1, &m_ImageProcessors[i]->texture.gl_texture));
    }
}

void ImageSystem::onGuiFrame()
{
    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        std::lock_guard<std::mutex> lock(m_ImageProcessors[i]->image_mutex);

        if (m_ImageProcessors[i]->image.holds)
        {
            GLCALL(glBindTexture(GL_TEXTURE_2D, m_ImageProcessors[i]->texture.gl_texture));

            if (
                m_ImageProcessors[i]->texture.width != m_ImageProcessors[i]->image.width || 
                m_ImageProcessors[i]->texture.height != m_ImageProcessors[i]->image.height
            )
            {
                m_ImageProcessors[i]->texture.width = m_ImageProcessors[i]->image.width;
                m_ImageProcessors[i]->texture.height = m_ImageProcessors[i]->image.height;

                GLCALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, m_ImageProcessors[i]->texture.width, m_ImageProcessors[i]->texture.height, 0, m_ImageProcessors[i]->image.image_layout, GL_UNSIGNED_BYTE, m_ImageProcessors[i]->image.data.data()));                
            }
            else
            {
                GLCALL(glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_ImageProcessors[i]->texture.width, m_ImageProcessors[i]->texture.height, m_ImageProcessors[i]->image.image_layout, GL_UNSIGNED_BYTE, m_ImageProcessors[i]->image.data.data()));
            }
        
            GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
        }

        if (ImGui::Begin(m_ImageProcessors[i]->imgui_panel_name.c_str()))
        {
            if (m_ImageProcessors[i]->image.holds)
            {

                ImGui::Checkbox("Flip vertically", &m_ImageProcessors[i]->flip_vertically);
                ImGui::Checkbox("Flip horizontally", &m_ImageProcessors[i]->flip_horizontally);

                ImVec2 widget_size = ImGui::GetContentRegionAvail();

                float image_aspect = (float)m_ImageProcessors[i]->texture.width / (float)m_ImageProcessors[i]->texture.height;
                float widget_aspect = widget_size.x / widget_size.y;

                ImVec2 display_size;

                if (widget_aspect > image_aspect)
                    display_size = ImVec2(widget_size.y * image_aspect, widget_size.y);
                else 
                    display_size = ImVec2(widget_size.x, widget_size.x / image_aspect);
                
                ImVec2 uv0 = ImVec2(m_ImageProcessors[i]->flip_horizontally ? 1.f : 0.f, m_ImageProcessors[i]->flip_vertically ? 1.f : 0.f);
                ImVec2 uv1 = ImVec2(m_ImageProcessors[i]->flip_horizontally ? 0.f : 1.f, m_ImageProcessors[i]->flip_vertically ? 0.f : 1.f);
                ImGui::Image((ImTextureID)m_ImageProcessors[i]->texture.gl_texture, display_size, uv0, uv1);
            }
            else 
                ImGui::Text("No image yet");
        }
        ImGui::End();
    }
}