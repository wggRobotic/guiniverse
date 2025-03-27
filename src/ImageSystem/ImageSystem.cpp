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

    m_ImageProcessors[size]->imgui.panel_name = imgui_panel_name;
    
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

        //qrcode
        if (m_ImageProcessors[index]->addons.flags & ImageSystemAddOn_QRCode) 
        {
            int width, height;
            {
                std::lock_guard<std::mutex> lock(m_ImageProcessors[index]->image.mutex);

                width = m_ImageProcessors[index]->image.width;
                height = m_ImageProcessors[index]->image.height;
                const unsigned char* data = m_ImageProcessors[index]->image.data.data();

                m_ImageProcessors[index]->addons.qrcode.gray_scale.reserve(width * height);

                if (m_ImageProcessors[index]->image.is_bgr) 
                {
                    for (int i = 0, offset = 0; i < width * height; i++, offset+=3) {
                        m_ImageProcessors[index]->addons.qrcode.gray_scale[i] = (unsigned char)(
                            0.299f * (float)data[offset + 2] + 
                            0.587f * (float)data[offset + 1] + 
                            0.114f * (float)data[offset + 0]
                        );
                    }
                }
                else
                {
                    for (int i = 0, offset = 0; i < width * height; i++, offset+=3) {
                        m_ImageProcessors[index]->addons.qrcode.gray_scale[i] = (unsigned char)(
                            0.299f * (float)data[offset + 0] + 
                            0.587f * (float)data[offset + 1] + 
                            0.114f * (float)data[offset + 2]
                        );
                    }
                }
            }

            quirc_resize(m_ImageProcessors[index]->addons.qrcode.quirc_instance, width, height);

            uint8_t *qr_image = quirc_begin(m_ImageProcessors[index]->addons.qrcode.quirc_instance, &width, &height);
            memcpy(qr_image,  m_ImageProcessors[index]->addons.qrcode.gray_scale.data(), width * height);
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

void ImageSystem::ImageCallback(int index, bool is_bgr, int width, int height, unsigned char* data)
{
    std::lock_guard<std::mutex> lock(m_ImageProcessors[index]->image.mutex);
    std::lock_guard<std::mutex> lock_diff(m_ImageProcessors[index]->addons.diff.image.mutex);

    bool change_of_dimensions = false;

    if (m_ImageProcessors[index]->image.width != width || m_ImageProcessors[index]->image.height != height) change_of_dimensions = true;


    if ((m_ImageProcessors[index]->addons.flags & ImageSystemAddOn_Diff) && change_of_dimensions == false && m_ImageProcessors[index]->image.holds == true)
    {

        float diff_intensity = m_ImageProcessors[index]->addons.diff.imgui.diff_intensity.load();

        m_ImageProcessors[index]->addons.diff.image.data.resize(3 * width * height);
        unsigned char* old_data = m_ImageProcessors[index]->image.data.data();

        unsigned char* diff_data =  m_ImageProcessors[index]->addons.diff.image.data.data();

        for (int i = 0; i < width * height * 3; i+=3)
        {
            int pixel_diff = data[i] + data[i+1] + data[i+2] - (old_data[i] + old_data[i+1] + old_data[i+2]);

            int intensity = std::min((int)(float)(std::abs(pixel_diff) / 3.f * diff_intensity), 255);


            diff_data[i] = 0;
            diff_data[i+1] = 0;
            diff_data[i+2] = 0;

            if (pixel_diff > 0)
                diff_data[i + 2] = intensity;
            else if (pixel_diff < 0)
                diff_data[i] = intensity;

        }
        m_ImageProcessors[index]->addons.diff.image.dirty = true;
        m_ImageProcessors[index]->addons.diff.image.holds = true;
        m_ImageProcessors[index]->addons.diff.image.width = width;
        m_ImageProcessors[index]->addons.diff.image.height = height;
    }

    

    m_ImageProcessors[index]->image.dirty = true;
    m_ImageProcessors[index]->image.holds = true;
    m_ImageProcessors[index]->image.width = width;
    m_ImageProcessors[index]->image.height = height;
    m_ImageProcessors[index]->image.is_bgr = is_bgr;
    m_ImageProcessors[index]->image.data.assign(data, data + 3 * width * height);

    
}

void ImageSystem::onGuiStartup()
{
    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        std::lock_guard<std::mutex> lock(m_ImageProcessors[i]->image.mutex);

        GLCALL(glGenTextures(1, &m_ImageProcessors[i]->imgui.gl_texture));
        GLCALL(glBindTexture(GL_TEXTURE_2D, m_ImageProcessors[i]->imgui.gl_texture));

        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

        GLCALL(glBindTexture(GL_TEXTURE_2D, 0));

        if (m_ImageProcessors[i]->addons.flags & ImageSystemAddOn_Diff)
        {
            GLCALL(glGenTextures(1, &m_ImageProcessors[i]->addons.diff.imgui.gl_texture));

            GLCALL(glBindTexture(GL_TEXTURE_2D, m_ImageProcessors[i]->addons.diff.imgui.gl_texture));

            GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
            GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
            GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
            GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

            GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
        }
    }
}

void ImageSystem::onGuiShutdown()
{

    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        std::lock_guard<std::mutex> lock(m_ImageProcessors[i]->image.mutex);

        GLCALL(glDeleteTextures(1, &m_ImageProcessors[i]->imgui.gl_texture));
        m_ImageProcessors[i]->imgui.height = 0;
        m_ImageProcessors[i]->imgui.width = 0;

        if (m_ImageProcessors[i]->addons.flags & ImageSystemAddOn_Diff)
        {
            GLCALL(glDeleteTextures(1, &m_ImageProcessors[i]->addons.diff.imgui.gl_texture));
            m_ImageProcessors[i]->addons.diff.imgui.height = 0;
            m_ImageProcessors[i]->addons.diff.imgui.width = 0;
        }
    }
}

void ImageSystem::onGuiFrame()
{
    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {

        bool image_holds;
        bool diff_image_holds;

        {
            std::lock_guard<std::mutex> lock(m_ImageProcessors[i]->image.mutex);
            image_holds = m_ImageProcessors[i]->image.holds;

            if (m_ImageProcessors[i]->image.holds && m_ImageProcessors[i]->image.dirty)
            {

                GLCALL(glBindTexture(GL_TEXTURE_2D, m_ImageProcessors[i]->imgui.gl_texture));
                if (
                    m_ImageProcessors[i]->imgui.width != m_ImageProcessors[i]->image.width || 
                    m_ImageProcessors[i]->imgui.height != m_ImageProcessors[i]->image.height
                )
                {
                    m_ImageProcessors[i]->imgui.width = m_ImageProcessors[i]->image.width;
                    m_ImageProcessors[i]->imgui.height = m_ImageProcessors[i]->image.height;
                    GLCALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, m_ImageProcessors[i]->imgui.width, m_ImageProcessors[i]->imgui.height, 0, m_ImageProcessors[i]->image.is_bgr ? GL_BGR : GL_RGB, GL_UNSIGNED_BYTE, m_ImageProcessors[i]->image.data.data()));                
                }
                else
                {
                    GLCALL(glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_ImageProcessors[i]->imgui.width, m_ImageProcessors[i]->imgui.height, m_ImageProcessors[i]->image.is_bgr ? GL_BGR : GL_RGB, GL_UNSIGNED_BYTE, m_ImageProcessors[i]->image.data.data()));
                }
                GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
                
            }
        }

        {
            std::lock_guard<std::mutex> lock(m_ImageProcessors[i]->addons.diff.image.mutex);
            diff_image_holds = m_ImageProcessors[i]->addons.diff.image.holds;

            if (m_ImageProcessors[i]->addons.diff.image.holds && m_ImageProcessors[i]->addons.diff.image.dirty)
            {

                GLCALL(glBindTexture(GL_TEXTURE_2D, m_ImageProcessors[i]->addons.diff.imgui.gl_texture));
                if (
                    m_ImageProcessors[i]->addons.diff.imgui.width != m_ImageProcessors[i]->addons.diff.image.width || 
                    m_ImageProcessors[i]->addons.diff.imgui.height != m_ImageProcessors[i]->addons.diff.image.height
                )
                {
                    m_ImageProcessors[i]->addons.diff.imgui.width = m_ImageProcessors[i]->addons.diff.image.width;
                    m_ImageProcessors[i]->addons.diff.imgui.height = m_ImageProcessors[i]->addons.diff.image.height;
                    GLCALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, m_ImageProcessors[i]->addons.diff.imgui.width, m_ImageProcessors[i]->addons.diff.imgui.height, 0, GL_RGB, GL_UNSIGNED_BYTE, m_ImageProcessors[i]->addons.diff.image.data.data()));                
                }
                else
                {
                    GLCALL(glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_ImageProcessors[i]->addons.diff.imgui.width, m_ImageProcessors[i]->addons.diff.imgui.height, GL_RGB, GL_UNSIGNED_BYTE, m_ImageProcessors[i]->addons.diff.image.data.data()));
                }
                GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
                
            }
        }

        if (ImGui::Begin(m_ImageProcessors[i]->imgui.panel_name.c_str()))
        {
            if (image_holds)
            {

                ImGui::Checkbox("Flip vertically", &m_ImageProcessors[i]->imgui.flip_vertically);
                ImGui::Checkbox("Flip horizontally", &m_ImageProcessors[i]->imgui.flip_horizontally);

                ImVec2 widget_size = ImGui::GetContentRegionAvail();

                float image_aspect = (float)m_ImageProcessors[i]->imgui.width / (float)m_ImageProcessors[i]->imgui.height;
                float widget_aspect = widget_size.x / widget_size.y;

                ImVec2 display_size;

                if (widget_aspect > image_aspect)
                    display_size = ImVec2(widget_size.y * image_aspect, widget_size.y);
                else 
                    display_size = ImVec2(widget_size.x, widget_size.x / image_aspect);
                
                ImVec2 uv0 = ImVec2(m_ImageProcessors[i]->imgui.flip_horizontally ? 1.f : 0.f, m_ImageProcessors[i]->imgui.flip_vertically ? 1.f : 0.f);
                ImVec2 uv1 = ImVec2(m_ImageProcessors[i]->imgui.flip_horizontally ? 0.f : 1.f, m_ImageProcessors[i]->imgui.flip_vertically ? 0.f : 1.f);
                ImGui::Image((ImTextureID)m_ImageProcessors[i]->imgui.gl_texture, display_size, uv0, uv1);
            }
            else 
                ImGui::Text("No image yet");
        }
        ImGui::End();

        

        if (m_ImageProcessors[i]->addons.flags & ImageSystemAddOn_Diff)
        {
            if (ImGui::Begin((m_ImageProcessors[i]->imgui.panel_name + " - diff").c_str()))
            {
                if (diff_image_holds)
                {
                    float diff_intensity = m_ImageProcessors[i]->addons.diff.imgui.diff_intensity.load();
                    ImGui::SliderFloat(("##" + m_ImageProcessors[i]->imgui.panel_name + " diff slider").c_str(), &diff_intensity, 1.f, 10.f);
                    m_ImageProcessors[i]->addons.diff.imgui.diff_intensity.store(diff_intensity);

                    ImVec2 widget_size = ImGui::GetContentRegionAvail();

                    float image_aspect = (float)m_ImageProcessors[i]->addons.diff.imgui.width / (float)m_ImageProcessors[i]->addons.diff.imgui.height;
                    float widget_aspect = widget_size.x / widget_size.y;

                    ImVec2 display_size;

                    if (widget_aspect > image_aspect)
                        display_size = ImVec2(widget_size.y * image_aspect, widget_size.y);
                    else 
                        display_size = ImVec2(widget_size.x, widget_size.x / image_aspect);
                    
                    ImVec2 uv0 = ImVec2(m_ImageProcessors[i]->imgui.flip_horizontally ? 1.f : 0.f, m_ImageProcessors[i]->imgui.flip_vertically ? 1.f : 0.f);
                    ImVec2 uv1 = ImVec2(m_ImageProcessors[i]->imgui.flip_horizontally ? 0.f : 1.f, m_ImageProcessors[i]->imgui.flip_vertically ? 0.f : 1.f);
                    ImGui::Image((ImTextureID)m_ImageProcessors[i]->addons.diff.imgui.gl_texture, display_size, uv0, uv1);
                }
                else 
                    ImGui::Text("No image yet");
            }
            ImGui::End();
        }
    }
}