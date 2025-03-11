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

void ImageSystem::addImageSystemBackend(ImageSystemBackend* backend)
{
    int backends_size = m_Backends.size();
    m_Backends.resize(backends_size + 1);
    m_Backends[backends_size] = backend;

    int processors_size = m_ImageProcessors.size();
    int backend_processor_count = backend->getProcessorCount();
    m_ImageProcessors.resize(processors_size + backend_processor_count);

    for (int i = 0; i < backend_processor_count; i++)
    {
        m_ImageProcessors[i + processors_size].backend_index = backends_size;
        m_ImageProcessors[i + processors_size].backend_processor_index = i;
        m_ImageProcessors[i + processors_size].mutex = std::make_unique<std::mutex>();
    }

}

void ImageSystem::onFrame()
{

    for (int i = 0; i < m_Backends.size(); i++)
        m_Backends[i]->onFrame();

    for (int i = 0; i < m_ImageProcessors.size(); i++)
    {
        std::lock_guard<std::mutex> lock(*m_ImageProcessors[i].mutex);

        int image_layout = GL_RGB;
        int width = 0;
        int height = 0;
        unsigned char* data;

        if (m_Backends[m_ImageProcessors[i].backend_index]->onFramegetImage(m_ImageProcessors[i].backend_processor_index, &image_layout, &width, &height, &data))
        {
            m_ImageProcessors[i].image.dirty = true;
            m_ImageProcessors[i].image.holds = true;
            m_ImageProcessors[i].image.width = width;
            m_ImageProcessors[i].image.height = height;
            m_ImageProcessors[i].image.image_layout = image_layout;
            m_ImageProcessors[i].image.data.assign(data, data + ImageSystemImageLayoutPixelSize(image_layout) * width * height);

            m_Backends[m_ImageProcessors[i].backend_index]->onFramegotImage(m_ImageProcessors[i].backend_processor_index);
        }        
    }
}

void ImageSystem::onGuiStartup()
{
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

        if (ImGui::Begin(m_Backends[m_ImageProcessors[i].backend_index]->ImGuiPanelName(m_ImageProcessors[i].backend_processor_index).c_str()))
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