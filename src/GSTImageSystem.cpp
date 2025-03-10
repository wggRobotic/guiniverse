#include <GL/glew.h>

#include <guiniverse/GSTImageSystem.hpp>

#include <imgui.h>

#define GLCALL(call) \
    do { \
        call; \
        GLenum err = glGetError(); \
        if (err != GL_NO_ERROR) { \
            fprintf(stderr, "OpenGL error in %s at %s:%d: %d\n", #call, __FILE__, __LINE__, err); \
        } \
    } while (0)

GSTImageSystem::GSTImageSystem()
{
    m_ImageProcessors.reserve(2);
}

void GSTImageSystem::addSink(short port)
{
    std::lock_guard<std::mutex> lock(m_ImageProcessorMutex);

    int current_size = m_ImageProcessors.size();
    m_ImageProcessors.resize(current_size + 1);

    m_ImageProcessors.at(current_size).port = port;

    std::string sink_name = "sink" + std::to_string(current_size);
    std::string launch_string = "udpsrc port=" + std::to_string(port) + " ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=RGB ! appsink name=" + sink_name + " max-buffers=1 drop=true";

    printf(launch_string.c_str());

    m_ImageProcessors.at(current_size).pipeline = gst_parse_launch(
        launch_string.c_str(),
        NULL
    );

    if (!m_ImageProcessors.at(current_size).pipeline) {
        printf("Failed to create pipeline\n");
    }

    m_ImageProcessors.at(current_size).appsink = gst_bin_get_by_name(GST_BIN(m_ImageProcessors.at(current_size).pipeline), sink_name.c_str());
    m_ImageProcessors.at(current_size).sink = GST_APP_SINK(m_ImageProcessors.at(current_size).appsink);
    gst_element_set_state(m_ImageProcessors.at(current_size).pipeline, GST_STATE_PLAYING);
}

void GSTImageSystem::ImGuiPanels()
{
    std::lock_guard<std::mutex> lock(m_ImageProcessorMutex);

    for (int i = 0; i < m_ImageProcessors.size(); i++) 
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

        GstSample *sample = gst_app_sink_try_pull_sample(m_ImageProcessors.at(i).sink, 0);  // 1-second timeout

        if (sample) {

            GstBuffer *buffer = gst_sample_get_buffer(sample);

            if (buffer) {
                GstCaps *caps = gst_sample_get_caps(sample);

                GstClockTime timestamp = GST_BUFFER_PTS(buffer);

                if (caps && (!m_ImageProcessors.at(i).holds_image || timestamp != m_ImageProcessors.at(i).last_frame_timestamp)) {
                    GstStructure *structure = gst_caps_get_structure(caps, 0);

                    if (structure) {
                        gint width, height;

                        if (gst_structure_get_int(structure, "width", &width) &&
                            gst_structure_get_int(structure, "height", &height)) {

                            GstMapInfo map;

                            if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {

                                GLCALL(glBindTexture(GL_TEXTURE_2D, m_ImageProcessors.at(i).texture));

                                if (m_ImageProcessors.at(i).texture_width != width || m_ImageProcessors.at(i).texture_height != height)
                                {
                                    m_ImageProcessors.at(i).texture_width = width;
                                    m_ImageProcessors.at(i).texture_height = height;

                                    GLCALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, map.data));                
                                }
                                else
                                {
                                    GLCALL(glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, map.data));
                                }
                            
                                GLCALL(glBindTexture(GL_TEXTURE_2D, 0));

                                m_ImageProcessors.at(i).holds_image = true;
                                m_ImageProcessors.at(i).last_frame_timestamp = timestamp;

                                gst_buffer_unmap(buffer, &map);
                            }
                        }
                    }
                }
            }

            gst_sample_unref(sample);
        }        

        if (ImGui::Begin(("UDP port " + std::to_string(m_ImageProcessors.at(i).port)).c_str()))
        {

            if (m_ImageProcessors.at(i).holds_image)
            {

                ImGui::Checkbox("Flip vertically", &m_ImageProcessors.at(i).flip_vertically);
                ImGui::Checkbox("Flip horizontally", &m_ImageProcessors.at(i).flip_horizontally);

                ImVec2 widget_size = ImGui::GetContentRegionAvail();

                float image_aspect = (float)m_ImageProcessors.at(i).texture_width / (float)m_ImageProcessors.at(i).texture_height;
                float widget_aspect = widget_size.x / widget_size.y;

                ImVec2 display_size;

                if (widget_aspect > image_aspect)
                    display_size = ImVec2(widget_size.y * image_aspect, widget_size.y);
                else 
                    display_size = ImVec2(widget_size.x, widget_size.x / image_aspect);
                
                ImVec2 uv0 = ImVec2(m_ImageProcessors.at(i).flip_horizontally ? 1.f : 0.f, m_ImageProcessors.at(i).flip_vertically ? 1.f : 0.f);
                ImVec2 uv1 = ImVec2(m_ImageProcessors.at(i).flip_horizontally ? 0.f : 1.f, m_ImageProcessors.at(i).flip_vertically ? 0.f : 1.f);
                ImGui::Image((ImTextureID)m_ImageProcessors.at(i).texture, display_size, uv0, uv1);
            }
            else 
                ImGui::Text("No image yet");
        }
        ImGui::End();
    }

}

void GSTImageSystem::onGuiShutdown()
{
    std::lock_guard<std::mutex> lock(m_ImageProcessorMutex);

    for (int i = 0; i < m_ImageProcessors.size(); i++) if (m_ImageProcessors.at(i).init)
        GLCALL(glDeleteTextures(1, &m_ImageProcessors.at(i).texture));
}