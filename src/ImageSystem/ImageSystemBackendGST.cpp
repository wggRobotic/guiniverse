#include <guiniverse/ImageSystem/ImageSystemBackendGST.hpp>

#include <string>

ImageSystemBackendGST::ImageSystemBackendGST()
{
}

ImageSystemBackendGST::~ImageSystemBackendGST()
{
    for (int i = 0; i < m_Processors.size(); i++)
    {
        gst_element_set_state(m_Processors[i].pipeline, GST_STATE_NULL);
        gst_object_unref(m_Processors[i].pipeline);
    }
}

int ImageSystemBackendGST::getProcessorCount()
{
    return m_Processors.size();
}

std::string ImageSystemBackendGST::ImGuiPanelName(int index)
{
    return ("Gstreamer sink on UDP port " + std::to_string(m_Processors[index].port));
}

void ImageSystemBackendGST::onFrame()
{
}

bool ImageSystemBackendGST::onFramegetImage(int index, int* image_layout, int* width, int* height, unsigned char** data)
{
    m_Processors[index].sample = gst_app_sink_try_pull_sample(m_Processors[index].sink, 0);

    if (m_Processors[index].sample) {
        m_Processors[index].buffer = gst_sample_get_buffer(m_Processors[index].sample);

        if (m_Processors[index].buffer) {
            GstCaps *caps = gst_sample_get_caps(m_Processors[index].sample);

            GstClockTime timestamp = GST_BUFFER_PTS(m_Processors[index].buffer);

            if (caps && (timestamp != m_Processors[index].last_frame_timestamp)) {
                GstStructure *structure = gst_caps_get_structure(caps, 0);

                if (structure) {

                    if (gst_structure_get_int(structure, "width", width) &&
                        gst_structure_get_int(structure, "height", height)) {

                        if (gst_buffer_map(m_Processors[index].buffer, &m_Processors[index].map, GST_MAP_READ)) {

                            m_Processors.at(index).last_frame_timestamp = timestamp;

                            *image_layout = GL_RGB;
                            *data = m_Processors[index].map.data;
                            return true;
                        }
                    }
                }
            }
        }

        gst_sample_unref(m_Processors[index].sample);
    }        

    return false;
}

void ImageSystemBackendGST::onFramegotImage(int index)
{
    gst_buffer_unmap(m_Processors[index].buffer, &m_Processors[index].map);
    gst_sample_unref(m_Processors[index].sample);
}

void ImageSystemBackendGST::addSink(short port)
{
    int size = m_Processors.size();
    m_Processors.resize(size + 1);

    m_Processors.at(size).port = port;

    std::string sink_name = "sink" + std::to_string(size);
    std::string launch_string = "udpsrc port=" + std::to_string(port) + " ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=RGB ! appsink name=" + sink_name + " max-buffers=1 drop=true";

    printf(launch_string.c_str());

    m_Processors.at(size).pipeline = gst_parse_launch(
        launch_string.c_str(),
        NULL
    );

    if (!m_Processors.at(size).pipeline) {
        printf("Failed to create pipeline\n");
    }

    m_Processors.at(size).appsink = gst_bin_get_by_name(GST_BIN(m_Processors.at(size).pipeline), sink_name.c_str());
    m_Processors.at(size).sink = GST_APP_SINK(m_Processors.at(size).appsink);

    gst_element_set_state(m_Processors.at(size).pipeline, GST_STATE_PLAYING);
}