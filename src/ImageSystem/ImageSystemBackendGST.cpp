#include <guiniverse/ImageSystem/ImageSystemBackendGST.hpp>

#include <string>

ImageSystemBackendGST::ImageSystemBackendGST(std::shared_ptr<ImageSystem> image_system) : ImageSystemBackend(image_system)
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

void ImageSystemBackendGST::onFrame()
{
    for (int i = 0; i < m_Processors.size(); i++)
    {
        GstSample* sample = gst_app_sink_try_pull_sample(m_Processors[i].sink, 0);

        if (sample) {
            GstBuffer* buffer = gst_sample_get_buffer(sample);

            if (buffer) {
                GstCaps* caps = gst_sample_get_caps(sample);

                GstClockTime timestamp = GST_BUFFER_PTS(buffer);

                if (caps && (timestamp != m_Processors[i].last_frame_timestamp)) {
                    GstStructure* structure = gst_caps_get_structure(caps, 0);

                    if (structure) {
                        int width, height;

                        if (gst_structure_get_int(structure, "width", &width) &&
                            gst_structure_get_int(structure, "height", &height)) {

                            GstMapInfo map;

                            if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {

                                m_ImageSystem->ImageCallback(m_Processors[i].index, GL_RGB, width, height, map.data);
                                m_Processors[i].last_frame_timestamp = timestamp;

                                gst_buffer_unmap(buffer, &map);
                            }
                        }
                    }
                }
            }

            gst_sample_unref(sample);
        }        
    }
}

void ImageSystemBackendGST::addSink(short port, int addons)
{
    int size = m_Processors.size();
    m_Processors.resize(size + 1);

    m_Processors[size].port = port;

    std::string sink_name = "sink" + std::to_string(size);
    std::string launch_string = "udpsrc port=" + std::to_string(port) + " ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=RGB ! appsink name=" + sink_name + " max-buffers=1 drop=true";

    printf(launch_string.c_str());

    m_Processors[size].pipeline = gst_parse_launch(
        launch_string.c_str(),
        NULL
    );

    if (!m_Processors.at(size).pipeline) {
        printf("Failed to create pipeline\n");
    }

    m_Processors[size].appsink = gst_bin_get_by_name(GST_BIN(m_Processors[size].pipeline), sink_name.c_str());
    m_Processors[size].sink = GST_APP_SINK(m_Processors[size].appsink);

    gst_element_set_state(m_Processors[size].pipeline, GST_STATE_PLAYING);

    m_Processors[size].index = m_ImageSystem->addImageProcessor(addons, "GStreamer sink on UDP port " + std::to_string(port));
}