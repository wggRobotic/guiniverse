#include <guiniverse/ImageSystem/ImageSystemBackendGST.hpp>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <string>

ImageSystemBackendGST::ImageSystemBackendGST(std::shared_ptr<ImageSystem> image_system)
    : ImageSystemBackend(image_system)
{
    gst_init(nullptr, nullptr); // Initialize GStreamer
}

ImageSystemBackendGST::~ImageSystemBackendGST()
{
    for (auto &proc : m_Processors) {
        gst_element_set_state(proc.pipeline, GST_STATE_NULL);
        gst_object_unref(proc.pipeline);
    }
}

void ImageSystemBackendGST::onFrame()
{
    for (auto &proc : m_Processors) {
        // Pull a new sample (blocking until a frame is ready)
        GstSample* sample = gst_app_sink_try_pull_sample(proc.sink, 10000000); // 10ms timeout

        if (!sample) continue;

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstCaps* caps = gst_sample_get_caps(sample);
        GstClockTime timestamp = GST_BUFFER_PTS(buffer);

        if (caps && (timestamp != proc.last_frame_timestamp)) {
            GstStructure* structure = gst_caps_get_structure(caps, 0);
            int width = 0, height = 0, par_num = 1, par_den = 1;

            if (structure &&
                gst_structure_get_int(structure, "width", &width) &&
                gst_structure_get_int(structure, "height", &height) &&
                gst_structure_get_fraction(structure, "pixel-aspect-ratio", &par_num, &par_den)
            ) {

                GstMapInfo map;
                if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                    // Wrap raw RGB data into OpenCV Mat
                    cv::Mat image_mat(height, width, CV_8UC3, (void*)map.data);
                    m_ImageSystem->ImageCallback(proc.index, image_mat, (float)par_num/(float)par_den);
                    proc.last_frame_timestamp = timestamp;
                    gst_buffer_unmap(buffer, &map);
                }
            }
        }

        gst_sample_unref(sample);
    }
}

void ImageSystemBackendGST::addSink(short port)
{
    int index = static_cast<int>(m_Processors.size());
    m_Processors.resize(index + 1);

    auto &proc = m_Processors[index];
    proc.port = port;

    std::string sink_name = "sink" + std::to_string(index);

    // Construct the pipeline
    std::string launch_string =
        "udpsrc port=" + std::to_string(port) + " "
        "caps=\"application/x-rtp, media=video, encoding-name=H264, payload=96\" ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "video/x-raw,format=BGR ! queue max-size-buffers=1 leaky=downstream ! "
        "appsink name=" + sink_name;

    printf("Launching: %s\n", launch_string.c_str());

    proc.pipeline = gst_parse_launch(launch_string.c_str(), nullptr);
    if (!proc.pipeline) {
        printf("Failed to create pipeline\n");
        return;
    }

    proc.appsink = gst_bin_get_by_name(GST_BIN(proc.pipeline), sink_name.c_str());
    proc.sink = GST_APP_SINK(proc.appsink);

    // Configure appsink for low-latency, dropping old frames
    g_object_set(proc.sink,
                 "emit-signals", TRUE,
                 "max-buffers", 1,
                 "drop", TRUE,
                 nullptr);

    gst_element_set_state(proc.pipeline, GST_STATE_PLAYING);

    proc.index = m_ImageSystem->addImageProcessor(
        "GStreamer sink on UDP port " + std::to_string(port));
}