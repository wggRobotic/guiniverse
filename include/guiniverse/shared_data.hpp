#pragma once

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <guiniverse/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <imgui.h>

using namespace std::string_view_literals;

using ImageData = std::pair<Image, bool>;

// Shared data mutexes
extern std::atomic<bool> running;

extern std::mutex image_mutex;
extern std::vector<ImageData> shared_image_data; // Shared image data buffer

extern std::mutex twist_mutex;
extern geometry_msgs::msg::Twist shared_twist;

extern std::mutex gripper_mutex;
extern std_msgs::msg::Float32MultiArray shared_gripper;

extern std::mutex barcode_mutex;
extern std::map<std::string, size_t> shared_barcodes;

extern std::mutex shared_data_mutex;
extern std::string shared_data;

constexpr std::array<std::string_view, 4> IMAGE_TOPICS{
    "n10/front/color"sv,
    "/n10/cam_dif"sv,
    "n10/rear/color"sv,
    "n10/gripper/color"sv,
};

enum motor_service_status
{
    MOTOR_SERVICE_NONE,
    MOTOR_SERVICE_ENABLE,
    MOTOR_SERVICE_DISABLE,
};

extern std::mutex motor_service_mutex;
extern int shared_motor_service_request;