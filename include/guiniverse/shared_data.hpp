#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <map>
#include <vector>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

// Shared data mutexes
extern std::atomic<bool> running;

extern std::mutex image_mutex;
extern std::vector<cv::Mat> shared_image_data;  // Shared image data buffer

extern std::mutex image_topics_mutex;
extern std::vector<std::string> shared_image_topics;

extern std::mutex twist_mutex;
extern geometry_msgs::msg::Twist shared_twist;

extern std::mutex gripper_mutex;
extern std_msgs::msg::Float32MultiArray shared_gripper;

extern std::mutex barcode_mutex;
extern std::map<std::string, size_t> shared_barcodes;

extern std::mutex shared_data_mutex;
extern std::string shared_data;
