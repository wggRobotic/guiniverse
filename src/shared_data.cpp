#include <guiniverse/shared_data.hpp>

std::atomic<bool> running{true};

std::mutex image_mutex;
std::vector<ImageData> shared_image_data;

std::mutex twist_mutex;
geometry_msgs::msg::Twist shared_twist;

std::mutex gripper_mutex;
std_msgs::msg::Float32MultiArray shared_gripper;

std::mutex barcode_mutex;
std::map<std::string, size_t> shared_barcodes;

std::mutex shared_data_mutex;
std::string shared_data;

std::mutex input_data_mutex;
input_data shared_input_data;