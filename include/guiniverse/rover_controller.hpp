#pragma once

#include <guiniverse/input.hpp>
#include <vector>

struct rover_controller_wheel
{
    ImVec2 position = ImVec2(0, 0);

    float last_angle = 0;
    float last_speed = 0;

};

struct rover_controller
{
public:

    rover_controller();

    void add_wheel(ImVec2 position);

    void process(input_state& input);
    void transfer_data_ros();

    ImVec2 get_linear_velocity() {return linear_velocity;};
    float get_angular_velocity() {return angular_velocity;};

private:

    std::vector<rover_controller_wheel> wheels;

    ImVec2 linear_velocity = ImVec2(0, 0);
    float angular_velocity = 0;

    int motor_status = 0;
};