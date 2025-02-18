#include <guiniverse/rover_controller.hpp>
#include <cmath>
#include <guiniverse/shared_data.hpp>

rover_controller::rover_controller() {

    wheels.resize(6);
}

void rover_controller::add_wheel(ImVec2 position) {

    rover_controller_wheel wheel;
    wheel.position = position;
    wheels.push_back(wheel);

}

void rover_controller::process(input_state& input) {

    ImVec2 main_axes = input.get_main_axes();

    linear_velocity.x = main_axes.y;
    angular_velocity = main_axes.x * (main_axes.y < 0 ? -1.f : 1.f);

    should_set_motor_status = ( input.get_enable_button() | input.get_disable_button() );
    motor_status = (input.get_enable_button() ? true : false);
}

void rover_controller::transfer_data_ros() {

    {
        std::lock_guard<std::mutex> lock(twist_mutex);
    
        shared_twist.linear.x = linear_velocity.x;
        shared_twist.linear.y = linear_velocity.y;
        shared_twist.angular.z = angular_velocity;
    }

    {
        std::lock_guard<std::mutex> lock(motor_enable_service_mutex);

        motor_enable_service_set_status = motor_status;
        motor_enable_service_is_set_status = should_set_motor_status;
    }

}