#include <guiniverse/rover_controller.hpp>
#include <cmath>

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
}