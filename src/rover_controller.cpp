#include <guiniverse/rover_controller.hpp>
#include <cmath>
#include <guiniverse/shared_data.hpp>

ImVec2 imgui_joystick(const char* label, float size, ImVec2 dead_ranges, ImVec2* position) {
    ImVec2 cursorPos = ImGui::GetCursorScreenPos();

    ImGui::InvisibleButton(label, ImVec2(size, size));

    ImVec2 joystick_position;

    if (position != NULL) joystick_position = *position;
    else if (ImGui::IsItemActive()) 
    {
        ImVec2 mouse_pos = ImGui::GetMousePos();

        joystick_position.x = 1.0f - 2.0f * ((mouse_pos.x - cursorPos.x) / size);
        joystick_position.y = 1.0f - 2.0f * ((mouse_pos.y - cursorPos.y) / size);

    } 
    else joystick_position = ImVec2(0.f, 0.f);

    float length = sqrtf(joystick_position.x * joystick_position.x + joystick_position.y * joystick_position.y);

    if (length > 1.0f) {
        joystick_position.x = (joystick_position.x / length);
        joystick_position.y = (joystick_position.y / length);
    }
    
    if (fabsf(joystick_position.x) < dead_ranges.x) joystick_position.x = 0.0f;
    if (fabsf(joystick_position.y) < dead_ranges.y) joystick_position.y = 0.0f;

    ImVec2 handle_pos = ImVec2(
        cursorPos.x + (1.0f - joystick_position.x) * 0.5f * size, 
        cursorPos.y + (1.0f - joystick_position.y) * 0.5f * size
    );

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddCircleFilled(ImVec2(cursorPos.x + size * 0.5f, cursorPos.y + size * 0.5f), size * 0.5f, IM_COL32(255, 255, 255, 100));
    draw_list->AddCircle(ImVec2(cursorPos.x + size * 0.5f, cursorPos.y + size * 0.5f), size * 0.5f, IM_COL32(255, 255, 255, 100));
    draw_list->AddCircleFilled(handle_pos, size * 0.1f, IM_COL32(255, 0, 0, 255), 32);

    ImGui::SetCursorScreenPos(ImVec2(cursorPos.x, cursorPos.y + size + ImGui::GetStyle().ItemSpacing.y));

    return joystick_position;
}

rover_controller::rover_controller() {

    wheels.resize(6);
}

void rover_controller::add_wheel(ImVec2 position) {

    rover_controller_wheel wheel;
    wheel.position = position;
    wheels.push_back(wheel);

}

void rover_controller::get_input(input_state& input) {

    switch (input.get_device_profile())
    {
        case INPUT_PROFILE_LOGITECH_JOYSTICK: {
            main_axes = ImVec2(input.get_axis(0), input.get_axis(1));

            float new_joystick_scalar = input.get_axis(4) / 2.f + 0.5f;
            if (new_joystick_scalar != joystick_scalar) {
                joystick_scalar = new_joystick_scalar;
                scalar = new_joystick_scalar;
            }

            enable_button = input.get_button(10);
            disable_button = input.get_button(11);
        } break;

        default: {
            main_axes = ImVec2(0.f, 0.f);
        }
    }
    
}

void rover_controller::process() {

    ImVec2 main_axes_scaled = ImVec2(main_axes.x * scalar, main_axes.y * scalar);

    linear_velocity.x = main_axes_scaled.y;
    angular_velocity = main_axes_scaled.x * (main_axes_scaled.y < 0 ? -1.f : 1.f);

    should_set_motor_status = ( enable_button | disable_button );
    motor_status = enable_button & should_set_motor_status;
}

void rover_controller::ImGui_panel_control(char* label) {

    if (ImGui::Begin(label))
    {
        ImVec2 pos = ImGui::GetCursorScreenPos();

        main_axes = imgui_joystick("virtual joystick", 200.f, ImVec2(0.1f, 0.1f), (main_axes.x == 0.f && main_axes.y == 0.f) ? NULL : &main_axes);

        ImGui::Text("Joystick %f %f", main_axes.x, main_axes.y);

        if (ImGui::Button("Enable")) enable_button = true;
        if (ImGui::Button("Disable")) disable_button = true;

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 220.f, pos.y + ImGui::GetStyle().ItemSpacing.y));

        ImGui::VSliderFloat("##vslider", ImVec2(20, 200), &scalar, 0.0f, 1.0f, "%.2f");

        ImGui::End();
    }

}

void rover_controller::ImGui_panel_visualisation(char* label) {

    if (ImGui::Begin(label))
    {
        
        ImGui::End();
    }

}

void rover_controller::exchange_ros_data() {

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