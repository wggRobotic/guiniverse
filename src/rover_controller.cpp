#include <guiniverse/rover_controller.hpp>
#include <cmath>
#include <guiniverse/shared_data.hpp>

ImVec2 imgui_joystick(const char* label, float size, ImVec2 dead_ranges, ImVec2* position, ImU32 color) {
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
    draw_list->AddCircleFilled(ImVec2(cursorPos.x + size * 0.5f, cursorPos.y + size * 0.5f), size * 0.5f, color);
    draw_list->AddCircle(ImVec2(cursorPos.x + size * 0.5f, cursorPos.y + size * 0.5f), size * 0.5f, IM_COL32(255, 255, 255, 255));
    draw_list->AddCircleFilled(handle_pos, size * 0.1f, IM_COL32(255, 0, 0, 255), 32);

    ImGui::SetCursorScreenPos(ImVec2(cursorPos.x, cursorPos.y + size + ImGui::GetStyle().ItemSpacing.y));

    return joystick_position;
}

rover_controller::rover_controller(GLFWwindow* w) : window(w) {

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

            float new_joystick_scalar = input.get_axis(3) / 2.f + 0.5f;
            if (new_joystick_scalar != joystick_scalar) {
                joystick_scalar = new_joystick_scalar;
                scalar = new_joystick_scalar;
            }

            gas_button = input.get_button(0);

            bool new_drive_mode_button = input.get_button(8);
            bool new_gripper_mode_button = input.get_button(9);
            
            if (new_drive_mode_button) drive_mode_button = true;
            else if (new_gripper_mode_button) gripper_mode_button = true;

            bool new_physical_enable_button = input.get_button(10);
            if (new_physical_enable_button && !physical_enable_button) enable_button = true;
            physical_enable_button = new_physical_enable_button;

            bool new_physical_disable_button = input.get_button(11);
            if (new_physical_disable_button && !physical_disable_button) disable_button = true;
            physical_disable_button = new_physical_disable_button;

            dog_walk_button = input.get_button(1);

        } break;

        default: {
            main_axes = ImVec2(0.f, 0.f);
        }

    }

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) main_axes.x = 1.f;
    else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) main_axes.x = -1.f;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) main_axes.y = 1.f;
    else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) main_axes.y = -1.f;

    float length = sqrtf(main_axes.x * main_axes.x + main_axes.y * main_axes.y);
    if (length > 1.0f) main_axes = ImVec2(main_axes.x / length, main_axes.y / length);

    gas_button |= (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS);

    if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS) {drive_mode_button = true; gripper_mode_button = false;}
    else if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS) {gripper_mode_button = true; drive_mode_button = false;}

    bool new_physical_enable_button = (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS);
    if (new_physical_enable_button && !physical_enable_button) enable_button = true;
    physical_enable_button = new_physical_enable_button;

    bool new_physical_disable_button = (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS);
    if (new_physical_disable_button && !physical_disable_button) disable_button = true;
    physical_disable_button = new_physical_disable_button;

    dog_walk_button |= (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS);

}

void rover_controller::process() {

    if (gripper_mode_button) gripper_mode = true;
    else if (drive_mode_button) gripper_mode = false;

    ImVec2 main_axes_scaled = ImVec2(main_axes.x * scalar * (gas_button ? 1.f : 0.f), main_axes.y * scalar * (gas_button ? 1.f : 0.f));
 

    if (dog_walk_button) {
        linear_velocity.x = main_axes_scaled.y;
        linear_velocity.y = main_axes_scaled.x;
        angular_velocity = 0.f;
    }
    else {
        linear_velocity.x = main_axes_scaled.y;
        linear_velocity.y = 0.f;
        angular_velocity = main_axes_scaled.x * (main_axes_scaled.y < 0 ? -1.f : 1.f);
    }

    should_set_motor_status = ( enable_button | disable_button );
    motor_status = enable_button & should_set_motor_status;

    main_axes = ImVec2(0.f, 0.f);
    gas_button = false;

    drive_mode_button = false;
    gripper_mode_button = false;

    enable_button = false;
    disable_button = false;

    dog_walk_button = false;
    
}

void rover_controller::ImGui_panel_control(char* label) {

    if (ImGui::Begin(label))
    {

        if (ImGui::RadioButton("Drive", !gripper_mode)) gripper_mode = false;
        if (ImGui::RadioButton("Gripper", gripper_mode)) gripper_mode = true;

        ImVec2 pos = ImGui::GetCursorScreenPos();

        main_axes = imgui_joystick("virtual joystick", 200.f, ImVec2(0.1f, 0.1f), (main_axes.x == 0.f && main_axes.y == 0.f) ? NULL : &main_axes, (gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

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