#include <cmath>
#include <guiniverse/input.hpp>
#include <stdio.h>
#include <string.h>
#include <iostream>

ImVec2 imgui_joystick_rect(const char* label, float size, float deadzone, ImVec2* position) {
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

    joystick_position.x = fmaxf(-1.0f, fminf(1.0f, joystick_position.x));
    joystick_position.y = fmaxf(-1.0f, fminf(1.0f, joystick_position.y));

    if (fabsf(joystick_position.x) < deadzone) joystick_position.x = 0.0f;
    if (fabsf(joystick_position.y) < deadzone) joystick_position.y = 0.0f;

    ImVec2 handle_pos = ImVec2(
        cursorPos.x + (1.0f - joystick_position.x) * 0.5f * size, 
        cursorPos.y + (1.0f - joystick_position.y) * 0.5f * size
    );

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(cursorPos, ImVec2(cursorPos.x + size, cursorPos.y + size), IM_COL32(50, 50, 50, 255), 10.0f);
    draw_list->AddRect(cursorPos, ImVec2(cursorPos.x + size, cursorPos.y + size), IM_COL32(255, 255, 255, 100), 2.0f);
    draw_list->AddCircleFilled(handle_pos, size * 0.1f, IM_COL32(255, 0, 0, 255), 32);

    ImGui::SetCursorScreenPos(ImVec2(cursorPos.x, cursorPos.y + size + ImGui::GetStyle().ItemSpacing.y));

    return joystick_position;
}

ImVec2 imgui_joystick_round(const char* label, float size, float dead_angle, ImVec2* position) {
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
    
    if (fabsf(joystick_position.x) < dead_angle) joystick_position.x = 0.0f;
    if (fabsf(joystick_position.y) < dead_angle) joystick_position.y = 0.0f;

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

void input_state::update() {

    int axes_count;
    float* axes;
    int button_count;
    unsigned char* buttons;

    if (refresh_button)
    {
        refresh_button = false;
        for (int i = 0; i <= GLFW_JOYSTICK_LAST; i++) 
        {
            input_devices_connected[i] = false;
            if ((input_devices_connected[i] = glfwJoystickPresent(GLFW_JOYSTICK_1 + i)) != 0) 
            {
                const char* name = glfwGetJoystickName(GLFW_JOYSTICK_1 + i);
                if (name) snprintf(input_device_names[i], sizeof(input_device_names[i]), "%s", name);
                else snprintf(input_device_names[i], sizeof(input_device_names[i]), "Couldn't read device name");
                
                input_device_profiles[i] = -1;

                for (int j = 0; j < INPUT_PROFILE_MAX_ENUM; j++)
                {
                    if (strcmp(known_input_device_names[j], input_device_names[i]) == 0)
                    {
                        input_device_profiles[i] = j;
                        break;
                    }
                }

            }
            else if (i == input_device_selected_index) input_device_selected_index = -1;
        }
    }    

    if (input_device_selected_index != -1) 
    {
        axes = (float*) glfwGetJoystickAxes(input_device_selected_index, &axes_count);
        buttons = (unsigned char*) glfwGetJoystickButtons(input_device_selected_index, &button_count);

        if (buttons == NULL || axes == NULL) {
            input_devices_connected[input_device_selected_index] = false;
            input_device_selected_index = -1;
        }
    }

    main_axes = ImVec2(0.f, 0.f);


    if(input_device_selected_index != -1) if (input_device_profiles[input_device_selected_index] != -1) {

        switch (input_device_profiles[input_device_selected_index]) {

            case INPUT_PROFILE_LOGITECH_JOYSTICK: {

                if (fabsf(axes[0]) < 0.1) axes[0] = 0.f;
                if (fabsf(axes[1]) < 0.1) axes[1] = 0.f;
                main_axes = ImVec2(-axes[0] * buttons[0], -axes[1] * buttons[0]);
            } break;

        }
    }

}

void input_state::imgui_panel() {

    if (ImGui::BeginCombo("Select input device", input_device_selected_index == -1 ? "No device selected" : input_device_names[input_device_selected_index]))
    {

        if (ImGui::Selectable("No device selected", input_device_selected_index == -1))
            input_device_selected_index = -1;

        for (int i = 0; i <= GLFW_JOYSTICK_LAST; i++)
        {
            if (input_devices_connected[i] == true) 
            {
                if (ImGui::Selectable(input_device_names[i], input_device_selected_index == i)) 
                    input_device_selected_index = i;
            }
        }
        
        ImGui::EndCombo();
    }

    if (ImGui::Button("Refresh"))
        refresh_button = true;

    if (input_device_profiles[input_device_selected_index] == -1) ImGui::TextColored(ImVec4(1.0, 0.0, 0.0, 1.0), "Couldn't find input profile for this device");

    main_axes = imgui_joystick_round("virtual joystick", 200.f, 0.1f, (main_axes.x == 0.f && main_axes.y == 0.f) ? NULL : &main_axes);
    ImGui::Text("Position %f %f", main_axes.x, main_axes.y);

}