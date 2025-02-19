#include <cmath>
#include <guiniverse/input.hpp>
#include <stdio.h>
#include <string.h>

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

void input_state::update() {

    int joystick_axes_count;
    float* joystick_axes;
    int joystick_button_count;
    unsigned char* joystick_buttons;

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
        joystick_axes = (float*) glfwGetJoystickAxes(input_device_selected_index, &joystick_axes_count);
        joystick_buttons = (unsigned char*) glfwGetJoystickButtons(input_device_selected_index, &joystick_button_count);

        if (joystick_buttons == NULL || joystick_axes == NULL) {
            input_devices_connected[input_device_selected_index] = false;
            input_device_selected_index = -1;
        }
    }

    for (int i = 0; i < INPUT_BUTTON_MAX_ENUM; i++) buttons[i] = false;
    axes[INPUT_AXIS_MAIN_X] = 0.f;
    axes[INPUT_AXIS_MAIN_Y] = 0.f;

    if(input_device_selected_index != -1) if (input_device_profiles[input_device_selected_index] != -1) {

        for (int i = 0; i < INPUT_BUTTON_MAX_ENUM; i++) 
            buttons[i] = joystick_buttons[input_button_mappings[input_device_profiles[input_device_selected_index]][i]];
        for (int i = 0; i < INPUT_AXIS_MAX_ENUM; i++) 
            axes[i] = joystick_axes[input_axes_mappings[input_device_profiles[input_device_selected_index]][i]];
    }

}

void input_state::imgui_panel() {

    if (ImGui::BeginCombo("Select input device", input_device_selected_index == -1 ? "No device selected" : input_device_names[input_device_selected_index]))
    {
        if (ImGui::Selectable("No device selected", input_device_selected_index == -1))
            input_device_selected_index = -1;

        for (int i = 0; i <= GLFW_JOYSTICK_LAST; i++) if (input_devices_connected[i] == true) 
        {
            if (ImGui::Selectable(input_device_names[i], input_device_selected_index == i)) 
                input_device_selected_index = i;
        }
        
        ImGui::EndCombo();
    }

    if (ImGui::Button("Refresh"))
        refresh_button = true;

    if (input_device_profiles[input_device_selected_index] == -1) ImGui::TextColored(ImVec4(1.0, 0.0, 0.0, 1.0), "Couldn't find input profile for this device");

    ImVec2 pos = ImGui::GetCursorScreenPos();

    ImVec2 main_axes = ImVec2(axes[INPUT_AXIS_MAIN_X], axes[INPUT_AXIS_MAIN_Y]);
    main_axes = imgui_joystick("virtual joystick", 200.f, ImVec2(0.1f, 0.1f), (main_axes.x == 0.f && main_axes.y == 0.f) ? NULL : &main_axes);
    axes[INPUT_AXIS_MAIN_X] = main_axes.x;
    axes[INPUT_AXIS_MAIN_Y] = main_axes.y;

    ImGui::Text("Joystick %f %f", main_axes.x, main_axes.y);

    if (ImGui::Button("Enable")) buttons[INPUT_BUTTON_ENABLE] = true;
    if (ImGui::Button("Disable")) buttons[INPUT_BUTTON_DISABLE] = true;

    ImGui::SetCursorScreenPos(ImVec2(pos.x + 220.f, pos.y + ImGui::GetStyle().ItemSpacing.y));

    ImGui::VSliderFloat("##vslider", ImVec2(20, 200), &axes[INPUT_AXIS_SCALAR], 0.0f, 1.0f, "%.2f");

}