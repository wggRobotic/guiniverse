#include <cmath>
#include <guiniverse/input.hpp>
#include <stdio.h>
#include <string.h>
#include <imgui.h>

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
                
                input_device_profiles[i] = INPUT_PROFILE_UNKNOWN;

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

        if (joystick_buttons == NULL || joystick_axes == NULL) 
        {
            input_devices_connected[input_device_selected_index] = false;
            input_device_selected_index = -1;
        }
    }

    for (int i = 0; i < 32; i++) buttons[i] = false;
    for (int i = 0; i < 16; i++) axes[i] = false;

    if (input_device_selected_index != -1) if (input_device_profiles[input_device_selected_index] != INPUT_PROFILE_UNKNOWN) 
    {

        for (int i = 0; i < joystick_button_count; i++) 
            buttons[i] = joystick_buttons[i];
        for (int i = 0; i < joystick_axes_count; i++) 
            axes[i] = joystick_axes[i];
    }

}

void input_state::ImGui_panel(char* label) {

    if (ImGui::Begin(label))
    {
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

        if (input_device_selected_index != -1) 
        {
            if (input_device_profiles[input_device_selected_index] == INPUT_PROFILE_UNKNOWN) ImGui::TextColored(ImVec4(1.0, 0.0, 0.0, 1.0), "Couldn't find input profile for this device");
            else 
            {
                ImGui::Text("Invert axes");
                char invert_text[128];

                for (int i = 0; i < 16; i++) 
                {
                    snprintf(invert_text, sizeof(invert_text), "INPUT_AXIS_%d", i);
                    ImGui::Checkbox(invert_text, &invert_axes[input_device_selected_index][i]);
                }
            }
        }

        

        

        ImGui::End();
    }

}