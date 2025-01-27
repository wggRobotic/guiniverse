#include <cmath>
#include <guiniverse/input.hpp>

ImVec2 imgui_joystick_rect(const char* label, float size, float deadzone, ImVec2* position) {
    ImVec2 cursorPos = ImGui::GetCursorScreenPos();  // Panel position in the UI

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 topLeft = cursorPos;
    ImVec2 bottomRight = ImVec2(cursorPos.x + size, cursorPos.y + size);

    // Draw joystick base (outer square)
    draw_list->AddRectFilled(topLeft, bottomRight, IM_COL32(50, 50, 50, 255), 10.0f);
    draw_list->AddRect(topLeft, bottomRight, IM_COL32(255, 255, 255, 100), 2.0f);

    // Handle mouse interaction
    ImGui::InvisibleButton(label, ImVec2(size, size));

    ImVec2 joystick_position;

    if (position != NULL) {
        joystick_position = *position;
    }
    else if (ImGui::IsItemActive()) {
        ImVec2 mouse_pos = ImGui::GetMousePos();

        joystick_position.x = 1.0f - 2.0f * ((mouse_pos.x - topLeft.x) / size);
        joystick_position.y = 1.0f - 2.0f * ((mouse_pos.y - topLeft.y) / size);

    } 
    else joystick_position = ImVec2(0.f, 0.f);

    joystick_position.x = fmaxf(-1.0f, fminf(1.0f, joystick_position.x));
    joystick_position.y = fmaxf(-1.0f, fminf(1.0f, joystick_position.y));

    if (fabsf(joystick_position.x) < deadzone) joystick_position.x = 0.0f;
    if (fabsf(joystick_position.y) < deadzone) joystick_position.y = 0.0f;

    ImVec2 handle_pos = ImVec2(
        topLeft.x + (1.0f - joystick_position.x) * 0.5f * size, 
        topLeft.y + (1.0f - joystick_position.y) * 0.5f * size
    );

    draw_list->AddCircleFilled(handle_pos, size * 0.1f, IM_COL32(255, 0, 0, 255), 32);

    ImGui::SetCursorScreenPos(ImVec2(cursorPos.x, cursorPos.y + size + ImGui::GetStyle().ItemSpacing.y));

    return joystick_position;
}

void input_state::control_panal_imgui() {

    int axes_count;
    float* axes;
    int button_count;
    unsigned char* buttons;

    {
        joystick_connected = true;

        axes = (float*) glfwGetJoystickAxes(GLFW_JOYSTICK_1, &axes_count);
        buttons = (unsigned char*) glfwGetJoystickButtons(GLFW_JOYSTICK_1, &button_count);

        if (buttons == NULL || axes == NULL) joystick_connected = false;
    } 

    if (ImGui::Begin("Control"))
    {

        if (joystick_connected) 
            ImGui::TextColored(ImVec4(0.f, 1.f, 0.f, 1.f), "Joystick connected");
        else 
            ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "Joystick not connected");

        // input method selection
        if (joystick_connected) 
        {
            if (ImGui::RadioButton("Joystick Logitech", input_mode == INPUT_JOYSTICK_LOGITECH)) 
                input_mode = INPUT_JOYSTICK_LOGITECH;
            if (ImGui::RadioButton("Joystick XBOX", input_mode == INPUT_JOYSTICK_XBOX)) 
                input_mode = INPUT_JOYSTICK_XBOX;
        }
        else input_mode = INPUT_KEYBOARD;

        if (ImGui::RadioButton("Keyboard", input_mode == INPUT_KEYBOARD)) 
            input_mode = INPUT_KEYBOARD;

        //input methods
        switch (input_mode) 
        {

        case INPUT_KEYBOARD: 
        {
            main_axes = ImVec2(0.f, 0.f);

            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) main_axes = ImVec2(0.f, 1.f);
            else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) main_axes = ImVec2(0.f, -1.f);
            else if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) main_axes = ImVec2(1.f, 1.f);
            else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) main_axes = ImVec2(-1.f, 1.f);
        
        } break;

        case INPUT_JOYSTICK_LOGITECH: 
        {
            if (fabsf(axes[0]) < 0.1) axes[0] = 0.f;
            if (fabsf(axes[1]) < 0.1) axes[1] = 0.f;
            main_axes = ImVec2(-axes[0] * buttons[0], -axes[1] * buttons[0]);
        } break;

        case INPUT_JOYSTICK_XBOX: 
        {
            main_axes = ImVec2(0.f, 0.f);
        } break;

        }

        main_axes = imgui_joystick_rect("virtual joystick", 200.f, 0.1f, (main_axes.x == 0.f && main_axes.y == 0.f) ? NULL : &main_axes);
        ImGui::Text("Position %f %f", main_axes.x, main_axes.y);

    }
    ImGui::End();

}