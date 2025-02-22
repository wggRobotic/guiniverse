#include <guiniverse/N10/N10.hpp>

#include <guiniverse/imgui_utils.hpp>

N10::N10() : RobotController("N10", "n10")
{
}

N10::~N10()
{
}

void N10::Init()
{
    m_ImageSystem = std::make_shared<ImageSystem>(shared_from_this());
}

void N10::ImGuiPanels(GLFWwindow* window, JoystickInput& input) {

    switch (input.getDeviceProfile())
    {
        case INPUT_PROFILE_LOGITECH_JOYSTICK: {
            main_axes = ImVec2(input.getAxis(0), input.getAxis(1));

            float new_joystick_scalar = input.getAxis(3) / 2.f + 0.5f;
            if (new_joystick_scalar != joystick_scalar) {
                joystick_scalar = new_joystick_scalar;
                scalar = new_joystick_scalar;
            }

            gas_button = input.getButton(0);

            bool new_drive_mode_button = input.getButton(8);
            bool new_gripper_mode_button = input.getButton(9);
            
            if (new_drive_mode_button) drive_mode_button = true;
            else if (new_gripper_mode_button) gripper_mode_button = true;

            bool new_physical_enable_button = input.getButton(10);
            if (new_physical_enable_button && !physical_enable_button) enable_button = true;
            physical_enable_button = new_physical_enable_button;

            bool new_physical_disable_button = input.getButton(11);
            if (new_physical_disable_button && !physical_disable_button) disable_button = true;
            physical_disable_button = new_physical_disable_button;

            dog_walk_button = input.getButton(1);

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

    if (ImGui::Begin("Control"))
    {

        if (ImGui::RadioButton("Drive", !gripper_mode)) gripper_mode = false;
        if (ImGui::RadioButton("Gripper", gripper_mode)) gripper_mode = true;

        ImVec2 pos = ImGui::GetCursorScreenPos();

        main_axes = imgui_joystick("virtual joystick", 200.f, ImVec2(0.1f, 0.1f), (main_axes.x == 0.f && main_axes.y == 0.f) ? 0 : &main_axes, (gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        ImGui::Text("Joystick %f %f", main_axes.x, main_axes.y);

        if (ImGui::Button("Enable")) enable_button = true;
        if (ImGui::Button("Disable")) disable_button = true;

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 220.f, pos.y + ImGui::GetStyle().ItemSpacing.y));

        ImGui::VSliderFloat("##vslider", ImVec2(20, 200), &scalar, 0.0f, 1.0f, "%.2f");

        ImGui::End();
    }

    if (ImGui::Begin("Visualization"))
    {
        

        ImGui::End();
    }
    
}

void N10::onFrame() {

}