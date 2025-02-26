#include <guiniverse/imgui_utils.hpp>

#include <cmath>
#include <stdio.h>

ImVec2 imgui_joystick(const char* label, float size, ImVec2 dead_ranges, ImVec2* position, ImU32 color) {
    ImVec2 cursorPos = ImGui::GetCursorScreenPos();

    ImGui::InvisibleButton(label, ImVec2(size, size));

    ImVec2 joystick_position;

    if (position != 0) joystick_position = *position;
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

void imgui_arrow(ImVec2 start, float angle, float magnitude, ImU32 color, float thickness, float arrowSize, bool showAmplitude) 
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 windowPos = ImGui::GetWindowPos();
    start = ImVec2(start.x + windowPos.x, start.y + windowPos.y);

    ImVec2 dir = ImVec2(-sinf(angle), -cosf(angle));

    float mag_arrow = (magnitude >= 0.f ? (magnitude - arrowSize > 0.f ? magnitude - arrowSize : 0.f) : (magnitude + arrowSize < 0.f ? magnitude + arrowSize : 0.f));

    ImVec2 end = ImVec2(start.x + magnitude * dir.x, start.y + magnitude * dir.y);

    draw_list->AddLine(start, ImVec2(start.x + mag_arrow * dir.x, start.y + mag_arrow * dir.y), color, thickness);

    ImVec2 perp = ImVec2(-dir.y, dir.x);

    ImVec2 arrowLeft = ImVec2(end.x + dir.x * arrowSize * (magnitude >= 0 ? -1.f : 1.f) + perp.x * (arrowSize * 0.5f), 
                              end.y + dir.y * arrowSize * (magnitude >= 0 ? -1.f : 1.f) + perp.y * (arrowSize * 0.5f));
    ImVec2 arrowRight = ImVec2(end.x + dir.x * arrowSize * (magnitude >= 0 ? -1.f : 1.f) - perp.x * (arrowSize * 0.5f), 
                               end.y + dir.y * arrowSize * (magnitude >= 0 ? -1.f : 1.f) - perp.y * (arrowSize * 0.5f));

    draw_list->AddTriangleFilled(end, arrowLeft, arrowRight, color);

    if (showAmplitude) {
        char label[32];
        snprintf(label, sizeof(label), "%.2f", magnitude);
        ImVec2 textPos = ImVec2((start.x + end.x) * 0.5f, (start.y + end.y) * 0.5f);
        draw_list->AddText(textPos, IM_COL32(255, 255, 255, 255), label);
    }
}
