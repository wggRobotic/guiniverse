#include <imgui.h>

ImVec2 imgui_joystick(const char* label, float size, ImVec2 dead_ranges, ImVec2* position, ImU32 color);

void imgui_arrow(ImVec2 start, float angle, float magnitude, ImU32 color, float thickness, float arrowSize, bool showAmplitude);