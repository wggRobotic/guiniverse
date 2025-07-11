#include <fcntl.h>
#include <guiniverse/control_gui/console.hpp>
#include <imgui.h>
#include <mutex>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <vector>

Console::Console()
{
}

Console::~Console()
{
    m_Running = false;
    if (m_CaptureThread.joinable())
    {
        m_CaptureThread.join();
    }
}

void Console::Init()
{
    if (pipe(m_PipeFd) == -1)
    {
        perror("pipe failed");
        return;
    }

    // Set non-blocking mode
    fcntl(m_PipeFd[0], F_SETFL, O_NONBLOCK);

    // Redirect stdout and stderr
    dup2(m_PipeFd[1], STDOUT_FILENO);
    dup2(m_PipeFd[1], STDERR_FILENO);

    // Disable buffering for stdout and stderr
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    m_Running = true;
    m_CaptureThread = std::thread([this] { CaptureOutput(); });
}

void Console::CaptureOutput()
{
    char buffer[512];
    while (m_Running)
    {
        auto count = read(m_PipeFd[0], buffer, sizeof(buffer) - 1);
        if (count > 0)
        {
            buffer[count] = 0;

            std::lock_guard<std::mutex> lock(m_LogMutex);
            m_LogLines.emplace_back(buffer);
        }
    }
}

void Console::ImGuiPanel()
{
    if (!ImGui::Begin("Console"))
    {
        ImGui::End();
        return;
    }

    if (ImGui::BeginChild("ScrollingRegion", ImVec2(0, -ImGui::GetTextLineHeightWithSpacing()), false, ImGuiWindowFlags_HorizontalScrollbar))
    {
        std::lock_guard<std::mutex> lock(m_LogMutex);
        for (const auto& line : m_LogLines)
        {
            ImGui::TextUnformatted(line.c_str());
        }

        if (m_AutoScroll)
        {
            ImGui::SetScrollHereY(1.0f);
        }
    }

    ImGui::EndChild();
    ImGui::End();
}
