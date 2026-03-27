#include <guiniverse/DataCaptureSystem.hpp>

#include <fstream>
#include <imgui.h>

DataCaptureSystem::DataCaptureSystem(std::shared_ptr<rclcpp::Node> Node) : m_Node(Node)
{
    m_Sections.reserve(3);
}

#define MAKE_DATA_CAPURE_SYSTEM_CALLBACK(INDEX) [this, INDEX](const std_msgs::msg::String::SharedPtr msg) { Callback(INDEX, msg); }

void DataCaptureSystem::addSection(const std::string& Name, const std::string& TopicName)
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    int current_size = m_Sections.size();
    m_Sections.resize(current_size + 1);

    m_Sections.at(current_size).SectionName = Name;
    m_Sections.at(current_size).Subscriber = m_Node->create_subscription<std_msgs::msg::String>(TopicName, 10, MAKE_DATA_CAPURE_SYSTEM_CALLBACK(current_size));
}

void DataCaptureSystem::Callback(size_t index, const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    m_Sections.at(index).Data[msg->data]++;
}

void DataCaptureSystem::ImGuiPanels() {

    std::lock_guard<std::mutex> lock(m_Mutex);

    for(int i = 0; i < m_Sections.size(); i++)
    {
        if (ImGui::Begin(m_Sections.at(i).SectionName.c_str()))
        {
            for(const auto &[code,has_code]: m_Sections.at(i).Data) ImGui::Text("%s: %lu", code.c_str(), has_code);

            if(m_Sections.at(i).Data.size() > 0)
            {
                if(ImGui::Button("Export"))
                {
                    std::string file_data;
                    for(const auto &pair: m_Sections.at(i).Data)
                    {
                        std::string keyStr = pair.first;
                        std::string valueStr = std::to_string(pair.second);
                        std:: string line = keyStr + ": " + valueStr;
                        file_data += line +"\n";
                    }
                    std::ofstream outputFile(m_Sections.at(i).SectionName + ".txt");
                    outputFile << file_data;
                    outputFile.close();
                }
            }
        }
        ImGui::End();
    }

}