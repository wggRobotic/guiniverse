#include <guiniverse/control_gui/control_gui.hpp>
#include <thread>

ControlGui::ControlGui(int robot_controller_refresh_rate)
    : m_RosRate(robot_controller_refresh_rate)
{
    // m_Console.Init();
}

void ControlGui::AddController(std::shared_ptr<RobotController> controller)
{
    if (!controller)
        return;

    m_Controllers.push_back(controller);
}

void ControlGui::Run()
{
    m_Running = true;
    m_RobotSelected = NO_ROBOT_SELECTED;

    std::thread robot_controller_thread(
        [this] { ControlGui::RobotControllerThreadFunction(); });

    GuiFunction();

    robot_controller_thread.join();
}

void ControlGui::Stop()
{
    m_Running = false;
}

void ControlGui::RobotControllerThreadFunction()
{
    auto robot_selected = NO_ROBOT_SELECTED;

    while (m_Running)
    {
        if (m_ChangeState)
        {
            if (robot_selected != NO_ROBOT_SELECTED)
                m_Controllers.at(robot_selected)->Shutdown();

            robot_selected = m_RobotSelected;

            if (robot_selected != NO_ROBOT_SELECTED)
                m_Controllers.at(robot_selected)->Start();

            m_ChangeState = false;
        }

        if (robot_selected != NO_ROBOT_SELECTED)
        {
            m_Controllers.at(robot_selected)->OnFrame();
        }

        m_RosRate.sleep();
    }
}
