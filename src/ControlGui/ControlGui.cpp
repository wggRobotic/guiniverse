#include <guiniverse/ControlGui/ControlGui.hpp>

#include <thread>

ControlGui::ControlGui(int RobotControllerRefreshRate) : m_RosRate(RobotControllerRefreshRate) {
    //m_Console.Init();
}

void ControlGui::addController(std::shared_ptr<RobotController> Controller)
{
    if (Controller == NULL) return; 

    m_Controllers.push_back(Controller);
}

void ControlGui::run()
{

    m_Running.store(true);
    m_RobotSelected.store(NO_ROBOT_SELECTED);

    std::thread robot_controller_thread(&ControlGui::RobotControllerThreadFunction, this);

    GuiFunction();

    robot_controller_thread.join();
}

void ControlGui::stop()
{
    m_Running.store(false);
}

void ControlGui::RobotControllerThreadFunction() {

    int robot_selected = NO_ROBOT_SELECTED;

    while (m_Running.load())
    {

        if (m_ChangeState.load())
        {
            if (robot_selected != NO_ROBOT_SELECTED)
                m_Controllers.at(robot_selected)->onStartup();
            
            robot_selected = m_RobotSelected.load();

            if (robot_selected != NO_ROBOT_SELECTED)
                m_Controllers.at(robot_selected)->onShutdown();
            
            m_ChangeState.store(false);
        }

        if (robot_selected != NO_ROBOT_SELECTED)
        {  
            m_Controllers.at(robot_selected)->onFrame();
        }

        m_RosRate.sleep();
    }
}
