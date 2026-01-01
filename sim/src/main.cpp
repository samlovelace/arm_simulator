
// #include <ament_index_cpp/get_package_share_directory.hpp>
#include "SimulatorFactory.hpp"

int main()
{
    std::cout << "############ Starting Simulation ############\n"; 
    //std::signal(SIGINT, signalHandler);
    //createLogger();

    // std::string configFilePath = ament_index_cpp::get_package_share_directory("robot_simulator") + "/configuration/config.yaml";

    // if(!ConfigManager::getInstance()->load(configFilePath))
    // {
    //     printf("Could not load config file at %s\n", configFilePath.c_str());
    //     return 0;
    // }

    std::string simConfig = "abv"; 
    std::shared_ptr<ISimulator> sim = SimulatorFactory::create(simConfig);
    sim->run(); 
}
