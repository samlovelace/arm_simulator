#ifndef SIMULATORFACTORY_HPP
#define SIMULATORFACTORY_HPP

#include <memory>
#include "ISimulator.hpp"
#include "SimulatorImpl.h"
#include "States.hpp"
#include "Controls.hpp"

// Input Fetcher(s)
#include "IInputFetcher.hpp" 
#include "AbvUdpInputFetcher.h"

// Dynamic Models 
#include "AbvDynamics.h"

// Integrators 
#include "EulerIntegrator.h"
 
class SimulatorFactory 
{ 
public:
    SimulatorFactory();
    ~SimulatorFactory();

    static std::shared_ptr<ISimulator> create(const std::string& aSimConfig)
    {
        if("abv" == aSimConfig)
        {
            using State = ThreeDofPlanar; 
            using Control = AbvControl; 

            // integrator 
            std::unique_ptr<IIntegrator<State, Control>> integrator =
                std::make_unique<EulerIntegrator<State, Control>>();
            
            // model 
            std::unique_ptr<IDynamicsModel<State, Control>> model = 
                std::make_unique<AbvDynamics<State, Control>>(12.7, 0.3); 

            // input fetcher 
            std::unique_ptr<IInputFetcher<Control>> input = std::make_unique<AbvUdpInputFetcher<Control>>(); 

            // setup state publisher
            RosTopicManager::getInstance()->createPublisher<robot_idl::msg::AbvState>("abv/sim/state");
            RosTopicManager::getInstance()->spinNode();     
        

            return std::make_shared<SimulatorImpl<State, Control>>(std::move(model), 
                                                                   std::move(integrator), 
                                                                   std::move(input), 
                                                                   std::bind(&StatePublish::threeDofPlanarRosPublishFunc, 
                                                                             std::placeholders::_1)); 
        }
        else
        {
            throw std::runtime_error("Unsupported simulation config: " + aSimConfig);
        }

        return nullptr; 
    }

private:
   
};
#endif //SIMULATORFACTORY_HPP