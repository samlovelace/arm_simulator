#ifndef SIMULATORIMPL_H
#define SIMULATORIMPL_H

#include <iostream> 
#include <memory> 
#include <thread>

#include "ISimulator.hpp"
#include "IDynamics.hpp"
#include "IIntegrator.hpp"
#include "IInputFetcher.hpp"
#include "RateController.hpp"

template<typename State, typename Control>
class SimulatorImpl : public ISimulator
{ 
public:
    SimulatorImpl(std::unique_ptr<IDynamicsModel<State, Control>> aModel, 
                  std::unique_ptr<IIntegrator<State, Control>> anIntegrator, 
                  std::unique_ptr<IInputFetcher<Control>> anInput, 
                  std::function<void(const State&)> aPublishFunc) : 
        mModel(std::move(aModel)),
        mIntegrator(std::move(anIntegrator)), 
        mInputFetcher(std::move(anInput)), 
        mRate(50),
        mPublishStateFunc(aPublishFunc),
        mState(),
        mLatestInput()
        {}
    
    ~SimulatorImpl() = default; 

    void run() override 
    {
        if(!mInputFetcher->startListening())
        {
            std::cout << "failed to start input listening thread" << std::endl; 
            return; 
        }

        std::cout << "Input Fetcher initialized successfully!\n"; 

        while(true)
        {
            mRate.start();  

            step();
            mPublishStateFunc(mState); 
            std::cout << mState << std::endl; 
            
            mRate.block(); 
        }
    } 

    void step() override 
    {
        mLatestInput = mInputFetcher->getLatestInput(); 
        mState = mIntegrator->step(*mModel, mState, mLatestInput, mRate.getDeltaTime());
    }

private:

    State mState; 
    Control mLatestInput; 

    std::function<void(const State&)> mPublishStateFunc; 

    std::unique_ptr<IDynamicsModel<State, Control>> mModel; 
    std::unique_ptr<IIntegrator<State, Control>> mIntegrator;
    std::unique_ptr<IInputFetcher<Control>> mInputFetcher;  
   
    RateController mRate; 

};
#endif //SIMULATORIMPL_H