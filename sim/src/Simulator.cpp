
#include "Simulator.h"
#include "RateController.hpp"
#include "AbvDynamics.h"
#include "AbvUdpInputFetcher.h"

Simulator::Simulator() : mDynamics(std::make_shared<AbvDynamics>()),
                         mInputFetcher(std::make_shared<AbvUdpInputFetcher>()), 
                         mSolverMethod(SOLVER_METHOD::EULER), 
                         mStateVector(Eigen::Matrix<double, 12, 1>::Zero()) 
{

}

Simulator::~Simulator()
{

}

void Simulator::run()
{
    RateController rate(50); 

    if(!mInputFetcher->startListening())
    {
        std::cout << "failed to start input listening thread" << std::endl; 
        return; 
    }

    while(true)
    {
        rate.start(); 
        
        Eigen::VectorXd input = mInputFetcher->getLatestInput(); 
        stepDynamics(input);  
        
        rate.block(); 
    }
}

void Simulator::stepDynamics(const Eigen::VectorXd& aLatestInput)
{
    switch (mSolverMethod)
    {
    case SOLVER_METHOD::EULER:
        euler(aLatestInput); 
        break;
    case SOLVER_METHOD::RK4:
        rk4(aLatestInput); 
        break; 
    default:
        euler(aLatestInput); 
        break; 
    }
}

void Simulator::euler(const Eigen::VectorXd& aLatestInput)
{

}

void Simulator::rk4(const Eigen::VectorXd& aLatestInput)
{
    
}