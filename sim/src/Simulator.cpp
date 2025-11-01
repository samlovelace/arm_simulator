
#include "Simulator.h"
#include "RateController.hpp"
#include "TestDynamics.hpp"

Simulator::Simulator() : mDynamics(std::make_shared<TestDynamics>())
{

}

Simulator::~Simulator()
{

}

void Simulator::run()
{
    RateController rate(50); 

    while(true)
    {
        rate.start(); 
        mDynamics->step(); 
        rate.block(); 
    }
}