#ifndef TESTDYNAMICS_HPP
#define TESTDYNAMICS_HPP
 
#include <iostream>
#include "IDynamics.hpp" 

class TestDynamics : public IDynamics
{ 
public:
    TestDynamics() {}
    ~TestDynamics() override {} 

    Eigen::VectorXd step() override {
        std::cout << "step test dynamics..." << std::endl; 
    }

private:
   
};
#endif //TESTDYNAMICS_HPP