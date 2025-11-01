#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <memory> 
#include "IDynamics.hpp"
 
class Simulator 
{ 
public:
    Simulator();
    ~Simulator();

    void run(); 

private:
    std::shared_ptr<IDynamics> mDynamics; 
};
#endif //SIMULATOR_H