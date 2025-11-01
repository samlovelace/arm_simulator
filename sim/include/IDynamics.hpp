#ifndef IDYNAMICS_HPP
#define IDYNAMICS_HPP
 
#include <Eigen/Dense>  

class IDynamics 
{ 
public:
    virtual ~IDynamics() = default; 
    virtual Eigen::VectorXd step() = 0; 

private:
   
};
#endif //IDYNAMICS_HPP