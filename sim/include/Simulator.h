#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <memory> 
#include "IDynamics.hpp"
#include "IInputFetcher.hpp"
class Simulator 
{ 
public:
    Simulator();
    ~Simulator();

    enum class SOLVER_METHOD
    {
        EULER, 
        RK4, 
        NUM_TYPES
    }; 

    void run(); 

private: 
    void stepDynamics(const Eigen::VectorXd& aLatestInput);

    void euler(const Eigen::VectorXd& aLatestInput); 
    void rk4(const Eigen::VectorXd& aLatestInput); 

private:
    std::shared_ptr<IDynamics> mDynamics; 
    std::shared_ptr<IInputFetcher> mInputFetcher; 

    Eigen::Matrix<double, 12, 1> mStateVector; 

    SOLVER_METHOD mSolverMethod; 
};
#endif //SIMULATOR_H