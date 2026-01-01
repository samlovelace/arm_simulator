#ifndef EULERINTEGRATOR_H
#define EULERINTEGRATOR_H
 
#include "IIntegrator.hpp"
 
template<typename State, typename Control> 
class EulerIntegrator : public IIntegrator<State, Control> 
{ 
public:
    EulerIntegrator() {}
    ~EulerIntegrator() override = default; 

    State step(const IDynamicsModel<State, Control>& aModel, 
               const State& x, 
               const Control& u, 
               double dt) override 
    {
        State xdot = aModel.computeDerivative(x, u, -1);  // TODO: do i need this t var?
        return x + dt * xdot;
    }

private:
   
};
#endif //EULERINTEGRATOR_H