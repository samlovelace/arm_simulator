#ifndef ABVDYNAMICS_H
#define ABVDYNAMICS_H
 
#include <iostream>
#include "IDynamics.hpp"
 
template<typename State, typename Control> 
class AbvDynamics : public IDynamicsModel<State, Control> 
{ 
public:
    AbvDynamics(double mass, double Iz) : mMass(mass), mIz(Iz) {}
    ~AbvDynamics() override = default;  

    State computeDerivative(const State& x,
                            const Control& u,
                            double t) const 
    {
        (void)t; 

        State xdot; 

        xdot.x = x.vx; 
        xdot.y = x.vy; 
        xdot.theta = x.omega; 

        xdot.vx = u.Fx / mMass; 
        xdot.vy = u.Fy / mMass; 
        xdot.omega = u.tau / mIz; 

        return xdot; 
    }

private:

    double mMass; 
    double mIz; 
   
};
#endif //ABVDYNAMICS_H