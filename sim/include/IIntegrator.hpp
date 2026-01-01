#ifndef IINTEGRATOR_HPP
#define IINTEGRATOR_HPP

template<typename State, typename Control> 
class IIntegrator 
{ 
public:

    virtual ~IIntegrator() = default; 
    virtual State step(const IDynamicsModel<State, Control>& aModel, 
                       const State& x, 
                       const Control& u, 
                       double dt) = 0; 

private:
   
};
#endif //IINTEGRATOR_HPP