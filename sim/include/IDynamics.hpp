#ifndef IDYNAMICS_HPP
#define IDYNAMICS_HPP
 
template<typename State, typename Control>
class IDynamicsModel
{
public:
    virtual ~IDynamicsModel() = default;

    virtual State computeDerivative(const State& x,
                                    const Control& u,
                                    double t) const = 0;
};

#endif //IDYNAMICS_HPP