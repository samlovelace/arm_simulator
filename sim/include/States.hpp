#ifndef STATES_HPP
#define STATES_HPP

#include "robot_idl/msg/vec3.hpp"
#include "robot_idl/msg/abv_state.hpp"
#include "RosTopicManager.hpp"

struct ThreeDofPlanar
{
    double x, y, theta;
    double vx, vy, omega;
};

// State * scalar
inline ThreeDofPlanar operator*(const ThreeDofPlanar& s, double k)
{
    return {
        s.x * k,
        s.y * k,
        s.theta * k,
        s.vx * k,
        s.vy * k,
        s.omega * k
    };
}

// scalar * State
inline ThreeDofPlanar operator*(double k, const ThreeDofPlanar& s)
{
    return s * k;
}

inline ThreeDofPlanar operator+(const ThreeDofPlanar& a,
                                const ThreeDofPlanar& b)
{
    return {
        a.x + b.x,
        a.y + b.y,
        a.theta + b.theta,
        a.vx + b.vx,
        a.vy + b.vy,
        a.omega + b.omega
    };
}

inline ThreeDofPlanar& operator+=(ThreeDofPlanar& a,
                                  const ThreeDofPlanar& b)
{
    a.x     += b.x;
    a.y     += b.y;
    a.theta += b.theta;
    a.vx    += b.vx;
    a.vy    += b.vy;
    a.omega += b.omega;
    return a;
}

inline std::ostream& operator<<(std::ostream& os,
                                const ThreeDofPlanar& s)
{
    os << "pos=("
       << s.x << ", "
       << s.y << ", "
       << s.theta << ") "
       << "vel=("
       << s.vx << ", "
       << s.vy << ", "
       << s.omega << ")";
    return os;
}

// TODO: a better name for this namespace? 
namespace StatePublish
{
    void threeDofPlanarRosPublishFunc(const ThreeDofPlanar& aState)
    {
        // convert to ROS2 msg type 
        robot_idl::msg::Vec3 position; 
        robot_idl::msg::Vec3 velocity;

        position.x = aState.x; 
        position.y = aState.y; 
        position.z = 0.0; 

        velocity.x = aState.vx; 
        velocity.y = aState.vy; 
        velocity.z = 0.0; 

        robot_idl::msg::Vec3 orientation; 
        orientation.x = 0.0; 
        orientation.y = 0.0; 
        orientation.z = aState.theta; 

        robot_idl::msg::Vec3 ang_vel; 
        ang_vel.x = 0.0; 
        ang_vel.y = 0.0; 
        ang_vel.z = aState.omega; 

        robot_idl::msg::AbvState state; 
        state.set__position(position); 
        state.set__velocity(velocity); 
        state.set__orientation(orientation); 
        state.set__ang_vel(ang_vel);

        RosTopicManager::getInstance()->publishMessage<robot_idl::msg::AbvState>("abv/sim/state", state); 
    }
}

#endif 