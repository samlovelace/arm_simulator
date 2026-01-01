#ifndef STATES_HPP
#define STATES_HPP

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

#endif 