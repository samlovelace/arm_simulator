#ifndef ISIMULATOR_HPP
#define ISIMULATOR_HPP
 
 
class ISimulator 
{ 
public:
    virtual ~ISimulator() = default; 

    virtual void run() = 0; 
    virtual void step() = 0; 

private:
   
};
#endif //ISIMULATOR_HPP