#ifndef IINPUTFETCHER_HPP
#define IINPUTFETCHER_HPP
 
#include <mutex> 
#include <Eigen/Dense>
 
template<typename Control> 
class IInputFetcher 
{ 
public:
    
    virtual ~IInputFetcher() = default; 
    virtual bool startListening() = 0; 

    Control getLatestInput() 
    {
        std::lock_guard<std::mutex> lock(mInputMutex); 
        return mLatestInput; 
    }

protected:
    
    void setLatestInput(const Control& anInput)
    {
        std::lock_guard<std::mutex> lock(mInputMutex); 
        mLatestInput = anInput; 
    }

private: 
    
    Control mLatestInput; 
    std::mutex mInputMutex; 
 
};
#endif //IINPUTFETCHER_HPP