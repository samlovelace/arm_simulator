#ifndef IINPUTFETCHER_HPP
#define IINPUTFETCHER_HPP
 
#include <mutex> 
#include <Eigen/Dense>
 
class IInputFetcher 
{ 
public:
    
    virtual ~IInputFetcher() = default; 
    virtual bool startListening() = 0; 

    Eigen::VectorXd getLatestInput() {
        std::lock_guard<std::mutex> lock(mInputMutex); 
        return mLatestInput; 
    }

protected:
    
    void setLatestInput(const Eigen::VectorXd& anInput)
    {
        std::lock_guard<std::mutex> lock(mInputMutex); 
        mLatestInput = anInput; 
    }

private: 
    
Eigen::VectorXd mLatestInput; 
    std::mutex mInputMutex; 

   
};
#endif //IINPUTFETCHER_HPP