#ifndef ABVUDPINPUTFETCHER_H
#define ABVUDPINPUTFETCHER_H
 
#include "IInputFetcher.hpp"
#include "UdpServer.h" 

class AbvUdpInputFetcher : public IInputFetcher
{ 
public:
    AbvUdpInputFetcher();
    ~AbvUdpInputFetcher() override; 

    bool startListening() override; 

private: 
    void onReceived(const std::string& message);
    void convertThrusterCommandToForce(const std::string& thrusterCommand);
    
private:
    std::unique_ptr<UdpServer> mUdpServer; 
    Eigen::Vector3d mThrustCmd; 
    double mThrusterForce; 
    double mMomentArm;
   
};
#endif //ABVUDPINPUTFETCHER_H