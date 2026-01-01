#ifndef ABVUDPINPUTFETCHER_H
#define ABVUDPINPUTFETCHER_H
 
#include "IInputFetcher.hpp"
#include "UdpServer.h" 

template<typename Control> 
class AbvUdpInputFetcher : public IInputFetcher<Control>
{ 
public:
    AbvUdpInputFetcher() : 
        mUdpServer(std::make_unique<UdpServer>(6969, std::bind(&AbvUdpInputFetcher::onReceived, this, std::placeholders::_1))), 
        mThrusterForce(0.15), mMomentArm(0.1235)
    {}

    ~AbvUdpInputFetcher() override {} 

    bool startListening()
    {
        mUdpServer->start();
        return true;  
    }

    void onReceived(const std::string& message)
    {
        convertThrusterCommandToForce(message); 
        
        Control latestInput;
        latestInput.Fx = mThrustCmd[0]; 
        latestInput.Fy = mThrustCmd[1]; 
        latestInput.tau = mThrustCmd[2]; 
        
        this->setLatestInput(latestInput);  
    }

    void convertThrusterCommandToForce(const std::string& thrusterCommand)
    {   
        // assume the thruster command comes in as a string of 0's and 1's
        // 0 = off, 1 = on
        // thrusterCommand = "00000000" means all thrusters are off
        // thrusterCommand = "10000000" means thruster 1 is on, all others are off

        mThrustCmd = Eigen::Vector3d::Zero(); // Default case

        if (thrusterCommand == "00000011") {
            mThrustCmd = Eigen::Vector3d(2*mThrusterForce, 0, 0); // +x
        } 
        else if (thrusterCommand == "00110000") 
        {
            mThrustCmd = Eigen::Vector3d(-2*mThrusterForce, 0, 0); // -x
        } 
        else if (thrusterCommand == "11000000") // +y 
        {
            mThrustCmd = Eigen::Vector3d(0, 2*mThrusterForce, 0);
        } 
        else if (thrusterCommand == "00001100") // -y  
        {
            mThrustCmd = Eigen::Vector3d(0, -2*mThrusterForce, 0);
        } 
        else if (thrusterCommand == "01000100") // +phi  
        {
            mThrustCmd = Eigen::Vector3d(0, 0, 2*mThrusterForce*mMomentArm); 
        } 
        else if (thrusterCommand == "10001000") // -phi
        {
            mThrustCmd = Eigen::Vector3d(0, 0, -2*mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "01000010") // +x, +y
        {
            mThrustCmd = Eigen::Vector3d(mThrusterForce, mThrusterForce, 0);
        } 
        else if (thrusterCommand == "00001001") // +x, -y
        {
            mThrustCmd = Eigen::Vector3d(mThrusterForce, -mThrusterForce, 0);
        } 
        else if (thrusterCommand == "10010000") // -x, +y
        {
            mThrustCmd = Eigen::Vector3d(-mThrusterForce, mThrusterForce, 0); 
        } 
        else if (thrusterCommand == "00100100") // -x, -y
        {
            mThrustCmd = Eigen::Vector3d(-mThrusterForce, -mThrusterForce, 0);
        } 
        else if (thrusterCommand == "00000001") // +x, +phi
        {
            mThrustCmd = Eigen::Vector3d(mThrusterForce, 0, mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "00000010") // +x, -phi
        {
            mThrustCmd = Eigen::Vector3d(mThrusterForce, 0, -mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "00010000") // -x, +phi
        {
            mThrustCmd = Eigen::Vector3d(-mThrusterForce, 0, mThrusterForce*mMomentArm);
        }
        else if (thrusterCommand == "00100000") // -x, -phi
        {
            mThrustCmd = Eigen::Vector3d(-mThrusterForce, 0, -mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "01000000") // +y, +phi
        {
            mThrustCmd = Eigen::Vector3d(0, mThrusterForce, mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "10000000") // +y, -phi
        {
            mThrustCmd = Eigen::Vector3d(0, mThrusterForce, -mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "00000100") //-y, +phi
        {
            mThrustCmd = Eigen::Vector3d(0, -mThrusterForce, mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "00001000") //-y, -phi
        {
            mThrustCmd = Eigen::Vector3d(0, -mThrusterForce, -mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "01000001") // +x, +y, +phi
        {
            mThrustCmd = Eigen::Vector3d(mThrusterForce, mThrusterForce, mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "00000101") // +x, -y, +phi
        {
            mThrustCmd = Eigen::Vector3d(mThrusterForce, -mThrusterForce, mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "01010000") // -x, +y, +phi
        {
            mThrustCmd = Eigen::Vector3d(-mThrusterForce, mThrusterForce, mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "00010100") // -x, -y, +phi
        {
            mThrustCmd = Eigen::Vector3d(-mThrusterForce, -mThrusterForce, mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "10000010") // +x, +y, -phi
        {
            mThrustCmd = Eigen::Vector3d(mThrusterForce, mThrusterForce, -mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "00001010") // +x, -y, -phi
        {
            mThrustCmd = Eigen::Vector3d(mThrusterForce, -mThrusterForce, -mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "10100000") // -x, +y, -phi
        {
            mThrustCmd = Eigen::Vector3d(-mThrusterForce, mThrusterForce, -mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "00101000") // -x, -y, -phi
        {
            mThrustCmd = Eigen::Vector3d(-mThrusterForce, -mThrusterForce, -mThrusterForce*mMomentArm);
        } 
        else if (thrusterCommand == "00000000") // all off
        {
            mThrustCmd = Eigen::Vector3d::Zero();
        }
        else 
        {
            mThrustCmd = Eigen::Vector3d::Zero();
        }

    }
    
private:
    std::unique_ptr<UdpServer> mUdpServer; 
    Eigen::Vector3d mThrustCmd; 
    double mThrusterForce; 
    double mMomentArm;
   
};
#endif //ABVUDPINPUTFETCHER_H