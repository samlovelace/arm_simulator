#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Joint.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <thread>
#include <memory> 
#include <mutex> 
#include "RateController.h"

class JointCommandSystemPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
public:
  JointCommandSystemPlugin() = default;

  void Configure(const ignition::gazebo::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &,
                 ignition::gazebo::EntityComponentManager &ecm,
                 ignition::gazebo::EventManager &) override;

  void PreUpdate(const ignition::gazebo::UpdateInfo &,
                 ignition::gazebo::EntityComponentManager &ecm) override;

  ~JointCommandSystemPlugin() override;

private:
  std::string jointTypeToString(const sdf::JointType& aType);
  void commandCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg);
  std::vector<double> getLatestJointCommand();
  bool getJointPositions(ignition::gazebo::EntityComponentManager &ecm, std::vector<double>& aJointVecOut);
  void jointPositionPublishLoop(ignition::gazebo::EntityComponentManager &ecm);

  ignition::gazebo::Model mModel{ignition::gazebo::kNullEntity};
  std::shared_ptr<rclcpp::Node> mRosNode;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mCmdSub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mPosPub; 
  std::vector<double> mJointCommands;
  std::thread mRosSpinThread;
  std::thread mPublishThread; 
  std::unique_ptr<RateController> mPublishRate; 


  std::vector<std::shared_ptr<ignition::gazebo::Joint>> mJoints; 
  std::vector<double> mKp; 
  std::vector<double> mKd; 
  std::vector<double> mPrevPosErr;
  std::chrono::time_point<std::chrono::steady_clock> mPrevTime;  

  std::mutex mCommandMutex; 
};

// Plugin registration
IGNITION_ADD_PLUGIN(
    JointCommandSystemPlugin,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate)
