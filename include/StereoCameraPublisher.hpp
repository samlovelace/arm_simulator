#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/Sensor.hh>

#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <memory> 
#include <mutex> 
#include "RateController.h"

class StereoCameraPublisher
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
public:
	StereoCameraPublisher() = default;
    ~StereoCameraPublisher() override; 

	void Configure(const ignition::gazebo::Entity &entity,
					const std::shared_ptr<const sdf::Element> &,
					ignition::gazebo::EntityComponentManager &ecm,
					ignition::gazebo::EventManager &) override;

	void PreUpdate(const ignition::gazebo::UpdateInfo &,
					ignition::gazebo::EntityComponentManager &ecm) override;

private:

	ignition::gazebo::Model mModel{ignition::gazebo::kNullEntity};
	std::shared_ptr<rclcpp::Node> mRosNode;
	//rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mPosPub; 
	std::thread mRosSpinThread;
	std::thread mPublishThread; 
	std::unique_ptr<RateController> mPublishRate; 

};

// Plugin registration
IGNITION_ADD_PLUGIN(
    StereoCameraPublisher,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate)
