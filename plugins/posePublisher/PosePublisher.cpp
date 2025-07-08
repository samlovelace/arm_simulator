
#include "PosePublisher.h"
#include <ignition/common/Console.hh>
#include <thread> 
#include <chrono> 
#include <vector> 


void PosePublisher::Configure(const ignition::gazebo::Entity &entity,
                                         const std::shared_ptr<const sdf::Element> &anSdf,
                                         ignition::gazebo::EntityComponentManager &ecm,
                                         ignition::gazebo::EventManager &)
{
	mModel = ignition::gazebo::Model(entity);
	if (!mModel.Valid(ecm))
	{
		std::cerr << "invalid model entity." << std::endl;
		return;
	}
	
	std::string topicName = "/robot/state";
	if(anSdf->HasElement("topic_name"))
	{
		topicName = anSdf->Get<std::string>("topic_name"); 
	}

	int rate = 10; 
	if(anSdf->HasElement("rate"))
	{
		rate = anSdf->Get<int>("rate"); 
	}

	ignmsg << "Configured to publish " << mModel.Name(ecm) << " state on " << topicName << " at " << rate << "hz" << std::endl; 

}

void PosePublisher::PostUpdate(const ignition::gazebo::UpdateInfo&, const ignition::gazebo::EntityComponentManager &ecm)
{
  auto now = std::chrono::steady_clock::now();
}

PosePublisher::~PosePublisher()
{
  if(mRosSpinThread.joinable())
  {
    mRosSpinThread.join(); 
  }

  if(mPublishThread.joinable())
  {
    mPublishThread.join(); 
  }

  mRosNode = nullptr; 
  rclcpp::shutdown(); 

  while(rclcpp::ok())
  {
    std::cout << "shutting down ROS2 plugin" << std::endl; 
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));  
  }
}
