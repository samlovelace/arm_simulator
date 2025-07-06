
#include "PosePublisher.hpp"
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/WorldPose.hh>
#include <ignition/math/Pose3.hh>

PosePublisher::~PosePublisher()
{

}

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

    std::string topicName = "/robot/pose"; 
    if(anSdf->HasElement("topic_name"))
    {
        topicName = anSdf->Get<std::string>("topic_name"); 
    }

    int publishRate = 10; 
    if(anSdf->HasElement("rate"))
    {
        publishRate = anSdf->Get<int>("rate"); 
    }

    // rclcpp::init(0, nullptr); 
    // mRosNode = rclcpp::Node::make_shared("pose_publisher");
    // mPosPub = mRosNode->create_publisher<nora_idl::msg::RobotState>(topicName, 10); 

    // mRosSpinThread = std::thread([this](){
    //     rclcpp::spin(mRosNode); 
    // }); 

    // mPublishRate = std::make_unique<RateController>(publishRate); 
  
    // mPublishThread = std::thread([&](){
    //     robotStatePublishLoop(ecm); 
    // });

}

void PosePublisher::PostUpdate(const ignition::gazebo::UpdateInfo &,
                               const ignition::gazebo::EntityComponentManager &ecm)
{
    auto worldPoseComp =
        ecm.Component<components::WorldPose>(this->modelEntity);

    if (worldPoseComp)
    {
    auto pose = worldPoseComp->Data();
    std::cout << "World pose: " << pose << std::endl;
    }

}

void PosePublisher::robotStatePublishLoop(ignition::gazebo::EntityComponentManager& ecm)
{



}

