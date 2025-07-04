
#include "StereoCameraPublisher.hpp"
#include <ignition/common/Console.hh>
#include <thread> 
#include <chrono> 
#include <vector> 

StereoCameraPublisher::~StereoCameraPublisher()
{

}

void StereoCameraPublisher::Configure(const ignition::gazebo::Entity &entity,
                                         const std::shared_ptr<const sdf::Element> &anSdf,
                                         ignition::gazebo::EntityComponentManager &ecm,
                                         ignition::gazebo::EventManager &)
{
  mModel = ignition::gazebo::Model(entity);
  if (!mModel.Valid(ecm))
  {
    std::cerr << " #################### invalid model entity. #####################" << std::endl;
    return;
  }
}

void StereoCameraPublisher::PreUpdate(const ignition::gazebo::UpdateInfo &,
					ignition::gazebo::EntityComponentManager &ecm)
{


}
