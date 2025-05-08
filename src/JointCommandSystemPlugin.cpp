
#include "JointCommandSystemPlugin.hpp"
#include <ignition/common/Console.hh>

void JointCommandSystemPlugin::Configure(const ignition::gazebo::Entity &entity,
                const std::shared_ptr<const sdf::Element> &,
                ignition::gazebo::EntityComponentManager &ecm,
                ignition::gazebo::EventManager &)
{
  mModel = ignition::gazebo::Model(entity);
  if (!mModel.Valid(ecm))
  {
    std::cerr << "invalid model entity." << std::endl;

    return;
  }

  for(int i = 0; i < mModel.JointCount(ecm); i++)
  {
    auto jnt = std::make_shared<ignition::gazebo::Joint>(mModel.Joints(ecm)[i]); 

    if(sdf::JointType::FIXED == jnt->Type(ecm).value())
    {
      continue; 
    }

    ignmsg << "Sam found a " << jointTypeToString(jnt->Type(ecm).value()) << " joint with name: " << jnt->Name(ecm).value() << "\n"; 
    jnt->EnablePositionCheck(ecm, true);
    mJoints.push_back(jnt); 
  }

  mJointCommands = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57}; 
  mPrevPosErr = mJointCommands; 
  mKp = {1000, 1000, 1000, 1000, 500, 1000}; 
  mKd = {100, 100, 100, 100, 50, 100}; 
  mPrevTime = std::chrono::steady_clock::now(); 
}

void JointCommandSystemPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &,
                ignition::gazebo::EntityComponentManager &ecm)
{
  auto now = std::chrono::steady_clock::now();

  for(int i = 0; i < mJoints.size(); i++)
  {
    auto jnt = mJoints[i]; 
    if(jnt->Position(ecm).has_value())
    {
      auto posVec = jnt->Position(ecm).value();
      
      if(0 == posVec.size())
      {
        return; 
      }

      //ignmsg << "joint: " << jnt->Name(ecm).value() << " Pos: " << posVec[0] << "\n";
 
      auto dt = std::chrono::duration<double>(now - mPrevTime).count(); 

      double posErr = mJointCommands[i] - posVec[0]; 
      double errDeriv = (posErr - mPrevPosErr[i]) / dt; 

      double forceVal = mKp[i] * posErr + mKd[i] * errDeriv; 

      std::vector<double> force(1, forceVal); 
      jnt->SetForce(ecm, force); 
      mPrevPosErr[i] = posErr;

    }

  }
  mPrevTime = now;

}

std::string JointCommandSystemPlugin::jointTypeToString(const sdf::JointType& aType)
{
  switch (aType)
  {
  case sdf::JointType::REVOLUTE:
    return "Revolute";
  default:
    return "I'm to lazy to convert the other types right now"; 
  }
}


JointCommandSystemPlugin::~JointCommandSystemPlugin()
{

}
