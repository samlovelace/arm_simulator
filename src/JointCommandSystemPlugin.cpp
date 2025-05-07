#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <thread>

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
                 ignition::gazebo::EventManager &) override
  {
    this->model = ignition::gazebo::Model(entity);
    if (!this->model.Valid(ecm))
    {
      std::cerr << "#################### Invalid model entity. #####################" << std::endl;
      return;
    }

    rclcpp::init(0, nullptr);
    this->rosNode = rclcpp::Node::make_shared("joint_command_node");

    this->sub = this->rosNode->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/joint_commands", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
          this->jointCommands = msg->data;
        });

    this->rosSpinThread = std::thread([this]()
                                      { rclcpp::spin(this->rosNode); });
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo &,
                 ignition::gazebo::EntityComponentManager &ecm) override
  { 
    if (this->jointCommands.empty())
      return;

    auto joints = this->model.Joints(ecm);
    for (size_t i = 0; i < joints.size() && i < this->jointCommands.size(); ++i)
    {  
      auto posComp = ecm.Component<ignition::gazebo::components::JointPosition>(joints[i]);
      if (!posComp)
      {
        ecm.CreateComponent(joints[i],
                            ignition::gazebo::components::JointPosition({this->jointCommands[i]}));
      }
      else
      {
        posComp->Data()[0] = this->jointCommands[i];
      }
    }
  }

  ~JointCommandSystemPlugin() override
  {
    if (this->rosSpinThread.joinable())
    {
      rclcpp::shutdown();
      this->rosSpinThread.join();
    }
  }

private:
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
  std::shared_ptr<rclcpp::Node> rosNode;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub;
  std::vector<double> jointCommands;
  std::thread rosSpinThread;
};

// Plugin registration
IGNITION_ADD_PLUGIN(
    JointCommandSystemPlugin,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate)
