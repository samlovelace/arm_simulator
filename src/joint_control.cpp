#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <vector>

namespace gazebo
{
  class JointForcePlugin : public ModelPlugin
  {
    public:
      // Constructor to initialize rclcpp node
      JointForcePlugin()
        : ModelPlugin(), node(nullptr)
      {
        // Initialize ROS 2 node
        rclcpp::init(0, nullptr);  // Initialize rclcpp (you should typically pass arguments from main())
        node = rclcpp::Node::make_shared("joint_force_plugin_node");

        // Create publisher for joint forces
        joint_force_pub = node->create_publisher<std_msgs::msg::Float64>("/joint_forces", 10);
      }

      // Called when the plugin is loaded
      void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
      {
        this->model = model;

        // Retrieve all joints of the model
        joint_names = model->GetJoints();
        RCLCPP_INFO(node->get_logger(), "Found %zu joints in the model.", joint_names.size());

        // Connect to the update event
        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&JointForcePlugin::OnUpdate, this));

        // Initialize force values
        force_values.resize(joint_names.size(), 0.0);
      }

      // Called every simulation update
      void OnUpdate()
      {
        // Apply forces to each joint
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
          physics::JointPtr joint = joint_names[i];
          joint->SetForce(100, force_values[i]); // Apply force to the first axis (typically revolute joints)

          // Optionally, publish force values
          std_msgs::msg::Float64 msg;
          msg.data = force_values[i];
          joint_force_pub->publish(msg);
        }

        // Spin the rclcpp node to handle any incoming callbacks
        rclcpp::spin_some(node);
      }

      // Set the force on a specific joint
      void SetForce(size_t joint_index, double force)
      {
        if (joint_index < force_values.size())
        {
          force_values[joint_index] = force;
        }
      }

    private:
      physics::ModelPtr model;
      event::ConnectionPtr updateConnection;
      std::vector<physics::JointPtr> joint_names;
      std::vector<double> force_values;

      // ROS 2 publisher for joint forces
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint_force_pub;
      rclcpp::Node::SharedPtr node;
  };

  // Register the plugin with Gazebo
  GZ_REGISTER_MODEL_PLUGIN(JointForcePlugin)
}
