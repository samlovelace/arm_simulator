
#include "VehicleController.h"
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

void VehicleController::Configure(const ignition::gazebo::Entity &entity,
					              const std::shared_ptr<const sdf::Element> &anSdf,
					              ignition::gazebo::EntityComponentManager &ecm,
					              ignition::gazebo::EventManager &)
{
    mNavRecvd = false; 
    mCmdRecvd = false; 
    mControlLoopLaunched = false; 

    setRunning(false); 

    mModel = ignition::gazebo::Model(entity); 
    if(!mModel.Valid(ecm))
    {
        std::cerr << "Invalid model entity" << std::endl; 
        return; 
    }    

    auto ctx = rclcpp::contexts::get_global_default_context(); 

    if(!ctx->is_valid())
    {
        rclcpp::init(0, nullptr); 
    }

    mRosNode = rclcpp::Node::make_shared("vehicle_controller"); 
    mCmdSub = mRosNode->create_subscription<robot_idl::msg::VehicleWaypoint>("/vehicle/waypoint", 10, 
                                                                             std::bind(&VehicleController::commandCallback, 
                                                                                       this, 
                                                                                       std::placeholders::_1));
    mNavSub = mRosNode->create_subscription<robot_idl::msg::RobotState>("/robot/pose", 10, 
                                                                         std::bind(&VehicleController::navCallback, 
                                                                                    this, 
                                                                                    std::placeholders::_1));

    mRosSpinThread = std::thread([this](){
        rclcpp::spin(mRosNode); 
    }); 

    // initial joint pos here  
    mPrevPosErr = {0, 0, 0}; 
    mKp = {1000, 5000, 1000};  
    mKd = {100, 100, 100};  
    mPrevTime = std::chrono::steady_clock::now();
    
    // TODO: make config 
    mControlRate = std::make_unique<RateController>(10); 

    mCmdVelPub = mNode.Advertise<ignition::msgs::Twist>("/cmd_vel"); 

}

void VehicleController::PreUpdate(const ignition::gazebo::UpdateInfo&, ignition::gazebo::EntityComponentManager &ecm)
{
    if(mNavRecvd && mCmdRecvd && !mControlLoopLaunched)
    {
        setRunning(true); 

        mControlThread = std::thread([this](){
            controlLoop(); 
        }); 

        mControlLoopLaunched = true; 
    }

}

void VehicleController::setLatestNav(const robot_idl::msg::RobotState::SharedPtr aNavState)
{
    std::lock_guard<std::mutex> lock(mNavMutex); 
    mNav = aNavState; 
}

void VehicleController::setLatestCmd(const robot_idl::msg::VehicleWaypoint::SharedPtr aWaypoint)
{
    std::lock_guard<std::mutex> lock(mCmdMutex); 
    mCmd = aWaypoint; 
} 

robot_idl::msg::RobotState::SharedPtr VehicleController::getLatestNav()
{
    std::lock_guard<std::mutex> lock(mNavMutex); 
    return mNav; 
} 

robot_idl::msg::VehicleWaypoint::SharedPtr VehicleController::getLatestCmd()
{
    std::lock_guard<std::mutex> lock(mCmdMutex); 
    return mCmd; 
}

void VehicleController::commandCallback(robot_idl::msg::VehicleWaypoint::SharedPtr aMsg)
{
    if(!mCmdRecvd)
    {
        std::cout << "############### GOT CMD ###################" << std::endl; 
        mCmdRecvd = true; 
    }
    
    setLatestCmd(aMsg); 
}

void VehicleController::navCallback(robot_idl::msg::RobotState::SharedPtr aMsg)
{
    if(!mNavRecvd)
    {
        std::cout << "$$$$$$$$$$$$$$$$$$ GOT NAV $$$$$$$$$$$$$$$$$$$" << std::endl; 
        mNavRecvd = true; 
    }

    setLatestNav(aMsg); 
}

void VehicleController::controlLoop()
{
    // Go-to-pose gains (ensure k_alpha > k_r and k_beta < 0)
    const double k_r     = 8.0;   // distance gain
    const double k_alpha = 10.0;   // heading-to-goal gain
    const double k_beta  = -1.0;  // goal-heading (final yaw) gain

    // Limits & tolerances
    const double max_linear_vel   = 2.0;   // m/s
    const double max_angular_vel  = 2.0;   // rad/s
    const double position_tolerance = 0.01; // m
    const double yaw_tolerance      = 0.05; // rad

    // Slowdown distance so we don’t overshoot near goal
    const double slow_radius = 0.6; // m

    auto wrap = [](double a) {
        return std::atan2(std::sin(a), std::cos(a)); // (-pi, pi]
    };

    static int logCounter = 0;

    while (isRunning())
    {
        mControlRate->start();

        auto cmd = getLatestCmd();
        auto nav = getLatestNav();

        // Desired yaw from quaternion
        ignition::math::Quaterniond q(cmd->orientation.w,
                                      cmd->orientation.x,
                                      cmd->orientation.y,
                                      cmd->orientation.z);
        ignition::math::Vector3d goal_euler = q.Euler();
        const double yaw_goal = goal_euler.Z();

        // Errors in world frame
        const double dx = cmd->position.x - nav->position.x;
        const double dy = cmd->position.y - nav->position.y;
        const double r  = std::hypot(dx, dy);

        // Bearing from robot -> goal (world frame)
        const double bearing = std::atan2(dy, dx);

        // Heading errors
        const double alpha   = wrap(bearing - nav->euler.yaw); // how much to turn to face goal
        const double beta    = wrap(yaw_goal - bearing);       // how goal yaw differs from LOS
        const double yaw_err = wrap(yaw_goal - nav->euler.yaw);

        // If fully converged, hold still (controller keeps running)
        if (r < position_tolerance && std::abs(yaw_err) < yaw_tolerance)
        {
            publishTwistCmd(0.0, 0.0, 0.0);

            if (++logCounter % 20 == 0) {
                std::cout << "[Hold] Nav(x y yaw): " << nav->position.x << ", " << nav->position.y << ", " << nav->euler.yaw
                          << " | Cmd(x y yaw): " << cmd->position.x << ", " << cmd->position.y << ", " << yaw_goal
                          << " | r: " << r << " yaw_err: " << yaw_err << std::endl;
            }

            mControlRate->block();
            continue;
        }

        // Go-to-pose control law
        double v = k_r * r * std::cos(alpha);
        double w = k_alpha * alpha + k_beta * beta;

        // Smooth approach near goal
        const double slow = std::clamp(r / slow_radius, 0.0, 1.0);
        v *= slow;

        // (Optional) forward-only: uncomment to avoid reversing
        // if (v < 0.0) v = 0.0;

        // Clamp to limits
        v = std::clamp(v, -max_linear_vel,  max_linear_vel);
        w = std::clamp(w, -max_angular_vel, max_angular_vel);

        publishTwistCmd(v, 0.0, w);

        // Log periodically or when far
        if (++logCounter % 10 == 0 || r > 0.5)
        {
            std::cout << "Nav (x y yaw): " << nav->position.x << ", " << nav->position.y << ", " << nav->euler.yaw
                      << " | Cmd (x y yaw): " << cmd->position.x << ", " << cmd->position.y << ", " << yaw_goal
                      << " | r: " << r << " alpha: " << alpha << " beta: " << beta
                      << " | v: " << v << " w: " << w << std::endl;
        }

        mControlRate->block();
    }
}

void VehicleController::publishTwistCmd(double x, double y, double h)
{
    ignition::msgs::Twist twistMsg;
    twistMsg.mutable_linear()->set_x(x);   // Forward velocity
    twistMsg.mutable_linear()->set_y(0);
    twistMsg.mutable_linear()->set_z(0.0);
    twistMsg.mutable_angular()->set_x(0.0);
    twistMsg.mutable_angular()->set_y(0.0);
    twistMsg.mutable_angular()->set_z(h);  // Yaw rotation

    mCmdVelPub.Publish(twistMsg);
}

VehicleController::~VehicleController()
{
    setRunning(false); 
    if(mRosSpinThread.joinable())
    {
        mRosSpinThread.join();
    }
    if(mControlThread.joinable())
    {
        mControlThread.join(); 
    }
         
	mRosNode = nullptr; 
	
	if(rclcpp::ok())
	{
		rclcpp::shutdown(); 
	}
	
	while(rclcpp::ok())
	{
		std::cout << "shutting down ROS2 plugin" << std::endl; 
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));  
	}
}
