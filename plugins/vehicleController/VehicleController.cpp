
#include "VehicleController.h"

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
}

void VehicleController::PreUpdate(const ignition::gazebo::UpdateInfo&, ignition::gazebo::EntityComponentManager &ecm)
{
    if(mNavRecvd && mCmdRecvd && !mControlLoopLaunched)
    {
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
    while(isRunning())
    {
        mControlRate->start(); 

        // do control
        auto cmd = getLatestCmd(); 
        auto nav = getLatestNav(); 

        mControlRate->block(); 
    }

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
