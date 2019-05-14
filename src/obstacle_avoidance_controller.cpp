
/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <turtlebot_trajectory_testing/obstacle_avoidance_controller.h>
#include <turtlebot_trajectory_functions/angled_straight.h>


namespace turtlebot_trajectory_testing
{
  
  typedef TurtlebotObstacleAvoidanceController::traj_func_ptr traj_func_ptr;
  
  TurtlebotObstacleAvoidanceController::TurtlebotObstacleAvoidanceController(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name) : 
    TurtlebotObstacleAvoidanceController::Controller(nh, pnh)

  {
    
  };
  
/*  
  bool ObstacleAvoidanceController::isReady(const std_msgs::Header& header)
  {
    if(!curr_odom_)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(5 , name_,  "No odometry received!");
      return false;
    }
    else
    {
      ros::Duration delta_t = curr_odom_->header.stamp - header.stamp;
      ROS_DEBUG_STREAM_NAMED(name_, "Odometry is " << delta_t << " newer than current sensor data");
    }
    return true;
  }*/
  
  void TurtlebotObstacleAvoidanceController::setupPublishersSubscribers()
  {
    ROS_DEBUG_STREAM_NAMED(name_,  "Setting up publishers and subscribers");
    
    button_sub_ = nh_.subscribe("mobile_base/events/button", 10, &TurtlebotObstacleAvoidanceController::buttonCB, this);
    bumper_sub_ = nh_.subscribe("mobile_base/events/bumper", 10, &TurtlebotObstacleAvoidanceController::bumperCB, this);
    
    commanded_trajectory_publisher_ = turtlebot_trajectory_controller::TrajectoryController::pnh_.advertise< pips_trajectory_msgs::trajectory_points >("desired_trajectory", 1, true);
  }
  
  //Pressing button 0 activates wander mode
  void TurtlebotObstacleAvoidanceController::buttonCB(const kobuki_msgs::ButtonEvent::ConstPtr& msg)
  {
    if (msg->button == kobuki_msgs::ButtonEvent::Button0 && msg->state == kobuki_msgs::ButtonEvent::RELEASED )
    {
      wander_ = true;
      ROS_INFO_STREAM_NAMED(name_,  "Activating Wander");
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED(name_,  "Non-handled Button event");
    }
  };
  
  //Hitting the bumper deactivates wander mode
  void TurtlebotObstacleAvoidanceController::bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
      stop();
      if(wander_ == true)
      {
        wander_ = false;
        ROS_INFO_STREAM_NAMED(name_,  "Robot collided with obstacle! Deactivating Wander");
      }
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED(name_,  "Non-handled Bumper event");
    }
  };
  
  
  
  
  std::vector<desired_traj_func::Ptr> TurtlebotObstacleAvoidanceController::getTrajectoryFunctions(unsigned int num_paths, double velocity, double path_limits)
  {
    
    //Set trajectory departure angles and speed
    std::vector<double> dep_angles = {-path_limits/2,path_limits/2}; //,.6,.8,1,1.2,1.6,2,2.4};
    
    std::vector<desired_traj_func::Ptr> trajectory_functions(num_paths);
    
    for(size_t i = 0; i < num_paths; i++)
    {
      double dep_angle;
      if(num_paths == 1)
      {
        dep_angle = 0;
      }
      else
      {
        dep_angle = dep_angles[0] + i*(dep_angles[1] - dep_angles[0])/(num_paths - 1); 
      }
      trajectory_functions[i] = std::make_shared<turtlebot_trajectory_functions::AngledStraight>(dep_angle, velocity);
      
    }
    return trajectory_functions;
  }
  
  std::vector<desired_traj_func::Ptr> TurtlebotObstacleAvoidanceController::getTrajectoryFunctions(const std::vector<double>& dep_angles, double velocity)
  {
    unsigned int num_paths = dep_angles.size();
    
    std::vector<desired_traj_func::Ptr> trajectory_functions(num_paths);
    
    for(size_t i = 0; i < num_paths; i++)
    {
      trajectory_functions[i] = std::make_shared<turtlebot_trajectory_functions::AngledStraight>(dep_angles[i], velocity);
    }
    return trajectory_functions;
  }
  
  std::vector<traj_func_ptr> TurtlebotObstacleAvoidanceController::getTrajectoryFunctions()
  {
    std::vector<desired_traj_func::Ptr> funcs = TurtlebotObstacleAvoidanceController::getTrajectoryFunctions(num_paths_, v_des_, path_limits_);
    
    
    
    std::vector<traj_func_ptr> trajs(funcs.size());
    
    double v_max=.5;
    double w_max=4;
    double a_max=.55;
    double w_dot_max=1.78;
    
    near_identity ni(1,5,1,.01,v_max,w_max,a_max,w_dot_max);    
    
    
    for(size_t i = 0; i < funcs.size(); i++)
    {
      traj_func_ptr traj = std::make_shared<TurtlebotObstacleAvoidanceController::traj_func_type>(ni);
      traj->setTrajFunc(funcs[i]);

      trajs[i] = traj;
    }
    
    return trajs;
    
  }
  
  




} // namespace kobuki
// %EndTag(FULLTEXT)%

