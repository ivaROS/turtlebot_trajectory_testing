
  #include <turtlebot_trajectory_testing/depth_controller.h>
  #include <ros/ros.h>
  
namespace turtlebot_trajectory_testing
{
  
  class EnhancedDepthController : public DepthController
  {
  public:

  protected:

    virtual typename TrajBridge::trajectory_ptr findPath(nav_msgs::Odometry::ConstPtr odom )
    {
      auto longest_traj = DepthController::findPath(odom);

      if(longest_traj && longest_traj->getDuration() > min_ttc_)
      {
        return longest_traj;
      }
      else
      {
        
      }
    }
  
    virtual typename TrajBridge::trajectory_ptr rotateInPlace(nav_msgs::Odometry::ConstPtr odom)
    {
      double v = odom->twist.twist.linear.x;
      double w = odom->twist.twist.angular.z;
      
      double des_w = 1;
      double delta_t = 0.1;
      
      auto trajectory = std::make_shared<GenAndTest::TrajectoryStates>();
      
      decltype(trajectory->x0_) x0(*odom);
      
      //ni_state state;
      decltype(trajectory->x0_) state;
      
      double min_ttc = min_ttc_.toSec();
      
      state.w=des_w;
      for(double t=0; t<= min_ttc; t+=delta_t)
      {
        trajectory->x_vec.push_back(state);
        trajectory->times.push_back(t);
        
        state.theta += delta_t*des_w;
      }
      
    }
  };
  
} //end namespace turtlebot_trajectory_testing
  

  
  int main(int argc, char **argv)
  {
    std::string name= "depth_controller";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    turtlebot_trajectory_testing::DepthController controller(nh, pnh);
    controller.init();
    
    
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
    
    //ros::shutdown();
    
    return 0;
  }
  
  
  
