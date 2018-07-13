#include <turtlebot_trajectory_testing/depth_controller.h>
#include <pips_trajectory_testing/depth_image_cc_wrapper.h>


namespace turtlebot_trajectory_testing
{
  
  DepthController::DepthController(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name) :
    TurtlebotObstacleAvoidanceController(nh, pnh, "obstacle_avoidance"), 
    name_(name),
    pnh_(pnh)
  {

      
  }

  
  void DepthController::setupTrajectoryTesters()
  {
        traj_tester_ = std::make_shared<TurtlebotGenAndTest>(nh_, TurtlebotObstacleAvoidanceController::pnh_);
        traj_tester_->init();
        traj_tester2_ = traj_tester_;
        
        cc_wrapper_ = std::make_shared<pips_trajectory_testing::DepthImageCCWrapper>(nh_, pnh_,tfBuffer_);
        traj_tester_->setCollisionChecker(cc_wrapper_->getCC());
        
        cc_wrapper_->setBaseFrame(base_frame_id_);
  }
  
     



  // Note: I haven't fully thought through other implementations, but this may be generic after all...
  // If so, then this code will probably move back to the main controller but be renamed 'transformReady' or something
  bool DepthController::isReady(const std_msgs::Header& header)
  {
    if(!TurtlebotObstacleAvoidanceController::isReady(header))
    {
      return false;
    }

    else
    {
        if(cc_wrapper_->isReady(header))
        {
            cc_wrapper_->update();
            return true;
        }
        else
        {
            return false;
        }
    }
  }
  
  void DepthController::generateTrajectories()
  {
      std_msgs::Header header = cc_wrapper_->getCurrentHeader();
      TurtlebotObstacleAvoidanceController::sensorCb(header);
  }
  
  bool DepthController::init()
  {
    TurtlebotObstacleAvoidanceController::init();
    
    cc_wrapper_->init();
    cc_wrapper_->setCallback(boost::bind(&DepthController::generateTrajectories, this));
    
    return true;
  }
  
}
