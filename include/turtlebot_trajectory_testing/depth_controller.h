
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef TURTLEBOT_DEPTH_CONTROLLER_H_
#define TURTLEBOT_DEPTH_CONTROLLER_H_

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <turtlebot_trajectory_testing/obstacle_avoidance_controller.h>

#include <pips_trajectory_testing/pips_cc_wrapper.h>


//#include <dynamic_reconfigure/server.h>

#include <memory>


namespace turtlebot_trajectory_testing
{




/**
 * @ brief 
 *
 * A simple nodelet-based controller intended to avoid obstacles using PIPS.
 */
class DepthController : public TurtlebotObstacleAvoidanceController
{
public:
  DepthController(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME);
  ~DepthController(){};

  virtual bool init();
  
  static constexpr const char* DEFAULT_NAME="DepthController";


protected:
  bool isReady(const std_msgs::Header& header);
  
  void sensorCb(const std_msgs::Header& header);
  
  void generateTrajectories();
  
  virtual void setupTrajectoryTesters();
 


  
private:
  std::string name_;
  ros::NodeHandle pnh_;
  
  std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_;

};

} //ns kobuki

#endif /* TURTLEBOT_DEPTH_CONTROLLER_H_ */

