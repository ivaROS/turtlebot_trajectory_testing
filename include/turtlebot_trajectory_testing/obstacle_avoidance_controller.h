
#ifndef TURTLEBOT_OBSTACLE_AVOIDANCE_CONTROLLER_H_
#define TURTLEBOT_OBSTACLE_AVOIDANCE_CONTROLLER_H_

#include <pips_trajectory_testing/obstacle_avoidance_controller.h>
#include <turtlebot_trajectory_testing/turtlebot_trajectory_tester.h>
#include <turtlebot_trajectory_controller/trajectory_controller.h>

#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/BumperEvent.h>


namespace turtlebot_trajectory_testing
{

  typedef pips_trajectory_testing::ObstacleAvoidanceController<turtlebot_trajectory_controller::TrajectoryController, TurtlebotGenAndTest> controller_type;

class TurtlebotObstacleAvoidanceController : public controller_type
{
  
public:
  typedef controller_type Controller;
  
private:
  static constexpr const char* DEFAULT_NAME="ObstacleAvoidanceController";
  
  
public:
  TurtlebotObstacleAvoidanceController(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME);
  
//   virtual bool isReady(const std_msgs::Header& header);
  
  
protected:
  ros::Subscriber button_sub_, bumper_sub_;

  void buttonCB(const kobuki_msgs::ButtonEvent::ConstPtr& msg);
  void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg);

  virtual void setupPublishersSubscribers();
  
  
  std::vector<desired_traj_func::Ptr> getTrajectoryFunctions(unsigned int num_paths, double velocity);
  
  std::vector<desired_traj_func::Ptr> getTrajectoryFunctions(unsigned int num_paths, double velocity, double path_limits);
 
  std::vector<desired_traj_func::Ptr> getTrajectoryFunctions(const std::vector<double>& dep_angles, double velocity);
  
  std::vector<traj_func_ptr> getTrajectoryFunctions();
  

};

} //end namespace


#endif //TURTLEBOT_OBSTACLE_AVOIDANCE_CONTROLLER_H_
