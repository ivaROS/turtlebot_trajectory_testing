/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <turtlebot_trajectory_testing/turtlebot_trajectory_tester.h>
#include <memory>

#include <path_smoothing_ros/cubic_spline_interpolator.h>
#include <tf/tf.h>


namespace kobuki
{

/**
 * @ brief Sends a simple trajectory
 *
 * A simple program that sends a trajectory to the controller when a button is pressed.
 */
 


/* The rhs of x' = f(x) defined as a class */
class circle_traj_func : public desired_traj_func{
    double vf_; //Forward vel
    double r_;  //radius of circle

public:
    circle_traj_func( double vf, double r) : vf_(vf), r_(r) { }
    

    void dState ( const ni_state &x , ni_state &dxdt , const double  t  )
    {
        
        dxdt[ni_state::YD_IND] = vf_*sin( - (vf_/r_) * t );
        dxdt[ni_state::XD_IND] = vf_*cos( - (vf_/r_) * t );
    }
    
    
};


/* The rhs of x' = f(x) defined as a class */
class sin_traj_func : public desired_traj_func{
  double vf_; //Forward vel
  double mag_;
  double period_;
  
public:
  sin_traj_func( double vf, double mag, double period) : vf_(vf), mag_(mag), period_(period) { }
  
  
  void dState ( const ni_state &x , ni_state &dxdt , const double  t  )
  {
    double dx = std::sqrt(vf_*vf_/(1+mag_*mag_*period_*period_*std::cos(period_*x.xd-period_/2)*std::cos(period_*x.xd-period_/2)));
    dxdt[ni_state::YD_IND] = mag_*period_*std::cos(period_*x[ni_state::XD_IND]-period_/2) * dx;
    dxdt[ni_state::XD_IND] = dx;
    
    dxdt.xd = dx;
  }
  
  
};
 


/* The rhs of x' = f(x) defined as a class */
class spline_traj_func : public desired_traj_func{
  
  double vf_; //Forward vel
  double ang_vel_lim_; // Angular vel lim / upper bound
  
  nav_msgs::Path path_, path_smoothed_;
  
  std::vector<double> cummul_distances_;
  
  int st_idx;
  
public:
  spline_traj_func( double vf, double ang_vel_lim, std::string waypoints_yaml_) {
    
    //
  int pointsPerUnit, skipPoints;
  bool useEndConditions, useMiddleConditions;
  
  pointsPerUnit = 10; // 100;
  skipPoints = 0;
  useEndConditions = true;
  useMiddleConditions = true;
  
    // load waypoints from waypoints_yaml_
  YAML::Node wpoints_ = YAML::LoadFile(waypoints_yaml_);
/*  
  std::cout << "done with loading yaml!" << std::endl;
  std::cout << "number of waypoints = " << wpoints_["path_poses"].size() << std::endl;
  std::cout << "try visiting each waypoint " << wpoints_["path_poses"][0] << std::endl 
   << wpoints_["path_poses"][0]["x"] << "; " 
   << wpoints_["path_poses"][0]["y"] << "; "
   << wpoints_["path_poses"][0]["yaw"] << "; " << std::endl;
  */

  path_.header.frame_id = "map";
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";

  for (int i = 0; i < wpoints_["path_poses"].size(); i++)
  {
    pose.pose.position.x = static_cast<double>((wpoints_["path_poses"][i]["x"]).as<double>());
    pose.pose.position.y = static_cast<double>((wpoints_["path_poses"][i]["y"]).as<double>());
    pose.pose.orientation = tf::createQuaternionMsgFromYaw((wpoints_["path_poses"][i]["yaw"]).as<double>() * 3.14159265359 / 180.0);
    path_.poses.push_back(pose);
  }
  
  // create a cubic spline interpolator
  // path_smoothing::CubicSplineInterpolator csi("cubic_spline");
  path_smoothing::CubicSplineInterpolator csi(pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
 
  std::cout << "size of input waypoints = " << path_.poses.size() << std::endl;
  csi.interpolatePath(path_, path_smoothed_);
  std::cout << "size of smoothed sample points = " << path_smoothed_.poses.size() << std::endl;
  
      // create cummulative distances vector
    cummul_distances_.clear();
    csi.calcCummulativeDistances(path_smoothed_.poses, cummul_distances_);
    
    double total_dist = csi.calcTotalDistance(path_smoothed_.poses);
    assert(total_dist > 0);
    
   for (int i=0; i<cummul_distances_.size(); ++i) {
     cummul_distances_[i] *= total_dist;
  }
    
  assert(path_smoothed_.poses.size() == cummul_distances_.size());

  /*
    std::cout << "CummulativeDistances: " << std::endl;
  std::cout << cummul_distances_[1] << "; " ;
  std::cout << cummul_distances_[(cummul_distances_.size()-1)/2] << "; " ;
  std::cout << cummul_distances_[cummul_distances_.size()-1] << "; " ;
  
  
    std::cout << "path_smoothed_: " << std::endl;
  std::cout << path_smoothed_.poses[1] << "; " ;
  std::cout << path_smoothed_.poses[(path_smoothed_.poses.size()-1)/2] << "; " ;
  std::cout << path_smoothed_.poses[path_smoothed_.poses.size()-1] << "; " ;
  */

    //
    vf_ = vf;
    assert(vf_ > 0);
    ang_vel_lim_ = ang_vel_lim;
    assert(ang_vel_lim_ > 0);
    
    st_idx = 0;
  }
  
  
  void dState ( const ni_state &x , ni_state &dxdt , const double  t  )
  {
 /*   double dx = std::sqrt(vf_*vf_/(1+mag_*mag_*period_*period_*std::cos(period_*x.xd-period_/2)*std::cos(period_*x.xd-period_/2)));
    dxdt[ni_state::YD_IND] = mag_*period_*std::cos(period_*x[ni_state::XD_IND]-period_/2) * dx;
    dxdt[ni_state::XD_IND] = dx;
    
    dxdt.xd = dx; */
 
 // std::cout << t << std::endl;
 
  // grab the nearest sample points as the reference
  // compute the gradient based on the reference point
  double dist_traveled =  t * vf_;
  int tar_idx;
  for (tar_idx=0; tar_idx<cummul_distances_.size(); ++tar_idx) {
     if (cummul_distances_[tar_idx] > dist_traveled)
	break ;
  }
  assert(tar_idx > 0);
  // std::cout << tar_idx << std::endl;
  // st_idx = tar_idx;
  
  if (tar_idx >= cummul_distances_.size()) {
    dxdt[ni_state::XD_IND] = 0;
    dxdt[ni_state::YD_IND] = 0;
  }
  else {

    double dx = path_smoothed_.poses[tar_idx].pose.position.x - x[ni_state::X_IND];
    double dy = path_smoothed_.poses[tar_idx].pose.position.y - x[ni_state::Y_IND];
//double dx = path_smoothed_.poses[tar_idx].pose.position.x - path_smoothed_.poses[tar_idx-1].pose.position.x;
//double dy = path_smoothed_.poses[tar_idx].pose.position.y - path_smoothed_.poses[tar_idx-1].pose.position.y;
    
    // TODO
    // chech the rotation rate from previous vel to current vel
    
double dnorm = std::sqrt(dx*dx + dy*dy);
  
  dxdt.xd = dx / dnorm * vf_;
  dxdt[ni_state::YD_IND] = dy / dnorm * vf_;
  }

  
  // dxdt.xd = dxdt[ni_state::XD_IND]; 
  
  //std::cout << dxdt[ni_state::XD_IND] << "; " << dxdt[ni_state::YD_IND] << std::endl;
    
  }
  
  
};

 
class TrajectoryTester 
{
public:
  TrajectoryTester(ros::NodeHandle& nh, std::string& name) : nh_(nh), pnh_("~"), name_(name), traj_tester_(nh_, pnh_)
  {
  }
  ~TrajectoryTester(){
  };

  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  bool init()
  {
    
    traj_tester_.init();
    //
   traj_tester_.setParam_TF(60.0);
    
    button_subscriber_ = nh_.subscribe("/mobile_base/events/button", 10, &TrajectoryTester::buttonCB, this);

    odom_subscriber_ = nh_.subscribe("visual/odom", 1, &TrajectoryTester::OdomCB, this);
    trajectory_publisher_ = nh_.advertise< pips_trajectory_msgs::trajectory_points >("/turtlebot_controller/trajectory_controller/desired_trajectory", 1000);
    path_publisher_ = nh_.advertise<nav_msgs::Path>("/desired_path", 1000);
    
    nav_msgs::OdometryPtr init_odom(new nav_msgs::Odometry);

    curOdom_ = init_odom;
    
    nh_.param<std::string>("/mobile_base/base_frame", base_frame_id_, "base_footprint");

    return true;
  };
  
  
//
  void desTrajPublish() {
    trajectory_publisher_.publish(trajectory_);
  };

  
private:
  ros::NodeHandle nh_, pnh_;
  std::string name_;
  std::string base_frame_id_;
  ros::Subscriber button_subscriber_, odom_subscriber_;
  ros::Publisher trajectory_publisher_, path_publisher_;
  nav_msgs::OdometryPtr curOdom_;
  TurtlebotGenAndTest traj_tester_;
  pips_trajectory_msgs::trajectory_points trajectory_;
  
  /**
   * @brief ROS logging output for enabling the controller
   * @param msg incoming topic message
   */
  void buttonCB(const kobuki_msgs::ButtonEventPtr& msg);

 
  /**
   * @brief Turns on/off a LED, when a bumper is pressed/released
   * @param msg incoming topic message
   */
  void OdomCB(const nav_msgs::OdometryPtr& msg);

  pips_trajectory_msgs::trajectory_points generate_trajectory(const nav_msgs::OdometryPtr& msg);
  //pips_trajectory_msgs::trajectory_points generate_straight_trajectory(const nav_msgs::OdometryPtr& msg);

};

void TrajectoryTester::buttonCB(const kobuki_msgs::ButtonEventPtr& msg)
{
  if (msg->button == kobuki_msgs::ButtonEvent::Button0 && msg->state == kobuki_msgs::ButtonEvent::RELEASED )
  {
    ROS_INFO_STREAM("Button pressed: sending trajectory");

    nav_msgs::OdometryPtr odom = nav_msgs::OdometryPtr(curOdom_);
    trajectory_ = TrajectoryTester::generate_trajectory(odom);
    
    trajectory_publisher_.publish(trajectory_);
  }
  else
  if (msg->button == kobuki_msgs::ButtonEvent::Button1 && msg->state == kobuki_msgs::ButtonEvent::RELEASED )
  {
    ROS_INFO_STREAM("Button pressed: sending trajectory");

    nav_msgs::OdometryPtr odom = nav_msgs::OdometryPtr(curOdom_);
    trajectory_ = TrajectoryTester::generate_trajectory(odom);
    
    trajectory_publisher_.publish(trajectory_);
  }
  else
  {
    ROS_INFO_STREAM("Button event");
  }
};


pips_trajectory_msgs::trajectory_points TrajectoryTester::generate_trajectory(const nav_msgs::OdometryPtr& odom_msg)
{
    std::string per_key, fw_vel_key, ang_vel_key, mag_key, path_key;
    double fw_vel = .5; // m/s
    double ang_vel_lim = 3.14; // rad/s
    double r = .5;

    double period = 3;
    double mag=1;
    
    std::string waypoint_yaml = "/home/yipuzhao/catkin_ws/src/turtlebot_trajectory_testing/config/path.yaml";
    
    if(ros::param::search("period", per_key))
    {
        ros::param::get(per_key, period); 
    }
    
    if(ros::param::search("fw_vel", fw_vel_key))
    {
        ros::param::get(fw_vel_key, fw_vel); 
    }
    
    if(ros::param::search("ang_vel_lim", ang_vel_key))
    {
        ros::param::get(ang_vel_key, ang_vel_lim); 
    }
    
    if(ros::param::search("mag", mag_key))
    {
      ros::param::get(mag_key, mag); 
    }
    
    if(ros::param::search("waypoint_yaml", path_key))
    {
        ros::param::get(path_key, waypoint_yaml); 
    }
    
    //circle_traj_func trajf(fw_vel,r);
    
    std::vector<TurtlebotGenAndTest::traj_func_ptr> trajectory_functions;
    // near_identity ni(1,5,1,.01);    
    near_identity ni(100,100,100,.01);   
    TurtlebotGenAndTest::traj_func_ptr traj = std::make_shared<TurtlebotGenAndTest::traj_func_type>(ni);
 //   desired_traj_func::Ptr des_traj = std::make_shared<circle_traj_func>(fw_vel,mag,period);
 //   desired_traj_func::Ptr des_traj = std::make_shared<sin_traj_func>(fw_vel,mag,period);
    desired_traj_func::Ptr des_traj = std::make_shared<spline_traj_func>(fw_vel, ang_vel_lim, waypoint_yaml);
    traj->setTrajFunc(des_traj);
    
    trajectory_functions.push_back(traj);
    
    // auto valid_trajs = traj_tester_.run(trajectory_functions, odom_msg);
    // traj_tester_.setParam_TF(40.0);
    auto valid_trajs = traj_tester_.run(trajectory_functions, odom_msg);

    pips_trajectory_msgs::trajectory_points trajectory_msg = valid_trajs[0]->toMsg();
    
    auto path = valid_trajs[0]->toPathMsg();
    path->header.frame_id="odom";
    
    path_publisher_.publish(path);
    
    
    return trajectory_msg;
}



void TrajectoryTester::OdomCB(const nav_msgs::OdometryPtr& msg)
{

  curOdom_ = msg;
}

/*
void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

*/

} // namespace kobuki
// %EndTag(FULLTEXT)%


//http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_trajectory_sender"); //, ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    std::string name = ros::this_node::getName();
    kobuki::TrajectoryTester tester(nh,name);
    
    //signal(SIGINT, mySigintHandler);

    if (tester.init())
    {
      ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
      tester.desTrajPublish();
      ros::spinOnce();
      r.sleep();
    }
      //  ros::spin();
    }
    else
    {
        ROS_ERROR_STREAM("Couldn't initialise test_trajectory_sender!");
    }

    ROS_INFO_STREAM("Program exiting");
    return 0;

}

