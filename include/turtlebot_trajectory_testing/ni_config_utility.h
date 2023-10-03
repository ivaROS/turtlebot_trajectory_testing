#ifndef NI_CONFIG_UTILITY_H
#define NI_CONFIG_UTILITY_H

#include <turtlebot_trajectory_generator/near_identity.h>
#include <turtlebot_trajectory_testing/NIConfigUtilityConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>


namespace turtlebot_trajectory_testing
{
  class NIConfigUtility
  {
    typedef turtlebot_trajectory_generator::near_identity near_identity;
    typedef turtlebot_trajectory_generator::desired_traj_func desired_traj_func;
    typedef turtlebot_trajectory_generator::traj_func_type traj_func_type;
    typedef turtlebot_trajectory_generator::ni_params ni_params;

    typedef boost::shared_mutex Mutex;
    typedef boost::unique_lock< Mutex > WriteLock;
    typedef boost::shared_lock< Mutex > ReadLock;
    
    ni_params params;
    
    typedef dynamic_reconfigure::Server<turtlebot_trajectory_testing::NIConfigUtilityConfig> ReconfigureServer;
    std::shared_ptr<ReconfigureServer> reconfigure_server_;
    Mutex config_mutex_; // Allows simultaneous read operations; prevents anything else from happening while writing 
    
  public:
    traj_func_type::Ptr getTrajFunc(const desired_traj_func::Ptr& des_traj)
    {
      near_identity ni(getParams());
      
      traj_func_type::Ptr traj = std::make_shared<traj_func_type>(ni);
      traj->setTrajFunc(des_traj);
      return traj;
    }
    
    std::vector<traj_func_type::Ptr> getTrajFunc(const std::vector<desired_traj_func::Ptr>& des_trajs)
    {
      near_identity ni(getParams());
      
      int num_trajs = des_trajs.size();
      std::vector<traj_func_type::Ptr> trajs(num_trajs);
      for(int i = 0; i < num_trajs; ++i)
      {
        traj_func_type::Ptr traj = std::make_shared<traj_func_type>(ni);
        traj->setTrajFunc(des_trajs[i]);
        trajs[i]=traj;
      }
      return trajs;
    }
    
    NIConfigUtility() : 
      params(1,5,1,.01,.5,4,.55,1.78) //initialize with reasonable defaults
    {
    }
    
    void setParams(ni_params new_params)
    {
      WriteLock lock(config_mutex_);
      params = new_params;
    }
    
    ni_params getParams()
    {
      ReadLock lock(config_mutex_);
      return params;
    }
    
    void init(ros::NodeHandle pnh=ros::NodeHandle("~"))
    {
      reconfigure_server_=std::make_shared<ReconfigureServer>(pnh);
      reconfigure_server_->setCallback(boost::bind(&NIConfigUtility::configCB, this, _1, _2));
    }
    
    void configCB(turtlebot_trajectory_testing::NIConfigUtilityConfig &config, uint32_t level)
    {
      ni_params p(config.c_p, config.c_d, config.c_lambda, config.epsilon, config.v_max, config.w_max, config.a_max, config.w_dot_max);
      setParams(p);
    }
  };
}

#endif // NI_CONFIG_UTILITY_H