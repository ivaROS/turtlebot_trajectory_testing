#ifndef TURTLEBOT_TRAJECTORY_TESTER
#define TURTLEBOT_TRAJECTORY_TESTER

#include <pips_trajectory_testing/pips_trajectory_tester.h>
#include <turtlebot_trajectory_generator/near_identity.h>

extern template class GenAndTest<ni_state, ni_controller>;

typedef GenAndTest<ni_state, ni_controller> TurtlebotGenAndTest;

namespace turtlebot_trajectory_testing
{
  typedef TurtlebotGenAndTest GenAndTest;
  typedef ni_state state_type;
  typedef TurtlebotGenAndTest::traj_func_type traj_func_type;
  typedef traj_func_type::Ptr traj_func_ptr;
  typedef TurtlebotGenAndTest::trajectory_ptr trajectory_ptr;
  typedef TurtlebotGenAndTest::pips_trajectory_ptr pips_trajectory_ptr;
  
  typedef desired_traj_func::Ptr des_traj_func_ptr;
}


#endif //TURTLEBOT_TRAJECTORY_TESTER
