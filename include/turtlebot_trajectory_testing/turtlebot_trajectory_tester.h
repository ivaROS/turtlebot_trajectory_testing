#ifndef TURTLEBOT_TRAJECTORY_TESTER
#define TURTLEBOT_TRAJECTORY_TESTER

#include <pips_trajectory_testing/pips_trajectory_tester.h>
#include <turtlebot_trajectory_generator/near_identity.h>

extern template class GenAndTest<ni_state, ni_controller>;

typedef GenAndTest<ni_state, ni_controller> TurtlebotGenAndTest;


#endif //TURTLEBOT_TRAJECTORY_TESTER
