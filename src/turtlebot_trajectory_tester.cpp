#include <turtlebot_trajectory_testing/turtlebot_trajectory_tester.h>

//The purpose of this file is to ensure that a compilation unit exists for the object so it doesn't have to be rebuilt from scratch each time
template class GenAndTest<turtlebot_trajectory_generator::ni_state, turtlebot_trajectory_generator::ni_controller>;
