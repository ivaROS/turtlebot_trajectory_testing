
#include <turtlebot_trajectory_testing/depth_controller.h>
#include <ros/ros.h>


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

