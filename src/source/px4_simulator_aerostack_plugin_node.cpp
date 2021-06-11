#include "px4_simulator_aerostack_plugin.hpp"
#include "ros/ros.h"
#include "ros_utils_lib/ros_utils.hpp"

int main(int argc, char **argv)
{

  ros::init(argc, argv, ros_utils_lib::getNodeName("px4_simulation_aerostack"));
  std::cout << "Node starting "<< std::endl;
  Px4AerostackPlugin px4_aerostack_interface;
  px4_aerostack_interface.setUp();
  px4_aerostack_interface.start();
  ros::Rate r(250);
  while (ros::ok())
  {
    px4_aerostack_interface.run();
    ros::spinOnce();
    r.sleep();
  }
  px4_aerostack_interface.stop();
  
  return 0;
}
