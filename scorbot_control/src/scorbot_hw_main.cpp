
#include <ros/ros.h>

#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include "scorbot_control/scorbot_hw_interface.h"

int main(int argc, char** argv)
{
 ros::init(argc, argv, "scorbot_hw_interface");
 ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
 ros::AsyncSpinner spinner(3);
 spinner.start();

  // Create the hardware interface specific to your robot
 std::shared_ptr<scorbot_namespace::ScorbotHWInterface> scorbot_hw_interface(
    new scorbot_namespace::ScorbotHWInterface(nh));
 scorbot_hw_interface->init();

  // Start the control loop
 ros_control_boilerplate::GenericHWControlLoop control_loop(nh, scorbot_hw_interface);
 control_loop.run();  // Blocks until shutdown signal recieved

 return 0;
} 
