
#ifndef SCORBOT_HW_INTERFACE_H
#define SCORBOT_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <ros/ros.h>
#include <scorbot_control/arm_command.h>
#include <scorbot_control/scorbot_telemetry.h>

#define DEG_TO_RAD .01745329251
#define RAD_TO_DEG 57.295795131

namespace scorbot_namespace
{
         /** \brie(f) Hardware interface for a robot */
class ScorbotHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
       /**
        * \brie(f) Constructor
        * \ - Node handle for topics.
       */
    ScorbotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

        /** \brie(f) Initialize the robot hardware interface */
    virtual void init();

       /** \brie(f) Read the state from the robot hardware. */
    virtual void read(ros::Duration& elapsed_time);

       /** \brie(f) Write the command to the robot hardware. */
    virtual void write(ros::Duration& elapsed_time);

        /** \breif Enforce limits for all values before writing */
    virtual void enforceLimits(ros::Duration& period);
 
protected:
  
   ros::Subscriber telemetry_sub;
   void telemetryCallback(const scorbot_control::scorbot_telemetry::ConstPtr &msg); 

   ros::Publisher cmd_pub;

   
};        // class

}                                 // namespace ros_control_boilerplate
   
#endif