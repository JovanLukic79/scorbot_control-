#include <ros_control_boilerplate/generic_hw_interface.h>
#include "scorbot_control/scorbot_hw_interface.h"



namespace scorbot_namespace
{
ScorbotHWInterface::ScorbotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  telemetry_sub=nh.subscribe("/Mega/scorbot_telemetry",1,&ScorbotHWInterface::telemetryCallback, this);
 
  cmd_pub=nh.advertise<scorbot_control::arm_command>("/Mega/armCmd",1);

  ROS_INFO("ScorbotHWInterface constructed");
 
}

void ScorbotHWInterface::telemetryCallback(const scorbot_control::scorbot_telemetry::ConstPtr &msg){





 /*
 float32[6] angle #deg
 float32[6] current #amps
 float32[6] vel #deg/s
 time startSyncTime
 uint32 isrTicks # this would overlow if the robot is ledt on for too long
 uint8 bufferhealth 
 */

 /*
 //States
 std::vector<double> joint_position
 std::vector<double> joint_velocity
 std::vector<double> joint_effort__;
 */

 /* joint_position_[0] = scorbot_input_msg_.joint0;
 joint_position_[1] = scorbot_input_msg_.joint0;
 joint_position_[2] = scorbot_input_msg_.joint0;
 joint_position_[3] = scorbot_input_msg_.joint0;
 joint_position_[4] = scorbot_input_msg_.joint0;

 */
 /* joint_position_[0] = scorbot_input_msg_.joint0; */



 /* for(int i=0; i<num_joints_; i++){
    joint_position_[i]=msg->angle[i]*DEG_TO_RAD;
    joint_velocity_[i]=msg->vel[i]*DEG_TO_RAD;


  }
 */
}

void ScorbotHWInterface::init()
{


  // Call parent class version of this function
  GenericHWInterface::init();

  ROS_INFO("ScorbotHWInterface Ready.");
}
 

void ScorbotHWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since our write() command populates our state for us

  ros::spinOnce();
}

void ScorbotHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  //enforceLimits(elapsed_time);

 /* scorbot_output_msg_.cmd0 = joint_effort_command_[0];
 scorbot_output_msg_.cmd1 = joint_effort_command_[1];
 scorbot_output_msg_.cmd2 = joint_effort_command_[2];
 scorbot_output_msg_.cmd3 = joint_effort_command_[3];
 scorbot_output_msg_.cmd4 = joint_effort_command_[4];
*/

 /*
 float32[6] current #amps
 float32[6] accel #deg/^2
 float32[6] vel #deg/s
 float32[6] angle #deg void telemetryCallback(const scorbot_telemetry::ConstPtr &msg); 
 uint32 msg_ctr # count sent msgs to detected missed messages
  */

 /*
 std::vector<double> joint_position
  std::vector<double> joint_velocity
  std::vector<double> joint_effort__;
     */

 static scorbot_control::arm_command arm_cmd;

 /*for(int i=0; i<num_joints_; i++){
     arm_cmd.angle[i]=joint_position_command_[i]*RAD_TO_DEG;
     arm_cmd.vel[i]=joint_velocity_command_[i]*RAD_TO_DEG;
   }
*/


 cmd_pub.publish(arm_cmd);

}

void ScorbotHWInterface::enforceLimits(ros::Duration& period)

{
  // Enforces position and velocity
  //pos_jnt_sat_interface_.enforceLimits(period);
}

}
// namespace scorbot_hw_interface