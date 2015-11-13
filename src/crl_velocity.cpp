
#include <ros/ros.h>
#include <brics_actuator/JointVelocities.h>

#define DOF 6

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vel_publisher");
  ros::NodeHandle node;

  ros::Publisher velPub = node.advertise<brics_actuator::JointVelocities>("arm_controller/command_vel", 1);

  sleep(5);
  
  brics_actuator::JointVelocities msg;
  msg.velocities = std::vector<brics_actuator::JointValue>(DOF); 

  msg.velocities[0].joint_uri = "arm_1_joint";
  msg.velocities[0].unit = "rad";
  msg.velocities[0].value = 0.0;
  msg.velocities[1].joint_uri = "arm_2_joint";
  msg.velocities[1].unit = "rad";
  msg.velocities[1].value = 0.0;
  msg.velocities[2].joint_uri = "arm_3_joint";
  msg.velocities[2].unit = "rad";
  msg.velocities[2].value = -0.1;
  msg.velocities[3].joint_uri = "arm_4_joint";
  msg.velocities[3].unit = "rad";
  msg.velocities[3].value = 0.0;
  msg.velocities[4].joint_uri = "arm_5_joint";
  msg.velocities[4].unit = "rad";
  msg.velocities[4].value = 0.1;
  msg.velocities[5].joint_uri = "arm_6_joint";
  msg.velocities[5].unit = "rad";
  msg.velocities[5].value = 0.0;

  velPub.publish(msg);

  sleep(3);

  brics_actuator::JointVelocities msg2;
  msg2.velocities = std::vector<brics_actuator::JointValue>(DOF); 

  msg2.velocities[0].joint_uri = "arm_1_joint";
  msg2.velocities[0].unit = "rad";
  msg2.velocities[0].value = 0.0;
  msg2.velocities[1].joint_uri = "arm_2_joint";
  msg2.velocities[1].unit = "rad";
  msg2.velocities[1].value = 0.0;
  msg2.velocities[2].joint_uri = "arm_3_joint";
  msg2.velocities[2].unit = "rad";
  msg2.velocities[2].value = 0.0;
  msg2.velocities[3].joint_uri = "arm_4_joint";
  msg2.velocities[3].unit = "rad";
  msg2.velocities[3].value = 0.0;
  msg2.velocities[4].joint_uri = "arm_5_joint";
  msg2.velocities[4].unit = "rad";
  msg2.velocities[4].value = 0.0;
  msg2.velocities[5].joint_uri = "arm_6_joint";
  msg2.velocities[5].unit = "rad";
  msg2.velocities[5].value = 0.0;

  velPub.publish(msg2);

  return 0;
}
