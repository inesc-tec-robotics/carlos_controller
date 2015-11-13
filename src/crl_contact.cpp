
#include "carlos_controller/crl_contact.h"
#include "carlos_controller/crl_defines.h"

#include <amn_common/amn_mnl_defines.h>
#include <amn_common/amn_ik_lib.h>
#include <amn_common/amn_lico_lib.h>
#include <std_msgs/UInt8.h>
#include <brics_actuator/JointVelocities.h>
#include <ftm_msgs/ftm_defines.h>
#include <std_srvs/Empty.h>

using namespace carlos;

ContactController::ContactController(ros::NodeHandle node,
				     double sensor_offset, double period)
{
  this->node = node;
  this->offset = sensor_offset;
  this->period = period;
  this->mode = CONTACT_MODE_STOPPED;

  ContactController::register_messages();
  ContactController::register_services();
  ContactController::install_params();

  this->timer = node.createTimer(ros::Duration(period), 
				 &ContactController::timerCallback, this);
  this->timer.stop();
}

/* timer callback */
void ContactController::timerCallback(const ros::TimerEvent& event)
{
  //double torques[DOF];
  //double velocities[DOF];
  brics_actuator::JointVelocities msg;
  
  
  if(!ft_received || !arm_received)
    return;

  
  switch(this->mode){
  case CONTACT_MODE_STOPPED:

    break;
  case CONTACT_MODE_AUTO:
    //get_joints_torque(&this->ft_readings, this->joints, torques, this->offset);
    //d_velocities_friction(this->joints, torques, velocities, period, DOF);

    msg.velocities = std::vector<brics_actuator::JointValue>(DOF);
    for(int i=0; i<DOF; i++){
      msg.velocities[i].joint_uri = std::string(this->joints[i].name);
      msg.velocities[i].unit = "rad";
      msg.velocities[i].value = this->velocities[i]*(1.0 - this->factor);
    }

    this->velPub.publish(msg);

    break;
  }
}

/* Communication funtions */
void ContactController::install_params(void)
{
  XmlRpc::XmlRpcValue joint_names;
  this->node.getParam(JOINTS_NAMES, joint_names);
  ROS_ASSERT(joint_names.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int32_t i=0; i<joint_names.size(); i++){
    ROS_ASSERT(joint_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    this->joints[i].name = static_cast<std::string>(joint_names[i]).c_str();
  }
  
  XmlRpc::XmlRpcValue max_acc;
  this->node.getParam(JOINTS_MAX_ACC, max_acc);
  ROS_ASSERT(max_acc.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int32_t i=0; i<max_acc.size(); i++){
    ROS_ASSERT(max_acc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->joints[i].max_acc = max_acc[i];
  }

  XmlRpc::XmlRpcValue max_vel;
  this->node.getParam(JOINTS_MAX_VEL, max_vel);
  ROS_ASSERT(max_vel.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int32_t i=0; i<max_vel.size(); i++){
    ROS_ASSERT(max_vel[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->joints[i].max_vel = max_vel[i];
  }

  XmlRpc::XmlRpcValue inertias;
  this->node.getParam(JOINTS_INERTIAS, inertias);
  ROS_ASSERT(inertias.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int32_t i=0; i<inertias.size(); i++){
    ROS_ASSERT(inertias[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->joints[i].inertia = inertias[i];
  }

  XmlRpc::XmlRpcValue frictions;
  this->node.getParam(JOINTS_FRICTIONS, frictions);
  ROS_ASSERT(frictions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int32_t i=0; i<frictions.size(); i++){
    ROS_ASSERT(frictions[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->joints[i].friction = frictions[i];
  }

  XmlRpc::XmlRpcValue min_pos;
  this->node.getParam(JOINTS_MIN_POS, min_pos);
  ROS_ASSERT(min_pos.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int32_t i=0; i<min_pos.size(); i++){
    ROS_ASSERT(min_pos[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->joints[i].min_pos = min_pos[i];
  }

  XmlRpc::XmlRpcValue links_list;
  this->node.getParam(JOINTS_LINKS, links_list);
  ROS_ASSERT(links_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int32_t i=0; i<links_list.size(); i++){
    ROS_ASSERT(links_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->joints[i].link = links_list[i];
  }

  XmlRpc::XmlRpcValue max_pos;
  this->node.getParam(JOINTS_MAX_POS, max_pos);
  ROS_ASSERT(max_pos.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int32_t i=0; i<max_pos.size(); i++){
    ROS_ASSERT(max_pos[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->joints[i].max_pos = max_pos[i];
  }

  /*
  XmlRpc::XmlRpcValue press;
  this->node.getParam(PARAM_STUD_PRESS, press);
  ROS_ASSERT(press.getType() == XmlRpc::XmlRpcValue::TypeDouble);
  this->stud_press = press;
  */
}

void ContactController::register_messages(void)
{
  this->velPub = this->node.advertise<brics_actuator::JointVelocities>(COMMAND_VEL_TOPIC, 10);
}

void ContactController::subscribe_messages(void)
{
  fprintf(stdout, "Arm state name %s\n", ARM_CONTROLLER_STATE_TOPIC);
  this->armSubs = this->node.subscribe(ARM_CONTROLLER_STATE_TOPIC,
				       1, &ContactController::armStateHnd, this);
  fprintf(stdout, "ftm data topic --> %s\n", FTM_DATA_TOPIC);
  this->ftSubs = this->node.subscribe("/ftm75/measurement",
				      1, &ContactController::ftHnd, this);
}

void ContactController::unsubscribe_messages(void)
{
  this->armSubs.shutdown();
  this->ftSubs.shutdown();
}

void ContactController::register_services(void)
{
  this->resetSrv = this->node.serviceClient<std_srvs::Empty>(FTM_CALIBRATE_SRV);
}

/* Class functions */
contact_mode_t ContactController::set_mode(contact_mode_t mode)
{
  fprintf(stdout, "Contact controller -> set mode hnd: %d\n", this->mode);
  switch(this->mode){
  case CONTACT_MODE_STOPPED:
    this->mode = mode;
    if(this->mode == CONTACT_MODE_AUTO){
      this->ft_received = 0;
      this->arm_received = 0;
      subscribe_messages();
      this->timer.start();     
    }

    break;
  case CONTACT_MODE_AUTO:
    if(mode == CONTACT_MODE_STOPPED){
      unsubscribe_messages();
      this->mode = CONTACT_MODE_STOPPED;
      this->timer.stop();
      this->stop_arm();
    }

    break;
  }
  
  return this->mode;
}

contact_mode_t ContactController::get_mode(void)
{
  return this->mode;
}

void ContactController::set_joint_velocities(double* vel, double press)
{
  for(int i=0; i<DOF; i++)
    this->velocities[i] = vel[i];
  this->stud_press = press;
}

void ContactController::set_joint_limits(double* min, double* max)
{
  for(int i=0; i<DOF; i++){
    this->joints[i].min_pos = min[i];
    this->joints[i].max_pos = max[i];
  }

  return;
}

void ContactController::stop_arm(void)
{
  brics_actuator::JointVelocities msg;
  msg.velocities = std::vector<brics_actuator::JointValue>(DOF);
  for(int i=0; i<DOF; i++){
    msg.velocities[i].joint_uri = std::string(this->joints[i].name);
    msg.velocities[i].unit = "rad";
    msg.velocities[i].value = 0.0;
  }
  this->velPub.publish(msg);
}

/* message handlers */
void ContactController::armStateHnd(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
  if(arm_received == 0)
    arm_received = 1;

  for(int i=0; i<DOF; i++){
    this->joints[i].position = msg->actual.positions[i];
    this->joints[i].velocity = msg->actual.velocities[i];
  }
}

void ContactController::ftHnd(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  double goal[DOF];

  switch(this->mode){
  case CONTACT_MODE_STOPPED:CONTACT_MODE_REACHED:
    
    break;
  case CONTACT_MODE_AUTO:
    if(!this->ft_received){
      this->ft_readings.fx = msg->wrench.force.x;
      this->ft_readings.fy = msg->wrench.force.y;
      this->ft_readings.fz = msg->wrench.force.z;
      this->ft_readings.mx = msg->wrench.torque.x;
      this->ft_readings.my = msg->wrench.torque.y;
      this->ft_readings.mz = msg->wrench.torque.z;
      this->ft_received = 1;
    }else{
      this->ft_readings.fx = (this->ft_readings.fx + msg->wrench.force.x) / 2.0;
      this->ft_readings.fy = (this->ft_readings.fy + msg->wrench.force.y) / 2.0;
      this->ft_readings.fz = (this->ft_readings.fz + msg->wrench.force.z) / 2.0;
      this->ft_readings.mx = (this->ft_readings.mx + msg->wrench.torque.x) / 2.0;
      this->ft_readings.my = (this->ft_readings.my + msg->wrench.torque.y) / 2.0;
      this->ft_readings.mz = (this->ft_readings.mz + msg->wrench.torque.z) / 2.0;
    }

    if((this->stud_press + this->ft_readings.fz) < 0.0){
      fprintf(stdout, "Contact controller mode changed to STOPPED\n");
      unsubscribe_messages();
      this->mode = CONTACT_MODE_STOPPED;
      this->timer.stop();
      stop_arm();
    }else{
      this->factor = fabs(this->ft_readings.fz / this->stud_press);
      if(this->factor > 1.0){
	fprintf(stdout, "Contact controller mode changed to STOPPED\n");
	unsubscribe_messages();
	this->mode = CONTACT_MODE_STOPPED;
	this->timer.stop();
	stop_arm();
      }
    }
    
    break;
  }
}

