
#include "carlos_controller/arm_interface.h"
#include "carlos_controller/crl_defines.h"
#include <mission_ctrl_msgs/hardware_state.h>
#include <amn_welding_iface/SetValue.h>
#include <amn_welding_iface/SetSignal.h>

using namespace carlos;

ArmInterface::ArmInterface(ros::NodeHandle node)
{
  this->node = node;
  this->state = hardware::IDLE;

  pthread_mutex_init(&this->state_mutex, NULL);
  pthread_mutex_init(&this->modules_mutex, NULL);
  pthread_mutex_init(&this->last_pose_mutex, NULL);
  pthread_mutex_init(&this->last_orient_mutex, NULL);
  
  install_params();
  register_messages();
  subscribe_messages();

  mission_ctrl_msgs::GetMode srv;
  this->ctrlGetModeClt.call(srv);
  this->controller_state = (arm_mode_t)srv.response.mode;

  this->feederGetModeClt.call(srv);
  this->feeder_state = srv.response.mode;

  this->visionGetModeClt.call(srv);
  this->vision_state = (vision_mode_t)srv.response.mode;
  
  this->distribution_server = new DistributionServer(node, CARLOS_DISTRIBUTION_ACTION,
						     boost::bind(&ArmInterface::generateStudDistributionHnd,
								 this, _1), false);
  this->distribution_server->start();
  this->move_server = new MoveServer(node, CARLOS_MOVE_ARM_ACTION,
				     boost::bind(&ArmInterface::moveArmHnd, this, _1), false);
  this->move_server->start();
  this->projection_server = new ProjectionServer(node, CARLOS_PROJECTION_ACTION,
						 boost::bind(&ArmInterface::projectionPoseHnd,
							     this, _1), false);
  this->projection_server->start();
  this->weld_server = new WeldServer(node, CARLOS_WELD_ACTION,
				     boost::bind(&ArmInterface::executeWeldActHnd, this, _1), false);
  this->weld_server->start();
  
  //geometry_msgs::Pose task_pose;
  //std::vector<stud_t> pending_studs;
  //ArmInterface::read_stud_params("task004", &task_pose, &pending_studs);

  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  wall_tf.setRotation(q);
}

void ArmInterface::timerCallback(const ros::TimerEvent& time)
{
  fprintf(stdout, "Timer callback\n");
  publish_state();
}

void ArmInterface::tfCallback(const ros::TimerEvent& time)
{
  pthread_mutex_lock(&wall_tf_mutex);
  this->broadcaster.sendTransform(tf::StampedTransform(wall_tf, ros::Time::now(), "base_footprint", "wall"));
  pthread_mutex_unlock(&wall_tf_mutex);
}

void ArmInterface::install_params(void)
{
  XmlRpc::XmlRpcValue home_pos;
  this->node.getParam(ARM_PARAM_HOME_POS, home_pos);
  ROS_ASSERT(home_pos.getType() == XmlRpc::XmlRpcValue::TypeArray);
  if(home_pos.size() != DOF){
    fprintf(stdout, "Error reading %s, wrong number of DOF: %d\n", 
	    ARM_PARAM_HOME_POS, home_pos.size());
    ros::shutdown();
  }
  for(int32_t i=0; i<home_pos.size(); i++){
    ROS_ASSERT(home_pos[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->home_position[i] = home_pos[i];
  }

  XmlRpc::XmlRpcValue init_pos;
  this->node.getParam(ARM_PARAM_INIT_POS, init_pos);
  ROS_ASSERT(init_pos.getType() == XmlRpc::XmlRpcValue::TypeArray);
  if(init_pos.size() != DOF){
    fprintf(stdout, "Error reading %s, wrong number of DOF: %d\n",
	    ARM_PARAM_INIT_POS, init_pos.size());
    ros::shutdown();
  }
  for(int32_t i=0; i<init_pos.size(); i++){
    ROS_ASSERT(init_pos[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->init_position[i] = init_pos[i];
  }

  XmlRpc::XmlRpcValue vis_pos;
  this->node.getParam(ARM_PARAM_VISION_POS, vis_pos);
  ROS_ASSERT(vis_pos.getType() == XmlRpc::XmlRpcValue::TypeArray);
  if(vis_pos.size() != DOF){
    fprintf(stdout, "Error reading %s, wrong number of DOF: %d\n",
	    ARM_PARAM_VISION_POS, vis_pos.size());
    ros::shutdown();
  }
  for(int32_t i=0; i<vis_pos.size(); i++){
    ROS_ASSERT(vis_pos[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->vision_position[i] = vis_pos[i];
  }

  XmlRpc::XmlRpcValue proj_pos;
  this->node.getParam(ARM_PARAM_PROJECTION_POS, proj_pos);
  ROS_ASSERT(proj_pos.getType() == XmlRpc::XmlRpcValue::TypeArray);
  if(proj_pos.size() != DOF){
    fprintf(stdout, "Error reading %s, wrong number of DOF: %d\n",
	    ARM_PARAM_PROJECTION_POS, proj_pos.size());
    ros::shutdown();
  }
  for(int32_t i=0; i<proj_pos.size(); i++){
    ROS_ASSERT(proj_pos[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->projection_position[i] = proj_pos[i];
  }

  XmlRpc::XmlRpcValue fed_pos;
  this->node.getParam(ARM_PARAM_FEEDER_RELOAD, fed_pos);
  ROS_ASSERT(fed_pos.getType() == XmlRpc::XmlRpcValue::TypeArray);
  if(fed_pos.size() != DOF){
    fprintf(stdout, "Error reading %s, wrong number of DOF: %d\n",
	    ARM_PARAM_FEEDER_RELOAD, fed_pos.size());
    ros::shutdown();
  }
  for(int32_t i=0; i<fed_pos.size(); i++){
    ROS_ASSERT(fed_pos[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->feeder_reload[i] = fed_pos[i];
  }

  XmlRpc::XmlRpcValue fed_app;
  this->node.getParam(ARM_PARAM_FEEDER_APPROACH, fed_app);
  ROS_ASSERT(fed_app.getType() == XmlRpc::XmlRpcValue::TypeArray);
  if(fed_app.size() != DOF){
    fprintf(stdout, "Error reading %s, wrong number of DOF: %d\n",
	    ARM_PARAM_FEEDER_APPROACH, fed_app.size());
    ros::shutdown();
  }
  for(int32_t i=0; i<fed_app.size(); i++){
    ROS_ASSERT(fed_app[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->feeder_approach[i] = fed_app[i];
  }

  XmlRpc::XmlRpcValue fed_exit;
  this->node.getParam(ARM_PARAM_FEEDER_EXIT, fed_exit);
  ROS_ASSERT(fed_exit.getType() == XmlRpc::XmlRpcValue::TypeArray);
  if(fed_exit.size() != DOF){
    fprintf(stdout, "Error reading %s, wrong number of DOF: %d\n",
	    ARM_PARAM_FEEDER_EXIT, fed_exit.size());
    ros::shutdown();
  }
  for(int32_t i=0; i<fed_exit.size(); i++){
    ROS_ASSERT(fed_exit[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    this->feeder_exit[i] = fed_exit[i];
  }

  XmlRpc::XmlRpcValue jnt_names;
  this->node.getParam(ARM_PARAM_JOINT_NAMES, jnt_names);
  ROS_ASSERT(jnt_names.getType() == XmlRpc::XmlRpcValue::TypeArray);
  if(jnt_names.size() != DOF){
    fprintf(stdout, "Error reading %s, wrong number of joints: %d\n",
	    ARM_PARAM_JOINT_NAMES, jnt_names.size());
    ros::shutdown();
  }
  for(int32_t i=0; i<jnt_names.size(); i++){
    ROS_ASSERT(jnt_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    this->joint_names.push_back(static_cast<std::string>(jnt_names[i]));
  }

  XmlRpc::XmlRpcValue prim_cons;
  this->node.getParam(ARM_PARAM_PRIMARY_CONS, prim_cons);
  ROS_ASSERT(prim_cons.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int32_t i=0; i<prim_cons.size(); i++){
    ROS_ASSERT(prim_cons[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    if(prim_cons[i].size() != 5){
      fprintf(stdout, "Error reading %s, wrong number of parameters in row %d\n",
	      ARM_PARAM_PRIMARY_CONS, i);
      ros::shutdown();
    }
    moveit_msgs::JointConstraint tmp_msg;
    if(prim_cons[i][0].getType() != XmlRpc::XmlRpcValue::TypeString){
      fprintf(stdout, "Error reading %s, wrong parameter format in row %d column 1\n",
	      ARM_PARAM_PRIMARY_CONS, i);
      ros::shutdown();
    }
    tmp_msg.joint_name = static_cast<std::string>(prim_cons[i][0]);
    if(prim_cons[i][1].getType() != XmlRpc::XmlRpcValue::TypeDouble){
      fprintf(stdout, "Error reading %s, wrong parameter format in row %d column 2\n",
	      ARM_PARAM_PRIMARY_CONS, i);
      ros::shutdown();
    }
    tmp_msg.position = prim_cons[i][1];
    if(prim_cons[i][2].getType() != XmlRpc::XmlRpcValue::TypeDouble){
      fprintf(stdout, "Error reading %s, wrong parameter format in row %d column 3\n",
	      ARM_PARAM_PRIMARY_CONS, i);
      ros::shutdown();
    }
    tmp_msg.tolerance_above = prim_cons[i][2];
    if(prim_cons[i][3].getType() != XmlRpc::XmlRpcValue::TypeDouble){
      fprintf(stdout, "Error reading %s, wrong parameter format in row %d column 4\n",
	      ARM_PARAM_PRIMARY_CONS, i);
      ros::shutdown();
    }
    tmp_msg.tolerance_below = prim_cons[i][3];
    if(prim_cons[i][4].getType() != XmlRpc::XmlRpcValue::TypeDouble){
      fprintf(stdout, "Error reading %s, wrong parameter format in row %d column 5\n",
	      ARM_PARAM_PRIMARY_CONS, i);
      ros::shutdown();
    }
    tmp_msg.weight = prim_cons[i][4];
    this->primary_cons.push_back(tmp_msg);
  }

  XmlRpc::XmlRpcValue sec_cons;
  this->node.getParam(ARM_PARAM_SECONDARY_CONS, sec_cons);
  ROS_ASSERT(sec_cons.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int32_t i=0; i<sec_cons.size(); i++){
    ROS_ASSERT(sec_cons[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    if(sec_cons[i].size() != 5){
      fprintf(stdout, "Error reading %s, wrong number of parameters in row %d\n",
	      ARM_PARAM_SECONDARY_CONS, i);
      ros::shutdown();
    }
    moveit_msgs::JointConstraint tmp_msg;
    if(sec_cons[i][0].getType() != XmlRpc::XmlRpcValue::TypeString){
      fprintf(stdout, "Error reading %s, wrong parameter format in row %d column 1\n",
	      ARM_PARAM_SECONDARY_CONS, i);
      ros::shutdown();
    }
    tmp_msg.joint_name = static_cast<std::string>(sec_cons[i][0]);
    if(sec_cons[i][1].getType() != XmlRpc::XmlRpcValue::TypeDouble){
      fprintf(stdout, "Error reading %s, wrong parameter format in row %d column 2\n",
	      ARM_PARAM_SECONDARY_CONS, i);
      ros::shutdown();
    }
    tmp_msg.position = sec_cons[i][1];
    if(sec_cons[i][2].getType() != XmlRpc::XmlRpcValue::TypeDouble){
      fprintf(stdout, "Error reading %s, wrong parameter format in row %d column 3\n",
	      ARM_PARAM_SECONDARY_CONS, i);
      ros::shutdown();
    }
    tmp_msg.tolerance_above = sec_cons[i][2];
    if(sec_cons[i][3].getType() != XmlRpc::XmlRpcValue::TypeDouble){
      fprintf(stdout, "Error reading %s, wrong parameter format in row %d column 4\n",
	      ARM_PARAM_SECONDARY_CONS, i);
      ros::shutdown();
    }
    tmp_msg.tolerance_below = sec_cons[i][3];
    if(sec_cons[i][3].getType() != XmlRpc::XmlRpcValue::TypeDouble){
      fprintf(stdout, "Error reading %s, wrong parameter format in row %d column 5\n",
	      ARM_PARAM_SECONDARY_CONS, i);
      ros::shutdown();
    }
    tmp_msg.weight = sec_cons[i][4];
    this->secondary_cons.push_back(tmp_msg);
  }

  XmlRpc::XmlRpcValue pos_cons;
  this->node.getParam(ARM_PARAM_POSITION_CONS, pos_cons);
  ROS_ASSERT(pos_cons.getType() == XmlRpc::XmlRpcValue::TypeDouble);
  this->position_cons = pos_cons;

  XmlRpc::XmlRpcValue orient_cons;
  this->node.getParam(ARM_PARAM_ORIENTATION_CONS, orient_cons);
  ROS_ASSERT(orient_cons.getType() == XmlRpc::XmlRpcValue::TypeDouble);
  this->orientation_cons = orient_cons;
}

void ArmInterface::register_messages(void)
{
  this->modePub = this->node.advertise<mission_ctrl_msgs::hardware_state>(CARLOS_ARM_STATE_MSG, 1);
  this->goHomeSrv = this->node.advertiseService(CARLOS_ARM_HOME_SRV, &ArmInterface::goHomeHnd, this);
  this->exitFeederSrv = this->node.advertiseService(CARLOS_ARM_FEEDER_EXIT_SRV, &ArmInterface::exitFeederHnd, this);

  /* Service clients */
  this->getCurrentPoseClt = this->node.serviceClient<carlos_controller::CurrentPose>(CRL_GET_CURRENT_POSE_SRV);
  this->getCurrentJointsClt = this->node.serviceClient<carlos_controller::CurrentJoints>(CRL_GET_CURRENT_JOINTS_SRV);
  this->ftMoveClt = this->node.serviceClient<carlos_controller::FTMove>(CRL_FT_MOVE_SRV);
  this->jointMoveClt = this->node.serviceClient<carlos_controller::JointMove>(CRL_JOINT_MOVE_SRV);
  this->poseMoveClt = this->node.serviceClient<carlos_controller::PoseMove>(CRL_POSE_MOVE_SRV);
  this->rpyMoveClt = this->node.serviceClient<carlos_controller::RPYMove>(CRL_RPY_MOVE_SRV);
  this->setConsClt = this->node.serviceClient<carlos_controller::JointConstraints>(CRL_SET_CONSTRAINTS_SRV);

  this->ctrlGetModeClt = this->node.serviceClient<mission_ctrl_msgs::GetMode>(CRL_CTRL_GET_MODE_SRV);
  this->feederGetModeClt = this->node.serviceClient<mission_ctrl_msgs::GetMode>(CRL_FEEDER_GET_MODE_SRV);
  this->feederSetModeClt = this->node.serviceClient<mission_ctrl_msgs::SetMode>(CRL_FEEDER_SET_MODE_SRV);
  this->feederLoadClt = this->node.serviceClient<std_srvs::Empty>(CRL_FEEDER_LOAD_SRV);
  this->visionGetModeClt = this->node.serviceClient<mission_ctrl_msgs::GetMode>(CRL_VISION_GET_MODE_SRV);
  this->visionSetModeClt = this->node.serviceClient<mission_ctrl_msgs::SetMode>(CRL_VISION_SET_MODE_SRV);
  this->setPowerClt = this->node.serviceClient<amn_welding_iface::SetValue>(WLD_SET_POWER_SRV);
  this->setLaserClt = this->node.serviceClient<amn_welding_iface::SetSignal>(WLD_SET_LASER_SRV);
  this->fireGunClt = this->node.serviceClient<std_srvs::Empty>(WLD_FIRE_GUN_SRV);
}

void ArmInterface::subscribe_messages(void)
{
  this->ctrlModeSubs = this->node.subscribe(CRL_CTRL_MODE_MSG, 1,
					    &ArmInterface::controllerModeHnd, this);
  this->feederModeSubs = this->node.subscribe(CRL_FEEDER_MODE_MSG, 1,
					      &ArmInterface::feederModeHnd, this);
  this->visionModeSubs = this->node.subscribe(CRL_VISION_MODE_MSG, 1,
					      &ArmInterface::visionModeHnd, this);
  this->visionOrientSubs = this->node.subscribe(CRL_WALL_ORIENT_MSG, 1,
						&ArmInterface::visionOrientHnd, this);
  this->visionStudsSubs = this->node.subscribe(CRL_STUDS_POS_MSG, 1,
					       &ArmInterface::visionStudsHnd, this);
  this->wldGroundSubs = this->node.subscribe(WLD_GROUND_STATE_MSG, 1,
					     &ArmInterface::wldGroundHnd, this);
  this->wldLaserSubs = this->node.subscribe(WLD_LASER_STATE_MSG, 1,
					    &ArmInterface::wldLaserHnd, this);
  this->wldPowerStateSubs = this->node.subscribe(WLD_POWER_STATE_MSG, 1,
						 &ArmInterface::wldPowerStateHnd, this);
  this->wldPowerValueSubs = this->node.subscribe(WLD_POWER_VALUE_MSG, 1,
						 &ArmInterface::wldPowerValueHnd, this);
}

void ArmInterface::publish_state(void)
{
  std::stringstream sts;
  mission_ctrl_msgs::hardware_state msg;
  msg.state = get_state();
  switch(msg.state){
  case hardware::IDLE:
    sts << IDLE_DESCRIPTION; 
    break;
  case hardware::BUSY:
    sts << BUSY_DESCRIPTION;
    break;
  case hardware::ERROR:
    sts << ERROR_DESCRIPTION;
    break;
  case hardware::NOT_CONNECTED:
    sts << NOT_CONNECTED_DESCRIPTION;
    break;
  default:
    sts << " ";
    break;
  }
  msg.description = sts.str();
  this->modePub.publish(msg);
}

bool ArmInterface::goHomeHnd(std_srvs::Empty::Request& request,
			     std_srvs::Empty::Response& response)
{
  return move_home();
}

bool ArmInterface::move_home(void)
{
  carlos_controller::JointMove home_mv;
  home_mv.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
  home_mv.request.overwrite = std::vector<uint8_t>(DOF);
  for(int32_t i=0; i<DOF; i++){
    home_mv.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
    home_mv.request.move.positions[i].unit = "rad";
    home_mv.request.move.positions[i].value = this->home_position[i];
    home_mv.request.overwrite[i] = false;
  }

  return jointMoveClt.call(home_mv);
}

bool ArmInterface::exitFeederHnd(std_srvs::Empty::Request& request,
			     std_srvs::Empty::Response& response)
{
  if (!move_arm_exit_feeder_reload())
  {
    ROS_ERROR("Could not reach the feeder exit position");
    return false;
  }

  return true;
}

bool ArmInterface::move_arm_exit_feeder_reload(void)
{
  //Feeder exit
  carlos_controller::JointMove feeder_ex;
  feeder_ex.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
  feeder_ex.request.overwrite = std::vector<uint8_t>(DOF);
  for(int32_t i=0; i<DOF; i++){
    feeder_ex.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
    feeder_ex.request.move.positions[i].unit = "rad";
    feeder_ex.request.move.positions[i].value = this->feeder_exit[i];
    feeder_ex.request.overwrite[i] = false;
  }

  if(!this->jointMoveClt.call(feeder_ex))
  {
    set_state(hardware::IDLE);
    return false;
  }

  //Feeder approach
  carlos_controller::JointMove feeder_mv;
  feeder_mv.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
  feeder_mv.request.overwrite = std::vector<uint8_t>(DOF);
  for(int32_t i=0; i<DOF; i++){
    feeder_mv.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
    feeder_mv.request.move.positions[i].unit = "rad";
    feeder_mv.request.move.positions[i].value = this->feeder_approach[i];
    feeder_mv.request.overwrite[i] = false;
  }

  if(!this->jointMoveClt.call(feeder_mv))
  {
    set_state(hardware::IDLE);
    return false;
  }

  set_state(hardware::IDLE);
  return true;
}


hardware::states ArmInterface::get_state(void)
{
  hardware::states tmp;

  pthread_mutex_lock(&this->state_mutex);
  tmp = this->state;
  pthread_mutex_unlock(&this->state_mutex);

  return tmp;
}

hardware::states ArmInterface::set_state(hardware::states state)
{
  hardware::states tmp;
  pthread_mutex_lock(&this->state_mutex);
  tmp = this->state;
  this->state = state;
  pthread_mutex_unlock(&this->state_mutex);

  return tmp;
}

arm_mode_t ArmInterface::get_controller_mode(void)
{
  arm_mode_t tmp;
  pthread_mutex_lock(&this->modules_mutex);
  tmp = this->controller_state;
  pthread_mutex_unlock(&this->modules_mutex);

  return tmp;
}

arm_mode_t ArmInterface::set_controller_mode(arm_mode_t mode)
{
  arm_mode_t tmp;
  pthread_mutex_lock(&this->modules_mutex);
  tmp = this->controller_state;
  this->controller_state = mode;
  pthread_mutex_unlock(&this->modules_mutex);
  
  return tmp;
}

vision_mode_t ArmInterface::get_vision_mode(void)
{
  vision_mode_t tmp;
  pthread_mutex_lock(&this->modules_mutex);
  tmp = this->vision_state;
  pthread_mutex_unlock(&this->modules_mutex);

  return tmp;
}

vision_mode_t ArmInterface::set_vision_mode(vision_mode_t mode)
{
  vision_mode_t tmp;
  pthread_mutex_lock(&this->modules_mutex);
  tmp = this->vision_state;
  this->vision_state = mode;
  pthread_mutex_unlock(&this->modules_mutex);

  return tmp;
}

int ArmInterface::get_last_pose(void)
{
  int tmp;
  pthread_mutex_lock(&this->last_pose_mutex);
  tmp = this->last_pose;
  pthread_mutex_unlock(&this->last_pose_mutex);

  return tmp;
}

int ArmInterface::get_last_orient(void)
{
  int tmp;
  pthread_mutex_lock(&this->last_orient_mutex);
  tmp = this->last_orient;
  pthread_mutex_unlock(&this->last_orient_mutex);

  return tmp;
}

bool ArmInterface::get_gnd_contact(void)
{
  bool tmp;
  pthread_mutex_lock(&this->modules_mutex);
  tmp = this->wld_gnd_in;
  pthread_mutex_unlock(&this->modules_mutex);

  return tmp;
}

int ArmInterface::read_stud_params(std::string task_name, geometry_msgs::Pose* task_pose,
				   std::vector<stud_t>* pending_studs, int* side, int* voltage, double* pressure)
{
  double tmp = 0.0;
  std::stringstream str;
  str << MISSION_TASKS << SEPARATOR << task_name << PARAM_NAV_GOAL_X;
  
  XmlRpc::XmlRpcValue x_pos;
  this->node.getParam(str.str().c_str(), x_pos);
  if(x_pos.getType() != XmlRpc::XmlRpcValue::TypeDouble){
    fprintf(stdout, "Error reading param double %s\n", str.str().c_str());
    return -1;
  }

  str.str("");
  str.clear();
  str << MISSION_TASKS << SEPARATOR << task_name << PARAM_NAV_GOAL_Y;

  XmlRpc::XmlRpcValue y_pos;
  this->node.getParam(str.str().c_str(), y_pos);
  if(y_pos.getType() != XmlRpc::XmlRpcValue::TypeDouble){
    fprintf(stdout, "Error reading param double %s\n", str.str().c_str());
    return -1;
  }

  str.str("");
  str.clear();
  str << MISSION_TASKS << SEPARATOR << task_name << PARAM_NAV_GOAL_YAW;

  XmlRpc::XmlRpcValue yaw_pos;
  this->node.getParam(str.str().c_str(), yaw_pos);
  if(yaw_pos.getType() != XmlRpc::XmlRpcValue::TypeDouble){
    fprintf(stdout, "Error reading param double %s\n", str.str().c_str());
    return -1;
  }

  tmp = (double)(yaw_pos) / 2.0;
  task_pose->position.x = x_pos;
  task_pose->position.y = y_pos;
  task_pose->position.z = 0.0;
  task_pose->orientation.x = 0.0;
  task_pose->orientation.y = 0.0;
  task_pose->orientation.z = sin(tmp);
  task_pose->orientation.w = cos(tmp);

  str.str("");
  str.clear();
  str << MISSION_TASKS << SEPARATOR << task_name << PARAM_DIRECTION;

  XmlRpc::XmlRpcValue dir;
  this->node.getParam(str.str().c_str(), dir);
  if(dir.getType() != XmlRpc::XmlRpcValue::TypeInt){
    fprintf(stdout, "Error reading param integer %s\n", str.str().c_str());
    return -1;
  }
  *side = dir;

  str.str("");
  str.clear();
  str << MISSION_TASKS << SEPARATOR << task_name << PARAM_VOLTAGE;

  XmlRpc::XmlRpcValue volt;
  this->node.getParam(str.str().c_str(), volt);
  if(volt.getType() != XmlRpc::XmlRpcValue::TypeInt){
    fprintf(stdout, "Error reading param integer %s\n", str.str().c_str());
    return -1;
  }
  *voltage = volt;

  str.str("");
  str.clear();
  str << MISSION_TASKS << SEPARATOR << task_name << PARAM_STUD_PRESS;

  XmlRpc::XmlRpcValue press;
  this->node.getParam(str.str().c_str(), press);
  if(press.getType() != XmlRpc::XmlRpcValue::TypeDouble){
    fprintf(stdout, "Error reading param double %s\n", str.str().c_str());
    return -1;
  }
  *pressure = press;

  for(int i=1;; i++){
    XmlRpc::XmlRpcValue stud_state;
    XmlRpc::XmlRpcValue stud_time;
    XmlRpc::XmlRpcValue stud_x;
    XmlRpc::XmlRpcValue stud_y;
    stud_t stud_info;
    str.str("");
    str.clear();
    str << PARAM_STUD_ID << i;
    stud_info.id = std::string(str.str().c_str());

    str.str("");
    str.clear();
    str << MISSION_TASKS << SEPARATOR << task_name << PARAM_STUD_NAME << i << PARAM_STUD_STATE;
    fprintf(stdout, "Stud param: %s\n", str.str().c_str());
    this->node.getParam(str.str().c_str(), stud_state);
    if(stud_state.getType() != XmlRpc::XmlRpcValue::TypeInt){
      break;
    }
    stud_info.state = (stud::states)((int)stud_state);
    if(stud_info.state != stud::PENDING)
      continue;
    
    str.str("");
    str.clear();
    str << MISSION_TASKS << SEPARATOR << task_name << PARAM_STUD_NAME << i << PARAM_STUD_X;
    this->node.getParam(str.str().c_str(), stud_x);
    if(stud_x.getType() != XmlRpc::XmlRpcValue::TypeDouble)
      break;
    stud_info.x = stud_x;

    str.str("");
    str.clear();
    str << MISSION_TASKS << SEPARATOR << task_name << PARAM_STUD_NAME << i << PARAM_STUD_Y;
    this->node.getParam(str.str().c_str(), stud_y);
    if(stud_y.getType() != XmlRpc::XmlRpcValue::TypeDouble)
      break;
    stud_info.y = stud_y;

    str.str("");
    str.clear();
    str << MISSION_TASKS << SEPARATOR << task_name << PARAM_STUD_NAME << i << PARAM_STUD_TIME;
    this->node.getParam(str.str().c_str(), stud_time);
    if(stud_time.getType() != XmlRpc::XmlRpcValue::TypeDouble)
      break;
    stud_info.timestamp = stud_time;

    pending_studs->push_back(stud_info);
  }
  
  return 0;
}

/* Incomming handlers */
void ArmInterface::controllerModeHnd(const std_msgs::UInt8::ConstPtr& msg)
{
  pthread_mutex_lock(&modules_mutex);
  this->controller_state = (arm_mode_t)msg->data;
  pthread_mutex_unlock(&modules_mutex);
}

void ArmInterface::feederModeHnd(const std_msgs::UInt8::ConstPtr& msg)
{
  pthread_mutex_lock(&modules_mutex);
  this->feeder_state = msg->data;
  pthread_mutex_unlock(&modules_mutex);
}

void ArmInterface::visionModeHnd(const std_msgs::UInt8::ConstPtr& msg)
{
  pthread_mutex_lock(&modules_mutex);
  this->vision_state = (vision_mode_t)msg->data;
  pthread_mutex_unlock(&modules_mutex);
}

void ArmInterface::visionOrientHnd(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  pthread_mutex_lock(&last_orient_mutex);
  last_orient++;
  pthread_mutex_unlock(&last_orient_mutex);
  if(get_last_orient() < 6)
    return;
  
  if(fabs(msg->point.x) < 0.01 && fabs(msg->point.y) < 0.01 && fabs(msg->point.z) < 0.01){
    mission_ctrl_msgs::SetMode set_mode;
    set_mode.request.mode = CRL_VIS_DETECT;
    this->visionSetModeClt.call(set_mode);
    fprintf(stdout, "-->Orientation mode finished\n");
  }else{
    if(fabs(msg->point.z) > 0.01 ){
      carlos_controller::RPYMove rpy_move;
      rpy_move.request.move.header.frame_id = std::string(msg->header.frame_id.c_str());
      rpy_move.request.move.point.x = 0;
      rpy_move.request.move.point.y = 0;
      rpy_move.request.move.point.z = msg->point.z;
      if(!this->rpyMoveClt.call(rpy_move)){
	fprintf(stdout, "Error moving the robot arm (Yaw)\n");
      }
    }else if(fabs(msg->point.y) > 0.01 ){
        carlos_controller::RPYMove rpy_move;
        rpy_move.request.move.header.frame_id = std::string(msg->header.frame_id.c_str());
        rpy_move.request.move.point.x = 0;
        rpy_move.request.move.point.y = msg->point.y;
        rpy_move.request.move.point.z = 0;
        if(!this->rpyMoveClt.call(rpy_move)){
          fprintf(stdout, "Error moving the robot arm (Pitch)\n");
        }
    }else if(fabs(msg->point.x) > 0.01 ){
        carlos_controller::RPYMove rpy_move;
        rpy_move.request.move.header.frame_id = std::string(msg->header.frame_id.c_str());
        rpy_move.request.move.point.x = msg->point.x;
        rpy_move.request.move.point.y = 0;
        rpy_move.request.move.point.z = 0;
        if(!this->rpyMoveClt.call(rpy_move)){
          fprintf(stdout, "Error moving the robot arm (Roll)\n");
        }
    }
    pthread_mutex_lock(&this->last_orient_mutex);
    this->last_orient = 0;
    pthread_mutex_unlock(&this->last_orient_mutex);
  }
}

void ArmInterface::visionStudsHnd(const carlos_controller::StudsPoses::ConstPtr& msg)
{
  if(get_state() != hardware::BUSY)
    return;

  poses.pose_array.header.frame_id = std::string(msg->pose_array.header.frame_id.c_str());
  poses.pose_array.poses = std::vector<geometry_msgs::Pose>(msg->pose_array.poses.size());
  for(int32_t i=0; i<poses.pose_array.poses.size(); i++){
    poses.pose_array.poses.at(i).position.x = msg->pose_array.poses.at(i).position.x;
    poses.pose_array.poses.at(i).position.y = msg->pose_array.poses.at(i).position.y;
    poses.pose_array.poses.at(i).position.z = msg->pose_array.poses.at(i).position.z;
    poses.pose_array.poses.at(i).orientation.x = msg->pose_array.poses.at(i).orientation.x;
    poses.pose_array.poses.at(i).orientation.y = msg->pose_array.poses.at(i).orientation.y;
    poses.pose_array.poses.at(i).orientation.z = msg->pose_array.poses.at(i).orientation.z;
    poses.pose_array.poses.at(i).orientation.w = msg->pose_array.poses.at(i).orientation.w;
  }
  poses.stiff1 = msg->stiff1;
  poses.stiff2 = msg->stiff2;

  pthread_mutex_lock(&this->last_pose_mutex);
  this->last_pose++;
  pthread_mutex_unlock(&this->last_pose_mutex);
}

void ArmInterface::wldGroundHnd(const std_msgs::Bool::ConstPtr& msg)
{
  pthread_mutex_lock(&modules_mutex);
  this->wld_gnd_in = msg->data;
  pthread_mutex_unlock(&modules_mutex);
}

void ArmInterface::wldLaserHnd(const std_msgs::Bool::ConstPtr& msg)
{
  pthread_mutex_lock(&modules_mutex);
  this->wld_laser_out = msg->data;
  pthread_mutex_unlock(&modules_mutex);
}

void ArmInterface::wldPowerStateHnd(const std_msgs::Bool::ConstPtr& msg)
{
  pthread_mutex_lock(&modules_mutex);
  this->wld_power_state = msg->data;
  pthread_mutex_unlock(&modules_mutex);
}

void ArmInterface::wldPowerValueHnd(const std_msgs::UInt32::ConstPtr& msg)
{
  pthread_mutex_lock(&modules_mutex);
  this->wld_power_value = msg->data;
  pthread_mutex_unlock(&modules_mutex);
}

/* Action servers */

void ArmInterface::generateStudDistributionHnd(const mission_ctrl_msgs::generateStudDistributionGoalConstPtr& goal)
{
  fprintf(stdout, "Generate distribution handler\n");
  mission_ctrl_msgs::generateStudDistributionResult result;

  pthread_mutex_lock(&this->state_mutex);
  if(this->state != hardware::IDLE){
    pthread_mutex_unlock(&this->state_mutex);
    result.error_string = std::string("The arm interface is busy");
    this->distribution_server->setAborted(result);
    return;
  }

  this->state = hardware::BUSY;
  pthread_mutex_unlock(&this->state_mutex);

  while(get_controller_mode() != CRL_ARM_STOPPED){
    fprintf(stdout, "Waiting for the arm controller\n");
    sleep(1);
  }

  carlos_controller::JointMove srv;
  srv.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
  srv.request.overwrite = std::vector<uint8_t>(DOF);
  for(int32_t i=0; i<DOF; i++){
    srv.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
    srv.request.move.positions[i].unit = "rad";
    srv.request.move.positions[i].value = this->vision_position[i];
    srv.request.overwrite[i] = false;
  }

  if(goal->side == 0)
    srv.request.move.positions[0].value = -this->vision_position[0];

  if(!this->jointMoveClt.call(srv)){
    result.error_string = std::string("Could not reach the detection position");
    this->distribution_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  //set_controller_mode((arm_mode_t)srv.response.mode.data);

  while(get_controller_mode() == CRL_ARM_MOVING){
    fprintf(stdout, "Controller executing a movement\n");
    sleep(1);
  }

  if(get_controller_mode() != CRL_ARM_STOPPED){
    result.error_string = std::string("Error moving the robot arm");
    this->distribution_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  amn_welding_iface::SetSignal set_signal;
  set_signal.request.active = true;
  if(!this->setLaserClt.call(set_signal)){
    result.error_string = std::string("Error activating laser line");
    this->distribution_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  pthread_mutex_lock(&last_orient_mutex);
  this->last_orient = 0;
  pthread_mutex_unlock(&last_orient_mutex);

  mission_ctrl_msgs::SetMode set_mode;
  set_mode.request.mode = CRL_VIS_ORIENT;
  if(!this->visionSetModeClt.call(set_mode)){
    result.error_string = std::string("Error changing the vision module working mode");
    this->distribution_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  pthread_mutex_lock(&last_pose_mutex);
  this->last_pose = 0;
  pthread_mutex_unlock(&last_pose_mutex);

  while(get_last_pose() < 10){
    fprintf(stdout, "Waiting for studs distribution\n");
    sleep(1);
  }

  set_mode.request.mode = CRL_VIS_STOPPED;
  if(!this->visionSetModeClt.call(set_mode)){
    fprintf(stdout, "Error changing the vision module working mode\n");
  }

  set_signal.request.active = false;
  if(!this->setLaserClt.call(set_signal)){
    fprintf(stdout, "Error turning off the laser line\n");
  }

  //DPL: Transform and publish result
  tf::StampedTransform transform;
  try{
    this->listener.waitForTransform("base_footprint", "arm_6_link",
				    ros::Time(0), ros::Duration(2.0));
    this->listener.lookupTransform("base_footprint", "arm_6_link",
				   ros::Time(0), transform);
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
  }

  tf::Vector3 tmp_v(0.0, 0.0, 0.0);
  double height = transform(tmp_v).z() + BASE_HEIGHT;
  fprintf(stdout, "--> Tool position in z: %f\n", transform(tmp_v).z());
  
  carlos_controller::CurrentJoints current_joints;
  this->getCurrentJointsClt.call(current_joints);

  double z_pos = 0.0;
  for(int32_t i=0; i<poses.pose_array.poses.size(); i++){
    z_pos += poses.pose_array.poses.at(i).position.z;
  }
  if(poses.pose_array.poses.size() > 0)
    z_pos /= poses.pose_array.poses.size();

  //DPL: transform
  tf::StampedTransform tool_tf;
  try{
    this->listener.waitForTransform("base_footprint", "tool_link",
				    ros::Time(0), ros::Duration(2.0));
    this->listener.lookupTransform("base_footprint", "tool_link",
				   ros::Time(0), tool_tf);
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
  }
  
  tf::Quaternion q;
  tf::Vector3 trans(tool_tf.getOrigin().x() + cos(1.5708 + current_joints.response.joints.at(0))*(-poses.stiff1)+
		    z_pos*sin(1.5708 + current_joints.response.joints.at(0)), 
		    tool_tf.getOrigin().y() + sin(1.5708 + current_joints.response.joints.at(0))*(-poses.stiff1)-
		    z_pos*cos(1.5708 + current_joints.response.joints.at(0)), 0.0);
  q.setRPY(1.5708, 0, 1.5708-current_joints.response.joints.at(0));
  pthread_mutex_lock(&this->wall_tf_mutex);
  wall_tf.setOrigin(trans);
  wall_tf.setRotation(q);
  pthread_mutex_unlock(&this->wall_tf_mutex);
  
  result.positions = std::vector<geometry_msgs::Point>(poses.pose_array.poses.size());

  for(int32_t i=0; i<poses.pose_array.poses.size(); i++){
    result.positions.at(i).x = poses.pose_array.poses.at(i).position.y - poses.stiff1;
    result.positions.at(i).y = height - poses.pose_array.poses.at(i).position.x;
    result.positions.at(i).z = poses.pose_array.poses.at(i).position.z;
  }

  /*
  if(!this->move_home()){
    result.error_string = std::string("Could not reach the home position");
  }
  */

  this->distribution_server->setSucceeded(result);
  set_state(hardware::IDLE);
}

void ArmInterface::moveArmHnd(const mission_ctrl_msgs::moveArmGoalConstPtr& goal)
{
  mission_ctrl_msgs::moveArmResult result;

  pthread_mutex_lock(&this->state_mutex);
  if(this->state != hardware::IDLE){
    pthread_mutex_unlock(&this->state_mutex);
    result.succeeded = false;
    this->move_server->setAborted(result);
    return;
  }

  this->state = hardware::BUSY;
  pthread_mutex_unlock(&this->state_mutex);

  while(get_controller_mode() != CRL_ARM_STOPPED){
    fprintf(stdout, "Waiting for the arm controller\n");
    sleep(1);
  }

 /*
  carlos_controller::JointMove srv;
  srv.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
  srv.request.overwrite = std::vector<uint8_t>(DOF);
  for(int32_t i=0; i<DOF; i++){
    srv.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
    srv.request.move.positions[i].unit = "rad";
    srv.request.move.positions[i].value = this->vision_position[i];
    srv.request.overwrite[i] = false;
  }

  if(!this->jointMoveClt.call(srv)){
    //    result.error_string = std::string("Could not reach the detection position");
    this->move_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  //set_controller_mode((arm_mode_t)srv.response.mode.data);

  while(get_controller_mode() == CRL_ARM_MOVING){
    fprintf(stdout, "Controller executing a movement\n");
    sleep(1);
  }

  if(get_controller_mode() != CRL_ARM_STOPPED){
    //result.error_string = std::string("Error moving the robot arm");
    this->move_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  amn_welding_iface::SetSignal set_signal;
  set_signal.request.active = true;
  if(!this->setLaserClt.call(set_signal)){
    //result.error_string = std::string("Error activating laser line");
    this->move_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  pthread_mutex_lock(&last_orient_mutex);
  this->last_orient = 0;
  pthread_mutex_unlock(&last_orient_mutex);

  mission_ctrl_msgs::SetMode set_mode;
  set_mode.request.mode = CRL_VIS_ORIENT;
  if(!this->visionSetModeClt.call(set_mode)){
    //result.error_string = std::string("Error changing the vision module working mode");
    this->move_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  pthread_mutex_lock(&last_pose_mutex);
  this->last_pose = 0;
  pthread_mutex_unlock(&last_pose_mutex);

  while(get_last_pose() < 10){
    fprintf(stdout, "Waiting for studs distribution\n");
    sleep(1);
  }

  set_mode.request.mode = CRL_VIS_STOPPED;
  if(!this->visionSetModeClt.call(set_mode)){
    fprintf(stdout, "Error changing the vision module working mode\n");
  }

  set_signal.request.active = false;
  if(!this->setLaserClt.call(set_signal)){
    fprintf(stdout, "Error turning off the laser line\n");
  }
*/

  carlos_controller::CurrentPose current_pose;
  this->getCurrentPoseClt.call(current_pose);

  carlos_controller::JointConstraints pr_cons;
  pr_cons.request.constraints = std::vector<moveit_msgs::JointConstraint>(this->primary_cons.size());
  for(int32_t i=0; i<pr_cons.request.constraints.size(); i++){
    pr_cons.request.constraints[i].joint_name = primary_cons.at(i).joint_name;
    pr_cons.request.constraints[i].position = primary_cons.at(i).position;
    pr_cons.request.constraints[i].tolerance_above = primary_cons.at(i).tolerance_above;
    pr_cons.request.constraints[i].tolerance_below = primary_cons.at(i).tolerance_below;
    pr_cons.request.constraints[i].weight = primary_cons.at(i).weight;
  }
  pr_cons.request.orientation = this->orientation_cons;
  pr_cons.request.position = this->position_cons;
    
  if(!this->setConsClt.call(pr_cons)){
    result.succeeded = false;
    this->move_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  tf::StampedTransform wrist_tf;
  try{
    this->listener.waitForTransform("projector_link", "arm_6_link",
				    ros::Time(0), ros::Duration(2.0));
    this->listener.lookupTransform("projector_link", "arm_6_link",
				   ros::Time(0), wrist_tf);
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    result.succeeded = false;
    this->move_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  fprintf(stdout, "Wrist: %f, %f, %f\n", wrist_tf.getOrigin().x(),
	  wrist_tf.getOrigin().y(), wrist_tf.getOrigin().z());
  fprintf(stdout, "Wrist: %f, %f, %f, %f\n", wrist_tf.getRotation().x(),
	  wrist_tf.getRotation().y(), wrist_tf.getRotation().z(),
	  wrist_tf.getRotation().w()); 

  fprintf(stdout, "Position: %f, %f, %f\n", goal->pose.pose.position.x,
	  goal->pose.pose.position.y, goal->pose.pose.position.z);
  fprintf(stdout, "Rotation: %f, %f, %f, %f\n", goal->pose.pose.orientation.x,
	  goal->pose.pose.orientation.y, goal->pose.pose.orientation.z,
	  goal->pose.pose.orientation.w);

  tf::Transform tfr;
  tf::Vector3 pos_vector(goal->pose.pose.position.x,
			 goal->pose.pose.position.y, goal->pose.pose.position.z);
  tfr.setOrigin(pos_vector);
  tf::Quaternion pos_quat(goal->pose.pose.orientation.x, goal->pose.pose.orientation.y,
			  goal->pose.pose.orientation.z, goal->pose.pose.orientation.w);
  tfr.setRotation(pos_quat);
  tf::Transform movement = tfr*wrist_tf;
  
  fprintf(stdout, "Transform: %f, %f, %f\n", movement.getOrigin().x(),
	  movement.getOrigin().y(), movement.getOrigin().z());
  fprintf(stdout, "Transform: %f, %f, %f, %f\n", movement.getRotation().x(),
	  movement.getRotation().y(), movement.getRotation().z(),
	  movement.getRotation().w()); 

  carlos_controller::PoseMove pose_mv;
  pose_mv.request.move.header.frame_id = std::string(current_pose.response.pose.header.frame_id.c_str());
  pose_mv.request.move.pose.position.x = movement.getOrigin().x();
  pose_mv.request.move.pose.position.y = movement.getOrigin().y();
  pose_mv.request.move.pose.position.z = movement.getOrigin().z();
  pose_mv.request.move.pose.orientation.x = movement.getRotation().x();
  pose_mv.request.move.pose.orientation.y = movement.getRotation().y();
  pose_mv.request.move.pose.orientation.z = movement.getRotation().z();
  pose_mv.request.move.pose.orientation.w = movement.getRotation().w();

  if(!this->poseMoveClt.call(pose_mv)){
    result.succeeded = false;
    this->move_server->setAborted(result);
  }else{
    result.succeeded = true;
    this->move_server->setSucceeded(result);
  }

  this->set_state(hardware::IDLE);
}

void ArmInterface::projectionPoseHnd(const mission_ctrl_msgs::projectionPoseGoalConstPtr& goal)
{
  mission_ctrl_msgs::projectionPoseResult result;

  pthread_mutex_lock(&this->state_mutex);
  if(this->state != hardware::IDLE){
    pthread_mutex_unlock(&this->state_mutex);
    result.succeeded = false;
    this->projection_server->setAborted(result);
    return;
  }
  
  this->state = hardware::BUSY;
  pthread_mutex_unlock(&this->state_mutex);

  while(get_controller_mode() != CRL_ARM_STOPPED){
    fprintf(stdout, "Waiting for the arm controller\n");
    sleep(1);
  }

  carlos_controller::JointMove srv;
  /*
  srv.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
  srv.request.overwrite = std::vector<uint8_t>(DOF);
  for(int32_t i=0; i<DOF; i++){
    srv.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
    srv.request.move.positions[i].unit = "rad";
    srv.request.move.positions[i].value = this->vision_position[i];
    srv.request.overwrite[i] = false;
  }

  if(!this->jointMoveClt.call(srv)){
    //result.error_string = std::string("Could not reach the detection position");
    this->projection_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  //set_controller_mode((arm_mode_t)srv.response.mode.data);

  while(get_controller_mode() == CRL_ARM_MOVING){
    fprintf(stdout, "Controller executing a movement\n");
    sleep(1);
  }

  if(get_controller_mode() != CRL_ARM_STOPPED){
    //result.error_string = std::string("Error moving the robot arm");
    this->projection_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }


  amn_welding_iface::SetSignal set_signal;
  set_signal.request.active = true;
  if(!this->setLaserClt.call(set_signal)){
    //result.error_string = std::string("Error activating laser line");
    this->projection_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  pthread_mutex_lock(&last_orient_mutex);
  this->last_orient = 0;
  pthread_mutex_unlock(&last_orient_mutex);

  mission_ctrl_msgs::SetMode set_mode;
  set_mode.request.mode = CRL_VIS_ORIENT;
  if(!this->visionSetModeClt.call(set_mode)){
    //result.error_string = std::string("Error changing the vision module working mode");
    this->projection_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  pthread_mutex_lock(&last_pose_mutex);
  this->last_pose = 0;
  pthread_mutex_unlock(&last_pose_mutex);

  while(get_last_pose() < 10){
    fprintf(stdout, "Waiting for studs distribution\n");
    sleep(1);
  }

  set_mode.request.mode = CRL_VIS_STOPPED;
  if(!this->visionSetModeClt.call(set_mode)){
    fprintf(stdout, "Error changing the vision module working mode\n");
  }

  set_signal.request.active = false;
  if(!this->setLaserClt.call(set_signal)){
    fprintf(stdout, "Error turning off the laser line\n");
  }

  //DPL: Transform and publish result
  tf::StampedTransform transform;
  try{
    this->listener.waitForTransform("base_footprint", "arm_6_link",
				    ros::Time(0), ros::Duration(2.0));
    this->listener.lookupTransform("base_footprint", "arm_6_link",
				   ros::Time(0), transform);
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
  }

  carlos_controller::CurrentJoints current_joints;
  this->getCurrentJointsClt.call(current_joints);

  double z_pos = 0.0;
  for(int32_t i=0; i<poses.pose_array.poses.size(); i++){
    z_pos += poses.pose_array.poses.at(i).position.z;
  }
  if(poses.pose_array.poses.size() > 0)
    z_pos /= poses.pose_array.poses.size();

 tf::StampedTransform tool_tf;
  try{
    this->listener.waitForTransform("arm_link", "tool_link",
				    ros::Time(0), ros::Duration(2.0));
    this->listener.lookupTransform("arm_link", "tool_link",
				   ros::Time(0), tool_tf);
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
  }
  
  tf::Quaternion q;
  tf::Vector3 trans(tool_tf.getOrigin().x() + cos(1.5708 + current_joints.response.joints.at(0))*(-poses.stiff1)+
		    z_pos*sin(1.5708 + current_joints.response.joints.at(0)), 
		    tool_tf.getOrigin().y() + sin(1.5708 + current_joints.response.joints.at(0))*(-poses.stiff1)-
		    z_pos*cos(1.5708 + current_joints.response.joints.at(0)), 0.0);
  q.setRPY(1.5708, 0, 1.5708-current_joints.response.joints.at(0));
  pthread_mutex_lock(&this->wall_tf_mutex);
  wall_tf.setOrigin(trans);
  wall_tf.setRotation(q);
  pthread_mutex_unlock(&this->wall_tf_mutex);
*/


  //carlos_controller::JointMove srv;
  srv.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
  srv.request.overwrite = std::vector<uint8_t>(DOF);
  for(int32_t i=0; i<DOF; i++){
    srv.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
    srv.request.move.positions[i].unit = "rad";
    srv.request.move.positions[i].value = this->projection_position[i];
    srv.request.overwrite[i] = false;
  }

  //srv.request.move.positions[0].value = current_joints.response.joints.at(0);

  if(!this->jointMoveClt.call(srv)){
    result.succeeded = false;
    this->projection_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  set_controller_mode((arm_mode_t)srv.response.mode.data);

  while(get_controller_mode() == CRL_ARM_MOVING){
    fprintf(stdout, "Controller executing a movement\n");
    sleep(1);
  }

  if(get_controller_mode() == CRL_ARM_STOPPED){
    result.succeeded = true;
    this->projection_server->setSucceeded(result);
  }else{
    result.succeeded = false;
    this->projection_server->setAborted(result);
  }
  
  set_state(hardware::IDLE);
}

void ArmInterface::executeWeldActHnd(const mission_ctrl_msgs::executeWeldGoalConstPtr& goal)
{
  fprintf(stdout, "Execute welding action\n");
  mission_ctrl_msgs::executeWeldResult result;

  pthread_mutex_lock(&this->state_mutex);
  if(this->state != hardware::IDLE){
    pthread_mutex_unlock(&this->state_mutex);
    result.error_string = std::string("The arm interface is busy");
    result.result_state = false;
    this->weld_server->setAborted(result);
    return;
  }

  this->state = hardware::BUSY;
  pthread_mutex_unlock(&this->state_mutex);

  geometry_msgs::Pose task_pose;
  std::vector<stud_t> pending;
  double pressure;
  int side;
  int voltage;
  read_stud_params(goal->task_name, &task_pose, &pending, &side, &voltage, &pressure);

  if(pending.size() < 1){
    fprintf(stdout, "Task with no pending studs\n");
    result.error_string = std::string("No pending studs");
    result.result_state = true;
    this->weld_server->setSucceeded(result);
    set_state(hardware::IDLE);
    return;
  }

  //DPL: get stiffener distance
  while(get_controller_mode() != CRL_ARM_STOPPED){
    fprintf(stdout, "Waiting for the arm controller\n");
    sleep(1);
  }

  carlos_controller::JointMove joint_mv;
  joint_mv.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
  joint_mv.request.overwrite = std::vector<uint8_t>(DOF);
  for(int32_t i=0; i<DOF; i++){
    joint_mv.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
    joint_mv.request.move.positions[i].unit = "rad";
    joint_mv.request.move.positions[i].value = this->vision_position[i];
    joint_mv.request.overwrite[i] = false;
  }

  if(side == 0)
    joint_mv.request.move.positions[0].value = -this->vision_position[0];

  if(!this->jointMoveClt.call(joint_mv)){
    result.error_string = std::string("Could not reach the detection position");
    result.result_state = false;
    this->weld_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  //set_controller_mode((arm_mode_t)joint_mv.response.mode.data);

  while(get_controller_mode() == CRL_ARM_MOVING){
    fprintf(stdout, "Controller executing a movement\n");
    sleep(1);
  }

  if(get_controller_mode() != CRL_ARM_STOPPED){
    result.error_string = std::string("Error moving the robot arm");
    result.result_state = false;
    this->weld_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  amn_welding_iface::SetSignal set_signal;
  set_signal.request.active = true;
  if(!this->setLaserClt.call(set_signal)){
    result.error_string = std::string("Error activating laser line");
    result.result_state = false;
    this->weld_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  pthread_mutex_lock(&last_orient_mutex);
  this->last_orient = 0;
  pthread_mutex_unlock(&last_orient_mutex);

  mission_ctrl_msgs::SetMode set_mode;
  set_mode.request.mode = CRL_VIS_ORIENT;
  if(!this->visionSetModeClt.call(set_mode)){
    result.error_string = std::string("Error changing the vision module working mode");
    result.result_state = false;
    this->weld_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }

  pthread_mutex_lock(&this->last_pose_mutex);
  this->last_pose = 0;
  pthread_mutex_unlock(&this->last_pose_mutex);

  while(get_last_pose() < 10){
    fprintf(stdout, "Waiting for studs distribution\n");
    sleep(1);
  }

  set_mode.request.mode = CRL_VIS_STOPPED;
  if(!this->visionSetModeClt.call(set_mode)){
    fprintf(stdout, "Error changing the vision module working mode\n");
  }

  set_signal.request.active = false;
  if(!this->setLaserClt.call(set_signal)){
    fprintf(stdout, "Error turning off the laser line\n");
  }

  tf::StampedTransform transform;
  try{
    this->listener.waitForTransform("base_footprint", "arm_6_link",
				    ros::Time(0), ros::Duration(2.0));
    this->listener.lookupTransform("base_footprint", "arm_6_link",
				   ros::Time(0), transform);
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
  }

  tf::Vector3 tmp_v(0.0, 0.0, 0.0);
  double height = transform(tmp_v).z() + BASE_HEIGHT;
  double z_pos = 0.0;
  fprintf(stdout, "--> Tool height: %f\n", height);
  fprintf(stdout, "--> Tool position in z: %f\n", transform(tmp_v).z());
  
  
  if(poses.pose_array.poses.size() == 0){
    //DPL: return
  }

  for(int32_t i=0; i<poses.pose_array.poses.size(); i++){
    z_pos += poses.pose_array.poses.at(i).position.z;
  }
  z_pos /= poses.pose_array.poses.size();

  //DPL: call get current pos
  carlos_controller::CurrentPose current_pose;
  this->getCurrentPoseClt.call(current_pose);
  
  for(int i=0; i<pending.size(); i++){
    stud_t tmp_stud = pending.at(i);
    
    mission_ctrl_msgs::executeWeldFeedback feedback;
    feedback.stud_id = std::string(tmp_stud.id.c_str());

    carlos_controller::JointConstraints pr_cons;
    pr_cons.request.constraints = std::vector<moveit_msgs::JointConstraint>(this->primary_cons.size());
    for(int32_t i=0; i<pr_cons.request.constraints.size(); i++){
      pr_cons.request.constraints[i].joint_name = primary_cons.at(i).joint_name;
      pr_cons.request.constraints[i].position = primary_cons.at(i).position;
      pr_cons.request.constraints[i].tolerance_above = primary_cons.at(i).tolerance_above;
      pr_cons.request.constraints[i].tolerance_below = primary_cons.at(i).tolerance_below;
      pr_cons.request.constraints[i].weight = primary_cons.at(i).weight;
    }
    pr_cons.request.orientation = this->orientation_cons;
    pr_cons.request.position = this->position_cons;
    
    if(!this->setConsClt.call(pr_cons)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not set the motion constraints");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    carlos_controller::JointMove init_mv;
    init_mv.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
    init_mv.request.overwrite = std::vector<uint8_t>(DOF);
    for(int32_t i=0; i<DOF; i++){
      init_mv.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
      init_mv.request.move.positions[i].unit = "rad";
      init_mv.request.move.positions[i].value = this->init_position[i];
      init_mv.request.overwrite[i] = false;
    }

    if(!this->jointMoveClt.call(init_mv)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not reach the init position");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    if(!this->setConsClt.call(pr_cons)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not set the motion constraints");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    fprintf(stdout, "vector x: %f\n", tmp_stud.y + height);
    //tf::Vector3 tmp_v(0.0, 0.0, 0.05);
    tf::Vector3 tmp_v(tmp_stud.y - height, tmp_stud.x + poses.stiff1, z_pos - 0.11);

    //DPL: Move arm to point
    carlos_controller::PoseMove pose1_mv;
    pose1_mv.request.move.header.frame_id = std::string(current_pose.response.pose.header.frame_id.c_str());
    pose1_mv.request.move.pose.position.x = transform(tmp_v).x();
    pose1_mv.request.move.pose.position.y = transform(tmp_v).y();
    pose1_mv.request.move.pose.position.z = transform(tmp_v).z();
    pose1_mv.request.move.pose.orientation.x = current_pose.response.pose.pose.orientation.x;
    pose1_mv.request.move.pose.orientation.y = current_pose.response.pose.pose.orientation.y;
    pose1_mv.request.move.pose.orientation.z = current_pose.response.pose.pose.orientation.z;
    pose1_mv.request.move.pose.orientation.w = current_pose.response.pose.pose.orientation.w;
 
    if(!this->poseMoveClt.call(pose1_mv)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not reach the desired approach pose");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    carlos_controller::JointConstraints sc_cons;
    sc_cons.request.constraints = std::vector<moveit_msgs::JointConstraint>(this->secondary_cons.size());
    for(int32_t i=0; i<sc_cons.request.constraints.size(); i++){
      sc_cons.request.constraints[i].joint_name = secondary_cons.at(i).joint_name;
      sc_cons.request.constraints[i].position = secondary_cons.at(i).position;
      sc_cons.request.constraints[i].tolerance_above = secondary_cons.at(i).tolerance_above;
      sc_cons.request.constraints[i].tolerance_below = secondary_cons.at(i).tolerance_below;
      sc_cons.request.constraints[i].weight = secondary_cons.at(i).weight;
    }
    sc_cons.request.orientation = this->orientation_cons;
    sc_cons.request.position = this->position_cons;
    
    if(!this->setConsClt.call(sc_cons)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not set the motion constraints");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    carlos_controller::CurrentJoints current_joints;
    this->getCurrentJointsClt.call(current_joints);
    for(int32_t i=0; i<current_joints.response.joints.size(); i++)
      fprintf(stdout, "Captured current joint value: %f\n", current_joints.response.joints.at(i));

    tf::StampedTransform transform2;
    try{
      this->listener.waitForTransform("base_footprint", "arm_6_link",
				      ros::Time(0), ros::Duration(2.0));
      this->listener.lookupTransform("base_footprint", "arm_6_link",
				     ros::Time(0), transform2);
    }catch(tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
    }

    carlos_controller::CurrentPose current_pose2;
    this->getCurrentPoseClt.call(current_pose2);
    tf::Vector3 vec_apr(0.0,0.0,0.1);

    carlos_controller::PoseMove pose2_mv;
    pose2_mv.request.move.header.frame_id = std::string(current_pose2.response.pose.header.frame_id.c_str());
    pose2_mv.request.move.pose.position.x = transform2(vec_apr).x();
    pose2_mv.request.move.pose.position.y = transform2(vec_apr).y();
    pose2_mv.request.move.pose.position.z = transform2(vec_apr).z();
    pose2_mv.request.move.pose.orientation.x = current_pose2.response.pose.pose.orientation.x;
    pose2_mv.request.move.pose.orientation.y = current_pose2.response.pose.pose.orientation.y;
    pose2_mv.request.move.pose.orientation.z = current_pose2.response.pose.pose.orientation.z;
    pose2_mv.request.move.pose.orientation.w = current_pose2.response.pose.pose.orientation.w;

    if(!this->poseMoveClt.call(pose2_mv)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not reach the desired approach pose");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }
    
    carlos_controller::FTMove ft_mv;
    ft_mv.request.pressure = pressure;
    if(!this->ftMoveClt.call(ft_mv)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not perform the contact");
      result.result_state = false;
      this->weld_server->setAborted(result);
      
      if(!this->poseMoveClt.call(pose1_mv)){
	fprintf(stdout, "Could not move the arm to a safe position\n");
      }
      set_state(hardware::IDLE);
      return;
    }

    while(!get_gnd_contact()){
      fprintf(stdout, "Waiting for ground contact\n");
      sleep(1);
    }
    
    std_srvs::Empty gun_srv;
    if(!this->fireGunClt.call(gun_srv)){
      result.error_string = std::string("Error activating laser line");
      this->weld_server->setAborted(result);
      if(!this->poseMoveClt.call(pose1_mv)){
	fprintf(stdout, "Could not move the arm to a safe position\n");
      }
      set_state(hardware::IDLE);
      return;
    }
    
    sleep(1);
    //exit(0);

    //pose_mv.request.move.pose.position.z = -0.1;
    
    carlos_controller::JointMove back_mv;
    back_mv.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
    back_mv.request.overwrite = std::vector<uint8_t>(DOF);
    for(int32_t i=0; i<DOF; i++){
      back_mv.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
      back_mv.request.move.positions[i].unit = "rad";
      back_mv.request.move.positions[i].value = current_joints.response.joints.at(i);
      back_mv.request.overwrite[i] = false;
    }

    if(!this->jointMoveClt.call(back_mv)){
      result.error_string = std::string("Could not reach the desired joint position");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    /*
    if(!this->setConsClt.call(pr_cons)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not set the motion constraints");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    carlos_controller::PoseMove pose0_mv;
    pose2_mv.request.move.header.frame_id = std::string(current_pose.response.pose.header.frame_id.c_str());
    pose2_mv.request.move.pose.position.x = current_pose.response.pose.pose.position.x;
    pose2_mv.request.move.pose.position.y = current_pose.response.pose.pose.position.y;
    pose2_mv.request.move.pose.position.z = current_pose.response.pose.pose.position.z;
    pose2_mv.request.move.pose.orientation.x = current_pose.response.pose.pose.orientation.x;
    pose2_mv.request.move.pose.orientation.y = current_pose.response.pose.pose.orientation.y;
    pose2_mv.request.move.pose.orientation.z = current_pose.response.pose.pose.orientation.z;
    pose2_mv.request.move.pose.orientation.w = current_pose.response.pose.pose.orientation.w;

    if(!this->poseMoveClt.call(pose0_mv)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not reach the desired approach pose");
      result.result_state = false;
      this->weld_server->setAborted(result);
      return;
    }
    */
    
    if(!this->jointMoveClt.call(init_mv)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not reach the init position");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    if(!this->setConsClt.call(pr_cons)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not set the motion constraints");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }
    
    //Feeder approach
    carlos_controller::JointMove feeder_mv;
    feeder_mv.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
    feeder_mv.request.overwrite = std::vector<uint8_t>(DOF);
    for(int32_t i=0; i<DOF; i++){
      feeder_mv.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
      feeder_mv.request.move.positions[i].unit = "rad";
      feeder_mv.request.move.positions[i].value = this->feeder_approach[i];
      feeder_mv.request.overwrite[i] = false;
    }
    
    if(!this->jointMoveClt.call(feeder_mv)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not reach the feeder approach position");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    //Feeder reload
    carlos_controller::JointMove feeder_rl;
    feeder_rl.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
    feeder_rl.request.overwrite = std::vector<uint8_t>(DOF);
    for(int32_t i=0; i<DOF; i++){
      feeder_rl.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
      feeder_rl.request.move.positions[i].unit = "rad";
      feeder_rl.request.move.positions[i].value = this->feeder_reload[i];
      feeder_rl.request.overwrite[i] = false;
    }
    
    if(!this->jointMoveClt.call(feeder_rl)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not reach the feeder reload position");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    std_srvs::Empty reload;
    if(!this->feederLoadClt.call(reload)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not load the stud");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    //Feeder exit
    carlos_controller::JointMove feeder_ex;
    feeder_ex.request.move.positions = std::vector<brics_actuator::JointValue>(DOF);
    feeder_ex.request.overwrite = std::vector<uint8_t>(DOF);
    for(int32_t i=0; i<DOF; i++){
      feeder_ex.request.move.positions[i].joint_uri = std::string(this->joint_names.at(i).c_str());
      feeder_ex.request.move.positions[i].unit = "rad";
      feeder_ex.request.move.positions[i].value = this->feeder_exit[i];
      feeder_ex.request.overwrite[i] = false;
    }
    
    if(!this->jointMoveClt.call(feeder_ex)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not reach the feeder exit position");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }    

    if(!this->jointMoveClt.call(feeder_mv)){
      feedback.stud_state = false;
      this->weld_server->publishFeedback(feedback);
      result.error_string = std::string("Could not reach the feeder approach position");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    mission_ctrl_msgs::SetMode feeder_set_mode;
    feeder_set_mode.request.mode = 1;
    if(!this->feederSetModeClt.call(feeder_set_mode)){
      result.error_string = std::string("Error changing the feeder module working mode");
      result.result_state = false;
      this->weld_server->setAborted(result);
      set_state(hardware::IDLE);
      return;
    }

    feedback.stud_state = true;
    this->weld_server->publishFeedback(feedback);
  }
    
  if(!this->move_home()){
    result.error_string = std::string("Could not reach the feeder approach position");
    result.result_state = false;
    this->weld_server->setAborted(result);
    set_state(hardware::IDLE);
    return;
  }
  
  result.error_string = std::string(" ");
  result.result_state = true;
  this->weld_server->setSucceeded(result);
  set_state(hardware::IDLE);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, ARM_INTERFACE_NAME);
  ros::NodeHandle nh;

  ArmInterface *iface = new ArmInterface(nh);
  ros::Timer timer = nh.createTimer(ros::Duration(1/DEFAULT_STATE_FREQ),
				    &ArmInterface::timerCallback, iface);
  ros::Timer tfTimer = nh.createTimer(ros::Duration(0.02),
				      &ArmInterface::tfCallback, iface);
  ros::spin();

  return 0;
}
