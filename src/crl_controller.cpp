
#include "carlos_controller/crl_controller.h"

#include <mission_ctrl_msgs/mission_ctrl_defines.h>
#include <std_srvs/Empty.h>

using namespace carlos;
using namespace aimen;

CRLController::CRLController(ros::NodeHandle node)
{
    this->node = node;
    this->ctrl_mode = CRL_ARM_STOPPED;

    this->group = new move_group_interface::MoveGroup("arm");
    //  move_group_interface::MoveGroup group(CRL_MOVE_GROUP);

    if(!CRLController::initialize_controller())
        ROS_ERROR("Could not initialize CARLoS arm controller");

    CRLController::install_params();

    /*
    ros::NodeHandle n;
    ros::ServiceClient cliente_studs_mode= n.serviceClient<mission_ctrl_msgs::SetMode>("studs_dist/set_mode");
    mission_ctrl_msgs::SetMode srv;

    srv.request.mode=CRL_VIS_ORIENT;
    cliente_studs_mode.call(srv);
    */

    CRLController::register_messages();
    CRLController::register_services();
    CRLController::subscribe_messages();

    this->contact = new ContactController(node, PI_4, 0.05);

    this->reference_name = this->group->getPoseReferenceFrame();
    fprintf(stdout, "Reference name: %s\n", this->reference_name.c_str());
}

/* Communication functions */
void CRLController::install_params(void)
{
  XmlRpc::XmlRpcValue effector;
  this->node.getParam(CRL_PARAM_EFFECTOR, effector);
  ROS_ASSERT(effector.getType() == XmlRpc::XmlRpcValue::TypeString);
  this->effector_name = static_cast<std::string>(effector);
 
  XmlRpc::XmlRpcValue tool;
  this->node.getParam(CRL_PARAM_TOOL, tool);
  ROS_ASSERT(tool.getType() == XmlRpc::XmlRpcValue::TypeString);
  this->tool_name = static_cast<std::string>(tool);

  XmlRpc::XmlRpcValue camera;
  this->node.getParam(CRL_PARAM_CAMERA, camera);
  ROS_ASSERT(camera.getType() == XmlRpc::XmlRpcValue::TypeString);
  this->camera_name = static_cast<std::string>(camera);

  XmlRpc::XmlRpcValue projector;
  this->node.getParam(CRL_PARAM_PROJECTOR, projector);
  ROS_ASSERT(projector.getType() == XmlRpc::XmlRpcValue::TypeString);
  this->projector_name = static_cast<std::string>(projector);

  // Read constraints
  /*
  XmlRpc::XmlRpcValue constr;
  this->node.getParam(CRL_PARAM_CONSTRAINTS, constr);
  ROS_ASSERT(constr.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int32_t i=0; i<constr.size(); i++){
    crl_constraints_t tmp_const;
    ROS_ASSERT(constr[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(constr[i][0].getType() == XmlRpc::XmlRpcValue::TypeString);
    tmp_const.name = static_cast<std::string>(constr[i][0]).c_str();
    ROS_ASSERT(constr[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    tmp_const.position = constr[i][1];
    ROS_ASSERT(constr[i][2].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    tmp_const.above = constr[i][2];
    ROS_ASSERT(constr[i][3].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    tmp_const.below = constr[i][3];
    ROS_ASSERT(constr[i][4].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    tmp_const.weight = constr[i][4];
    this->constraints.push_back(tmp_const);
  }
  */
}

void CRLController::register_messages(void)
{
  this->ctrlModePub = this->node.advertise<std_msgs::UInt8>(CRL_CTRL_MODE_MSG, 1);
}

void CRLController::register_services(void)
{
  this->getCtrlModeSrv = this->node.advertiseService(CRL_CTRL_GET_MODE_SRV,
						     &CRLController::getCtrlModeHnd, this);
  this->setCtrlModeSrv = this->node.advertiseService(CRL_CTRL_SET_MODE_SRV,
						     &CRLController::setCtrlModeHnd, this);
  this->currentPoseSrv = this->node.advertiseService(CRL_GET_CURRENT_POSE_SRV,
						     &CRLController::getCurrentPoseHnd, this);
  this->currentJointsSrv = this->node.advertiseService(CRL_GET_CURRENT_JOINTS_SRV,
						       &CRLController::getCurrentJointsHnd, this);
  this->jointMoveSrv = this->node.advertiseService(CRL_JOINT_MOVE_SRV,
						   &CRLController::jointMoveHnd, this);
  this->poseMoveSrv = this->node.advertiseService(CRL_POSE_MOVE_SRV,
						    &CRLController::poseMoveHnd, this);
  this->rpyMoveSrv = this->node.advertiseService(CRL_RPY_MOVE_SRV,
						 &CRLController::rpyMoveHnd, this);
  this->ftMoveSrv = this->node.advertiseService(CRL_FT_MOVE_SRV,
						&CRLController::ftMoveHnd, this);
  this->setConsSrv = this->node.advertiseService(CRL_SET_CONSTRAINTS_SRV,
						 &CRLController::setConstraintsHnd, this);

  this->ftmCalClt = this->node.serviceClient<std_srvs::Empty>(CRL_FTM75_CALIBRATE_SRV);
}

void CRLController::subscribe_messages(void)
{
  //DPL: subscribe movement result
}

void CRLController::unsubscribe_messages(void)
{
  
}

bool CRLController::initialize_controller(void)
{
  /*
  tf::TransformListener listener;
  ros::Time now = ros::Time::now() - ros::Duration(2.0);

  geometry_msgs::PoseStamped arm_pose = group->getCurrentPose("arm_6_link");
  try{
    listener.waitForTransform("camera_link", "arm_6_link",
			      now, ros::Duration(2.0));
    listener.transformPose("camera_link", arm_pose, this->arm_trn);
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    return false;
  }
  geometry_msgs::PoseStamped tool_pose = group->getCurrentPose("tool_link");
  try{
    listener.waitForTransform("camera_link", "tool_link",
			      now, ros::Duration(2.0));
    listener.transformPose("camera_link", tool_pose, this->tool_trn);
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    return false;
  }
  geometry_msgs::PoseStamped proj_pose = group->getCurrentPose("projector_link");
  try{
    listener.waitForTransform("camera_link", "projector_link",
			      now, ros::Duration(2.0));
    listener.transformPose("camera_link", proj_pose, this->proj_trn);
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    return false;
  }

  ROS_INFO("CRLController configuration loaded...");
  */

  return true;
}

/* service handlers */
bool CRLController::setConstraintsHnd(carlos_controller::JointConstraints::Request& request,
				      carlos_controller::JointConstraints::Response& response)
{
  if(this->ctrl_mode != CRL_ARM_STOPPED){
    fprintf(stdout, "Set constraints error. Controller busy\n");
    response.error = std::string("Controller busy");
    return false;
  }

  std::vector<double> joints = group->getCurrentJointValues();
  moveit_msgs::Constraints constraints;
  constraints.joint_constraints = std::vector<moveit_msgs::JointConstraint>(request.constraints.size());
  for(int32_t i=0; i<constraints.joint_constraints.size(); i++){
    constraints.joint_constraints[i].joint_name = std::string(request.constraints[i].joint_name.c_str());
    constraints.joint_constraints[i].position = joints.at(i);
    constraints.joint_constraints[i].tolerance_above = request.constraints[i].tolerance_above;
    constraints.joint_constraints[i].tolerance_below = request.constraints[i].tolerance_below;
    constraints.joint_constraints[i].weight = request.constraints[i].weight;
  }
  group->setPathConstraints(constraints);
  
  group->setGoalOrientationTolerance(request.orientation);
  group->setGoalPositionTolerance(request.position);
  
  return true;
}

bool CRLController::jointMoveHnd(carlos_controller::JointMove::Request& request,
				 carlos_controller::JointMove::Response& response)
{
  if(this->ctrl_mode != CRL_ARM_STOPPED){
    fprintf(stdout, "JointMove error. Controller busy\n");
    response.mode.data = this->ctrl_mode;
    return false;
  }else if(request.move.positions.size() != DOF){
    fprintf(stdout, "JointMove error. Incorrect degrees of freedom\n");
    response.mode.data = this->ctrl_mode;
    return false;
  }
  ros::AsyncSpinner spinner(1);
  spinner.start();

  fprintf(stdout, "Joint movement requested\n");
  group->clearPathConstraints();

  std::vector<double> joints_pose_act = group->getCurrentJointValues();
  for(int32_t i=0; i<DOF; i++)
    this->previous_joints[i] = joints_pose_act.at(i);
  std::vector<double> pose(DOF);
  for(int i=0; i<DOF; i++){
    if(request.overwrite[i])
      pose.at(i) = joints_pose_act.at(i);
    else
      pose.at(i) = request.move.positions.at(i).value;
    fprintf(stdout, "\t joint value: %f\n", pose.at(i));
  }
  
  group->setJointValueTarget(pose);
  set_ctrl_mode(CRL_ARM_MOVING);
  int ret = group->move();
  fprintf(stdout, "Movement finished with ret: %d\n", ret);

  set_ctrl_mode(CRL_ARM_STOPPED);

  response.mode.data = this->ctrl_mode;

  spinner.stop();

  if(ret == 1)
    return true;
  else
    return false;
}

bool CRLController::poseMoveHnd(carlos_controller::PoseMove::Request& request,
				carlos_controller::PoseMove::Response& response)
{
  
  if(this->ctrl_mode != CRL_ARM_STOPPED){
    fprintf(stdout, "PoseMove error. Controller busy\n");
    response.mode.data = this->ctrl_mode;
    return false;
  }/*else if(!check_frame(request.move.header.frame_id)){
    ROS_ERROR("Unknown frame %s", request.move.header.frame_id.c_str());
    response.mode.data = this->ctrl_mode;
    return false;
  }
  */
  ros::AsyncSpinner spinner(1);
  spinner.start();

  geometry_msgs::PoseStamped tmp_pose;
  tmp_pose.header.frame_id = std::string(request.move.header.frame_id.c_str());
  tmp_pose.pose.position.x = request.move.pose.position.x;
  tmp_pose.pose.position.y = request.move.pose.position.y;
  tmp_pose.pose.position.z = request.move.pose.position.z;
  tmp_pose.pose.orientation.x = request.move.pose.orientation.x;
  tmp_pose.pose.orientation.y = request.move.pose.orientation.y;
  tmp_pose.pose.orientation.z = request.move.pose.orientation.z;
  tmp_pose.pose.orientation.w = request.move.pose.orientation.w;

  fprintf(stdout, "Moving to: %f, %f, %f\n\t %f, %f, %f, %f\n", tmp_pose.pose.position.x, 
	  tmp_pose.pose.position.y, tmp_pose.pose.position.z, tmp_pose.pose.orientation.x,
	  tmp_pose.pose.orientation.y, tmp_pose.pose.orientation.z, tmp_pose.pose.orientation.w);

  std::vector<double> joints_pose_act = group->getCurrentJointValues();
  for(int32_t i=0; i<DOF; i++)
    this->previous_joints[i] = joints_pose_act.at(i);

  this->group->setPoseTarget(tmp_pose);
  //this->group->move();
  set_ctrl_mode(CRL_ARM_MOVING);
  int ret = this->group->move();
  fprintf(stdout, "Return executing movement: %d\n", ret);
  set_ctrl_mode(CRL_ARM_STOPPED);

  response.mode.data = this->ctrl_mode;
  spinner.stop();
  
  if(ret == 1)
    return true;
  else
    return false;
}

bool CRLController::rpyMoveHnd(carlos_controller::RPYMove::Request& request,
			       carlos_controller::RPYMove::Response& response)
{
  if(this->ctrl_mode != CRL_ARM_STOPPED){
    fprintf(stdout, "RPYMove error. Controller busy\n");
    response.mode.data = this->ctrl_mode;
    return false;
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<double> current_pose = group->getCurrentJointValues();
  std::vector<double> pose(DOF);
  pose.at(0) = current_pose.at(0) + request.move.point.z;
  pose.at(1) = current_pose.at(1) - request.move.point.y;
  pose.at(2) = current_pose.at(2);
  pose.at(3) = current_pose.at(3);
  pose.at(4) = current_pose.at(4);
  pose.at(5) = current_pose.at(5) + request.move.point.x;

  group->setJointValueTarget(pose);
  //group->setMaxVelocityScalingFactor(0.3);
  set_ctrl_mode(CRL_ARM_MOVING);
  group->move();
  //group->setMaxVelocityScalingFactor(1.0);
  spinner.stop();

  set_ctrl_mode(CRL_ARM_STOPPED);

  response.mode.data = this->ctrl_mode;
  return true;
}

bool CRLController::ftMoveHnd(carlos_controller::FTMove::Request& request,
			      carlos_controller::FTMove::Response& response)
{
  std_srvs::Empty srv;
  if(!this->ftmCalClt.call(srv)){
    response.mode.data = this->ctrl_mode;
    return false;
  }

  std::vector<double> current_joints = group->getCurrentJointValues();
  double vel[DOF];
  for(int32_t i=0; i<DOF; i++)
    vel[i] = (current_joints.at(i) - this->previous_joints[i])/7.0;

  this->contact->set_joint_velocities(vel, request.pressure);
  set_ctrl_mode(CRL_ARM_MOVING);
  this->contact->set_mode(CONTACT_MODE_AUTO);

  int counter = 0;
  while(contact->get_mode() == CONTACT_MODE_AUTO){
    ros::spinOnce();
    usleep(10000);
    counter++;
    if(counter > 1000){
      this->contact->set_mode(CONTACT_MODE_STOPPED);
      break;
    }
  }

  set_ctrl_mode(CRL_ARM_STOPPED);
  
  response.mode.data = this->ctrl_mode; 

  if(counter > 1000)
    return false;
  else
    return true;
}

/* Services handlers */
bool CRLController::getCtrlModeHnd(mission_ctrl_msgs::GetMode::Request& request,
				   mission_ctrl_msgs::GetMode::Response& response)
{
  response.mode = this->ctrl_mode;
  return true;
}

bool CRLController::setCtrlModeHnd(mission_ctrl_msgs::SetMode::Request& request,
				   mission_ctrl_msgs::SetMode::Response& response)
{
  switch(this->ctrl_mode){
  case CRL_ARM_STOPPED:
    
    break;
  case CRL_ARM_MOVING:
    if(request.mode == CRL_ARM_STOPPED){
      // DPL: stop group movement;
      set_ctrl_mode(CRL_ARM_STOPPED);
    }
    break;
  case CRL_ARM_ERROR:
    if(request.mode == CRL_ARM_STOPPED)
      set_ctrl_mode(CRL_ARM_STOPPED);
    break;
  }

  response.mode = this->ctrl_mode;
  return true;
}

bool CRLController::getCurrentPoseHnd(carlos_controller::CurrentPose::Request& request,
				      carlos_controller::CurrentPose::Response& response)
{
  response.pose = this->group->getCurrentPose();

  return true;
}

bool CRLController::getCurrentJointsHnd(carlos_controller::CurrentJoints::Request& request,
					carlos_controller::CurrentJoints::Response& response)
{
  std::vector<double> joints = group->getCurrentJointValues();
  response.joints = std::vector<double>(joints.size());
  for(int32_t i=0; i<response.joints.size(); i++)
    response.joints.at(i) = joints.at(i);

  return true;
}


void CRLController::timerCallback(const ros::TimerEvent& event)
{
  //DPL: 
}

void CRLController::set_ctrl_mode(arm_mode_t mode)
{
  if(this->ctrl_mode == mode)
    return;

  this->ctrl_mode = mode;
  std_msgs::UInt8 msg;
  fprintf(stdout, "Controller mod -> %d\n", this->ctrl_mode);
  msg.data = this->ctrl_mode;
  this->ctrlModePub.publish(msg);
}

bool CRLController::check_frame(std::string frame)
{
  if(frame.compare(this->reference_name) == 0)
    return true;
  else if(frame.compare(this->effector_name) == 0)
    return true;
  else if(frame.compare(this->tool_name) == 0)
    return true;
  else if(frame.compare(this->camera_name) == 0)
    return true;
  else if(frame.compare(this->projector_name) == 0)
    return true;

  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, CRL_CONTROLLER_NAME);
  ros::NodeHandle nh;

  //ros::spinOnce();
  CRLController *ctrl = new CRLController(nh);
  ros::Timer timer = nh.createTimer(ros::Duration(1.0), &CRLController::timerCallback, ctrl);
  
  ros::spin();

  return 0;
}
