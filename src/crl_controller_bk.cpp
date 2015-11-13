
#include "carlos_controller/crl_controller.h"

#include <mission_ctrl_msgs/mission_ctrl_defines.h>

using namespace carlos;
using namespace aimen;

CRLController::CRLController(ros::NodeHandle node)
{
    this->node = node;
    this->ctrl_mode = AMN_MODE_MANUAL;
    this->arm_mode = CRL_ARM_STOPPED;
    this->wld_mode = CRL_WLD_GTPHS;
    this->vision_mode = CRL_VIS_ORIENT;

    this->group = new move_group_interface::MoveGroup("arm");
    //  move_group_interface::MoveGroup group(CRL_MOVE_GROUP);

    if(!CRLController::initialize_controller())
        ROS_ERROR("Could not initialize CARLoS arm controller");

    CRLController::install_params();

    ros::NodeHandle n;
    ros::ServiceClient cliente_studs_mode= n.serviceClient<mission_ctrl_msgs::SetMode>("studs_dist/set_mode");
    mission_ctrl_msgs::SetMode srv;

    srv.request.mode=CRL_VIS_ORIENT;
    cliente_studs_mode.call(srv);

    CRLController::move_arm_vision_pose();
    ros::Duration(1).sleep();
    this->vision_mode = CRL_VIS_ORIENT;

    CRLController::register_messages();
    CRLController::register_services();
    CRLController::subscribe_messages();

    this->teach = new MNLController(node, -PI_4, 0.1);
    this->contact = new ContactController(node, PI_4, 0.05);

    this->reference_name = this->group->getPoseReferenceFrame();

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
}

void CRLController::register_messages(void)
{
  this->ctrlModePub = this->node.advertise<std_msgs::UInt8>(CRL_CTRL_MODE_MSG, 1);
  this->armModePub = this->node.advertise<std_msgs::UInt8>(CRL_ARM_MODE_MSG, 1);
  this->wldModePub = this->node.advertise<std_msgs::UInt8>(CRL_WLD_MODE_MSG, 1);
  this->studsPub = this->node.advertise<geometry_msgs::PoseArray>(CARLOS_STUDS_PRJ_MSG, 1);
}

void CRLController::register_services(void)
{
  this->getCtrlModeSrv = this->node.advertiseService(CRL_CTRL_GET_MODE_SRV,
						     &CRLController::getCtrlModeHnd, this);
  this->setCtrlModeSrv = this->node.advertiseService(CRL_CTRL_SET_MODE_SRV,
						     &CRLController::setCtrlModeHnd, this);
  this->getArmModeSrv = this->node.advertiseService(CRL_ARM_GET_MODE_SRV,
						    &CRLController::getArmModeHnd, this);
  this->setArmModeSrv = this->node.advertiseService(CRL_ARM_SET_MODE_SRV,
						    &CRLController::setArmModeHnd, this);
  this->getWldModeSrv = this->node.advertiseService(CRL_WLD_GET_MODE_SRV,
						    &CRLController::getWldModeHnd, this);
  this->studsModeClt = this->node.serviceClient<mission_ctrl_msgs::SetMode>(CARLOS_VISION_SET_MODE_SRV);
}

void CRLController::subscribe_messages(void)
{
  this->studsSubs = this->node.subscribe(CARLOS_STUDS_POS_MSG,
					 1, &CRLController::studPosesHnd, this);
  this->jointSubs = this->node.subscribe(CARLOS_JOINT_MOVE_MSG,
					 10, &CRLController::jointMoveHnd, this);
  this->linearSubs = this->node.subscribe(CARLOS_POSE_MOVE_MSG,
					  10, &CRLController::linearMoveHnd, this);
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

/* Topics handlers */
void CRLController::jointMoveHnd(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  if(!check_frame(msg->header.frame_id)){
    ROS_ERROR("Unknown frame %s", msg->header.frame_id.c_str());
    return;
  }

  for(size_t i=0; i<msg->poses.size(); i++){
    crl_movement_t tmp_mv;
    tmp_mv.type = 0;
    tmp_mv.pose.header.frame_id = std::string(msg->header.frame_id.c_str());
    tmp_mv.pose.pose.position.x = msg->poses[i].position.x;
    tmp_mv.pose.pose.position.y = msg->poses[i].position.y;
    tmp_mv.pose.pose.position.z = msg->poses[i].position.z;
    tmp_mv.pose.pose.orientation.x = msg->poses[i].orientation.x;
    tmp_mv.pose.pose.orientation.y = msg->poses[i].orientation.y;
    tmp_mv.pose.pose.orientation.z = msg->poses[i].orientation.z;
    tmp_mv.pose.pose.orientation.w = msg->poses[i].orientation.w;

    this->movements.push_back(tmp_mv);
  }

}

void CRLController::linearMoveHnd(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  if(!check_frame(msg->header.frame_id)){
    ROS_ERROR("Unknown frame %s", msg->header.frame_id.c_str());
    return;
  }

  for(size_t i=0; i<msg->poses.size(); i++){
    crl_movement_t tmp_mv;
    tmp_mv.type = 1;
    tmp_mv.pose.header.frame_id = std::string(msg->header.frame_id.c_str());
    tmp_mv.pose.pose.position.x = msg->poses[i].position.x;
    tmp_mv.pose.pose.position.y = msg->poses[i].position.y;
    tmp_mv.pose.pose.position.z = msg->poses[i].position.z;
    tmp_mv.pose.pose.orientation.x = msg->poses[i].orientation.x;
    tmp_mv.pose.pose.orientation.y = msg->poses[i].orientation.y;
    tmp_mv.pose.pose.orientation.z = msg->poses[i].orientation.z;
    tmp_mv.pose.pose.orientation.w = msg->poses[i].orientation.w;

    this->movements.push_back(tmp_mv);
  }
}


void CRLController::studPosesHnd(const geometry_msgs::PoseArray::ConstPtr& msg)
{

	ros::NodeHandle n;
	ros::ServiceClient cliente_studs_mode= n.serviceClient<mission_ctrl_msgs::SetMode>("studs_dist/set_mode");
	mission_ctrl_msgs::SetMode srv;

	srv.request.mode=CRL_VIS_STOPPED;
	cliente_studs_mode.call(srv);
	ROS_INFO_STREAM("msg "<<msg->header.seq);
  
  if(this->vision_mode == CRL_VIS_ORIENT && msg->header.frame_id=="orientation"){//mensaje modo tracking
    
    double roll, pitch, yaw;
    roll=msg->poses[0].position.x;
    pitch=msg->poses[0].position.y;
    yaw=msg->poses[0].position.z;
    
    if(fabs(yaw)>0.02){
      ROS_INFO_STREAM("reorientar yaw");
      orient_arm(0, 0, yaw);
      srv.request.mode=CRL_VIS_ORIENT;
       
    }else if(fabs(pitch)>0.02){
      ROS_INFO_STREAM("reorientar pitch");
      orient_arm(0, pitch, 0);
      srv.request.mode=CRL_VIS_ORIENT;

    }else if(fabs(roll)>0.02){
      ROS_INFO_STREAM("reorientar roll");
      orient_arm(roll, 0, 0);
      srv.request.mode=CRL_VIS_ORIENT;
      
    }else{
      this->vision_mode = CRL_VIS_DETECT; 
      srv.request.mode=CRL_VIS_DETECT;//cambia a modo detección
      ROS_INFO_STREAM("orientado (" << roll << ", " << pitch << ", "  << yaw << ")" ) ;
    }
    
    this->studsModeClt.call(srv);
    ros::Duration(1).sleep();//espera 1s.
  
  }else if(this->vision_mode == CRL_VIS_DETECT && msg->header.frame_id!="idle"){
    this->studs_pos.clear();
    this->studs_proj.clear();
    
    publish_ctrl_mode();


    ros::AsyncSpinner spinner(1);
    spinner.start();
    
     
    geometry_msgs::PoseStamped current_pose;
    current_pose = group->getCurrentPose(); 
    
    //guarda pos actual
    group->rememberJointValues("posicion_inicial");
    
    
    tf::StampedTransform transform;
    listener.lookupTransform("base_link","arm_6_link",ros::Time(0), transform);
    
    crl_movement_t ini_pose;
    ini_pose.type = 1;
    ini_pose.pose.header.frame_id = "/base_link";
    ini_pose.pose.pose = current_pose.pose;

    
    for(size_t i=0; i<msg->poses.size(); i++){
      geometry_msgs::PoseStamped tmp_pose;
      geometry_msgs::PoseStamped tmp_proj;
      
      tmp_pose.header.frame_id = this->camera_name.c_str();
      tmp_pose.pose.position = msg->poses[i].position;
      tmp_pose.pose.orientation = msg->poses[i].orientation;

      //resta 0.1m en eje z de herramienta
      tf::Vector3 vectorTarg(msg->poses[i].position.x,msg->poses[i].position.y,msg->poses[i].position.z-0.1);
      
      crl_movement_t tmp_mv;
      tmp_mv.type = 1;
      tmp_mv.pose.header.frame_id = this->camera_name.c_str();
      tmp_mv.pose.pose.position.x = transform(vectorTarg).x();
      tmp_mv.pose.pose.position.y = transform(vectorTarg).y();
      tmp_mv.pose.pose.position.z = transform(vectorTarg).z();
      tmp_mv.pose.pose.orientation = current_pose.pose.orientation;
      
      
      try{
	move_point(&tmp_mv, i);
	
      }catch(tf::TransformException ex){
	ROS_ERROR("%s"," er00 ", ex.what());
      }catch(...){
	ROS_ERROR("%s"," er01 ");
      }
      
      this->studs_pos.push_back(tmp_pose);
      this->studs_proj.push_back(tmp_proj);
      
    }
    
    
    ROS_INFO_STREAM("volviendo a inicio: ");
    
    //vuelve a pos inicial
    group->setNamedTarget("posicion_inicial");
    group->move();
	
    spinner.stop();


    srv.request.mode=CRL_VIS_ORIENT;//cambia modulo vision a modo tracking
    this->vision_mode = CRL_VIS_ORIENT;
	cliente_studs_mode.call(srv);
	ros::Duration(1).sleep();

  }
  else{
    srv.request.mode=CRL_VIS_ORIENT;
    this->vision_mode = CRL_VIS_ORIENT;
    cliente_studs_mode.call(srv);
    ros::Duration(1).sleep();
  }
  
  publish_studs();
}

void CRLController::publish_studs(void)
{
  geometry_msgs::PoseArray msg;
  msg.poses = std::vector<geometry_msgs::Pose>(this->studs_proj.size());
  for(int i=0; i<this->studs_proj.size(); i++){
    msg.poses[i].position.x = studs_proj.at(i).pose.position.x;
    msg.poses[i].position.y = studs_proj.at(i).pose.position.y;
    msg.poses[i].position.z = studs_proj.at(i).pose.position.z;
    msg.poses[i].orientation.x = studs_proj.at(i).pose.orientation.x;
    msg.poses[i].orientation.y = studs_proj.at(i).pose.orientation.y;
    msg.poses[i].orientation.z = studs_proj.at(i).pose.orientation.z;
    msg.poses[i].orientation.w = studs_proj.at(i).pose.orientation.w;
  }
  this->studsPub.publish(msg);
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
  case AMN_MODE_MANUAL:
    this->ctrl_mode = (running_mode_t)request.mode;
    if(this->ctrl_mode == AMN_MODE_CALIBRATE){
      this->teach->set_mode(MNL_MODE_CALIBRATE);
    }
    
    break;
  case AMN_MODE_AUTO:
    if(request.mode != AMN_MODE_CALIBRATE)
      this->ctrl_mode = (running_mode_t)request.mode;

    break;
  case AMN_MODE_ALARM:
    if(request.mode == AMN_MODE_MANUAL)
      this->ctrl_mode = AMN_MODE_MANUAL;
    
    break;
  case AMN_MODE_ERROR:
    if(request.mode == AMN_MODE_MANUAL)
      this->ctrl_mode = AMN_MODE_MANUAL;
    
    break;
  case AMN_MODE_CALIBRATE:
    if(request.mode != AMN_MODE_AUTO)
      this->ctrl_mode = (running_mode_t)request.mode;
    
    break;
  }
  
  response.mode = this->ctrl_mode;
  if(request.mode != response.mode)
    this->publish_ctrl_mode();
  
  return true;
}

bool CRLController::getArmModeHnd(mission_ctrl_msgs::GetMode::Request& request,
				  mission_ctrl_msgs::GetMode::Response& response)
{
  response.mode = this->arm_mode;
  return true;
}

bool CRLController::setArmModeHnd(mission_ctrl_msgs::SetMode::Request& request,
				  mission_ctrl_msgs::SetMode::Response& response)
{
  if(this->ctrl_mode != AMN_MODE_AUTO)
    return false;

  switch(this->arm_mode){
  case CRL_ARM_STOPPED:
    this->arm_mode = (arm_mode_t)request.mode;
    
    break;
  case CRL_ARM_MOVING: CRL_ARM_WELDING: CRL_ARM_GUIDED:
    if((arm_mode_t)request.mode == CRL_ARM_STOPPED)
      this->arm_mode = (arm_mode_t)request.mode;
    
    break;
  }

  response.mode = this->ctrl_mode;
  if(request.mode != response.mode)
    this->publish_arm_mode();

  return true;
}

bool CRLController::getWldModeHnd(mission_ctrl_msgs::GetMode::Request& request,
				  mission_ctrl_msgs::GetMode::Response& response)
{
  response.mode = this->wld_mode;
  return true;
}

void CRLController::timerCallback(const ros::TimerEvent& event)
{
  switch(this->ctrl_mode){
  case AMN_MODE_CALIBRATE:

    switch(this->teach->get_mode()){
    case MNL_MODE_STOPPED:
      this->ctrl_mode = AMN_MODE_MANUAL;
      
      break;
    case MNL_MODE_ERROR:
      this->ctrl_mode = AMN_MODE_ERROR;
      
      break;
    }

    break;
  case AMN_MODE_AUTO:
    
    switch(this->arm_mode){
    case CRL_ARM_STOPPED:
      if(this->movements.size() != 0){
	//DPL: execute movement
	if(move_arm(&this->movements.at(0)) == 0){
	  this->movements.erase(this->movements.begin());
	  this->arm_mode = CRL_ARM_MOVING;
	}
      }

      if(this->studs_pos.size() != 0){
	//std::vector<geometry_msgs::Pose> w_points;
	//move_group_interface::MoveGroup group("arm");
	/*
	moveit_msgs::Constraints constraints;
	constraints.joint_constraints = std::vector<moveit_msgs::JointConstraint>(2);
	constraints.joint_constraints[0].joint_name = "arm_3_joint";
	constraints.joint_constraints[0].position = 1.5708;
	constraints.joint_constraints[0].tolerance_above = 1.5708;
	constraints.joint_constraints[0].tolerance_below = 1.5708;
	constraints.joint_constraints[0].weight = 1.0;
	constraints.joint_constraints[1].joint_name= "arm_2_joint";
	constraints.joint_constraints[1].position = 0.0;
	constraints.joint_constraints[1].tolerance_above = 0.7854;
	constraints.joint_constraints[1].tolerance_below = 1.5708;
	constraints.joint_constraints[1].weight = 1.0;
	group->setPathConstraints(constraints);
	*/
	geometry_msgs::PoseStamped tmp_pose = this->studs_pos.at(0);
	ROS_INFO("--> Camera detection");
	ROS_INFO("X Pos: %f", tmp_pose.pose.position.x);
	ROS_INFO("Y Pos: %f", tmp_pose.pose.position.y);
	ROS_INFO("Z Pos: %f", tmp_pose.pose.position.z);
	ROS_INFO("X Ori: %f", tmp_pose.pose.orientation.x);
	ROS_INFO("Y Ori: %f", tmp_pose.pose.orientation.y);
	ROS_INFO("Z Ori: %f", tmp_pose.pose.orientation.z);
	ROS_INFO("W Ori: %f", tmp_pose.pose.orientation.w);
	
	/*
	geometry_msgs::PoseStamped current_pose2 = group->getCurrentPose("camera_link");
	ROS_INFO("--> Camera current pose");
	ROS_INFO("X CPos: %f", current_pose2.pose.position.x);
	ROS_INFO("Y CPos: %f", current_pose2.pose.position.y);
	ROS_INFO("Z CPos: %f", current_pose2.pose.position.z);
	ROS_INFO("X COri: %f", current_pose2.pose.orientation.x);
	ROS_INFO("Y COri: %f", current_pose2.pose.orientation.y);
	ROS_INFO("Z COri: %f", current_pose2.pose.orientation.z);
	ROS_INFO("W COri: %f", current_pose2.pose.orientation.w);
	ROS_INFO("----------------------");
	*/

	//tf::TransformListener listener;
	tf::StampedTransform transform;
	try{
	  listener.waitForTransform("camera_link", "arm_6_link",
				    ros::Time(0), ros::Duration(2.0));
	  listener.lookupTransform("camera_link", "arm_6_link",
				   ros::Time(0), transform);
	}catch(tf::TransformException ex){
	  ROS_ERROR("%s", ex.what());
	}
	
	tf::Transform tfr;
	tfr.setOrigin(tf::Vector3(tmp_pose.pose.position.x,
				  tmp_pose.pose.position.y, tmp_pose.pose.position.z));
	tfr.setRotation(tf::Quaternion(tmp_pose.pose.orientation.x, tmp_pose.pose.orientation.y,
				       tmp_pose.pose.orientation.z, tmp_pose.pose.orientation.w));
	tf::Transform tr = tfr*transform;
	
	ROS_INFO("--> Transform from camera to arm");
	ROS_INFO("X TPos: %f", tr.getOrigin().x());
	ROS_INFO("Y TPos: %f", tr.getOrigin().y());
	ROS_INFO("Z TPos: %f", tr.getOrigin().z());
	ROS_INFO("X TOri: %f", tr.getRotation().x());
	ROS_INFO("Y TOri: %f", tr.getRotation().y());
	ROS_INFO("Z TOri: %f", tr.getRotation().z());
	ROS_INFO("W TOri: %f", tr.getRotation().w());
	ROS_INFO("----------------------");

	tmp_pose.pose.position.x = tr.getOrigin().x();
	tmp_pose.pose.position.y = tr.getOrigin().y();
	tmp_pose.pose.position.z = tr.getOrigin().z();
	tmp_pose.pose.orientation.x = tr.getRotation().x();
	tmp_pose.pose.orientation.y = tr.getRotation().y();
	tmp_pose.pose.orientation.z = tr.getRotation().z();
	tmp_pose.pose.orientation.w = tr.getRotation().w();

	geometry_msgs::PoseStamped tmp;
	try{
	  listener.waitForTransform("base_link", "arm_6_link",
				    ros::Time(0), ros::Duration(2.0));
	  listener.transformPose("base_link", tmp_pose, tmp);
	}catch(tf::TransformException ex){
	  ROS_ERROR("%s", ex.what());
	}
	ROS_INFO("--> Final position");
	ROS_INFO("X PPos: %f", tmp.pose.position.x);
	ROS_INFO("Y PPos: %f", tmp.pose.position.y);
	ROS_INFO("Z PPos: %f", tmp.pose.position.z);
	ROS_INFO("X POri: %f", tmp.pose.orientation.x);
	ROS_INFO("Y POri: %f", tmp.pose.orientation.y);
	ROS_INFO("Z POri: %f", tmp.pose.orientation.z);
	ROS_INFO("W POri: %f", tmp.pose.orientation.w);
	ROS_INFO("----------------------");

	//moveit_msgs::RobotTrajectory r_trj;
	//move_group_interface::MoveGroup::Plan plan;
	//double ccp = group.computeCartesianPath(w_points, 0.01, 0.0, plan.trajectory_, true);
	//ROS_INFO("Cartesian Path %f", ccp);
	//group.asyncExecute(plan);
	group->setPoseTarget(tmp);
	group->asyncMove();
	this->studs_pos.erase(studs_pos.begin());
      }

      break;
    case CRL_ARM_MOVING:
      
      break;
    case CRL_ARM_WELDING:
      
      switch(this->wld_mode){
      case CRL_WLD_STOPPED:
	//DPL: set goal and move

	break;
      case CRL_WLD_GTPHS:

	break;
      case CRL_WLD_GTPLS:

	break;
      case CRL_WLD_GTPC:

	break;
      case CRL_WLD_WELD:

	break;
      case CRL_WLD_MBLS:

	break;
      case CRL_WLD_CHECK:

	break;
      case CRL_WLD_MBHS:

	break;
      }
      
      break;
    case CRL_ARM_GUIDED:
      
      break;
    }

    break;
  }
}

void CRLController::publish_ctrl_mode(void)
{
  std_msgs::UInt8 msg;
  fprintf(stdout, "Controller mod -> %d\n", this->ctrl_mode);
  msg.data = this->ctrl_mode;
  this->ctrlModePub.publish(msg);
}

void CRLController::publish_arm_mode(void)
{
  std_msgs::UInt8 msg;
  fprintf(stdout, "Arm mode -> %d\n", this->arm_mode);
  msg.data = this->arm_mode;
  this->armModePub.publish(msg);
}

void CRLController::publish_wld_mode(void)
{
  std_msgs::UInt8 msg;
  msg.data = this->wld_mode;
  this->wldModePub.publish(msg);
}

bool CRLController::check_frame(std::string frame)
{
  if(frame.compare(this->effector_name) == 0)
    return true;
  else if(frame.compare(this->tool_name) == 0)
    return true;
  else if(frame.compare(this->camera_name) == 0)
    return true;
  else if(frame.compare(this->projector_name) == 0)
    return true;

  return false;
}




int CRLController::move_arm(crl_movement_t* mv)
{
  geometry_msgs::PoseStamped tmp_pose;
  geometry_msgs::PoseStamped ref_pose;
  
   try{
    listener.waitForTransform(this->effector_name, mv->pose.header.frame_id,
			      ros::Time(0), ros::Duration(TF_TOUT));
    listener.transformPose(this->effector_name, tmp_pose, mv->pose);
  }
  catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    return -1;
  }

  try{
    listener.waitForTransform(this->reference_name, this->effector_name,
			      ros::Time(0), ros::Duration(TF_TOUT));
    listener.transformPose(this->reference_name, ref_pose, tmp_pose);
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    return -1;
  }

  if(!this->group->setPoseTarget(ref_pose)){
    ROS_ERROR("It is not possible to set the target pose");
    //DPL: error setting the target
    return -1;
  }
  this->group->asyncMove();
  
 
  return 0;
}



void CRLController::orient_arm(double r,double p, double y){
    //reorienta en funcion de angulos RPY
   ros::AsyncSpinner spinner(1);
    spinner.start();

    std::vector<double>joints_pose_act=group->getCurrentJointValues();
    double joints_pose[]={	joints_pose_act[0]+y, 
				            joints_pose_act[1]-p,
				            joints_pose_act[2],
				            joints_pose_act[3],
				            joints_pose_act[4],
				            joints_pose_act[5]+r};
 
    const std::vector<double> pose(joints_pose, joints_pose+6);

    group->setJointValueTarget(pose);
    group->move();

    spinner.stop();



}


void CRLController::move_arm_vision_pose()
{
  //initial pose for vision
  ros::AsyncSpinner spinner(1);
  spinner.start();
  double joints_pose_vision[]={-1.5708, 1.6, 2, 0, 1.1708, 0};
  const std::vector<double> pose_vision (joints_pose_vision, joints_pose_vision+6);
  
  group->setJointValueTarget(pose_vision);
  group->move();
  
  spinner.stop();
}


int CRLController::move_point(crl_movement_t* mv, int i)
{
  geometry_msgs::PoseStamped target_pose;
  geometry_msgs::PoseStamped tmp_pose;
  geometry_msgs::PoseStamped tmp2_pose;
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped init_pose;
  geometry_msgs::PoseStamped reload_pose;
  
  current_pose = group->getCurrentPose();
  
  moveit_msgs::Constraints constraints;
  constraints.joint_constraints = std::vector<moveit_msgs::JointConstraint>(4);
  constraints.joint_constraints[0].joint_name= "arm_1_joint";
  constraints.joint_constraints[0].position = group->getCurrentJointValues()[0];
  constraints.joint_constraints[0].tolerance_above = 1.5708;
  constraints.joint_constraints[0].tolerance_below = 1.8708;
  constraints.joint_constraints[0].weight = 1.0;
  constraints.joint_constraints[1].joint_name = "arm_2_joint";
  constraints.joint_constraints[1].position = group->getCurrentJointValues()[1];
  constraints.joint_constraints[1].tolerance_above = 2.3562;
  constraints.joint_constraints[1].tolerance_below = 1.5708;
  constraints.joint_constraints[1].weight = 1.0;
  constraints.joint_constraints[2].joint_name = "arm_3_joint";
  constraints.joint_constraints[2].position = group->getCurrentJointValues()[2];
  constraints.joint_constraints[2].tolerance_above = 1.5708;
  constraints.joint_constraints[2].tolerance_below = 1.5708;
  constraints.joint_constraints[2].weight = 1.0;
  constraints.joint_constraints[3].joint_name = "arm_4_joint";
  constraints.joint_constraints[3].position = group->getCurrentJointValues()[3];
  constraints.joint_constraints[3].tolerance_above = 1.5708;
  constraints.joint_constraints[3].tolerance_below = 1.5708;
  constraints.joint_constraints[3].weight = 1.0;
  
  group->setPathConstraints(constraints);
  
  group->setGoalOrientationTolerance(0.002);
  group->setGoalPositionTolerance(0.002);
  
  double joints_ini_d[]={group->getCurrentJointValues()[0], 0.4, 2.2708, 0, -0.3, 0};
  const std::vector<double> joints_ini (joints_ini_d, joints_ini_d+6);
  
  double joints_reload_apr[]={-0.2, -0.4277, 1.2441, -0.0004, 1.4664, 1.3260};
  const std::vector<double> joints_reload1 (joints_reload_apr, joints_reload_apr+6);
  
  double joints_reload_entrada[]={-0.2, -0.5616, 1.5382, -0.0001, 1.0448, 1.3357};
  const std::vector<double> joints_reload2 (joints_reload_entrada, joints_reload_entrada+6);
  
  double joints_reload_salida[]={-0.2, -0.6776, 1.3519, 0.0, 1.1147, 1.3724};
  const std::vector<double> joints_reload3 (joints_reload_salida, joints_reload_salida+6);


    group->setJointValueTarget(joints_ini);
    group->move();

    constraints.joint_constraints[0].position = group->getCurrentJointValues()[0];
    constraints.joint_constraints[1].position = group->getCurrentJointValues()[1];
    constraints.joint_constraints[2].position = group->getCurrentJointValues()[2];
    constraints.joint_constraints[3].position = group->getCurrentJointValues()[3];

    group->setPathConstraints(constraints);

    target_pose = group->getCurrentPose();
    target_pose.pose    = mv->pose.pose;
   
    ROS_INFO_STREAM("punto " << i << ": (" << target_pose.pose.position.x << ", "<< target_pose.pose.position.y << ", "<< target_pose.pose.position.z << "): ");

    group->setPoseTarget(target_pose);//pose punto i

    int errT;
    errT=group->move();

    if(errT==1){


	    moveit_msgs::Constraints constraintsf;
	    constraintsf.joint_constraints = std::vector<moveit_msgs::JointConstraint>(4);
	    constraintsf.joint_constraints[0].joint_name= "arm_1_joint";
	    constraintsf.joint_constraints[0].position = group->getCurrentJointValues()[0];
	    constraintsf.joint_constraints[0].tolerance_above = 0.7;
	    constraintsf.joint_constraints[0].tolerance_below = 0.7;
	    constraintsf.joint_constraints[0].weight = 1.0;
	    constraintsf.joint_constraints[1].joint_name = "arm_2_joint";
	    constraintsf.joint_constraints[1].position = group->getCurrentJointValues()[1];
	    constraintsf.joint_constraints[1].tolerance_above = 0.9;
	    constraintsf.joint_constraints[1].tolerance_below = 0.9;
	    constraintsf.joint_constraints[1].weight = 1.0;
	    constraintsf.joint_constraints[2].joint_name = "arm_3_joint";
	    constraintsf.joint_constraints[2].position = group->getCurrentJointValues()[2];
	    constraintsf.joint_constraints[2].tolerance_above = 1.2;
	    constraintsf.joint_constraints[2].tolerance_below = 1.2;
	    constraintsf.joint_constraints[2].weight = 1.0;
	    constraintsf.joint_constraints[3].joint_name = "arm_4_joint";
	    constraintsf.joint_constraints[3].position = group->getCurrentJointValues()[3];
	    constraintsf.joint_constraints[3].tolerance_above = 1.2;
	    constraintsf.joint_constraints[3].tolerance_below = 1.2;
	    constraintsf.joint_constraints[3].weight = 1.0;

	    group->setPathConstraints(constraintsf);


	    tf::StampedTransform transform2;
	    listener.lookupTransform("base_link","arm_6_link",ros::Time(0), transform2);
	    tf::Vector3 vectorApr(0.0,0.0,0.1);//punto de contacto
	    tmp_pose = group->getCurrentPose();
	    tmp_pose.pose.position.x =transform2(vectorApr).x();
	    tmp_pose.pose.position.y =transform2(vectorApr).y();
	    tmp_pose.pose.position.z =transform2(vectorApr).z();

	    tf::Vector3 vectorApr2(0.0,0.0,0.12);//punto destino
	    tmp2_pose = group->getCurrentPose();
	    tmp2_pose.pose.position.x =transform2(vectorApr2).x();
	    tmp2_pose.pose.position.y =transform2(vectorApr2).y();
	    tmp2_pose.pose.position.z =transform2(vectorApr2).z();


	    int errT2;
	    int errT3;

        std::vector<geometry_msgs::Pose> waypoints;
        std::vector<geometry_msgs::Pose> waypoints2;
        moveit::planning_interface::MoveGroup::Plan plan;
        moveit_msgs::RobotTrajectory trajectory;

	    ROS_INFO_STREAM("aprox punto " << i );

        //movimiento cartesiano
        //waypoints.push_back(tmp2_pose.pose);
        //waypoints.push_back(tmp_pose.pose);
        //group->computeCartesianPath(waypoints,0.01,0.0,trajectory);
        //plan.trajectory_=trajectory;
        //errT2=group->execute(plan);
        

        std::vector<double> aprox_joints;
        aprox_joints=group->getCurrentJointValues();

        ros::ServiceClient client_ftm_calib= node.serviceClient<std_srvs::Empty>("ftm75/calibrate");
        std_srvs::Empty::Request req;
        std_srvs::Empty::Response res;
        client_ftm_calib.call(req, res);

	
        group->setPoseTarget(tmp_pose);
	    errT2=group->move();

        //group->setPoseTarget(tmp2_pose);
        //const robot_state::RobotState target_state_contact=group->getJointValueTarget();
        
        //std::vector<double> target_joints_contact;
        //target_state_contact.copyJointGroupPositions("arm",target_joints_contact);
 
 

        std::vector<double> current_joints_contact;
        current_joints_contact=group->getCurrentJointValues();



        double vel[DOF];
        vel[0] = (current_joints_contact[0]-aprox_joints[0])/5.0;
        vel[1] = (current_joints_contact[1]-aprox_joints[1])/5.0;
        vel[2] = (current_joints_contact[2]-aprox_joints[2])/5.0;
        vel[3] = (current_joints_contact[3]-aprox_joints[3])/5.0;
        vel[4] = (current_joints_contact[4]-aprox_joints[4])/5.0;
        vel[5] = (current_joints_contact[5]-aprox_joints[5])/5.0;
       
        this->contact->set_joint_velocities(vel);
        this->contact->set_mode(CONTACT_MODE_AUTO);

        ros::Duration(4).sleep();

        this->contact->set_mode(CONTACT_MODE_STOPPED);
        	
	    ROS_INFO_STREAM("ret punto " << i );
	    group->setPoseTarget(target_pose);

	    group->move();


	    group->setPathConstraints(constraints);

	      
	    group->setJointValueTarget(joints_ini);
	    errT3=group->move();

	    if(errT2==1 && errT3==1 ){
            //si se ha alcanzado el punto de soldadura va a posición de recarga

        /*
       */ 
            constraints.joint_constraints[0].position = group->getCurrentJointValues()[0];
            constraints.joint_constraints[1].position = group->getCurrentJointValues()[1];
            constraints.joint_constraints[2].position = group->getCurrentJointValues()[2];
            constraints.joint_constraints[3].position = group->getCurrentJointValues()[3];

            group->setPathConstraints(constraints);

	        group->setJointValueTarget(joints_reload1);//aprox recarga
	        group->move();

	        group->setJointValueTarget(joints_reload2);//recarga
	        group->move();

	        //group->setJointValueTarget(joints_reload3);//salida recarga
	        //group->move();

	        group->setJointValueTarget(joints_reload1);//vuelve a aprox recarga
	        group->move();




        /*
            double joints_reload_pruebas[]={1.57, 0.4, 2.07, 0, -0.1, 0};
            const std::vector<double> joints_reload4 (joints_reload_pruebas, joints_reload_pruebas+6);
	        group->setJointValueTarget(joints_reload4);//vuelve a recarga pruebas
	        group->move();
        */

	        group->setJointValueTarget(joints_ini);
	        group->move();


	
	    }
	       else{
		    ROS_INFO_STREAM("   contacto punto " << i << " no alcanzable");
	       }

   }
   else{
        ROS_INFO_STREAM("   aproximacion punto " << i << " no alcanzable");
   }
   

  return 0;
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
