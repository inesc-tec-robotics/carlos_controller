
#ifndef CRL_CONTROLLER_H
#define CRL_CONTROLLER_H

#include <ros/ros.h>
#include <amn_common/amn_common.h>
#include <mission_ctrl_msgs/GetMode.h>
#include <mission_ctrl_msgs/SetMode.h>
#include <std_msgs/UInt8.h>
#include <amn_common/amn_mnl_defines.h>
#include <amn_common/amn_mnl_lib.h>
#include <carlos_controller/CurrentPose.h>
#include <carlos_controller/CurrentJoints.h>
#include <carlos_controller/JointConstraints.h>
#include <carlos_controller/FTMove.h>
#include <carlos_controller/JointMove.h>
#include <carlos_controller/PoseMove.h>
#include <carlos_controller/RPYMove.h>
#include <geometry_msgs/PoseArray.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/transforms/transforms.h>
#include <tf/transform_listener.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

//#include <brics_actuator/JointPositions.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_srvs/Empty.h>

#include "carlos_controller/crl_defines.h"
#include "carlos_controller/crl_contact.h"

#define CRL_CONTROLLER_NAME "CARLoS_controller"
#define CRL_MOVE_GROUP "arm"
#define TF_TOUT 2.0

typedef enum{
  CRL_ARM_STOPPED = 0,
  CRL_ARM_MOVING  = 1,
  CRL_ARM_ERROR = 2
} arm_mode_t;

typedef enum{
  CRL_VIS_STOPPED = 0,
  CRL_VIS_DETECT  = 1,
  CRL_VIS_ORIENT  = 2,
} vision_mode_t;

typedef struct{
  const char* name;
  double position;
  double above;
  double below;
  double weight;
} crl_constraints_t;

typedef struct{
  int type; //0=JOINT, 1=LINEAR
  geometry_msgs::PoseStamped pose;
} crl_movement_t;

namespace carlos
{
  class CRLController{
  public:
    CRLController(ros::NodeHandle node);
    ~CRLController(){};

    void timerCallback(const ros::TimerEvent&);

  private:
    void install_params(void);
    void register_messages(void);
    void register_services(void);
    void subscribe_messages(void);
    void unsubscribe_messages(void);
    
    bool initialize_controller(void);

    /* messages handlers */
    //void jointMoveHnd(const geometry_msgs::PoseArray::ConstPtr&);
    //void linearMoveHnd(const geometry_msgs::PoseArray::ConstPtr&);
    //void studPosesHnd(const geometry_msgs::PoseArray::ConstPtr&);
    //void jointMoveHnd(const brics_actuator::JointPositions::ConstPtr&);
    //void linearMoveHnd(const geometry_msgs::PoseStamped::ConstPtr&);
    //void rpyMoveHnd(const geometry_msgs::PointStamped::ConstPtr&);

    /* services handlers */
    bool getCtrlModeHnd(mission_ctrl_msgs::GetMode::Request&,
			mission_ctrl_msgs::GetMode::Response&);
    bool setCtrlModeHnd(mission_ctrl_msgs::SetMode::Request&,
			mission_ctrl_msgs::SetMode::Response&);
    bool getCurrentPoseHnd(carlos_controller::CurrentPose::Request&,
			   carlos_controller::CurrentPose::Response&);
    bool getCurrentJointsHnd(carlos_controller::CurrentJoints::Request&,
			     carlos_controller::CurrentJoints::Response&);
    bool setConstraintsHnd(carlos_controller::JointConstraints::Request&,
			   carlos_controller::JointConstraints::Response&);
    bool jointMoveHnd(carlos_controller::JointMove::Request&,
		      carlos_controller::JointMove::Response&);
    bool poseMoveHnd(carlos_controller::PoseMove::Request&,
		     carlos_controller::PoseMove::Response&);
    bool rpyMoveHnd(carlos_controller::RPYMove::Request&,
		    carlos_controller::RPYMove::Response&);
    bool ftMoveHnd(carlos_controller::FTMove::Request&,
		   carlos_controller::FTMove::Response&);
 
  private:
    void set_ctrl_mode(arm_mode_t);
    bool check_frame(std::string);
    arm_mode_t ctrl_mode;

    /* Params */
    std::string reference_name;
    std::string effector_name;
    std::string tool_name;
    std::string camera_name;
    std::string projector_name;
    ContactController* contact;
    tf::TransformListener listener;
    //std::vector<geometry_msgs::PoseStamped> studs_pos;
    //std::vector<geometry_msgs::PoseStamped> studs_proj;
    //std::vector<crl_movement_t> movements;
    //std::vector<crl_constraints_t> constraints;
    move_group_interface::MoveGroup* group;
    double previous_joints[DOF];

    /* Transforms */
    //geometry_msgs::PoseStamped arm_trn;
    //geometry_msgs::PoseStamped tool_trn;
    //geometry_msgs::PoseStamped proj_trn;

    ros::NodeHandle node;
    ros::Subscriber modeSubs;
    ros::Subscriber studsSubs;
    ros::Publisher ctrlModePub;
    ros::Publisher studsPub;
    ros::ServiceServer getCtrlModeSrv;
    ros::ServiceServer setCtrlModeSrv;
    ros::ServiceServer currentPoseSrv;
    ros::ServiceServer currentJointsSrv;
    ros::ServiceServer jointMoveSrv;
    ros::ServiceServer poseMoveSrv;
    ros::ServiceServer rpyMoveSrv;
    ros::ServiceServer setConsSrv;
    ros::ServiceServer ftMoveSrv;
    ros::ServiceClient ftmCalClt;
  };
}

#endif /* CRL_CONTROLLER_H */
