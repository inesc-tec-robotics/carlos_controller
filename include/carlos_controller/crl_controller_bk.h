
#ifndef CRL_CONTROLLER_H
#define CRL_CONTROLLER_H

#include <ros/ros.h>
#include <amn_common/amn_common.h>
#include <mission_ctrl_msgs/GetMode.h>
#include <mission_ctrl_msgs/SetMode.h>
#include <std_msgs/UInt8.h>
#include <amn_common/amn_mnl_defines.h>
#include <amn_common/amn_mnl_lib.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/transforms/transforms.h>
#include <tf/transform_listener.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

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
  CRL_ARM_WELDING = 2,
  CRL_ARM_GUIDED  = 3
} arm_mode_t;

typedef enum{
  CRL_WLD_STOPPED = 0,
  CRL_WLD_GTPHS   = 1,
  CRL_WLD_GTPLS   = 2,
  CRL_WLD_GTPC    = 3,
  CRL_WLD_WELD    = 4,
  CRL_WLD_MBLS    = 5,
  CRL_WLD_CHECK   = 6,
  CRL_WLD_MBHS    = 7
} wld_mode_t;

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
    void publish_studs(void);
    
    bool initialize_controller(void);

    /* messages handlers */
    //void jointMoveHnd(const geometry_msgs::PoseArray::ConstPtr&);
    //void linearMoveHnd(const geometry_msgs::PoseArray::ConstPtr&);
    //void studPosesHnd(const geometry_msgs::PoseArray::ConstPtr&);

    /* services handlers */
    bool getCtrlModeHnd(mission_ctrl_msgs::GetMode::Request&,
			mission_ctrl_msgs::GetMode::Response&);
    bool setCtrlModeHnd(mission_ctrl_msgs::SetMode::Request&,
			mission_ctrl_msgs::SetMode::Response&);
    bool getArmModeHnd(mission_ctrl_msgs::GetMode::Request&,
		       mission_ctrl_msgs::GetMode::Response&);
    bool setArmModeHnd(mission_ctrl_msgs::SetMode::Request&,
		       mission_ctrl_msgs::SetMode::Response&);
    bool getWldModeHnd(mission_ctrl_msgs::GetMode::Request&,
		       mission_ctrl_msgs::GetMode::Response&);
 
  private:
    void publish_ctrl_mode(void);
    void publish_arm_mode(void);
    void publish_wld_mode(void);
    bool check_frame(std::string);
    int move_arm(crl_movement_t*);

    int move_point(crl_movement_t*, int);
    void orient_arm(double, double, double);
    void move_arm_vision_pose(void);
    running_mode_t ctrl_mode;
    arm_mode_t arm_mode;
    wld_mode_t wld_mode;
    vision_mode_t vision_mode;
    

    /* Params */
    std::string reference_name;
    std::string effector_name;
    std::string tool_name;
    std::string camera_name;
    std::string projector_name;
    aimen::MNLController* teach;
    ContactController* contact;
    tf::TransformListener listener;
    std::vector<geometry_msgs::PoseStamped> studs_pos;
    std::vector<geometry_msgs::PoseStamped> studs_proj;
    std::vector<crl_movement_t> movements;
    std::vector<crl_constraints_t> constraints;
    move_group_interface::MoveGroup* group;

    /* Transforms */
    geometry_msgs::PoseStamped arm_trn;
    geometry_msgs::PoseStamped tool_trn;
    geometry_msgs::PoseStamped proj_trn;

    ros::NodeHandle node;
    ros::Subscriber modeSubs;
    ros::Subscriber studsSubs;
    ros::Subscriber jointSubs;
    ros::Subscriber linearSubs;
    ros::Publisher ctrlModePub;
    ros::Publisher armModePub;
    ros::Publisher wldModePub;
    ros::Publisher studsPub;
    ros::ServiceServer getCtrlModeSrv;
    ros::ServiceServer setCtrlModeSrv;
    ros::ServiceServer getArmModeSrv;
    ros::ServiceServer setArmModeSrv;
    ros::ServiceServer getWldModeSrv;
    ros::ServiceClient studsModeClt;
  };
}

#endif /* CRL_CONTROLLER_H */
