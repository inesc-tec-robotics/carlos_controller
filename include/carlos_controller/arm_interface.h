
#ifndef CARLOS_ARM_INTERFACE_H
#define CARLOS_ARM_INTERFACE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <amn_welding_iface/wld_defines.h>
#include <brics_actuator/JointPositions.h>
#include <carlos_controller/crl_controller.h>
#include <carlos_controller/StudsPoses.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <mission_ctrl_msgs/executeWeldAction.h>
#include <mission_ctrl_msgs/generateStudDistributionAction.h>
#include <mission_ctrl_msgs/moveArmAction.h>
#include <mission_ctrl_msgs/projectionPoseAction.h>
#include <mission_ctrl_msgs/mission_ctrl_defines.h>
#include <moveit_msgs/JointConstraint.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <pthread.h>

#define DOF 6
#define ARM_INTERFACE_NAME "carlos_arm_interface"
#define BASE_HEIGHT 0.0637

#define IDLE_DESCRIPTION          "Arm controller enabled"
#define BUSY_DESCRIPTION          "Arm controller executing action"
#define ERROR_DESCRIPTION         "Arm controller stopped in error mode"
#define NOT_CONNECTED_DESCRIPTION "Arm controller disabled"

typedef actionlib::SimpleActionServer<mission_ctrl_msgs::generateStudDistributionAction> DistributionServer;
typedef actionlib::SimpleActionServer<mission_ctrl_msgs::moveArmAction> MoveServer;
typedef actionlib::SimpleActionServer<mission_ctrl_msgs::projectionPoseAction> ProjectionServer;
typedef actionlib::SimpleActionServer<mission_ctrl_msgs::executeWeldAction> WeldServer;

typedef struct{
  std::string id;
  double x;
  double y;
  double timestamp;
  double press;
  stud::states state;
} stud_t;

namespace carlos
{
  class ArmInterface{
  public:
    ArmInterface(ros::NodeHandle);
    ~ArmInterface(){};
    
    //void executeWeldHnd(const mission_ctrl_msgs::ExecuteWeldGoalConstPtr&);
    void timerCallback(const ros::TimerEvent&);
    void tfCallback(const ros::TimerEvent&);

  private:
    void install_params(void);
    void register_messages(void);
    void subscribe_messages(void);
    void publish_state(void);

    bool move_home(void);
    bool move_arm_exit_feeder_reload(void);

    /* multi-thread functions */
    hardware::states get_state(void);
    hardware::states set_state(hardware::states);
    arm_mode_t get_controller_mode(void);
    arm_mode_t set_controller_mode(arm_mode_t);
    vision_mode_t get_vision_mode(void);
    vision_mode_t set_vision_mode(vision_mode_t);
    int get_last_pose(void);
    int get_last_orient(void);
    bool get_gnd_contact(void);

    int read_stud_params(std::string, geometry_msgs::Pose*, std::vector<stud_t>*, int*, int*, double*);

    /* Service handlers */
    bool goHomeHnd(std_srvs::Empty::Request&,
		   std_srvs::Empty::Response&);
    bool exitFeederHnd(std_srvs::Empty::Request&,
    		   std_srvs::Empty::Response&);
    
    /* Message handlers */
    void controllerModeHnd(const std_msgs::UInt8::ConstPtr&);
    void feederModeHnd(const std_msgs::UInt8::ConstPtr&);
    void visionModeHnd(const std_msgs::UInt8::ConstPtr&);
    void visionOrientHnd(const geometry_msgs::PointStamped::ConstPtr&);
    void visionStudsHnd(const carlos_controller::StudsPoses::ConstPtr&);

    void wldGroundHnd(const std_msgs::Bool::ConstPtr&);
    void wldLaserHnd(const std_msgs::Bool::ConstPtr&);
    void wldPowerStateHnd(const std_msgs::Bool::ConstPtr&);
    void wldPowerValueHnd(const std_msgs::UInt32::ConstPtr&);

    /* Action servers */
    void executeWeldActHnd(const mission_ctrl_msgs::executeWeldGoalConstPtr&);
    void generateStudDistributionHnd(const mission_ctrl_msgs::generateStudDistributionGoalConstPtr&);
    void moveArmHnd(const mission_ctrl_msgs::moveArmGoalConstPtr&);
    void projectionPoseHnd(const mission_ctrl_msgs::projectionPoseGoalConstPtr&);

  private:
    pthread_mutex_t state_mutex;
    hardware::states state;
    pthread_mutex_t modules_mutex;
    arm_mode_t controller_state;
    unsigned short feeder_state;
    vision_mode_t vision_state;
    bool wld_gnd_in;
    bool wld_laser_out;
    bool wld_power_state;
    int wld_power_value;

    carlos_controller::StudsPoses poses;
    tf::TransformListener listener;
    tf::Transform wall_tf;
    tf::TransformBroadcaster broadcaster;
    int last_pose;
    int last_orient;
    pthread_mutex_t last_pose_mutex;
    pthread_mutex_t last_orient_mutex;
    pthread_mutex_t wall_tf_mutex;

    std::vector<moveit_msgs::JointConstraint> primary_cons;
    std::vector<moveit_msgs::JointConstraint> secondary_cons;
    double position_cons;
    double orientation_cons;
    
    std::vector<std::string> joint_names;
    double home_position[DOF];
    double init_position[DOF];
    double projection_position[DOF];
    double vision_position[DOF];
    double feeder_approach[DOF];
    double feeder_reload[DOF];
    double feeder_exit[DOF];

    DistributionServer* distribution_server;
    MoveServer* move_server;
    ProjectionServer* projection_server;
    WeldServer* weld_server;
    
    //std::vector<stud_t> studs;

    ros::NodeHandle node;
    ros::Publisher modePub;

    ros::Subscriber ctrlModeSubs;
    ros::Subscriber feederModeSubs;
    ros::Subscriber visionModeSubs;
    ros::Subscriber visionOrientSubs;
    ros::Subscriber visionStudsSubs;
    ros::Subscriber wldGroundSubs;
    ros::Subscriber wldLaserSubs;
    ros::Subscriber wldPowerStateSubs;
    ros::Subscriber wldPowerValueSubs;

    ros::ServiceServer goHomeSrv;
    ros::ServiceServer exitFeederSrv;
    ros::ServiceClient getCurrentPoseClt;
    ros::ServiceClient getCurrentJointsClt;
    ros::ServiceClient ftMoveClt;
    ros::ServiceClient jointMoveClt;
    ros::ServiceClient poseMoveClt;
    ros::ServiceClient rpyMoveClt;
    ros::ServiceClient setConsClt;
    ros::ServiceClient ctrlGetModeClt;
    ros::ServiceClient feederGetModeClt;
    ros::ServiceClient feederSetModeClt;
    ros::ServiceClient feederLoadClt;
    ros::ServiceClient visionGetModeClt;
    ros::ServiceClient visionSetModeClt;
    ros::ServiceClient setPowerClt;
    ros::ServiceClient setLaserClt;
    ros::ServiceClient fireGunClt;

    //geometry_msgs::Pose task_pose;
    
  };
}

#endif /* CARLOS_ARM_INTERFACE_H */
