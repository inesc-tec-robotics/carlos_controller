

#ifndef CRL_DEFINES_H
#define CRL_DEFINES_H

#define CRL_CTRL_GET_MODE_SRV "carlos/controller/mode/get"
#define CRL_CTRL_SET_MODE_SRV "carlos/controller/mode/set"
#define CRL_GET_CURRENT_POSE_SRV "carlos/current_pose"
#define CRL_GET_CURRENT_JOINTS_SRV "carlos/current_joints"
#define CRL_ARM_MODE_MSG      "carlos/arm/mode/status"
//#define CRL_STUDS_DIST_SRV    "studs_dist/set_mode"
//#define TRAJECTORY_GOAL_TOPIC "arm_controller/follow_joint_trajectory/goal"
#define CRL_FEEDER_GET_MODE_SRV "carlos/feeder/get_mode"
#define CRL_FEEDER_SET_MODE_SRV "carlos/feeder/set_mode"
#define CRL_FEEDER_LOAD_SRV     "carlos/feeder/load"
#define CRL_VISION_GET_MODE_SRV "carlos/vision/get_mode"
#define CRL_VISION_SET_MODE_SRV "carlos/vision/set_mode"
#define CRL_FTM75_CALIBRATE_SRV "ftm75/calibrate"

#define CRL_JOINT_MOVE_SRV      "carlos/joint_movement"
#define CRL_POSE_MOVE_SRV       "carlos/pose_movement"
#define CRL_RPY_MOVE_SRV        "carlos/rpy_movement"
#define CRL_SET_CONSTRAINTS_SRV "carlos/set_constraints"
#define CRL_FT_MOVE_SRV         "carlos/ft_movement"

//#define MNL_SET_MODE_SRV   "carlos/guided/mode/set"
#define CRL_STUDS_POS_MSG   "carlos/vision/studs_pose"
#define CRL_WALL_ORIENT_MSG "carlos/vision/wall_orientation"
//#define CRL_STUDS_PRJ_MSG  "carlos/studs_projector"

#define CRL_CTRL_MODE_MSG      "carlos/controller/mode"
#define CRL_FEEDER_MODE_MSG    "carlos/feeder/mode"
#define CRL_VISION_MODE_MSG    "carlos/vision/mode"

/* Params */
#define CRL_PARAM_EFFECTOR    "/carlos/link_names/effector"
#define CRL_PARAM_TOOL        "/carlos/link_names/tool"
#define CRL_PARAM_CAMERA      "/carlos/link_names/camera"
#define CRL_PARAM_PROJECTOR   "/carlos/link_names/projector"
#define CRL_PARAM_CONSTRAINTS "/carlos/constraints"
//#define CRL_PARAM_STUD_PRESS  "/studs_pattern/press"

#define CRL_PARAM_DEVICE_NAME "/carlos/feeder/device"
#define CRL_PARAM_STACK_NAME  "/carlos/feeder/protocol"
#define CRL_PARAM_IFACE_NAME  "/carlos/feeder/interface"
#define CRL_PARAM_ROLL_DEVICE "/carlos/feeder/roll"
#define CRL_PARAM_LOAD_DEVICE "/carlos/feeder/load"
#define CRL_PARAM_BAUD_RATE   "/carlos/feeder/baudrate"
#define CRL_PARAM_NODE_ID     "/carlos/feeder/node"

/* Arm interface params */
#define ARM_PARAM_HOME_POS         "/carlos/arm_interface/home_pos"
#define ARM_PARAM_INIT_POS         "/carlos/arm_interface/init_pos"
#define ARM_PARAM_VISION_POS       "/carlos/arm_interface/vision_pos"
#define ARM_PARAM_PROJECTION_POS   "/carlos/arm_interface/projection_pos"
#define ARM_PARAM_FEEDER_RELOAD    "/carlos/arm_interface/feeder_reload"
#define ARM_PARAM_FEEDER_APPROACH  "/carlos/arm_interface/feeder_approach"
#define ARM_PARAM_FEEDER_EXIT      "/carlos/arm_interface/feeder_exit"
#define ARM_PARAM_JOINT_NAMES      "/carlos/arm_interface/joint_names"
#define ARM_PARAM_PRIMARY_CONS     "/carlos/arm_interface/primary_cons"
#define ARM_PARAM_SECONDARY_CONS   "/carlos/arm_interface/secondary_cons"
#define ARM_PARAM_POSITION_CONS    "/carlos/arm_interface/position_cons"
#define ARM_PARAM_ORIENTATION_CONS "/carlos/arm_interface/orientation_cons"

/* Mission params */
#define MISSION_TASKS           "/mission/tasks"
#define SEPARATOR               "/"
#define PARAM_NAV_GOAL_X        "/nav_goal/x"
#define PARAM_NAV_GOAL_Y        "/nav_goal/y"
#define PARAM_NAV_GOAL_YAW      "/nav_goal/yaw"
#define PARAM_DIRECTION         "/direction"
#define PARAM_VOLTAGE           "/voltage"
#define PARAM_STUD_DISTANCE     "/stud_pattern/distance"
#define PARAM_STUD_DISTRIBUTION "/stud_pattern/distribution"
#define PARAM_STUD_PROXIMITY    "/stud_pattern/proximity"
#define PARAM_STUD_PRESS        "/stud_pattern/press"
#define PARAM_STUD_X            "/x"
#define PARAM_STUD_Y            "/y"
#define PARAM_STUD_STATE        "/state"
#define PARAM_STUD_TIME         "/time_stamp"
#define PARAM_STUD_NAME         "/studs/stud_"
#define PARAM_STUD_ID           "stud_"

//#define MNL_JOINT_STD_VEL 0.1
//#define MNL_JOINT_STD_ACC 0.3

#endif /* CRL_DEFINES_H */
