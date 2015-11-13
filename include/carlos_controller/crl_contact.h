
#ifndef CRL_CONTACT_H
#define CRL_CONTACT_H

#include <ros/ros.h>

#include <amn_common/amn_common.h>
#include <amn_common/amn_ik_defines.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

#define DOF 6

typedef enum{
  CONTACT_MODE_STOPPED = 0,
  CONTACT_MODE_AUTO = 1,
  CONTACT_MODE_ERROR = 2
} contact_mode_t;

namespace carlos
{
  class ContactController{
  public:
    ContactController(ros::NodeHandle node, double sensor_offset, double period);
    ~ContactController();

    void timerCallback(const ros::TimerEvent&);
    contact_mode_t set_mode(contact_mode_t);
    contact_mode_t get_mode();
    void set_joint_velocities(double*, double);
    void set_joint_limits(double*, double*);

  private:
    void install_params(void);
    void register_messages(void);
    void subscribe_messages(void);
    void unsubscribe_messages(void);
    void register_services(void);

    void stop_arm(void);

    /* message handlers */
    void armStateHnd(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr&);
    void ftHnd(const geometry_msgs::WrenchStamped::ConstPtr&);

    contact_mode_t mode;
    int ft_received;
    int arm_received;
    double offset;
    double period;
    double stud_press;
    joint_t joints[DOF];
    double velocities[DOF];
    double factor;
    force_torque_t ft_readings;
    ros::NodeHandle node;

    ros::Subscriber armSubs;
    ros::Subscriber ftSubs;
    ros::Publisher velPub;
    ros::ServiceClient resetSrv;
    ros::Timer timer;
  };
}

#endif /* CRL_CONTACT_H */
