#ifndef AMN_STUDS_FEEDER_H
#define AMN_STUDS_FEEDER_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <mission_ctrl_msgs/GetMode.h>
#include <mission_ctrl_msgs/SetMode.h>

#include "carlos_controller/Definitions.h"

#ifndef MMC_SUCCESS
#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
#define MMC_FAILED 1
#endif

typedef void* HANDLE;

typedef enum{
  FEEDER_BUSY     = 0,
  FEEDER_PREPARED = 1,
  FEEDER_LOADED   = 2,
  FEEDER_ERROR    = 3
} feeder_state_t;

namespace carlos
{
  class StudsFeeder{
  public:
    StudsFeeder(ros::NodeHandle);
    ~StudsFeeder(){};

    int close_devices(void);

  private:
    int open_devices(void);
    int prepare_motors(void);
    int velocity_move_time(HANDLE, unsigned int*, long, long);
    int velocity_move_hall(HANDLE, unsigned int*, long, int, long);

    void install_params(void);
    void register_messages(void);
    void register_services(void);

    void set_state(feeder_state_t);

    /* service handlers */
    bool getStateHnd(mission_ctrl_msgs::GetMode::Request&,
		     mission_ctrl_msgs::GetMode::Response&);
    bool setStateHnd(mission_ctrl_msgs::SetMode::Request&,
		     mission_ctrl_msgs::SetMode::Response&);
    bool loadStudHnd(std_srvs::Empty::Request&,
		     std_srvs::Empty::Response&);

  private:
    feeder_state_t feeder_state;
    std::string device_name;
    std::string stack_name;
    std::string interface_name;
    std::string roll_device;
    std::string load_device;
    unsigned int baudrate;
    unsigned short node_id;

    void* roll_handle;
    void* load_handle;

    ros::NodeHandle node;
    ros::Publisher modePub;
    ros::ServiceServer getModeSrv;
    ros::ServiceServer setModeSrv;
    ros::ServiceServer loadStudSrv;
    
  };
}

#endif /* AMN_STUDS_FEEDER_H */

