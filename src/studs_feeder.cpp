
#include "carlos_controller/studs_feeder.h"
#include "carlos_controller/crl_defines.h"

#include <std_msgs/UInt8.h>
#include <signal.h>

using namespace carlos;

StudsFeeder::StudsFeeder(ros::NodeHandle node)
{
  this->node = node;
  this->roll_handle = 0;
  this->load_handle = 0;

  install_params();
  if(open_devices() != MMC_SUCCESS)
    exit(1);

  if(close_devices() == MMC_FAILED){
    fprintf(stdout, "Error closing devices\n");
  }
  //  if(prepare_motors() != MMC_SUCCESS)
  //    exit(1);
  register_messages();
  register_services();
 
  this->feeder_state = FEEDER_PREPARED;
}

int StudsFeeder::open_devices(void)
{
  char device[255];
  char stack[255];
  char interface[255];
  char port1[255];
  char port2[255];

  strcpy(device, this->device_name.c_str());
  strcpy(stack, this->stack_name.c_str());
  strcpy(interface, this->interface_name.c_str());
  strcpy(port1, this->roll_device.c_str());
  strcpy(port2, this->load_device.c_str());

  unsigned int error;
  unsigned int c_baudrate = 0;
  unsigned int c_timeout = 0;
  
  this->roll_handle = VCS_OpenDevice(device, stack, interface, port1, &error);
  if(this->roll_handle!=0 && error == 0){
    if(VCS_GetProtocolStackSettings(this->roll_handle, &c_baudrate, &c_timeout, &error) != 0){
      if(c_baudrate != this->baudrate){
	if(VCS_SetProtocolStackSettings(this->roll_handle, this->baudrate, c_timeout, &error) != 0){
	  if(VCS_GetProtocolStackSettings(this->roll_handle, &c_baudrate, &c_timeout, &error) != 0){
	    if(c_baudrate != this->baudrate){
	      fprintf(stdout, "Error setting the baudrate\n");
	      return MMC_FAILED;
	    }
	  }else{
	    fprintf(stdout, "Error getting the protocol stack: %d\n", error);
	    return MMC_FAILED;
	  }
	}else{
	  fprintf(stdout, "Error setting the baudrate: %d\n", error);
	  return MMC_FAILED;
	}
      }
    }else{
      fprintf(stdout, "Error getting the protocol stack: %d\n", error);
      return MMC_FAILED;
    }
  }else{
    fprintf(stdout, "Error opening the roll device: %d\n", error);
    this->roll_handle = 0;
    return MMC_FAILED;
  }

  fprintf(stdout, "First device loaded\n");

  this->load_handle = VCS_OpenDevice(device, stack, interface, port2, &error);
  if(this->load_handle!=0 && error == 0){
    if(VCS_GetProtocolStackSettings(this->load_handle, &c_baudrate, &c_timeout, &error) != 0){
      if(c_baudrate != this->baudrate){
	if(VCS_SetProtocolStackSettings(this->load_handle, this->baudrate, c_timeout, &error) != 0){
	  if(VCS_GetProtocolStackSettings(this->load_handle, &c_baudrate, &c_timeout, &error) != 0){
	    if(c_baudrate != this->baudrate){
	      fprintf(stdout, "Error setting the baudrate\n");
	      return MMC_FAILED;
	    }
	  }else{
	    fprintf(stdout, "Error getting the protocol stack: %d\n", error);
	    return MMC_FAILED;
	  }
	}else{
	  fprintf(stdout, "Error setting the baudrate: %d\n", error);
	  return MMC_FAILED;
	}
      }
    }else{
      fprintf(stdout, "Error getting the protocol stack: %d\n", error);
      return MMC_FAILED;
    }
  }else{
    fprintf(stdout, "Error opening the second device: %s\n", this->load_device.c_str());
    this->load_handle = 0;
    return MMC_FAILED;
  }
  
  return MMC_SUCCESS;
}

int StudsFeeder::prepare_motors(void)
{
  unsigned int error = 0;
  int fault = 0;
  int enabled = 0;

  fprintf(stdout, "--> Prepare motors\n");

  if(VCS_GetFaultState(this->roll_handle, this->node_id, &fault, &error) == 0){
    fprintf(stdout, "Error getting the fault state: %d\n", error);
    return MMC_FAILED;
  }else{
    if(fault){
      fprintf(stdout, "Fault state\n");
      if(VCS_ClearFault(this->roll_handle, this->node_id, &error) == 0){
	fprintf(stdout, "Error clearing the fault state: %d\n", error);
	return MMC_FAILED;
      }
    }
    if(VCS_GetEnableState(this->roll_handle, this->node_id, &enabled, &error) == 0){
      fprintf(stdout, "Error getting the state: %d\n", error);
      return MMC_FAILED;
    }else{
      if(!enabled){
	if(VCS_SetEnableState(this->roll_handle, this->node_id, &error) == 0){
	  fprintf(stdout, "Error setting the state: %d\n", error);
	  return MMC_FAILED;
	}
      }
    }
  }
  
  if(VCS_GetFaultState(this->load_handle, this->node_id, &fault, &error) == 0){
    fprintf(stdout, "Error getting the fault state: %d\n", error);
    return MMC_FAILED;
  }else{
    if(fault){
      if(VCS_ClearFault(this->load_handle, this->node_id, &error) == 0){
	fprintf(stdout, "Error clearing the fault state: %d\n", error);
	return MMC_FAILED;
      }
    }
    if(VCS_GetEnableState(this->load_handle, this->node_id, &enabled, &error) == 0){
      fprintf(stdout, "Error getting the state: %d\n", error);
      return MMC_FAILED;
    }else{
      if(!enabled){
	if(VCS_SetEnableState(this->load_handle, this->node_id, &error) == 0){
	  fprintf(stdout, "Error setting the state: %d\n", error);
	  return MMC_FAILED;
	}
      }
    }
  }

  fprintf(stdout, "<-- Prepare motors\n");

  return MMC_SUCCESS;
}

int StudsFeeder::close_devices(void)
{
  unsigned int error;

  if(VCS_CloseDevice(this->roll_handle, &error) == 0){
    fprintf(stdout, "Error closing the roll device: %d\n", error);
    return MMC_FAILED;
  }

  if(VCS_CloseDevice(this->load_handle, &error) == 0){
    fprintf(stdout, "Error closing the load device: %d\n", error);
    return MMC_FAILED;
  }

  return MMC_SUCCESS;
}

int StudsFeeder::velocity_move_time(HANDLE handle, unsigned int* error, long vel, long time)
{
  fprintf(stdout, "-->Velocity move time\n");
  std::cout << handle << std::endl;
  fflush(stdout);

  if(VCS_ActivateProfileVelocityMode(handle, this->node_id, error) == 0){
    fprintf(stdout, "Error activating velocity profile: %d\n", *error);
    return MMC_FAILED;
  }else{
    if(VCS_MoveWithVelocity(handle, this->node_id, vel, error) == 0){
      fprintf(stdout, "Error moving motor: %d\n", *error);
      return MMC_FAILED;
    }

    usleep(time);

    if(VCS_HaltVelocityMovement(handle, this->node_id, error) == 0){
      fprintf(stdout, "Error stopping the motor: %d\n", *error);
      return MMC_FAILED;
    }
  }

  fprintf(stdout, "<--Velocity move time\n");

  return MMC_SUCCESS;
} 

int StudsFeeder::velocity_move_hall(HANDLE handle, unsigned int* error, long vel, int pol, long delay)
{
  unsigned short inputs = pol;
  int counter = 0;

  if(VCS_ActivateProfileVelocityMode(handle, this->node_id, error) == 0){
    fprintf(stdout, "Error activating velocity profile: %d\n", *error);
    return MMC_FAILED;
  }else{
    if(VCS_MoveWithVelocity(handle, this->node_id, vel, error) == 0){
      fprintf(stdout, "Error moving motor: %d\n", *error);
      return MMC_FAILED;
    }
    
    while(inputs == pol){
      usleep(1000);
      VCS_GetAllDigitalInputs(this->load_handle, this->node_id, &inputs, error);
      inputs = inputs >> 15;
      counter++;
      if(counter > 2000)
	break;
    }
    usleep(delay);

    if(VCS_HaltVelocityMovement(handle, this->node_id, error) == 0){
      fprintf(stdout, "Error stopping the motor: %d\n", *error);
      return MMC_FAILED;
    }
  }

  return MMC_SUCCESS;
}

void StudsFeeder::install_params(void)
{
  XmlRpc::XmlRpcValue tmp_device;
  this->node.getParam(CRL_PARAM_DEVICE_NAME, tmp_device);
  ROS_ASSERT(tmp_device.getType() == XmlRpc::XmlRpcValue::TypeString);
  this->device_name = static_cast<std::string>(tmp_device);

  XmlRpc::XmlRpcValue tmp_stack;
  this->node.getParam(CRL_PARAM_STACK_NAME, tmp_stack);
  ROS_ASSERT(tmp_stack.getType() == XmlRpc::XmlRpcValue::TypeString);
  this->stack_name = static_cast<std::string>(tmp_stack);

  XmlRpc::XmlRpcValue tmp_interface;
  this->node.getParam(CRL_PARAM_IFACE_NAME, tmp_interface);
  ROS_ASSERT(tmp_interface.getType() == XmlRpc::XmlRpcValue::TypeString);
  this->interface_name = static_cast<std::string>(tmp_interface);
  
  XmlRpc::XmlRpcValue tmp_roll;
  this->node.getParam(CRL_PARAM_ROLL_DEVICE, tmp_roll);
  ROS_ASSERT(tmp_roll.getType() == XmlRpc::XmlRpcValue::TypeString);
  this->roll_device = static_cast<std::string>(tmp_roll);

  XmlRpc::XmlRpcValue tmp_load;
  this->node.getParam(CRL_PARAM_LOAD_DEVICE, tmp_load);
  ROS_ASSERT(tmp_load.getType() == XmlRpc::XmlRpcValue::TypeString);
  this->load_device = static_cast<std::string>(tmp_load);

  XmlRpc::XmlRpcValue tmp_baud;
  this->node.getParam(CRL_PARAM_BAUD_RATE, tmp_baud);
  ROS_ASSERT(tmp_baud.getType() == XmlRpc::XmlRpcValue::TypeInt);
  this->baudrate = static_cast<int>(tmp_baud);

  XmlRpc::XmlRpcValue tmp_node;
  this->node.getParam(CRL_PARAM_NODE_ID, tmp_node);
  ROS_ASSERT(tmp_node.getType() == XmlRpc::XmlRpcValue::TypeInt);
  this->node_id = static_cast<int>(tmp_node);
}

void StudsFeeder::register_messages(void)
{
  this->modePub = this->node.advertise<std_msgs::UInt8>(CRL_FEEDER_MODE_MSG, 1);
}

void StudsFeeder::register_services(void)
{
  this->getModeSrv = this->node.advertiseService(CRL_FEEDER_GET_MODE_SRV,
						 &StudsFeeder::getStateHnd, this);	
  this->setModeSrv = this->node.advertiseService(CRL_FEEDER_SET_MODE_SRV,
						 &StudsFeeder::setStateHnd, this);
  this->loadStudSrv = this->node.advertiseService(CRL_FEEDER_LOAD_SRV,
						  &StudsFeeder::loadStudHnd, this);
}

void StudsFeeder::set_state(feeder_state_t state)
{
  this->feeder_state = state;

  std_msgs::UInt8 msg;
  msg.data = this->feeder_state;
  this->modePub.publish(msg);
}

/* Services handlers */
bool StudsFeeder::loadStudHnd(std_srvs::Empty::Request& request,
			      std_srvs::Empty::Response& response)
{
  unsigned short input;
  unsigned int error = 0;

  fprintf(stdout, "New load message. State: %d\n", this->feeder_state);

  if(this->feeder_state == FEEDER_PREPARED){
    set_state(FEEDER_BUSY);

    
    if(open_devices() == MMC_FAILED){
      fprintf(stdout, "Error openning devices\n");
      set_state(FEEDER_ERROR);
      close_devices();
      return false;
    }
    
    if(prepare_motors() == MMC_FAILED){
      fprintf(stdout, "Error preparing motors\n");
      set_state(FEEDER_ERROR);
      close_devices();
      return false;
    }

    if(VCS_GetAllDigitalInputs(this->load_handle, this->node_id, &input, &error) == 0){
      fprintf(stdout, "Error reading signals\n");
      set_state(FEEDER_ERROR);
      close_devices();
      return false;
    }else{
      input = input>>15;
      if(input == 1){
	if(velocity_move_hall(this->roll_handle, &error, 500, 1, 300000) == MMC_FAILED){
	  set_state(FEEDER_ERROR);
	  fprintf(stdout, "Error moving hall 1\n");
	  close_devices();
	  return false;
	}
      }
      std::cout << this->load_handle << std::endl;
      if(velocity_move_time(this->load_handle, &error, 3000, 2200000) == MMC_FAILED){
	fprintf(stdout, "Error moving time 1\n");
	set_state(FEEDER_ERROR);
	close_devices();
	return false;
      }
      if(velocity_move_hall(this->load_handle, &error, 2000, 0, 20000) == MMC_FAILED){
	fprintf(stdout, "Error moving hall 2\n");
	set_state(FEEDER_ERROR);
	close_devices();
	return false;
      }
      
      set_state(FEEDER_LOADED);
      
      if(VCS_SetDisableState(this->roll_handle, this->node_id, &error) == 0)
	fprintf(stdout, "Error disabling motor: %d\n", error);
      if(VCS_SetDisableState(this->load_handle, this->node_id, &error) == 0)
	fprintf(stdout, "Error disabling motor: %d\n", error);
      
    }

    if(close_devices() == MMC_FAILED)
      fprintf(stdout, "Error closing devices\n");

    return true;
  }

  return false;
}

bool StudsFeeder::getStateHnd(mission_ctrl_msgs::GetMode::Request& request,
			      mission_ctrl_msgs::GetMode::Response& response)
{
  response.mode = this->feeder_state;

  return true;
}

bool StudsFeeder::setStateHnd(mission_ctrl_msgs::SetMode::Request& request,
			      mission_ctrl_msgs::SetMode::Response& response)
{
  if(feeder_state == FEEDER_LOADED && request.mode == FEEDER_PREPARED){
    set_state(FEEDER_PREPARED);
    response.mode = feeder_state;
    return true;
  }

  response.mode = feeder_state;

  return false;
}

/* Global vars for the main program */
StudsFeeder* c_feeder;

void close_program(int signal __attribute__((unused)))
{
  if(c_feeder->close_devices() == MMC_FAILED)
    fprintf(stdout, "Error closing devices\n");
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "studs_feeder_module");
  ros::NodeHandle nh;

  StudsFeeder *feeder = new StudsFeeder(nh);
  c_feeder = feeder;

  signal(SIGINT, close_program);
  signal(SIGTERM, close_program);
  
  ros::spin();

  return 0;
}
