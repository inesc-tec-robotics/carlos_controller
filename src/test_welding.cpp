
#include <mission_ctrl_msgs/executeWeldAction.h>
#include <mission_ctrl_msgs/mission_ctrl_defines.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<mission_ctrl_msgs::executeWeldAction> Client;

void finishedHnd(const actionlib::SimpleClientGoalState& state,
		 const mission_ctrl_msgs::executeWeldResultConstPtr& result)
{
  fprintf(stdout, "Action finisheed in state %s\n", state.toString().c_str());
  fprintf(stdout, "Error: %s\n", result->error_string.c_str());
  ros::shutdown();
}

void activeHnd()
{
  fprintf(stdout, "Goal active\n");
}

void feedbackHnd(const mission_ctrl_msgs::executeWeldFeedbackConstPtr& feedback)
{
  fprintf(stdout, "Feedback executed\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_execute_weld");
  Client client(CARLOS_WELD_ACTION, true);
  client.waitForServer();
  mission_ctrl_msgs::executeWeldGoal goal;
  
  fprintf(stdout, "Task name: %s\n", argv[1]);

  goal.task_name = std::string(argv[1]);

  client.sendGoal(goal, &finishedHnd, &activeHnd, &feedbackHnd);

  ros::spin();

  return 0;
}
