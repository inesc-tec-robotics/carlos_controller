
#include <mission_ctrl_msgs/projectionPoseAction.h>
#include <mission_ctrl_msgs/mission_ctrl_defines.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<mission_ctrl_msgs::projectionPoseAction> Client;

void finishedHnd(const actionlib::SimpleClientGoalState& state,
		 const mission_ctrl_msgs::projectionPoseResultConstPtr& result)
{
  fprintf(stdout, "Action finished in state: %s\n", state.toString().c_str());
  ros::shutdown();
}

void activeHnd()
{
  fprintf(stdout, "Goal active\n");
}

void feedbackHnd(const mission_ctrl_msgs::projectionPoseFeedbackConstPtr& feedback)
{
  fprintf(stdout, "Feedback executed\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_projection_pose");
  Client client(CARLOS_PROJECTION_ACTION, true);
  client.waitForServer();
  mission_ctrl_msgs::projectionPoseGoal goal;

  client.sendGoal(goal, &finishedHnd, &activeHnd, &feedbackHnd);

  ros::spin();

  return 0;
}
