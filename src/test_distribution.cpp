
#include <mission_ctrl_msgs/generateStudDistributionAction.h>
#include <mission_ctrl_msgs/mission_ctrl_defines.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<mission_ctrl_msgs::generateStudDistributionAction> Client;

void finishedHnd(const actionlib::SimpleClientGoalState& state,
		 const mission_ctrl_msgs::generateStudDistributionResultConstPtr& result)
{
  fprintf(stdout, "Action finished in state %s\n", state.toString().c_str());
  fprintf(stdout, "Error: %s\n", result->error_string.c_str());
  for(int i=0; i<result->positions.size(); i++){
    fprintf(stdout, "/tasks/test_task/studs/stud_%d/x: %f\n", i+1, result->positions[i].x);
    fprintf(stdout, "/tasks/test_task/studs/stud_%d/y: %f\n", i+1, result->positions[i].y);
  }
  ros::shutdown();
}

void activeHnd()
{
  fprintf(stdout, "Goal active\n");
}

void feedbackHnd(const mission_ctrl_msgs::generateStudDistributionFeedbackConstPtr& feedback)
{
  fprintf(stdout, "Feedback executed\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_generate_distribution");
  Client client(CARLOS_DISTRIBUTION_ACTION, true);
  client.waitForServer();
  mission_ctrl_msgs::generateStudDistributionGoal goal;

  goal.side = 1;

  client.sendGoal(goal, &finishedHnd, &activeHnd, &feedbackHnd);

  ros::spin();



  return 0;
}
