
#include <carlos_controller/DoTestAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<carlos_controller::DoTestAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_test_client2");
  Client client("do_test", true);
  client.waitForServer();
  carlos_controller::DoTestGoal goal;
  
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(20.0));
  if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    fprintf(stdout, "Action done.\n");
  fprintf(stdout, "Current State: %s\n", client.getState().toString().c_str());

  return 0;
}
