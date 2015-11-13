
#include <carlos_controller/DoTestAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<carlos_controller::DoTestAction> Server;

void timerCallback(const ros::TimerEvent& event)
{
  fprintf(stdout, "\t Timer event\n");
}

void execute(const carlos_controller::DoTestGoalConstPtr& goal, Server* as)
{
  fprintf(stdout, "Init DoTest execution --\n");
  fflush(stdout);
  sleep(10);
  fprintf(stdout, "Execution done.\n");
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_action_server");
  ros::NodeHandle nh;
  Server server(nh, "do_test", boost::bind(&execute, _1, &server), false);
  server.start();

  ros::Timer timer = nh.createTimer(ros::Duration(1.0), &timerCallback);

  ros::spin();

  return 0;
}
