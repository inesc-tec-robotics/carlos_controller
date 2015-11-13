
#include <mission_ctrl_msgs/moveArmAction.h>
#include <mission_ctrl_msgs/mission_ctrl_defines.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<mission_ctrl_msgs::moveArmAction> Client;

void finishedHnd(const actionlib::SimpleClientGoalState& state,
		 const mission_ctrl_msgs::moveArmResultConstPtr& result)
{
  fprintf(stdout, "Action finished in state %s\n", state.toString().c_str());
  ros::shutdown();
}

void activeHnd()
{
  fprintf(stdout, "Goal active\n");
}

void feedbackHnd(const mission_ctrl_msgs::moveArmFeedbackConstPtr& feedback)
{
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_move_arm");
  Client client(CARLOS_MOVE_ARM_ACTION, true);
  client.waitForServer();
  mission_ctrl_msgs::moveArmGoal goal;

  if(argc < 7){
    fprintf(stdout, "Wrong number of parameters\n");
  }

  tf::Quaternion q;
  q.setRPY(atof(argv[4]), atof(argv[5]), atof(argv[6]));

  goal.pose.header.frame_id = std::string("projector_link");
  goal.pose.pose.position.x = atof(argv[1]);
  goal.pose.pose.position.y = atof(argv[2]);
  goal.pose.pose.position.z = atof(argv[3]);
  goal.pose.pose.orientation.x = q.x();
  goal.pose.pose.orientation.y = q.y();
  goal.pose.pose.orientation.z = q.z();
  goal.pose.pose.orientation.w = q.w();

  client.sendGoal(goal, &finishedHnd, &activeHnd, &feedbackHnd);

  ros::spin();

  return 0;
}
