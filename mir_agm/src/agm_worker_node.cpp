#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <mir_agm/WebComm.h>
#include <string>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Robot_Class {
	public:	
		std::string name;
		ros::NodeHandle n;	
        mir_agm::WebComm job;			

		void agm_comm();
};

void Robot_Class::agm_comm()
{
	ros::ServiceClient agmClient = n.serviceClient<mir_agm::WebComm>("/web_comm");
    job.request.name = name;
    agmClient.call(job);
	cout<<job.response.status<<endl; 
    cout<<job.response.name<<endl;  
}



int main(int argc, char** argv){
  ros::init(argc, argv, "agm_worker_node");

  Robot_Class robot;
  robot.name = "Tesla";  
  robot.job.request.function = "NEXTJOB";
  robot.agm_comm();

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}