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
    void move();
};

void Robot_Class::agm_comm()
{
	ros::ServiceClient agmClient = n.serviceClient<mir_agm::WebComm>("/web_comm");
    job.request.name = name;
    agmClient.call(job);
}

void Robot_Class::move()
{
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
  {
    ROS_INFO("Hooray, the base moved 1 meter forward");    
  }
  else
  {
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "agm_worker_node");

  Robot_Class robot;
  robot.name = "Tesla";  

  //find next job
  robot.job.request.function = "START";

  while (ros::ok()) { 

    string job = robot.job.request.function;
    int status = robot.job.response.status;

    if (job=="START") {
      //ready for a new job
      robot.job.request.function = "NEXTJOB";
      robot.job.request.location = "";
      robot.agm_comm();
    } else if (job=="NEXTJOB" && status == 1) {
      //we have a new job to be activated
      robot.job.request.function = "ACTIVATEJOB";
      robot.job.request.location = "";
      robot.agm_comm();
    } else if (job=="ACTIVATEJOB" && status == 1){
      //move to source 
      robot.job.request.function = "MOVEWORKER";
      robot.job.request.location = "source";
      robot.move();
      robot.agm_comm();
    } else if (job=="MOVEWORKER" && status == 1){
      //either TAKEPART or LOADPART depending on location
      if (robot.job.request.location=="source"){
        robot.job.request.function = "TAKEPART";      
      } else {
        robot.job.request.function = "LOADPART";
      }
      robot.job.request.location = "";
      robot.agm_comm();
    } else if (job=="TAKEPART" && status == 1){
      //move to destination station
      robot.job.request.function = "MOVEWORKER";
      robot.job.request.location = "destination";
      robot.move();
      robot.agm_comm();
    } else if (job=="LOADPART" && status == 1) {
      //archive job
      robot.job.request.function = "ARCHIVEJOB";
      robot.job.request.location = "";
      robot.agm_comm();
    } else if (job=="ARCHIVEJOB" && status == 1) {
      //start over
      robot.job.request.function = "START";
      robot.job.request.location = "";
      cout<<"Starting again"<<endl;
    } else {
      //error
      cout<<job<<endl;
      cout<<status<<endl;
      if(status==10003){
        //there wasn't a job to do, reset and ask again
        robot.job.request.function = "START";
      }
    }
  }    

  return 0;
}