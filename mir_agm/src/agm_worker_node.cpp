#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <mir_agm/WebComm.h>
#include <string>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Robot_Class {
	public:	
		std::string key;
		ros::NodeHandle n;	
    mir_agm::WebComm job;			

		void agm_comm();
    void move(int posX, int posY, int posZ, int orientX, int orientY, int orientZ, int orientW);
};

void Robot_Class::agm_comm()
{
	ros::ServiceClient agmClient = n.serviceClient<mir_agm::WebComm>("/web_comm");
    job.request.key = key;
    agmClient.call(job);
}

void Robot_Class::move(int posX, int posY, int posZ, int orientX, int orientY, int orientZ, int orientW)
{
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot which is the next job received
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = posX;
  goal.target_pose.pose.position.y = posY;
  goal.target_pose.pose.position.z = posZ;
  
  goal.target_pose.pose.orientation.x = orientX;
  goal.target_pose.pose.orientation.y = orientY;
  goal.target_pose.pose.orientation.z = orientZ;
  goal.target_pose.pose.orientation.w = orientW;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the base moved to goal");    
  }
  else
  {
    ROS_INFO("The base failed to move to goal for some reason");
    job.request.function = "ERROR";
  }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "agm_worker_node");

  Robot_Class robot;
  if(argc > 1){
    robot.key = argv[1];  
  } else {
    cout<<"No key defined for the robot interface"<<endl;  
  }
  
  cout<<argc<<endl;
  cout<<argv<<endl;
  cout<<robot.key<<endl;

  //find next job
  robot.job.request.function = "START";
  //source coordinates
  int sPosX, sPosY, sPosZ, sOrientX, sOrientY, sOrientZ, sOrientW;
  int dPosX, dPosY, dPosZ, dOrientX, dOrientY, dOrientZ, dOrientW;

  //destination coordinates

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
      cout<<"Found next job and activating"<<endl;
      //source
      sPosX=robot.job.response.sourcePosX;
      sPosY=robot.job.response.sourcePosY;
      sPosZ=robot.job.response.sourcePosZ;
      sOrientX=robot.job.response.sourceOrientX;
      sOrientY=robot.job.response.sourceOrientY;
      sOrientZ=robot.job.response.sourceOrientZ;
      sOrientW=robot.job.response.sourceOrientW;
      //destination
      dPosX=robot.job.response.destPosX;
      dPosY=robot.job.response.destPosY;
      dPosZ=robot.job.response.destPosZ;
      dOrientX=robot.job.response.destOrientX;
      dOrientY=robot.job.response.destOrientY;
      dOrientZ=robot.job.response.destOrientZ;
      dOrientW=robot.job.response.destOrientW;

      robot.job.request.function = "ACTIVATEJOB";
      robot.job.request.location = "";
      robot.agm_comm();
    } else if (job=="ACTIVATEJOB" && status == 1){
      //move to source 
      cout<<"Moving to source"<<endl;
      robot.job.request.function = "MOVEWORKER";
      robot.job.request.location = "source";
      cout<<sPosX<<" "<<sPosY<<" "<<sPosZ<<" "<<sOrientX<<" "<<sOrientY<<" "<<sOrientZ<<" "<<sOrientW<<endl;
      robot.move(sPosX, sPosY, sPosZ, sOrientX, sOrientY, sOrientZ, sOrientW);
      robot.agm_comm();
    } else if (job=="MOVEWORKER" && status == 1){
      //either TAKEPART or LOADPART depending on location
      if (robot.job.request.location=="source"){
        cout<<"Taking part"<<endl;
        robot.job.request.function = "TAKEPART";      
      } else {
        cout<<"Loading workstation"<<endl;
        robot.job.request.function = "LOADPART";
      }
      robot.job.request.location = "";
      robot.agm_comm();
    } else if (job=="TAKEPART" && status == 1){
      //move to destination station
      cout<<"Moving to destination"<<endl;
      robot.job.request.function = "MOVEWORKER";
      robot.job.request.location = "destination";
      cout<<dPosX<<" "<<dPosY<<" "<<dPosZ<<" "<<dOrientX<<" "<<dOrientY<<" "<<dOrientZ<<" "<<dOrientW<<endl;
      robot.move(dPosX, dPosY, dPosZ, dOrientX, dOrientY, dOrientZ, dOrientW);
      robot.agm_comm();
    } else if (job=="LOADPART" && status == 1) {
      //archive job
      cout<<"Archiving job"<<endl;
      robot.job.request.function = "ARCHIVEJOB";
      robot.job.request.location = "";
      robot.agm_comm();
    } else if (job=="ARCHIVEJOB" && status == 1) {
      //start over
      robot.job.request.function = "START";
      robot.job.request.location = "";
      cout<<"Start again"<<endl;
    } else {
      //error
      if (robot.job.request.function!="ERROR"){
        cout<<job<<endl;
        cout<<status<<endl;
        cout<<robot.job.response.name<<endl;
      }
      
      if(status==10003 || status==10002 || (status==0 && job=="NEXTJOB")){
        //there wasn't a job to do, reset and ask again
        robot.job.request.function = "START";
      }
    }
  }    

  return 0;
}