#define LIBBOT_PRESENT 0

#include <iostream>
#include <ctime>
#include <sys/stat.h>
#include <fstream>
#include <cstdlib>
#include <list>
#include <string>
#include <ros/ros.h>
#include <vector>
#include <rrtstar_msgs/rrtStarSRV.h>
#include <rrtstar_msgs/Region.h>
#include <geometry_msgs/Vector3.h>
#include "std_msgs/String.h"
#include <time.h>
#include <knowledge_msgs/obstacleSRV.h>
#include <tf/transform_listener.h>
#include <controlCommand_msgs/control.h>
#include <controlCommand_msgs/controlGoalReachAck.h>

using namespace std;

bool ackReceived = true;

void ackCallback(const controlCommand_msgs::controlGoalReachAck& msg) {

		cout << "Acknowledgement received" << endl;
		ackReceived = true;
};


int main (int argc, char** argv) {

	ros::init(argc, argv, "main");
	ros::NodeHandle nh;

	ofstream obs;
	obs.open("./Obstacles.txt", ios::out);
	ofstream ws;
	ws.open("./Region.txt",ios::out);
	ofstream ee;
	ee.open("./EndEffector.txt",ios::out);
	ofstream file;
	file.open("./Trajectory.txt", ios::out);

	ros::ServiceClient obstacleClient = nh.serviceClient<knowledge_msgs::obstacleSRV>("obstacleService");
	ros::ServiceClient rrtstarClient = nh.serviceClient<rrtstar_msgs::rrtStarSRV>("rrtStarService");
	ros::Subscriber subCtrlAck	= nh.subscribe("robot_control_ack",80, ackCallback);
	ros::Publisher controlPub = nh.advertise<controlCommand_msgs::control>("robot_control_command",80);

	//get obstacles from knowledge
	knowledge_msgs::obstacleSRV srv;
	vector<rrtstar_msgs::Region> obstacles;
	if(obstacleClient.call(srv)) {
		for(int i=0; i<srv.response.obstacles.size(); i++) {
			rrtstar_msgs::Region obstacle;
			obstacle.center_x = srv.response.obstacles[i].center_x;
    	obstacle.center_y = srv.response.obstacles[i].center_y;
    	obstacle.center_z = srv.response.obstacles[i].center_z;
	    obstacle.size_x = fabs(srv.response.obstacles[i].size_x);
	    obstacle.size_y = fabs(srv.response.obstacles[i].size_y);
	    obstacle.size_z = fabs(srv.response.obstacles[i].size_z);
	    obstacles.push_back(obstacle);
		}
	}

	for(int i=0; i<obstacles.size(); i++) {
		obs << obstacles[i].center_x << " " << obstacles[i].center_y << " " << obstacles[i].center_z <<
		 " " << obstacles[i].size_x << " " << obstacles[i].size_y << " " << obstacles[i].size_z << endl;
	}
	obs.close();

	rrtstar_msgs::Region WS;
	WS.center_x = 0.55;
	WS.center_y = 0;
	WS.center_z = 0.1;
	WS.size_x = 0.8;
	WS.size_y = 1.2;
	WS.size_z = 0.4;

	ws << WS.center_x << " " << WS.center_y << " " << WS.center_z << " " << WS.size_x << " " << WS.size_y << " " << WS.size_z << endl;
	ws.close();

	rrtstar_msgs::Region Goal;
	Goal.center_x = 0.8;
	Goal.center_y = -0.05;
	Goal.center_z = 0;
	Goal.size_x = 0.1;
	Goal.size_y = 0.1;
	Goal.size_z = 0.1;

	tf::TransformListener listener;
	tf::StampedTransform transform;

	try {
		listener.waitForTransform("/base","/left_gripper",ros::Time(0),ros::Duration(10.0));
		listener.lookupTransform("/base","/left_gripper",ros::Time(0),transform);
	} catch(tf::TransformException ex) {

	}

	rrtstar_msgs::Region object;
	object.center_x = transform.getOrigin().x();
	object.center_y = transform.getOrigin().y();
	object.center_z = transform.getOrigin().z();
	tf::Quaternion q = transform.getRotation();
	//cout << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
	tf::Matrix3x3 m(q);
	double yaw,pitch,roll;
	m.getEulerYPR(yaw,pitch,roll);
	cout << "yaw: " << yaw << endl;
	cout << "pitch: " << pitch << endl;
	cout << "roll: " << roll << endl;
	cout << endl;
	object.size_x = 0.05;
	object.size_y = 0.3;
	object.size_z = 0.05;

	ee << object.size_x << " " << object.size_y << " " << object.size_z << endl;
	ee.close();

	rrtstar_msgs::rrtStarSRV rrtstarsrv;
	rrtstarsrv.request.WS = WS;
	rrtstarsrv.request.Goal = Goal;
	rrtstarsrv.request.Object = object;
	rrtstarsrv.request.Obstacles = obstacles;

	vector<geometry_msgs::Vector3> path;
	if(rrtstarClient.call(rrtstarsrv)) {
		for(int i=0; i<rrtstarsrv.response.path.size(); i++) {
			geometry_msgs::Vector3 point;
			point.x = rrtstarsrv.response.path[i].x;
			point.y = rrtstarsrv.response.path[i].y;
			point.z = rrtstarsrv.response.path[i].z;
			path.push_back(point);
		}
	}


	for(int i=0; i<path.size(); i++) {
		file  << path[i].x << " " << path[i].y << " " << path[i].z << endl;

	}
	file.close();

	int i = 0;
	int aux;
	ros::Rate loop_rate(10);
	while(ros::ok()) {

		if(ackReceived) {
			i++;
			if(i == path.size()) {
				break;
			}
			controlCommand_msgs::control controlMsg;
			controlMsg.Activation = 0;
			controlMsg.oneArm.armIndex = 0;
			controlMsg.oneArm.armCmndType = "cartPos";
			controlMsg.oneArm.cartGoal.cartesianPosition[0] = yaw;
			controlMsg.oneArm.cartGoal.cartesianPosition[1] = pitch;
			controlMsg.oneArm.cartGoal.cartesianPosition[2] = roll;
			controlMsg.oneArm.cartGoal.cartesianPosition[3] = path[i].x;
			controlMsg.oneArm.cartGoal.cartesianPosition[4] = path[i].y;
			controlMsg.oneArm.cartGoal.cartesianPosition[5] = path[i].z;
			cout << "x: " << path[i].x << endl;
			cout << "y: " << path[i].y << endl;
			cout << "z: " << path[i].z << endl;
			cout << endl;
			//cin >> aux;
			controlPub.publish(controlMsg);

			ackReceived = false;
		}

		ros::spinOnce();
		loop_rate.sleep();

	}

  return 1;
}
