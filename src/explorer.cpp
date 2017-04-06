/*
* explorer.cpp
*
* Example file for BLG436E - Assignment 3
*
*/
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <string>
#include <vector>
#include <queue>
#include <iostream>
#define LINEAR_VEL 1.8
#define ANGULAR_VEL 1.8
using namespace std;
int my_count=0;

void decideAndSetGeomMsg(const sensor_msgs::LaserScan::ConstPtr& msg, geometry_msgs::Twist &geom_msg){
	int sector=msg->ranges.size()/3; // laser scan data divided into 3 sectors: center, left, right, then center is divided into 3 small sectors

	float avrg_dist_center=0;       //avarage distance in center 
	float avrg_dist_left_from_center=0;       //avarage distance left_from_center 
	float avrg_dist_right_form_center=0;       //avarage distance right_form_center
	float avrg_dist_left=0;       //avarage distance in left
	float avrg_dist_right=0;       //avarage distance in right

	int i=0;
	for ( i = 0; i < sector ; i++)	
		avrg_dist_right += msg->ranges[i];	//right	
	for ( i = sector; i < sector*4/3 ; i++)	
		avrg_dist_right_form_center += msg->ranges[i];	//avrg_dist_right_form_center	
	for ( i = sector*4/3; i < sector*5/3; i++)	
		avrg_dist_center += msg->ranges[i];	//center	
	for ( i = sector*5/3; i < sector*2; i++)	
		avrg_dist_left_from_center += msg->ranges[i];	//avrg_dist_left_from_center	
	for ( i = sector*2; i < msg->ranges.size(); i++) 
		avrg_dist_left += msg->ranges[i];	 //left

	avrg_dist_right /= sector;
	avrg_dist_right_form_center /= sector/3;
	avrg_dist_center /= sector/3;
	avrg_dist_left_from_center /= sector/3;
	avrg_dist_left /= sector;

	printf("avrg_dist_right [%.2f]\n",avrg_dist_right);
	printf("avrg_dist_right_form_center [%.2f]\n",avrg_dist_right_form_center);
	printf("avrg_dist_center [%.2f]\n",avrg_dist_center);
	printf("avrg_dist_left_from_center [%.2f]\n",avrg_dist_left_from_center);
	printf("avrg_dist_left [%.2f]\n",avrg_dist_left);	

	string direction;
	if (avrg_dist_center < 1.0)
	{
	//~ if (avrg_dist_right_form_center < 1.0 && avrg_dist_left_from_center < 1.0){
	//~ 
	//~ }
	if (avrg_dist_right < avrg_dist_left)
	{
		geom_msg.linear.x = 0.0;	//stop 
		geom_msg.angular.z = 1.8;   //turn left
		direction = "left";
	}else
	{
		geom_msg.linear.x = 0.0;	//stop 
		geom_msg.angular.z = -1.8;   //turn right
		direction = "right";
	}	
	}else if ( (avrg_dist_right_form_center + avrg_dist_left_from_center + avrg_dist_center)/3 < 1.3)
	{
	if (avrg_dist_right < avrg_dist_left)
	{
		geom_msg.linear.x = 0.0;	//stop 
		geom_msg.angular.z = 1.8;   //turn left
		direction = "left";
	}else
	{
		geom_msg.linear.x = 0.0;	//stop 
		geom_msg.angular.z = -1.8;   //turn right
		direction = "right";
	}
	}	
	else if (avrg_dist_right_form_center < 1.3)
	{
		geom_msg.linear.x = 0.0;	//stop 
		geom_msg.angular.z = 1.8;   //turn left
		direction = "left";
	}else if (avrg_dist_left_from_center < 1.3)
	{
		geom_msg.linear.x = 0.0;	//stop 
		geom_msg.angular.z = -1.8;   //turn right
		direction = "right";
	}else
	{
		geom_msg.linear.x = 1.0;	//go 
		geom_msg.angular.z = 0.0;   //no turn 
		direction = "go";
	}

	cout<< direction << endl;





}
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	my_count++;
	cout<<"[count] >>> ["<< my_count << "]"<<endl;
	ros::NodeHandle n;	
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		geometry_msgs::Twist vel_msg; 

		vel_msg.linear.x = 0.0;
		vel_msg.linear.y = 0.0;
		vel_msg.linear.z = 0.0;
		vel_msg.angular.x = 0.0;
		vel_msg.angular.y = 0.0;
		vel_msg.angular.z = 0.0;

		cout << "\n===================================================== "<<endl;
		cout << "PUBLISH "<<endl;
		decideAndSetGeomMsg( msg, vel_msg);
		pub.publish(vel_msg);



		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		loop_rate.sleep();

	}
  
}
int main(int argc, char **argv)
{
 	ros::init(argc, argv, "explorer");
 	ros::NodeHandle n; 	
 	ros::Subscriber laser_sub = n.subscribe("/base_scan/scan", 100, laserCallback);

 	ros::spin();
 	return 0;
}
