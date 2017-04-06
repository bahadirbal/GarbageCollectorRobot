#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <boost/units/systems/si.hpp>
#include <sstream>
#include <iostream>
#include <assert.h>

#include <boost/shared_ptr.hpp>
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"

#include <boost/units/systems/si/torque.hpp>
#include <boost/units/systems/si/force.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

ros::Publisher pub;  
ros::Subscriber sub; 
boost::shared_ptr<tf::TransformListener> tf_listener; //smart pointer would be better than naked
geometry_msgs::Twist vel_msg;

#define CLOSENESS_THRESH 0.1 // 0.1m = 10cm - want to get this close to waypoint
#define LINEAR_VEL 5 //will go this fast in forward direction
#define ANGULAR_VEL 0.5 //will turn this fast
#define TURN_THRESH 0.2 // will be facing within 0.2 radians before will go forward

#define PI 3.141592653589793
void setArmPos(int pos){
	ros::NodeHandle n;
	ros::Publisher armPositionsPublisher;
	ros::Publisher gripperPositionPublisher;

	armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
	gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

	ros::Rate rate(20); //Hz
	double readValue;
	static const int numberOfArmJoints = 5;
	static const int numberOfGripperJoints = 2;
	while (n.ok()) {
		brics_actuator::JointPositions command;
		vector <brics_actuator::JointValue> armJointPositions;
		vector <brics_actuator::JointValue> gripperJointPositions;

		armJointPositions.resize(numberOfArmJoints); //TODO:change that
		gripperJointPositions.resize(numberOfGripperJoints);

		std::stringstream jointName;

		double POS[5];
		double GRIP_VAL=0;
		cout << "Please type POSE value  " << endl;
		cin >> readValue;
		if(readValue==1){
			POS[0]=3;
			POS[1]=2.5;
			POS[2]=-1;
			POS[3]=0.7;
			POS[4]=3;			
			GRIP_VAL=2;
		}else if(readValue==2){
			POS[0]=3;
			POS[1]=2.5;
			POS[2]=-1;
			POS[3]=0.7;
			POS[4]=3;			
			GRIP_VAL=0;
		}else if(readValue==3){
			POS[0]=3;
			POS[1]=2;
			POS[2]=-1;
			POS[3]=0.7;
			POS[4]=3;			
			GRIP_VAL=0;		 
		}else if(readValue==4){
			POS[0]=3;
			POS[1]=2;
			POS[2]=-1;
			POS[3]=0.7;
			POS[4]=3;			
			GRIP_VAL=2;
		}else if(readValue==5){
			POS[0]=0;
			POS[1]=1;
			POS[2]=-1;
			POS[3]=2;
			POS[4]=3;		
			GRIP_VAL=0;			
		}else if(readValue==6){
			POS[0]=0;
			POS[1]=1;
			POS[2]=-1;
			POS[3]=2;
			POS[4]=3;		
			GRIP_VAL=2;	
		}
		// ::io::base_unit_info <boost::units::si::angular_velocity>).name();
		for (int i = 0; i < numberOfArmJoints; ++i) {
			//~ cout << "Please type in value for joint " << i + 1 << endl;
			

			jointName.str("");
			jointName << "arm_joint_" << (i + 1);

			armJointPositions[i].joint_uri = jointName.str();
			armJointPositions[i].value = POS[i];

			armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;

		};

		cout << "Please type in value for a left jaw of the gripper " << endl;
		//~ cin >> readValue;
		gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
		gripperJointPositions[0].value = GRIP_VAL;
		gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

		cout << "Please type in value for a right jaw of the gripper " << endl;
		//~ cin >> readValue;
		gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
		gripperJointPositions[1].value = GRIP_VAL;
		gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

		cout << "sending command ..." << endl;

		command.positions = armJointPositions;
		armPositionsPublisher.publish(command);

		command.positions = gripperJointPositions;
        gripperPositionPublisher.publish(command);

		cout << "--------------------" << endl;
		rate.sleep();

	}
	//~ ros::Duration(4).sleep();
}
 // callback for laser 
void laserCallback (const sensor_msgs::LaserScan::ConstPtr& msg) { 
	
	ros::Rate loop_rate(61);
	while (ros::ok())
	{

		cout << "\n===================================================== "<<endl;
		cout << "[PUBLISH] "<<endl;
		
		
		int pose;
		cin>> pose;
		setArmPos( pose);
		
		//~ setArmPos(1);
		
		
		//~ setArmPos(3);
		//~ getTargetDirection();
		
		pub.publish(vel_msg);
		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		loop_rate.sleep();

	}
}


int main(int argc, char **argv)
{       
        ros::init(argc, argv, "green");
        ros::NodeHandle n; 
        pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
        sub = n.subscribe("/base_scan/scan", 1,laserCallback);  //Subscriber
        tf_listener = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());
      
        ros::spin();
        return 0;
}
