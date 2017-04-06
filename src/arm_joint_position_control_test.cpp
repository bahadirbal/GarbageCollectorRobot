#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"

#include <boost/units/systems/si/torque.hpp>
#include <boost/units/systems/si/force.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/JointPositions.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_arm_position_control_test");
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

	return 0;
}

/* EOF */
