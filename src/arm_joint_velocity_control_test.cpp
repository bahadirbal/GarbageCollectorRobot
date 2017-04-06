
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
#include "brics_actuator/JointVelocities.h"

#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_arm_velocity_control_test");
	ros::NodeHandle n;
	ros::Publisher armJointVelocitiesPublisher;

	armJointVelocitiesPublisher = n.advertise<brics_actuator::JointVelocities > ("arm_1/arm_controller/velocity_command", 1);

	ros::Rate rate(20); //Hz
	double readValue;
	static const int numberOfJoints = 5;
	while (n.ok()) {
		brics_actuator::JointVelocities command;
		vector <brics_actuator::JointValue> setPoints;

		setPoints.resize(numberOfJoints /*+ 2*/); //TODO:change that

		std::stringstream jointName;

		boost::units::quantity<boost::units::si::angular_velocity> g;
		cout << boost::units::to_string(boost::units::si::radian_per_second) << endl;
		// ::io::base_unit_info <boost::units::si::angular_velocity>).name();
		for (int i = 0; i < numberOfJoints; ++i) {
			cout << "Please type in value for joint " << i + 1 << endl;
			cin >> readValue;

			jointName.str("");
			jointName << "arm_joint_" << (i + 1);

			setPoints[i].joint_uri = jointName.str();
			setPoints[i].value = readValue;

			setPoints[i].unit = boost::units::to_string(boost::units::si::radian_per_second);
			cout << "Joint " << setPoints[i].joint_uri << " = " << setPoints[i].value << " " << setPoints[i].unit << endl;

		};

		/*cout << "Please type in value for gripper " << endl;
		cin >> readValue;
		setPoints[numberOfJoints].joint_uri = "gripper_finger_joint_l";
		setPoints[numberOfJoints].value = readValue;
		setPoints[numberOfJoints].unit = boost::units::to_string(boost::units::si::meter_per_second);

		setPoints[numberOfJoints + 1].joint_uri = "gripper_finger_joint_r";
		setPoints[numberOfJoints + 1].value = readValue;
		setPoints[numberOfJoints + 1].unit = boost::units::to_string(boost::units::si::meter_per_second);

		cout << "sending command ..." << endl;
		 */
		command.velocities = setPoints;
		armJointVelocitiesPublisher.publish(command);
		cout << "--------------------" << endl;
		rate.sleep();

	}

	return 0;
}

/* EOF */
