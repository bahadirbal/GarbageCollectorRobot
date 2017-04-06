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

boost::shared_ptr<tf::TransformListener> tf_listener_; //smart pointer would be better than naked

#define CLOSENESS_THRESH 0.1 // 0.1m = 10cm - want to get this close to waypoint
#define LINEAR_VEL 5 //will go this fast in forward direction
#define ANGULAR_VEL 0.5 //will turn this fast
#define TURN_THRESH 0.02 // will be facing within 0.05 radians before will go forward
#define TURN_ANGLE_THRESH 5  // in degrees 
#define PI 3.141592653589793

using namespace std;

class MyYoubot
{
  public:
    MyYoubot() :
				nh("~"), t_listener(), sub(nh.subscribe("/base_scan/scan", 1, &MyYoubot::laserCallback, this)),
				pub (nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000)),
				arm_pos_pub(nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1)) {	
		//constructor initially all variables are set to 0	
		objectTaken=false;
		garbage_container_x=0;
		garbage_container_y=0;
		garbage_container_dir=-1;
		at_garbage_container=false;
		isCloseToObj=false;
		next_obj=true;
		isStart=true;
		isDone=false;
		obj_coordinates.push_back(pair<double,double>(2,0));
		obj_coordinates.push_back(pair<double,double>(-0.5,2.2));
		obj_coordinates.push_back(pair<double,double>(1.1,3.2));
		
		vel_msg.linear.x = 0.0;
		vel_msg.linear.y = 0.0;
		vel_msg.linear.z = 0.0;
		vel_msg.angular.x = 0.0;
		vel_msg.angular.y = 0.0;
		vel_msg.angular.z = 0.0;		
		pub.publish (vel_msg);
    }
    // callback for laser 
    void laserCallback (const sensor_msgs::LaserScan::ConstPtr& msg) { 
      	//~ if(isStart){
			//~ for (int i = 0; i < 10; i++)
				//~ setArmPos(1);
			//~ ros::Duration(3).sleep();
			//~ isStart=false;
		//~ }
      	
		ros::Rate loop_rate(30);
		while (ros::ok())
		{		
		 	cout << "\n===================================================== "<<endl;
			cout << "[PUBLISH] "<<endl;	
							
			//~ int pose;
			//~ cin>> pose;
			//~ setArmPos( pose);
			//~ 
			//~ cout<<"[GRIPPERS] "<<isGrippersClosed()<<endl;
		 //~ 
			//~ if(0)			
			if(!obj_coordinates.empty() && !isCloseToObj){
				cout << "[MOVE TO GARBAGE] "<<endl;	
				waypoint_x=obj_coordinates.front().first;
				waypoint_y=obj_coordinates.front().second;	
				getTargetDirection();	
				if(dist < 1 ){ // if distance < 1 switch to adjustRobotPosition() function
					if(isInFront(msg)){
						isCloseToObj=true;
						vel_msg.linear.x = 0.0;					 
						vel_msg.angular.z = 0.0;
						pub.publish(vel_msg);
						setArmPos(1);
						cout << "[SWITCH] "<<endl;	
					}else
					{
						isCloseToObj=false;
						vel_msg.linear.x = 0.0;					 
						vel_msg.angular.z = 1.0;
						pub.publish(vel_msg);
					}						
				}
				
			}else if (isCloseToObj && !objectTaken)
			{
				cout << "[ADJUST ROBOT POSITION] "<<endl;	
				adjustRobotPosition(msg);
				if(objectTaken){
					vel_msg.linear.x = 0.0;					 
					vel_msg.angular.z = 0.0;
				}
					
			}else if(objectTaken){
				cout << "[MOVE TO GARBAGE CONTAINER] "<<endl;	
				waypoint_x=0;
				waypoint_y=0;	
				getTargetDirection();	
				if(dist < 0.3 ){
					isCloseToObj=false;
					vel_msg.linear.x = 0.0;					 
					vel_msg.angular.z = 0.0;
					pub.publish(vel_msg);
					setArmPos(4);
					objectTaken=false;
					obj_coordinates.pop_front();
					cout << "[CAME TO GARBAGE CONTAINER] "<<endl;						
				}
			}else{
				cout << "[WORK IS DONE] "<<endl;
				if(!isDone){
					setArmPos(5);
					isDone=true;
				}
			}
			pub.publish(vel_msg);
			 
			//Do this.
			ros::spinOnce();
			//Added a delay so not to spam
			loop_rate.sleep();

		}
    }
    // getting angle of any point according to global (/odom) frame 
    float getAngle(float z,float w){
		float angle=2*180/3.14159265*asin(z);	 
	    float tmp = 2 *180/3.14159265*acos(w);
	    if ((int)tmp != (int)angle){
			angle= abs(angle);
			angle *= -1;		
			cout << " SAG"<< endl;
		}
		else
			cout << " SOL"<< endl;
		return angle;	
	}
    float getAnguleBtwnRobotAndPoint(const float &point_ang){
		if ((int)robot_angl >=0 && (int)robot_angl<=180 && (int)point_ang >=0 && (int)point_ang <=180)// if robot angle[0,180] and point positive[0,180]
		{
			return point_ang-robot_angl;
		}
		if ((int)robot_angl <0 && (int)robot_angl >=-179 && (int)point_ang <0 && (int)point_ang >=-179)// if robot angle(0,179] and point negative[-179,0) 
		{
			return point_ang-robot_angl;
		}
		if ((int)robot_angl >=0 && (int)robot_angl<=90 && (int)point_ang <0 && (int)point_ang >=-90)// if robot angle positive [0,90] and point angle negative [-90,0)
		{
			return point_ang-robot_angl;
		}
		if ((int)robot_angl >90 && (int)robot_angl<=180 && (int)point_ang <-90 && (int)point_ang >=-179)// if robot angle positive(90,180] and point angle negative[-179,-90)
		{
			if ((int)robot_angl > (int)point_ang)// if robot angle positive and point angle negative
			{
				return 360-abs(point_ang)-abs(robot_angl);
			}
			if ((int)robot_angl < (int)point_ang) // if robot angle negative  and point angle positive
			{
				return (-1)*(360-abs(point_ang)-abs(robot_angl));
			}			
		}		
	}
	//SETTING ARM POSITION
	void setArmPos(int pos){

        int check = 0;
        ros::NodeHandle n;
        ros::Publisher armPositionsPublisher;
        ros::Publisher gripperPositionPublisher;

        armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
        gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

        ros::Rate rate(20); //Hz
        double readValue = pos;
        static const int numberOfArmJoints = 5;
        static const int numberOfGripperJoints = 2;
        while (n.ok()) {

            check++;
            cout << check << endl;

            brics_actuator::JointPositions command;
            vector <brics_actuator::JointValue> armJointPositions;
            vector <brics_actuator::JointValue> gripperJointPositions;

            armJointPositions.resize(numberOfArmJoints); //TODO:change that
            gripperJointPositions.resize(numberOfGripperJoints);

            std::stringstream jointName;

            double POS[5];
            double GRIP_VAL=0;
           // cout << "Please type POSE value  " << endl;
            //cin >> readValue;
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

            //cout << "Please type in value for a left jaw of the gripper " << endl;
            //~ cin >> readValue;
            gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
            gripperJointPositions[0].value = GRIP_VAL;
            gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

            //cout << "Please type in value for a right jaw of the gripper " << endl;
            //~ cin >> readValue;
            gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
            gripperJointPositions[1].value = GRIP_VAL;
            gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

            cout << "sending command ... " <<readValue << endl;

            command.positions = armJointPositions;
            armPositionsPublisher.publish(command);

            command.positions = gripperJointPositions;
            gripperPositionPublisher.publish(command);

            cout << "--------------------" << endl;
            rate.sleep();
                      
            if(readValue==1 && isGrippersClosed()!=0){// not closed
                //cout << "CIKTI" << endl;
                return;
            }else if (readValue==1 && isGrippersClosed()==0)
			{
				ros::Duration(0.3).sleep();
			}else if ((readValue==2 && isGrippersClosed()==0) || (readValue==3 && isGrippersClosed()==0)){// closed
				 return;
			}else if ((readValue==2 && isGrippersClosed()!=0) || (readValue==3 && isGrippersClosed()!=0)){// not closed
				ros::Duration(0.3).sleep();
				if(check>=15)
					return;
			}else if ( readValue==4 && isGrippersClosed()==1 && check>=15) {// open
				return;
			}else if ( readValue==4 &&  isGrippersClosed()!=1)  {
				ros::Duration(0.3).sleep();
			}
			if(readValue==5 && check>=5)
				return;
			 
        }
    }
	// DIRECTS THE ROBOT TO GARBAGE OR GARBAGE CONTAINER
	int getTargetDirection(){
		int dirctn;	
		string direction="STOP";
				
		//GET ROBOT LOCATION AND ORIENTATION
		tf::StampedTransform robot_transform;
		try{
			tf_listener_->lookupTransform("/odom","/base_link",ros::Time(0),robot_transform);
			robot_x = robot_transform.getOrigin().x();
			robot_y = robot_transform.getOrigin().y();
			robot_orientation=robot_transform.getRotation().getAngle()*robot_transform.getRotation().getAxis().getZ();
		}catch (tf::TransformException ex){		
		}
		
		//GET ANGLE OF VECTOR FROM ROBOT TO TARGET
		double direction_vector_angle = atan2(waypoint_y-robot_y,waypoint_x-robot_x);

		//LOOK AT DIFFERENCE BETWEEN THIS ANGLE AND CURRENT ANGLE
		double angle_diff=direction_vector_angle-robot_orientation;
		while(angle_diff<-PI) angle_diff+=2*PI; //normalise to between -pi and pi 
		while(angle_diff>PI) angle_diff-=2*PI; //normalise to between -pi and pi 
		
		dist = sqrt(pow(robot_x-waypoint_x, 2) + pow(robot_y-waypoint_y, 2)); //dist between robot and point
		
		cout<< "[robot_x] " << robot_x << endl;
		cout<< "[robot_y] " << robot_y << endl;
		cout<< "[angle_diff] " << angle_diff << endl;
		cout<< "[dist to point] " << dist << endl; //point may be garbage or garbage container
		//TURN TO TARGET 	
		if(angle_diff>TURN_THRESH ){
			vel_msg.linear.x=0.0;
			//~ if(angle_diff)
			vel_msg.angular.z=0.8;
			direction = "LEFT";			
			dirctn=1;		
		}else if(angle_diff<-TURN_THRESH){
			vel_msg.linear.x=0.0;
			vel_msg.angular.z=-0.8;
			direction = "RIGHT";	
			dirctn=-1;				
		}else if (dist>0.3)//IF DIRECTION IS CORRECT GO
		{
			vel_msg.linear.x=0.7;
			vel_msg.angular.z=0.0;
			direction = "GO";						 				
			dirctn=0;
			
		}else{	
			vel_msg.linear.x=0.0;
			vel_msg.angular.z=0.0;
			direction = "STOP";						 				
			dirctn=0;
		}				
		pub.publish(vel_msg);	
		cout<< "[getTargetDirection] " << direction << endl;
		return dirctn;
	}
	void getTargetDirection2(){
							 
//==============		
		tf::StampedTransform transform;
		try{
		  tf_listener_->lookupTransform("/odom", "/base_link", ros::Time(0), transform);//transforms /base_link from local to global(/odom) frame
		}
		catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
		}
			 
		robot_x=transform.getOrigin().x(); //x nad y positions of robot
		robot_y=transform.getOrigin().y();
		dist = sqrt(pow(robot_x-waypoint_x, 2) + pow(robot_y-waypoint_y, 2)); //dist between robot and point
		robot_angl = getAngle(transform.getRotation()[2],transform.getRotation()[3]);//get angle of robot according to global frame		
		cout<<"[========> DISTANSE] "<< dist << endl;
		float point_angl= atan((double)(waypoint_y/waypoint_x)); // get angle of point according to global frame
		if (waypoint_x > 0 && waypoint_y > 0)
		{
			point_angl=abs(point_angl*180/PI);
		}else if (waypoint_x < 0 && waypoint_y > 0)
		{
			point_angl=180-abs(point_angl*180/PI);
		}else if (waypoint_x < 0 && waypoint_y < 0)
		{
			point_angl=180+abs(point_angl*180/PI);
		}else if (waypoint_x > 0 && waypoint_y < 0)
		{
			point_angl=360-abs(point_angl*180/PI);
		}	
		
		
		float angl_btwn_robot_and_point= getAnguleBtwnRobotAndPoint(point_angl);//get angle between point and robot according to global frame
		float tmp_alfa= 180/3.14 * (float) atan((double)(abs(waypoint_y-robot_y) / abs(waypoint_x-robot_x)));
		
		cout<<"[Waypoint Angle] "<< point_angl << endl;
		
		//~ if(0)			 
		if (dist<0.8)// if dist smaller than 0.3 stop
		{
			if (angl_btwn_robot_and_point >= -6 && angl_btwn_robot_and_point <= 6   )
			{
				vel_msg.linear.x = 0.0;
				vel_msg.angular.z = 0.0 ;
			}
			else if( angl_btwn_robot_and_point < 0)
			{
				vel_msg.linear.x = 0.0;
				vel_msg.angular.z = -0.8 ;
			}
			else if( angl_btwn_robot_and_point > 0)
			{
				vel_msg.linear.x = 0.0;
				vel_msg.angular.z = 0.8 ;
			}
		}// if dist bigger than 0.3, there are four cases of location of robot and point location
		else if(dist >=0.3)//==============================robot in 4 quadrant point in 2 quadrant
		{
			cout<<"[dist] >= 0.3 "<<endl;
			if (waypoint_y>=robot_y && waypoint_x<=robot_x )//if point y>robot y and point x<robot x 
			{
				float tmp_an=0.0;
				if (robot_angl>=-179 && robot_angl<0)
				{
					tmp_an= (180-tmp_alfa)+abs(robot_angl);
					if (tmp_an>180)
						tmp_an=360-tmp_an;						
				}else
				{
					tmp_an= (180-tmp_alfa)-abs(robot_angl);
					tmp_an=abs(tmp_an); 
				}
				if (robot_angl >= -tmp_alfa && robot_angl <= 180-tmp_alfa)
				{
					vel_msg.linear.x = 0.0;						 
					vel_msg.angular.z = 0.8 ;
				}else
				{
					vel_msg.linear.x = 0.0;						 
					vel_msg.angular.z = -0.8 ;
				}					 
				if(tmp_an<TURN_ANGLE_THRESH){
					vel_msg.linear.x = 1.3;
					vel_msg.angular.z = 0.0 ;
				}
				cout<<"robot_angl "<<robot_angl<<endl;
				cout<<"tmp_alfa "<<tmp_alfa<<endl;
				cout<<"tmp_an "<<tmp_an<<endl;
			}//============================================robot in 3 quadrant point in 1 quadrant
			else if (waypoint_y>=robot_y && waypoint_x>=robot_x )//if point y>robot y and point x>robot x 
			{
				float tmp_an=0.0;
				if (robot_angl < 0 && robot_angl >=-179 )
				{
					 tmp_an= tmp_alfa+abs(robot_angl);
					 if(tmp_an>180)
						tmp_an=360-tmp_an;
				}else
				{						
					 tmp_an=abs(tmp_alfa-robot_angl);
				}
				if (robot_angl<=tmp_alfa && robot_angl>= 0-(180-tmp_alfa))
				{
					cout<<"solllll "<<endl;
					vel_msg.linear.x = 0.0;
					vel_msg.angular.z = 0.8 ;
				}else
				{
					cout<<"saggggg "<<endl;
					vel_msg.linear.x = 0.0;
					vel_msg.angular.z = -0.8 ;
				}					
				if(tmp_an<TURN_ANGLE_THRESH){
					vel_msg.linear.x = 1.3;
					vel_msg.angular.z = 0.0 ;
				}
				cout<<"robot_angl "<<robot_angl<<endl;
				cout<<"tmp_alfa "<<tmp_alfa<<endl;
				cout<<"tmp_an "<<tmp_an<<endl;
			}//=================================================robot in 1 quadrant point in 3 quadrant
			else if (waypoint_y<=robot_y && waypoint_x<=robot_x )//if point y>robot y and point x>robot x 
			{
				float tmp_an=0.0;
				if (robot_angl < 0 && robot_angl >=-179 )
				{
					 tmp_an= abs(180-(abs(robot_angl)+tmp_alfa));
					 
				}else
				{						
					 tmp_an=180-(tmp_alfa-robot_angl);
					 if(tmp_an>180)
						tmp_an=360-tmp_an;
				}
				if (robot_angl<=tmp_alfa && robot_angl >= tmp_alfa-180)
				{
					cout<<"sagggggGG "<<endl;
					vel_msg.linear.x = 0.0;
					vel_msg.angular.z = -0.8 ;
				}else
				{						
					cout<<"solllllLL "<<endl;
					vel_msg.linear.x = 0.0;
					vel_msg.angular.z = 0.8 ;
				}					
				if(tmp_an<TURN_ANGLE_THRESH){
					vel_msg.linear.x = 1.3;
					vel_msg.angular.z = 0.0 ;
				}
				cout<<"robot_angl "<<robot_angl<<endl;
				cout<<"tmp_alfa "<<tmp_alfa<<endl;
				cout<<"tmp_an "<<tmp_an<<endl;
			}//=======================================================robot in 2 quadrant, point in 4 quadrant
			else if (waypoint_y<=robot_y && waypoint_x>=robot_x )//if point y>robot y and point x>robot x 
			{
					float tmp_an=0.0;
					if (robot_angl < 0 && robot_angl >=-179 )
					{
						  tmp_an= abs(tmp_alfa+robot_angl);
						 
					}else
					{						
						 tmp_an=tmp_alfa+robot_angl;
						 if(tmp_an>180)
							tmp_an=360-tmp_an;						 
					}
					if (robot_angl>=0-tmp_alfa && robot_angl <= 0-tmp_alfa+180)
					{
						cout<<"saGGGG "<<endl;
						vel_msg.linear.x = 0.0;
						vel_msg.angular.z = -0.8 ;
					}else
					{						
						cout<<"soLLLLL "<<endl;
						vel_msg.linear.x = 0.0;
						vel_msg.angular.z = 0.8 ;
					}					
					if(tmp_an<TURN_ANGLE_THRESH){
						vel_msg.linear.x = 1.3;
						vel_msg.angular.z = 0.0 ;
					}
					cout<<"robot_angl "<<robot_angl<<endl;
					cout<<"tmp_alfa "<<tmp_alfa<<endl;
					cout<<"tmp_an "<<tmp_an<<endl;
				}
		}
		
		double robot_orientation=transform.getRotation().getAngle()*transform.getRotation().getAxis().getZ();
		// to see what is happening					
		cout<<"\nROBOT Orientation \n"<< robot_orientation;
		printf("-->Robot Orientation z & w: [%.2f] & [%.2f]\n", transform.getRotation()[2],transform.getRotation()[3]);		
		printf("Point Position x & y: [%.2f] & [%.2f]\n", waypoint_x, waypoint_y);				
		printf("-->robot_x: [%.2f] and robot_y: [%.2f]\n",robot_x, robot_y);
		printf("-->dist: [%.2f]\n",dist);
		printf("-->robot_angl : [%.2f]\n",robot_angl);
		printf("-->point_angl : [%.2f]\n",point_angl);
		printf("-->angl_btwn_robot_and_point: [%.2f]\n",angl_btwn_robot_and_point);				    
		printf("-->vel linear.x : [%.2f]\n",vel_msg.linear.x);
		printf("-->vel angular.z: [%.2f]\n",vel_msg.angular.z);	   
		
		 	
		
		//~ vel_msg.linear.x = 0.0;
		//~ vel_msg.angular.z = 0.0;
		//~ pub.publish(vel_msg);	 
		
//==============		
					
		//ros::Duration(0.5).sleep(); // sleep for half a second
		//sleep(1);
		//~ ++count;
	}
	void adjustRobotPosition(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
				
		int center=(laser_msg->ranges.size()-1)/2; // center index of laser msg ranges vector
 
		for (int i = center-190; i < center+175; i++){
			if (i>=center-190 && i<=center-17) // IF OBJECT IS ON RIGHT SIGHT
			{
				//~ cout << "[-100 <= RIGHT <= -18] "<<i<<" "<< laser_msg->ranges[i] <<endl; 
				if(laser_msg->ranges[i]<=0.35 ){
					vel_msg.linear.x = 0.0;			 
					vel_msg.angular.z = -0.015;
				}else if( laser_msg->ranges[i]>0.35 && laser_msg->ranges[i]<=0.5 ){
					vel_msg.linear.x = 0.0;			 
					vel_msg.angular.z = -0.05;
				}else if( laser_msg->ranges[i]>0.5 && laser_msg->ranges[i]<=1 ){
					vel_msg.linear.x = 0.0;			 
					vel_msg.angular.z = -0.06;
				}else if( laser_msg->ranges[i]>1 && laser_msg->ranges[i]<=1.5 ){
					vel_msg.linear.x = 0.0;			 
					vel_msg.angular.z = -0.1;
				}else if( laser_msg->ranges[i]>1.5 && laser_msg->ranges[i]<=2 ){
					vel_msg.linear.x = 0.0;			 
					vel_msg.angular.z = -0.15;
				}
			}else if (i>=center-4 && i<=center+175)// IF OBJECT IS ON LEFT SIGHT
			{
				//~ cout << "[-3 <= LEFT <= 85] "<<i<<" "<< laser_msg->ranges[i] <<endl;
				if( laser_msg->ranges[i]<=0.35 ){// l
					vel_msg.linear.x = 0.0;			 
					vel_msg.angular.z = 0.015;
				}else if(laser_msg->ranges[i]>0.35 && laser_msg->ranges[i]<=0.5 ){
					vel_msg.linear.x = 0.0;			 
					vel_msg.angular.z = 0.05;
				}else if( laser_msg->ranges[i]>0.5 && laser_msg->ranges[i]<=1 ){
					vel_msg.linear.x = 0.0;			 
					vel_msg.angular.z = 0.06;
				}else if( laser_msg->ranges[i]>1 && laser_msg->ranges[i]<=1.5 ){
					vel_msg.linear.x = 0.0;			 
					vel_msg.angular.z = 0.1;
				}else if( laser_msg->ranges[i]>1.5 && laser_msg->ranges[i]<=2 ){
					vel_msg.linear.x = 0.0;			 
					vel_msg.angular.z = 0.15;
				}
			}else if( 1 /*i<=center-4 && i>=center-17*/ ){ // IF BETWEEN ARM GRIPPERS 
				cout << "[i<center-3 && i>center-18: GO] "<<i<<" "<< laser_msg->ranges[i] <<endl;
				
				if (laser_msg->ranges[i]>0.25 && laser_msg->ranges[i]<=0.35 && betweenGrippers(center,laser_msg, 11, 9, 2)){
					vel_msg.linear.x = 0.1;			 
					vel_msg.angular.z = -0.08;
				}else if (laser_msg->ranges[i]>0.35 && laser_msg->ranges[i]<=0.5){
					vel_msg.linear.x = 0.05;			 
					vel_msg.angular.z = 0.0;
				}else if (laser_msg->ranges[i]>0.5 && laser_msg->ranges[i]<=1){
					vel_msg.linear.x = 0.07;			 
					vel_msg.angular.z = 0.0;
				}else if (laser_msg->ranges[i]>1 && laser_msg->ranges[i]<=1.5 ){
					vel_msg.linear.x = 0.3;			 
					vel_msg.angular.z = 0.0;
				}else if (laser_msg->ranges[i]>1.5 && laser_msg->ranges[i]<=2 ){
					vel_msg.linear.x = 0.4;			 
					vel_msg.angular.z = 0.0;
				}else if(laser_msg->ranges[i]>0.0 && laser_msg->ranges[i]<=0.25 ){
					
					if(isGrippersClosed()==1){// IF GRIPPERS ARE OPEN CLOSE THEM AND PICK UP
						setArmPos(2);			
						setArmPos(3);						
					}
					
					//if(betweenGrippers(center, laser_msg,11, 9, 0.85)){
					if(isGrippersClosed()==2){ // IF OBJECT TAKEN 
						objectTaken=true;
						cout<<"[OBJECT TAKEN] " << endl;
						return;
					}else if(isGrippersClosed()==0){ // IF GRIPPERS ARE CLOSED MEANS THAT ROBOT DROPED THE OJECT ACCIDENTALLY, WILL RETRY
						cout<<"[OBJECT NOT TAKEN] " << endl;
						objectTaken=false;							
						vel_msg.linear.x = -3.0;			 
						vel_msg.angular.z = 0.0;
						pub.publish(vel_msg);
						setArmPos(1);
						return;
					}else{
						cout<<"[ACIK] "<<"true "<<endl;
					}					
				}
				cout<<"[isGrippersClosed()] "<<isGrippersClosed()<<endl; // FOR DEBUGGING
				if(isGrippersClosed()==-1 || isInFrontAndVeryClose(laser_msg)){
					cout<<"[OBJECT NOT TAKEN] " << endl;
					objectTaken=false;							
					vel_msg.linear.x = -3.0;			 
					vel_msg.angular.z = 0.0;
					pub.publish(vel_msg);
					setArmPos(1);
					return;	
				}
			}
							
		}
		//~ vel_msg.linear.x = 0.0;			 
		//~ vel_msg.angular.z = 0.0;
		
	}
	// CHECKES IF GARBAGE IS BETWEEN GRIPPER FINGERS
	bool betweenGrippers(int center,const sensor_msgs::LaserScan::ConstPtr& laser_msg, int to_r, int to_l, double myrange){
		
		for (int i = center-to_r; i < center-to_l ; i++)
		{
			if(laser_msg->ranges[i]>myrange){
				cout<<"[Between Grippers] "<<"false "<< i<<endl;
				return false;				
			}
		}
		cout<<"[Between Grippers] "<<"true"<<endl;
		return true;
	}
	bool isInFront( const sensor_msgs::LaserScan::ConstPtr& laser_msg){
		int center=(laser_msg->ranges.size()-1)/2; // center index of laser msg ranges vector
		int to_r=50, to_l=50, myrange=2.0;
		for (int i = center-to_r; i < center+to_l ; i++)
		{
			//~ cout<<"[RANGES] " << i<<" "<< laser_msg->ranges[i] << endl;
			if(laser_msg->ranges[i] <= myrange){
				cout<<"[IN FRONT] "<<"TRUE "<< i<<" "<< laser_msg->ranges[i] << endl;
				return true;				
			}
		}
		cout<<"[NOT IN FRONT] "<<"FALSE"<<endl;
		return false;
	}
	bool isInFrontAndVeryClose( const sensor_msgs::LaserScan::ConstPtr& laser_msg){
		int center=(laser_msg->ranges.size()-1)/2; // center index of laser msg ranges vector
		int to_r=30, to_l=30, myrange=0.25;
		for (int i = center-to_r; i < center+to_l ; i++)
		{
			//~ cout<<"[RANGES] " << i<<" "<< laser_msg->ranges[i] << endl;
			if(laser_msg->ranges[i] < myrange){
				cout<<"[IN FRONT AND VERY CLOSE] "<<"TRUE "<< i<<" "<< laser_msg->ranges[i] << endl;
				return true;				
			}
		}
		cout<<"[NOT IN FRONT AND NOT CLOSE] "<<"FALSE"<<endl;
		return false;
	}
	int isGrippersClosed(){
		double finger_l_y, finger_r_y;
				
		//GET ROBOT LOCATION AND ORIENTATION
		tf::StampedTransform finger_l_transform, finger_r_transform;
		try{
			tf_listener_->lookupTransform("/gripper_palm_link","/gripper_finger_link_l",ros::Time(0),finger_l_transform);
			tf_listener_->lookupTransform("/gripper_palm_link","/gripper_finger_link_r",ros::Time(0),finger_r_transform);
			finger_l_y = finger_l_transform.getOrigin().y();
			finger_r_y = finger_r_transform.getOrigin().y();			 
			//~ robot_orientation=robot_transform.getRotation().getAngle()*robot_transform.getRotation().getAxis().getZ();
		}catch (tf::TransformException ex){		
		}
		cout<<"[finger_l_y] "<<finger_l_y<<endl;
		cout<<"[finger_r_y] "<<finger_r_y<<endl;
		cout<<"[fingers diff] "<<abs(finger_r_y-finger_l_y)<<endl;
		//if <0.02 gripers are closed  0
		//if >0.035 gripers are opened  1
		//if <0.032 && >0.028 object is taken 2
		if(abs(finger_r_y-finger_l_y) < 0.02)
			return 0;
		else if(abs(finger_r_y-finger_l_y) > 0.036)
			return 1;
		else if(abs(finger_r_y-finger_l_y) <= 0.036 && abs(finger_r_y-finger_l_y) >= 0.028)
			return 2;
		else
			return -1; // call again
		
	}
  private:
	double dist;// DISTACE TO GARBAGE OR GARBAGE CONTAINER			
	double waypoint_x,waypoint_y,robot_x,robot_y, robot_angl, robot_orientation, garbage_container_x, garbage_container_y;//SOME COORDINATES
    geometry_msgs::Twist vel_msg; // VELOCITY MSG
    ros::NodeHandle nh;    // NODE HANDLER
    ros::Publisher pub; // VELOCITY MSG PUBLISHER
    ros::Publisher arm_pos_pub;//arm position publitisher
    ros::Publisher grip_pos_pub;//gripper position publitisher
    ros::Subscriber sub;    // LASER MSG SUBSCRIBER
    tf::TransformListener t_listener;  // USED TO CHECK IF GRIPPER IS CLOSED 
    bool objectTaken;   // IF GARBAGE IS PICKED UP --> TRUE
    int garbage_container_dir; // IF ROBOT ORIENTED TO GARBAGE CONTAINER
    bool at_garbage_container; //IF ROBOT IS NEAR THE GARBAGE CONTAINER
    bool next_obj;
    deque< pair<double,double> > obj_coordinates;
    bool isStart;	// IS IT FIRST LOOP
    bool isDone;	// IS WORK IS DONE
    bool isCloseToObj; // IF ROBOT IS NEAR THE OBJECT
    
};

int main (int argc, char** argv){
  ros::init (argc, argv, "t");
  tf_listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());
  MyYoubot yb;
  
  ros::spin();
  
  return 0;
}
