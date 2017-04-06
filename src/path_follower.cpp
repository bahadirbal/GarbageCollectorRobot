#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Twist.h>

#include <sstream>
#include <iostream>

using namespace std;
int ct=0;
class PathFollower
{
  public:
    PathFollower() :
				nh("~"), listener(),
				pub (nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000)),
				sub (nh.subscribe ("/route_cmd", 1, &PathFollower::poseCallback, this)){	 
		//constractor initially all variables are set to 0	
		count = 0;
		rob_pos_x=0.0;
		rob_pos_y=0.0;
		dist= 0.0;
		robo_angl=0.0;
		twist_msg.linear.x = 0.0;
		twist_msg.linear.y = 0.0;
		twist_msg.linear.z = 0.0;
		twist_msg.angular.x = 0.0;
		twist_msg.angular.y = 0.0;
		twist_msg.angular.z = 0.0;	 
		
		pub.publish (twist_msg);
    }
    // callback for route 
    void poseCallback (const arm_navigation_msgs::MultiDOFJointTrajectory::ConstPtr& route_msg) { 
      	if(count==route_msg->points[0].poses.size())
			count=0;
		
		printf("-->poseCallback: [%d]\n",ct++); //to see how many times poseCallback is called
		float tx=route_msg->points[0].poses[0].position.x, ty=route_msg->points[0].poses[0].position.y;        
		
		ros::Rate rate(30);// times per sec
		
		while (ros::ok()){
			 
	//==============		
			tf::StampedTransform transform;
			try{
			  listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);//transforms /base_link from local to global(/odom) frame
			}
			catch (tf::TransformException ex){
			  ROS_ERROR("%s",ex.what());
			}
			
			dist = sqrt(pow(rob_pos_x+tx, 2) + pow(transform.getOrigin().y()+ty, 2)); //dist between robo and point
				
			count=0;
			rob_pos_x=transform.getOrigin().x(); //x nad y positions of robot
			rob_pos_y=transform.getOrigin().y();
			robo_angl = getAngle(transform.getRotation()[2],transform.getRotation()[3]);//get angle of robot according to global frame		
			float point_angl= getAngle(route_msg->points[0].poses[count].orientation.z,route_msg->points[0].poses[count].orientation.w);// get angle of point according to global frame
			float angl_btwn_robo_and_point= getAnguleBtwnRobotAndPoint(point_angl);//get angle between point and robot according to global frame
			float tmp_alfa= 180/3.14 * (float) atan((double)(abs(ty-rob_pos_y) / abs(tx-rob_pos_x)));
			geometry_msgs::Twist vel_msg;					
			 
			if (dist<0.3)// if dist smaller than 0.3
			{
				if (angl_btwn_robo_and_point >= -6 && angl_btwn_robo_and_point <= 6   )
				{
					vel_msg.linear.x = 0.8;
					vel_msg.angular.z = 0.0 ;
				}
				else if( angl_btwn_robo_and_point < 0)
				{
					vel_msg.linear.x = 0.0;
					vel_msg.angular.z = -0.8 ;
				}
				else if( angl_btwn_robo_and_point > 0)
				{
					vel_msg.linear.x = 0.0;
					vel_msg.angular.z = 0.8 ;
				}
			}// if dist bigger than 0.3, there are four cases of location of robot and point location
			else if(dist >=0.3)//==============================robot in 4 quadrant point in 2 quadrant
			{
				cout<<"dist >=0.3"<<endl;
				if (ty>=rob_pos_y && tx<=rob_pos_x )//if point y>robot y and point x<robot x 
				{
					float tmp_an=0.0;
					if (robo_angl>=-179 && robo_angl<0)
					{
						tmp_an= (180-tmp_alfa)+abs(robo_angl);
						if (tmp_an>180)
							tmp_an=360-tmp_an;						
					}else
					{
						tmp_an= (180-tmp_alfa)-abs(robo_angl);
						tmp_an=abs(tmp_an); 
					}
					if (robo_angl >= -tmp_alfa && robo_angl <= 180-tmp_alfa)
					{
						vel_msg.linear.x = 0.0;						 
						vel_msg.angular.z = 0.8 ;
					}else
					{
						vel_msg.linear.x = 0.0;						 
						vel_msg.angular.z = -0.8 ;
					}					 
					if(tmp_an<8){
						vel_msg.linear.x = 1.3;
						vel_msg.angular.z = 0.0 ;
					}
					cout<<"robo_angl "<<robo_angl<<endl;
					cout<<"tmp_alfa "<<tmp_alfa<<endl;
					cout<<"tmp_an "<<tmp_an<<endl;
				}//============================================robot in 3 quadrant point in 1 quadrant
				else if (ty>=rob_pos_y && tx>=rob_pos_x )//if point y>robot y and point x>robot x 
				{
					float tmp_an=0.0;
					if (robo_angl < 0 && robo_angl >=-179 )
					{
						 tmp_an= tmp_alfa+abs(robo_angl);
						 if(tmp_an>180)
							tmp_an=360-tmp_an;
					}else
					{						
						 tmp_an=abs(tmp_alfa-robo_angl);
					}
					if (robo_angl<=tmp_alfa && robo_angl>= 0-(180-tmp_alfa))
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
					if(tmp_an<8){
						vel_msg.linear.x = 1.3;
						vel_msg.angular.z = 0.0 ;
					}
					cout<<"robo_angl "<<robo_angl<<endl;
					cout<<"tmp_alfa "<<tmp_alfa<<endl;
					cout<<"tmp_an "<<tmp_an<<endl;
				}//=================================================robot in 1 quadrant point in 3 quadrant
				else if (ty<=rob_pos_y && tx<=rob_pos_x )//if point y>robot y and point x>robot x 
				{
					float tmp_an=0.0;
					if (robo_angl < 0 && robo_angl >=-179 )
					{
						 tmp_an= abs(180-(abs(robo_angl)+tmp_alfa));
						 
					}else
					{						
						 tmp_an=180-(tmp_alfa-robo_angl);
						 if(tmp_an>180)
							tmp_an=360-tmp_an;
					}
					if (robo_angl<=tmp_alfa && robo_angl >= tmp_alfa-180)
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
					if(tmp_an<8){
						vel_msg.linear.x = 1.3;
						vel_msg.angular.z = 0.0 ;
					}
					cout<<"robo_angl "<<robo_angl<<endl;
					cout<<"tmp_alfa "<<tmp_alfa<<endl;
					cout<<"tmp_an "<<tmp_an<<endl;
				}//=======================================================robot in 2 quadrant, point in 4 quadrant
				else if (ty<=rob_pos_y && tx>=rob_pos_x )//if point y>robot y and point x>robot x 
				{
					float tmp_an=0.0;
					if (robo_angl < 0 && robo_angl >=-179 )
					{
						  tmp_an= abs(tmp_alfa+robo_angl);
						 
					}else
					{						
						 tmp_an=tmp_alfa+robo_angl;
						 if(tmp_an>180)
							tmp_an=360-tmp_an;						 
					}
					if (robo_angl>=0-tmp_alfa && robo_angl <= 0-tmp_alfa+180)
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
					if(tmp_an<8){
						vel_msg.linear.x = 1.3;
						vel_msg.angular.z = 0.0 ;
					}
					cout<<"robo_angl "<<robo_angl<<endl;
					cout<<"tmp_alfa "<<tmp_alfa<<endl;
					cout<<"tmp_an "<<tmp_an<<endl;
				}
			}
			double robot_orientation=transform.getRotation().getAngle()*transform.getRotation().getAxis().getZ();
			// to see what is happening					
			cout<<"\nROBOT Orientation \n"<< robot_orientation;
			printf("\nRoute Orientation.z and w: [%.2f], [%.2f]\n", route_msg->points[0].poses[count].orientation.z,route_msg->points[0].poses[count].orientation.w);
			printf("-->Robot Orientation z & w: [%.2f] & [%.2f]\n", transform.getRotation()[2],transform.getRotation()[3]);		
			printf("Point Position x & y: [%.2f] & [%.2f]\n", tx, ty);				
			printf("-->rob_pos_x: [%.2f] and rob_pos_y: [%.2f]\n",rob_pos_x, rob_pos_y);
		    printf("-->dist: [%.2f]\n",dist);
		    printf("-->robo_angl : [%.2f]\n",robo_angl);
		    printf("-->point_angl : [%.2f]\n",point_angl);
		    printf("-->angl_btwn_robo_and_point: [%.2f]\n",angl_btwn_robo_and_point);	
		    printf("-->count: [%d]\n",count); 		    
		    printf("-->vel linear.x : [%.2f]\n",vel_msg.linear.x);
		    printf("-->vel angular.z: [%.2f]\n",vel_msg.angular.z);	   
		    printf("-->poseCallback: [%d]\n",ct); 	
		    printf("-->route_msg->points[0].poses.size: [%d]\n",route_msg->points.size()); 	
		    
		    //~ vel_msg.linear.x = 0.0;
			//~ vel_msg.angular.z = 0.0;
			pub.publish(vel_msg);	 
			
	//==============		
			 rate.sleep();
		 
			 ros::spinOnce();		
					
			//ros::Duration(0.5).sleep(); // sleep for half a second
			//sleep(1);
			//~ ++count;
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
		if ((int)robo_angl >=0 && (int)robo_angl<=180 && (int)point_ang >=0 && (int)point_ang <=180)// if robot angle[0,180] and point positive[0,180]
		{
			return point_ang-robo_angl;
		}
		if ((int)robo_angl <0 && (int)robo_angl >=-179 && (int)point_ang <0 && (int)point_ang >=-179)// if robot angle(0,179] and point negative[-179,0) 
		{
			return point_ang-robo_angl;
		}
		if ((int)robo_angl >=0 && (int)robo_angl<=90 && (int)point_ang <0 && (int)point_ang >=-90)// if robot angle positive [0,90] and point angle negative [-90,0)
		{
			return point_ang-robo_angl;
		}
		if ((int)robo_angl >90 && (int)robo_angl<=180 && (int)point_ang <-90 && (int)point_ang >=-179)// if robot angle positive(90,180] and point angle negative[-179,-90)
		{
			if ((int)robo_angl > (int)point_ang)// if robot angle positive and point angle negative
			{
				return 360-abs(point_ang)-abs(robo_angl);
			}
			if ((int)robo_angl < (int)point_ang) // if robot angle negative  and point angle positive
			{
				return (-1)*(360-abs(point_ang)-abs(robo_angl));
			}			
		}		
	}
  private:
	int count;	
	float dist;	
	float rob_pos_x;	
	float rob_pos_y;
	float robo_angl;	
    ros::NodeHandle nh;    
    ros::Publisher pub;
    ros::Subscriber sub;    
    tf::TransformListener listener;
    geometry_msgs::Twist twist_msg;
};

int main (int argc, char** argv){
  ros::init (argc, argv, "t");
  
  PathFollower test_node;
  
  ros::spin();
  
  return 0;
}
