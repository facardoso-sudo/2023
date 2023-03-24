#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <iostream>
#include <math.h>


#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"



 
using namespace std;
 


turtlesim::Pose feedback;
nav_msgs::Odometry odom;


void subCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	feedback.x = -msg->pose.pose.position.z;
	feedback.y = msg->pose.pose.position.y;

/*	
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

*/

	
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                  msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  feedback.theta = yaw + M_PI/2;
  //feedback.theta = msg->theta;

}
 
 
int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "pid3D");
 
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("odom", 1000, subCallback);
 
  ros::Rate loop_rate(10);
 
  if (ros::ok())
  {
    geometry_msgs::Twist msg;
    float posdesejada[2], oridesejada, dist=99, erroorie=99;
    float tolerance_orie = 0.005, tolerance_pos = 0.05;
    float Kpos = 10; 
    float Korie = 15;
    float angulo;
 
    cout << "Digite a posicao\nX>>";
    cin >> posdesejada[0];
 
    cout << "Y>>";
    cin >> posdesejada[1];
 
    ros::spinOnce();
 
 
 	ROS_WARN("angulo>>%f\n",angulo);
 
    // Controle da orientacao
 	while (abs(erroorie) > tolerance_orie && dist > tolerance_pos) {
 		
		angulo = atan2(posdesejada[1]-feedback.y,posdesejada[0]-feedback.x);
        	dist = sqrt(pow(posdesejada[0]-feedback.y,2)+pow(posdesejada[1]-feedback.y,2));

		erroorie = angulo-feedback.theta;
 
        	msg.angular.z = Korie*erroorie/(60/10);
 		msg.linear.x = cos(erroorie)*abs(Kpos*(dist)/(60/10));
         	
		ROS_INFO("theta>>%f,Erro>>%f",feedback.theta,erroorie);
		ROS_INFO("Dist>>%f" ,dist); 

        	pub.publish(msg);
 					
        	ros::spinOnce();
 
        	loop_rate.sleep();
	}
 	msg.angular.z = 0;
	msg.linear.x = 0;
  	pub.publish(msg);
	ros::spinOnce();
	ROS_WARN("...Orientacao alcancada...");
 
  }
  return 0;
}
