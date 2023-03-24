#include <tf/tf.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
using namespace std;
  
nav_msgs::Odometry feedback;
 
double yaw2;


//****************************************************************//
//			          	VARIÁVEIS GLOBAIS - v1                    //
//****************************************************************//
//tf::Pose pose;
//double x=0,y=0,theta, x_ant, y_ant, delta, travelled = 0;
//geometry_msgs::Twist msg;	
//bool ori_ok = false, pos_ok = false;
float posdesejada[2], oridesejada, erropos=99, erroorie=99, erropos_1, erropos_2;
//float tolerance_orie = 0.05, tolerance_pos = 0.1;
//float angulo;

double dist_obs, ang_obs;
bool is_obs = false;
int i,j,k;
double indice;

double range_desvio_far = 1;
double range_desvio_close = 0.7;
double range_desvio_very_close = 0.4;
double range_desvio = range_desvio_far;

double lidar_size, lidar_size_received;
double lidar_ignorar_percentage = 0.3;

//double count_lados;
//double count_frente;
//double count_ign;

//double max_lin_free = 1.2;
//double max_lin_obs = 0.6;

double size_1,size_2;

//****************************************************************//
//			          	VARIÁVEIS GLOBAIS - v2                  //
//****************************************************************//
tf::Pose _pose;
double _x=0,_y=0,_theta, _x_ant, _y_ant, _delta, _travelled = 0;
geometry_msgs::Twist _msg;	
bool _ori_ok = false, _pos_ok = false;
float _posdesejada[2], _oridesejada, _erropos=99, _erroorie=99, _erropos_1, _erropos_2;
float _tolerance_orie = 0.05, _tolerance_pos = 0.1;
float _angulo;

double _dist_obs, _ang_obs;
bool _is_obs = false;
int _i,_j,_k;
double _indice;

double _range_desvio_far = 1;
double _range_desvio_close = 0.7;
double _range_desvio_very_close = 0.4;
double _range_desvio = _range_desvio_far;

double _lidar_size, _lidar_size_received;
double _lidar_ignorar_percentage = 0.3;

double _count_lados;
double _count_frente;
double _count_ign;

double _max_lin_free = 1.2;
double _max_lin_obs = 0.6;

double _size_1,_size_2;


//Callback da Odometria.
void subCallback(const nav_msgs::Odometry::ConstPtr& msg){
  
	feedback.pose.pose.position.x = msg->pose.pose.position.x;
	feedback.pose.pose.position.y = msg->pose.pose.position.y;
	feedback.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  feedback.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  
	
  feedback.twist.twist.linear.x = msg->twist.twist.linear.x;
  feedback.twist.twist.angular.z = msg->twist.twist.angular.z;

   tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  yaw2 = yaw;
}
 
 
//Callback do LIDAR.
void subCallback_lidar(const sensor_msgs::LaserScan::ConstPtr& lidar)
{	
	lidar_size_received = lidar->ranges.size();
	lidar_size = floor(lidar_size_received*(1-lidar_ignorar_percentage));

	size_1 = floor((lidar_size_received - lidar_size)/2);
	size_2 = lidar_size_received - 2*size_1;

	dist_obs = 1.5;
	for (i=0;i<size_2;i++){
		if(lidar->ranges[i+size_1] < dist_obs && lidar->ranges[i+size_1] > 0.01){
			dist_obs = lidar->ranges[i+size_1];
			indice = i;
		}
	}
	
	range_desvio = erropos < range_desvio_close ? range_desvio_close : range_desvio_far;
	range_desvio = erropos < range_desvio_very_close ? range_desvio_very_close : range_desvio;

	//Verifica em qual percentual do range aceito do Lidar ele está.
	ang_obs = indice/size_2;

	//Se está nos 5% da região frontal, o range de desvio é maior.
	if (ang_obs > (0.5-0.025) && ang_obs < (0.5+0.025))
	{
		range_desvio = range_desvio*1.2;
	}

	//Verifica os objetos próximos e, se dentro do range escolhido, aciona a flag de desvio.
	is_obs = dist_obs < range_desvio ? true : false;

	//cout << dist_obs << "\n";
	//cout << is_obs << "\n";
	//cout << ang_obs << "\n\n";
}


int main(int argc, char **argv){
  ros::init(argc, argv, "pid_coppelia");
 
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("odom", 1000, subCallback);
  ros::Subscriber sub_lidar = n.subscribe("scan", 1000, subCallback_lidar);
  ros::Time   last_time, actual_time;
  ros::Rate loop_rate(100);
 
  //system("rosservice call reset");
  //system("rqt_plot /turtle1/pose/x:y &");
  
  double erroSumL, lastErrL;
  double erroSumA, lastErrA;
  

  double linearX = 0;
  double kpL = 1;
  double kiL = 0;
  double kdL = 0.5;
  
  double angularZ = 0;
  double kpA = 3;
  double kiA = 0;
  double kdA = 0.4;
  double PI = 3.141592654;
  

  while (ros::ok()){
    geometry_msgs::Twist msg;
    float posX = 1.8, posY = 1.5;
    float tolerance = 5;
    double errorLinear = 99;
    double errorAngular = 0;
    
    //cout << "Digite a posicao\nX>>";
    //cin >> posX;

    //cout << "Digite a posicao\nY>>";
    //cin >> posY;


    while (abs(errorLinear) > tolerance) {

        if(is_obs){

          }else{

        double yaw = PI * feedback.pose.pose.orientation.z;
      
        yaw = -(yaw*0.2)+yaw;

        yaw = yaw2;

        actual_time = ros::Time::now();
        last_time = actual_time;
        double timeChange = (actual_time = last_time).toSec();

        //linear PID   
        errorLinear = sqrt(pow((posX-feedback.pose.pose.position.x),2)+pow((posY-feedback.pose.pose.position.y),2));
        erroSumL += (errorLinear * timeChange);
        double dErrL =  (errorLinear - lastErrL) / timeChange;
        linearX = kpL * errorLinear + kiL * erroSumL + kdL * dErrL;

        //linear P
        // errorLinear = sqrt(pow((feedback.x-posX),2)+pow((feedback.y-posY),2));
        // linearX = kpL * errorLinear;

        //angular PID
        errorAngular = atan2(posY-feedback.pose.pose.position.y, posX-feedback.pose.pose.position.x);
        
        erroSumA += (errorAngular * timeChange);
        double dErrA =  (errorAngular - lastErrA) / timeChange;
        //double err =  (errorAngular - yaw);
        double err =  (errorAngular)-(yaw);
        
        //verifica se o erro esteja dentro do intevalo de -PI à PI
        if (err > PI) {
            err = err - (2 * PI);
        } 
        if (err < -PI) {
            err = err + (2 * PI);
        }    


        angularZ = (kpA * err) + (kiA * erroSumA) + (kdA * dErrA);
        
        //angular P
        // errorAngular = atan2(posY-feedback.y, posX-feedback.x);
        // double velAng =  (errorAngular - yaw);
        
        // //verifica se o erro esteja dentro do intevalo de -PI à PI
        // if (velAng > PI) {
        //     velAng = velAng - (2 * PI);
        // } 
        // if (velAng < -PI) {
        //     velAng = velAng + (2 * PI);
        // }    
        // angularZ = velAng * kpA;


        msg.linear.x = linearX;
        msg.angular.z = angularZ;
        
        //ROS_INFO("X = %f  erroL = %f    Y = %f erroA = %f ",linearX,errorLinear,angularZ, err);
        ROS_INFO("X=%f  Y=%f ",feedback.pose.pose.position.x,
                               feedback.pose.pose.position.y);
        
        ROS_INFO("LIN_X = %f     ANG_Z = %f", linearX, angularZ);
        ROS_INFO("----------------------");
        
        // erro = posX-feedback.x;
        // msg.linear.x = /Kpos*erro/(60/10);
        // ROS_INFO("TEMPO: %f   X>>%f,Erro>>%f",dt,feedback.x,erro);
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
      }
  	}
    ROS_WARN("...Posicao alcancada...");
    msg.linear.x = 0;
    msg.angular.z = 0;  
    pub.publish(msg);
    ros::spinOnce();
  }
 
  return 0;
}