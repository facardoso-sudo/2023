#include <tf/tf.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <iostream>
#include "FLIE-master/flie.h"
using namespace std;
  
nav_msgs::Odometry feedback;



double yaw2;

//nav_msgs::Odometry feedback;



//int threshold_occupied_;
//int threshold_free_;


//****************************************************************//
//			          	VARIÁVEIS GLOBAIS - v1                    //
//****************************************************************//
tf::Pose pose;
double x=0,y=0,theta, x_ant, y_ant, delta, travelled = 0;
geometry_msgs::Twist msg;	
bool ori_ok = false, pos_ok = false;
float posdesejada[2], oridesejada, erropos=99, erroorie=99, erropos_1, erropos_2;
float tolerance_orie = 0.05, tolerance_pos = 0.1;
float angulo;

double dist_obs, ang_obs;
bool is_obs = false;
int i,j,k;
double indice;

double range_desvio_far = 1;
double range_desvio_close = 0.4;
double range_desvio_very_close = 0.2;
double range_desvio = range_desvio_far;

double lidar_size, lidar_size_received;
double lidar_ignorar_percentage = 0.3;

double count_lados;
double count_frente;
double count_ign;

double max_lin_free = 1.2;
double max_lin_obs = 0.6;

double size_1,size_2;


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
		if(lidar->ranges[i+size_1] < dist_obs && lidar->ranges[i+size_1] > 0.1){
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
  double kpA = 4;
  double kiA = 0;
  double kdA = 0.4;
  double PI = 3.141592654;

//****************************************************************//
//	   			DEFINIÇÃO DAS VARIÁVEIS LINGUÍSTICAS              //
//****************************************************************//

fuzzy_set cat_linear[10];

/*** Erro Linear - begin ***/

		linguisticvariable erro_linear;

		cat_linear[0].setname("QZ");
		cat_linear[0].setrange(0,20);
		cat_linear[0].setval(0,0,0.7);

		cat_linear[1].setname("VERYCLOSE");
		cat_linear[1].setrange(0,20);
		cat_linear[1].setval(0.3,0.7,1.1);

		cat_linear[2].setname("CLOSE");
		cat_linear[2].setrange(0,20);
		cat_linear[2].setval(0.7,1.1,1.5);

		cat_linear[3].setname("FAR");
		cat_linear[3].setrange(0,20);
		cat_linear[3].setval(1.1,1.5,2);

		cat_linear[4].setname("VERYFAR");
		cat_linear[4].setrange(0,20);
		cat_linear[4].setval(1.5,2,20,20);

		erro_linear.setname("erro_linear");
		erro_linear.includecategory(&cat_linear[0]);
		erro_linear.includecategory(&cat_linear[1]);
		erro_linear.includecategory(&cat_linear[2]);
		erro_linear.includecategory(&cat_linear[3]);
		erro_linear.includecategory(&cat_linear[4]);

/*** Erro Linear - end ***/


/*** Velocidade Linear - begin ***/ 

		linguisticvariable vel_linear;

		cat_linear[5].setname("QZ");
		cat_linear[5].setrange(0,1);
		cat_linear[5].setval(0,0,0.2);

		cat_linear[6].setname("VERYSLOW");
		cat_linear[6].setrange(0,1);
		cat_linear[6].setval(0,0.2,0.35);

		cat_linear[7].setname("SLOW");
		cat_linear[7].setrange(0,1);
		cat_linear[7].setval(0.20,0.35,0.70);

		cat_linear[8].setname("FAST");
		cat_linear[8].setrange(0,1);
		cat_linear[8].setval(0.35,0.70,0.90);

		cat_linear[9].setname("VERYFAST");
		cat_linear[9].setrange(0,2);
		cat_linear[9].setval(0.90,1,1);

		vel_linear.setname("vel_linear");
		vel_linear.includecategory(&cat_linear[5]);
		vel_linear.includecategory(&cat_linear[6]);
		vel_linear.includecategory(&cat_linear[7]);
		vel_linear.includecategory(&cat_linear[8]);
		vel_linear.includecategory(&cat_linear[9]);
		
/*** Velocidade Linear - end ***/


fuzzy_set cat_angular[10];

/*** Erro Angular - begin ***/

		linguisticvariable erro_angular;

		cat_angular[0].setname("NB");
		cat_angular[0].setrange(-M_PI,0);
		cat_angular[0].setval(-M_PI,-M_PI, -M_PI/3, -M_PI/10);

		cat_angular[1].setname("NS");
		cat_angular[1].setrange(-M_PI,0);
		cat_angular[1].setval(-M_PI/3,-M_PI/10,0);

		cat_angular[2].setname("QZ");
		cat_angular[2].setrange(-M_PI,+M_PI);
		cat_angular[2].setval(-M_PI/10,0,+M_PI/10);

		cat_angular[3].setname("PS");
		cat_angular[3].setrange(0,+M_PI);
		cat_angular[3].setval(0,M_PI/10,M_PI/3);

		cat_angular[4].setname("PB");
		cat_angular[4].setrange(0,+M_PI);
		cat_angular[4].setval(M_PI/10,M_PI/3, M_PI, M_PI);

		erro_angular.setname("erro_angular");
		erro_angular.includecategory(&cat_angular[0]);
		erro_angular.includecategory(&cat_angular[1]);
		erro_angular.includecategory(&cat_angular[2]);
		erro_angular.includecategory(&cat_angular[3]);
		erro_angular.includecategory(&cat_angular[4]);

/*** Erro Angular - end ***/


/*** Velocidade Angular - begin ***/

		linguisticvariable vel_angular;

		cat_angular[5].setname("NB");
		cat_angular[5].setrange(-2*M_PI,0);
		cat_angular[5].setval(-2*M_PI,-2*M_PI,-M_PI/2,-0.3);

		cat_angular[6].setname("NS");
		cat_angular[6].setrange(-2*M_PI,0);
		cat_angular[6].setval(-M_PI/2,-0.3,0);

		cat_angular[7].setname("QZ");
		cat_angular[7].setrange(-2*M_PI,+2*M_PI);
		cat_angular[7].setval(-0.3,0,0.3);

		cat_angular[8].setname("PS");
		cat_angular[8].setrange(0,+2*M_PI);
		cat_angular[8].setval(0,0.3,M_PI/2);

		cat_angular[9].setname("PB");
		cat_angular[9].setrange(0,+2*M_PI);
		cat_angular[9].setval(0.3,M_PI/2,2*M_PI,2*M_PI);

		vel_angular.setname("vel_angular");
		vel_angular.includecategory(&cat_angular[5]);
		vel_angular.includecategory(&cat_angular[6]);
		vel_angular.includecategory(&cat_angular[7]);
		vel_angular.includecategory(&cat_angular[8]);
		vel_angular.includecategory(&cat_angular[9]);
		
/*** Velocidade Angular - end ***/



/*** Proximidade do obstáculo - begin ***/

		fuzzy_set cat_obstaculo[3];

		cat_obstaculo[0].setname("VERYCLOSE");
		cat_obstaculo[0].setrange(0, 2);
		cat_obstaculo[0].setval(0, 0, 0.4, 0.6);

		cat_obstaculo[1].setname("CLOSE");
		cat_obstaculo[1].setrange(0, 2);
		cat_obstaculo[1].setval(0.5, 0.7, 0.9);

		cat_obstaculo[2].setname("FAR");
		cat_obstaculo[2].setrange(0, 2);
		cat_obstaculo[2].setval(0.8, 1, 2, 2);

		linguisticvariable obstaculo_dist;
		obstaculo_dist.setname("obstaculo_dist");
		obstaculo_dist.includecategory(&cat_obstaculo[0]);
		obstaculo_dist.includecategory(&cat_obstaculo[1]);
		obstaculo_dist.includecategory(&cat_obstaculo[2]);

/*** Proximidade do obstáculo - end ***/



/*** Direção do obstáculo mais próximo - begin ***/

		fuzzy_set cat_direction[3];

		cat_direction[0].setname("DIREITA");
		cat_direction[0].setrange(0, 1);
		cat_direction[0].setval(0, 0, 0.5);

		cat_direction[1].setname("FRENTE");
		cat_direction[1].setrange(0, 1);
		cat_direction[1].setval(0.45, 0.5, 0.55);

		cat_direction[2].setname("ESQUERDA");
		cat_direction[2].setrange(0, 1);
		cat_direction[2].setval(0.5, 1, 1);

		linguisticvariable obstaculo_dir;
		obstaculo_dir.setname("obstaculo_dir");
		obstaculo_dir.includecategory(&cat_direction[0]);
		obstaculo_dir.includecategory(&cat_direction[1]);
		obstaculo_dir.includecategory(&cat_direction[2]);

/*** Direção do obstáculo mais próximo - end ***/



/*** Velocidade Linear para Desvio de Obstáculo - begin ***/

		fuzzy_set cat_vel_lin_obs[4];

		cat_vel_lin_obs[0].setname("QZ");
		cat_vel_lin_obs[0].setrange(0,1);
		cat_vel_lin_obs[0].setval(0,0,0.3);

		cat_vel_lin_obs[1].setname("VERYSLOW");
		cat_vel_lin_obs[1].setrange(0,1);
		cat_vel_lin_obs[1].setval(0,0.3,0.6);

		cat_vel_lin_obs[2].setname("SLOW");
		cat_vel_lin_obs[2].setrange(0,1);
		cat_vel_lin_obs[2].setval(0.3,0.6,1,1);

		linguisticvariable vel_linear_obs;
		vel_linear_obs.setname("vel_linear_obs");
		vel_linear_obs.includecategory(&cat_vel_lin_obs[0]);
		vel_linear_obs.includecategory(&cat_vel_lin_obs[1]);
		vel_linear_obs.includecategory(&cat_vel_lin_obs[2]);

/*** Velocidade Linear para Desvio de Obstáculo - end ***/


/*** Velocidade Angular para Desvio de Obstáculo - begin ***/

		fuzzy_set cat_vel_ang_obs[7];

		cat_vel_ang_obs[0].setname("NVB");
		cat_vel_ang_obs[0].setrange(-3,0);
		cat_vel_ang_obs[0].setval(-3, -3, -2.5, -2);

		cat_vel_ang_obs[1].setname("NB");
		cat_vel_ang_obs[1].setrange(-3,0);
		cat_vel_ang_obs[1].setval(-2.5, -2, -1);

		cat_vel_ang_obs[2].setname("NS");
		cat_vel_ang_obs[2].setrange(-3,0);
		cat_vel_ang_obs[2].setval(-2, -1, -0.4);

		cat_vel_ang_obs[3].setname("QZ");
		cat_vel_ang_obs[3].setrange(-3,3);
		cat_vel_ang_obs[3].setval(-0.7, 0, 0.7);

		cat_vel_ang_obs[4].setname("PS");
		cat_vel_ang_obs[4].setrange(0,3);
		cat_vel_ang_obs[4].setval(0.4, 1, 2);

		cat_vel_ang_obs[5].setname("PB");
		cat_vel_ang_obs[5].setrange(0,3);
		cat_vel_ang_obs[5].setval(1, 2, 2.5);

		cat_vel_ang_obs[6].setname("PVB");
		cat_vel_ang_obs[6].setrange(0,3);
		cat_vel_ang_obs[6].setval(2, 2.5, 3, 3);

		linguisticvariable vel_angular_obs;
		vel_angular_obs.setname("vel_angular");
		vel_angular_obs.includecategory(&cat_vel_ang_obs[0]);
		vel_angular_obs.includecategory(&cat_vel_ang_obs[1]);
		vel_angular_obs.includecategory(&cat_vel_ang_obs[2]);
		vel_angular_obs.includecategory(&cat_vel_ang_obs[3]);
		vel_angular_obs.includecategory(&cat_vel_ang_obs[4]);
		vel_angular_obs.includecategory(&cat_vel_ang_obs[5]);
		vel_angular_obs.includecategory(&cat_vel_ang_obs[6]);
		
/*** Velocidade Angular para Desvio de Obstáculo - end ***/



//****************************************************************//
//		      REGRAS FUZZY PARA CONTROLE DE POSIÇÃO	              //
//****************************************************************//

/*** Regras de controle de posição linear - begin ***/

	fuzzy_control fc_linear;
	fc_linear.set_defuzz(CENTROID);
	fc_linear.definevars(erro_linear, erro_angular, vel_linear);

	//Para a velocidade linear há uma não-linearidade na lógica.
	//Se o erro angular for grande, a velocidade linear tem que ser pequena.
	//Isso serve para ele não sair disparado na direção errada porque o erro é grande.
	//Logo, é necessário que o erro angular seja uma variável de entrada.
	
	//Se erro linear é "tanto faz" e erro angular é "negative big", então velocidade linear é "quase zero".
	fc_linear.insert_rule("QZ","NB","QZ");
	fc_linear.insert_rule("VERYCLOSE","NB","QZ");
	fc_linear.insert_rule("CLOSE","NB","QZ");
	fc_linear.insert_rule("FAR","NB","QZ");
	fc_linear.insert_rule("VERYFAR","NB","QZ");

	//Se erro linear é "tanto faz" e erro angular é "positive big", então velocidade linear é "quase zero".
	fc_linear.insert_rule("QZ","PB","QZ");
	fc_linear.insert_rule("VERYCLOSE","PB","QZ");
	fc_linear.insert_rule("CLOSE","PB","QZ");
	fc_linear.insert_rule("FAR","PB","QZ");
	fc_linear.insert_rule("VERYFAR","PB","QZ");

	//Se erro angular é "tanto faz" e erro linear é "quase zero", então velocidade linear é "quase zero".
	fc_linear.insert_rule("QZ","NB","QZ");
	fc_linear.insert_rule("QZ","NS","QZ");
	fc_linear.insert_rule("QZ","QZ","QZ");
	fc_linear.insert_rule("QZ","PS","QZ");
	fc_linear.insert_rule("QZ","PB","QZ");

	//Se erro angular é "tanto faz" e erro linear é "very close", então velocidade linear é "very slow".
	fc_linear.insert_rule("VERYCLOSE","NB","VERYSLOW");
	fc_linear.insert_rule("VERYCLOSE","NS","VERYSLOW");
	fc_linear.insert_rule("VERYCLOSE","QZ","VERYSLOW");
	fc_linear.insert_rule("VERYCLOSE","PS","VERYSLOW");
	fc_linear.insert_rule("VERYCLOSE","PB","VERYSLOW");

	//Se erro angular é "tanto faz" e erro linear é "close", então velocidade linear é "slow".
	fc_linear.insert_rule("CLOSE","NB","SLOW");
	fc_linear.insert_rule("CLOSE","NS","SLOW");
	fc_linear.insert_rule("CLOSE","QZ","SLOW");
	fc_linear.insert_rule("CLOSE","PS","SLOW");
	fc_linear.insert_rule("CLOSE","PB","SLOW");

	//Se erro angular é "tanto faz" e erro linear é "far", então velocidade linear é "fast".
	fc_linear.insert_rule("FAR","NB","FAST");
	fc_linear.insert_rule("FAR","NS","FAST");
	fc_linear.insert_rule("FAR","QZ","FAST");
	fc_linear.insert_rule("FAR","PS","FAST");
	fc_linear.insert_rule("FAR","PB","FAST");

	//Se erro angular é "tanto faz" e erro linear é "very far", então velocidade linear é "very fast".
	fc_linear.insert_rule("VERYFAR","NB","VERYFAST");
	fc_linear.insert_rule("VERYFAR","NS","VERYFAST");
	fc_linear.insert_rule("VERYFAR","QZ","VERYFAST");
	fc_linear.insert_rule("VERYFAR","PS","VERYFAST");
	fc_linear.insert_rule("VERYFAR","PB","VERYFAST");

/*** Regras de controle de posição linear - end ***/

/*** Regras de controle de posição angular - begin ***/

	fuzzy_control fc_angular;
	fc_angular.set_defuzz(CENTROID);
	fc_angular.definevars(erro_angular, vel_angular);
	fc_angular.insert_rule("NB","NB");
	fc_angular.insert_rule("NS","NS");
	fc_angular.insert_rule("QZ","QZ");
	fc_angular.insert_rule("PS","PS");
	fc_angular.insert_rule("PB","PB");
	
/*** Regras de controle de posição angular - end ***/
//****************************************************************//
//		      REGRAS FUZZY PARA DESVIO DE OBSTÁCULO	              //
//****************************************************************//

/*** Regras de controle de desvio linear - begin ***/

	fuzzy_control fc_desvio_linear;
	fc_desvio_linear.set_defuzz(CENTROID);
	fc_desvio_linear.definevars(obstaculo_dist, vel_linear_obs);

	fc_desvio_linear.insert_rule("VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("CLOSE","VERYSLOW");
	fc_desvio_linear.insert_rule("FAR","SLOW");
	
/*** Regras de controle de desvio linear - end ***/


/*** Regras de controle de desvio angular - begin ***/

	fuzzy_control fc_desvio_angular;
	fc_desvio_angular.set_defuzz(CENTROID);
	fc_desvio_angular.definevars(obstaculo_dir, obstaculo_dist, vel_angular_obs);

	fc_desvio_angular.insert_rule("DIREITA","VERYCLOSE","PVB");
	fc_desvio_angular.insert_rule("ESQUERDA","VERYCLOSE","NVB");
	fc_desvio_angular.insert_rule("FRENTE","VERYCLOSE","NVB");

	fc_desvio_angular.insert_rule("DIREITA","CLOSE","PB");
	fc_desvio_angular.insert_rule("ESQUERDA","CLOSE","NB");
	fc_desvio_angular.insert_rule("FRENTE","CLOSE","PB");

	fc_desvio_angular.insert_rule("DIREITA","FAR","PS");
	fc_desvio_angular.insert_rule("ESQUERDA","FAR","NS");
	fc_desvio_angular.insert_rule("FRENTE","FAR","PS");
	
/*** Regras de controle de desvio angular - end ***/


  while (ros::ok()){
    //geometry_msgs::Twist msg;
    //float posX = 1.8, posY = 1.5;
	posdesejada[0] = 1.8;
	posdesejada[1] = 1.5;

   
	//float tolerance = 5;
    double errorLinear = 99;
    double errorAngular = 0;
    
    //cout << "Digite a posicao\nX>>";
    //cin >> posX;

    //cout << "Digite a posicao\nY>>";
    //cin >> posY;


    while ((abs(erroorie) > tolerance_orie)) {

        if(is_obs){

        msg.angular.z = fc_desvio_angular.make_inference(ang_obs, dist_obs);
				msg.linear.x = fc_desvio_linear.make_inference(dist_obs)*max_lin_obs;

				erropos =  sqrt(pow(posdesejada[0]-feedback.pose.pose.position.x,2)+pow(posdesejada[1]-feedback.pose.pose.position.y,2));

				pub.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();

				system("clear");
				printf("Controle Ativo: DESVIO DE OBSTÁCULO.\n");
				printf("Distância total percorrida R1: %.2fm\n", travelled);
				printf("Erro de orientação R1: -\n");
				printf("Erro de posição R1: %.2f m\n",erropos);
				printf("Velocidade angular enviada R1: %.2f rad/s\n",msg.angular.z);
				printf("Velocidade linear enviada R1: %.2f m/s\n\n",msg.linear.x);

          }else{

        double yaw = PI * feedback.pose.pose.orientation.z;
      
        yaw = -(yaw*0.2)+yaw;

        yaw = yaw2;

        actual_time = ros::Time::now();
        last_time = actual_time;
        double timeChange = (actual_time = last_time).toSec();

        //linear PID   
        errorLinear = sqrt(pow((posdesejada[0]-feedback.pose.pose.position.x),2)+pow((posdesejada[1]-feedback.pose.pose.position.y),2));
        erroSumL += (errorLinear * timeChange);
        double dErrL =  (errorLinear - lastErrL) / timeChange;
        linearX = kpL * errorLinear + kiL * erroSumL + kdL * dErrL;

        //linear P
        //errorLinear = sqrt(pow((posX-feedback.pose.pose.position.x),2)+pow((posY-feedback.pose.pose.position.y),2));
        //msg.linear.x = fc_linear.make_inference(erropos, erroorie)*max_lin_free;
        //linearX = msg.linear.x * errorLinear;

        //angular PID
        errorAngular = atan2(posdesejada[1]-feedback.pose.pose.position.y, posdesejada[0]-feedback.pose.pose.position.x);
        
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
        ROS_INFO("X1=%f  Y1=%f ",feedback.pose.pose.position.x,
                               feedback.pose.pose.position.y);
        
        ROS_INFO("LIN_X1 = %f     ANG_Z1 = %f", linearX, angularZ);
        ROS_INFO("----------------------");
        
        // erro = posX-feedback.x;
        // msg.linear.x = /Kpos*erro/(60/10);
        // ROS_INFO("TEMPO: %f   X>>%f,Erro>>%f",dt,feedback.x,erro);
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        
		//Controle de posição
			errorLinear = sqrt(pow((posdesejada[0]-feedback.pose.pose.position.x),2)+pow((posdesejada[1]-feedback.pose.pose.position.y),2));
				if (errorLinear > tolerance_pos){
					pos_ok = false;
					msg.linear.x = fc_linear.make_inference(erropos, erroorie)*max_lin_free;
				}else{
					pos_ok = true;
					
          ROS_WARN("...Posicao R1 alcancada...");
					msg.linear.x = 0;
					msg.angular.z = 0;
					pub.publish(msg);
					ros::spinOnce();
					loop_rate.sleep();
				}
			}

        }

  	}

  return 0;
}