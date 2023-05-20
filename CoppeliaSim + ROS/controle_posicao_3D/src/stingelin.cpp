#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <math.h>
#include <tf/tf.h>
 
using namespace std;
 
turtlesim::Pose feedback;
turtlesim::Pose  _feedback;
turtlesim::Pose  _feedback_;
nav_msgs::Odometry odom;


double yaw2;

//****************************************************************//
//			          	VARIÁVEIS GLOBAIS - v1                    //
//****************************************************************//
tf::Pose pose;
double x=0,y=0,theta, x_ant, y_ant, delta, travelled = 0;
geometry_msgs::Twist msg;	
bool ori_ok = false, pos_ok = false;

bool flag_x = false;
bool flag_y = false;
float distx_R1 = 99;
float distx_R2 = 99;

float disty_R1 = 99;
float disty_R2 = 99;
float efeito_derivativo = 1;

float posdesejada[2], oridesejada, error_dist_robo=99,erropos=99, erroorie=99, erropos_1, erropos_2;
float tolerance_orie = 0.05, tolerance_pos = 0.1, tolerance_min = 0.01;
float angulo;

double dist_obs, ang_obs;
bool is_obs = false;


//****************************************************************//
//			          	VARIÁVEIS GLOBAIS - v2                  //
//****************************************************************//
tf::Pose _pose;
double _x=0,_y=0,_theta, _x_ant, _y_ant, _delta, _travelled = 0;
geometry_msgs::Twist _msg;	
bool _ori_ok = false, _pos_ok = false;
float _posdesejada[2], _oridesejada, _error_dist_robo=99,_erropos=99, _erroorie=99, _erropos_1, _erropos_2;
float _tolerance_orie = 0.05, _tolerance_pos = 0.1;
float _angulo;

double _dist_obs, _ang_obs;
bool _is_obs = false;



//****************************************************************//
//			          	VARIÁVEIS GLOBAIS - v3                  //
//****************************************************************//
tf::Pose _pose_;
double _x_=0,_y_=0,_theta_, _x_ant_, _y_ant_, _delta_, _travelled_ = 0;
geometry_msgs::Twist _msg_;	
bool _ori_ok_ = false, _pos_ok_ = false;
float _posdesejada_[2], _oridesejada_, _erropos_=99, _erroorie_=99, _erropos_1_, _erropos_2_;
float _tolerance_orie_ = 0.05, _tolerance_pos_ = 0.1;
float _angulo_;

double _dist_obs_, _ang_obs_;
bool _is_obs_ = false;






float dcml_coordX = 0.0;
float dcml_coordY = 0.0;


//****************************************************************//
//			          TRATAMENTO DO FEEDBACK - R1                 //
//****************************************************************//


//Callback da Odometria.
void subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg)
{

	tf::poseMsgToTF(msg->pose.pose, pose);
  	theta = tf::getYaw(pose.getRotation());

	feedback.x = -msg->pose.pose.position.z;
	feedback.y = msg->pose.pose.position.y;


	
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                  msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  feedback.theta = yaw + M_PI/2;
  
}



//****************************************************************//
//			          TRATAMENTO DO FEEDBACK - v2                 //
//****************************************************************//

//Callback da Odometria.
void _subCallback_odom(const nav_msgs::Odometry::ConstPtr& _msg)
{


 	tf::poseMsgToTF(_msg->pose.pose, pose);
  	_theta = tf::getYaw(pose.getRotation());

	_feedback.x = -_msg->pose.pose.position.z;
	_feedback.y = _msg->pose.pose.position.y;


	
  tf::Quaternion q(_msg->pose.pose.orientation.x, _msg->pose.pose.orientation.y,
                  _msg->pose.pose.orientation.z, _msg->pose.pose.orientation.w);
  
  tf::Matrix3x3 m(q);
  double _roll, _pitch, _yaw;
  m.getRPY(_roll, _pitch, _yaw);

  _feedback.theta = _yaw + M_PI/2;
	  
}



//****************************************************************//
//			          TRATAMENTO DO FEEDBACK - v3                 //
//****************************************************************//

//Callback da Odometria.
void _subCallback_odom_(const nav_msgs::Odometry::ConstPtr& _msg_)
{


   	tf::poseMsgToTF(_msg_->pose.pose, pose);
  	_theta_ = tf::getYaw(pose.getRotation());

	_feedback_.x = -_msg_->pose.pose.position.z;
	_feedback_.y = _msg_->pose.pose.position.y;


	
  tf::Quaternion q(_msg_->pose.pose.orientation.x, _msg_->pose.pose.orientation.y,
                  _msg_->pose.pose.orientation.z, _msg_->pose.pose.orientation.w);
  
  tf::Matrix3x3 m(q);
  double _roll_, _pitch_, _yaw_;
  m.getRPY(_roll_, _pitch_, _yaw_);

  _feedback_.theta = _yaw_ + M_PI/2;
	  
}


int main(int argc, char **argv)
{

//****************************************************************//
//	   					CONFIGURAÇÃO DO ROS 		              //
//****************************************************************//
ros::init(argc, argv, "stingelin");

ros::NodeHandle n;
ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel1", 1000);
ros::Subscriber sub = n.subscribe("pose1", 1000, subCallback_odom);


ros::NodeHandle _n;
ros::Publisher _pub = _n.advertise<geometry_msgs::Twist>("cmd_vel2", 1000);
ros::Subscriber _sub = _n.subscribe("pose2", 1000, _subCallback_odom);


ros::NodeHandle _n_;
ros::Publisher _pub_ = _n_.advertise<geometry_msgs::Twist>("cmd_vel3", 1000);
ros::Subscriber _sub_ = _n_.subscribe("pose3", 1000, _subCallback_odom_);


ros::Rate loop_rate(10);





//****************************************************************//
//			    PROMPT SETPOINT (tolerância de desvio)	
//
// Código de planejamento de trajetória, "caminho feliz", Coppeliasim,
// informa o parametro para se saber os percentuais de desvios do 
// virtual e real em relação ao planejado, então os percentuais
// serão aplicados em ambos os robos, como aumento ou diminuição da 
// velocidade para encontrar a centralidade do caminho planejado.
//****************************************************************//




//****************************************************************//
//			 			CONTROLE DE POSIÇÃO (DCML)	 	                  //
//
// Aplicar o DCML obtendo dados cinemáticos das 3 entidades, 
// planejador, virtual e real, usar saida percentual do DCML para informar
// o parametro de ajuste para o controle de posição e
// plotar grafico demonstrando o efeito da DCML sobre o controlador 
// fuzzy e pid.
//****************************************************************//

	



		//cout << "Digite a posicao R1\nX>>";
    	//cin >> posdesejada[0];
		//cout << "Digite a posicao R2\nX>>";
 		//cin >> _posdesejada[0];

		posdesejada[0] = 4.25;
		posdesejada[1] = 6.66;
		

    	//cout << "Y para R1>>";
    	//cin >> posdesejada[1];
		//cout << "Y para R2>>";
		//cin >> _posdesejada[1];

		_posdesejada[0] = 4.25;
		_posdesejada[1] = 6.66;

    	//ros::spinOnce();

		//pos_ok = false;
		//ori_ok = false;
		
		while ((abs(error_dist_robo) > tolerance_pos) && (abs(_error_dist_robo) > _tolerance_pos)) {
 
				//while (ros::ok())
	              //  {
		            dcml_coordX = ((_feedback_.x-feedback.x)-(_feedback_.x-_feedback.x)/(log(_feedback_.x-feedback.x/_feedback_.x-_feedback.x)));
                    printf("dcml_coordX: %.2f m\n",dcml_coordX);
                    dcml_coordY = ((_feedback_.y-feedback.y)-(_feedback_.y-_feedback.y)/(log(_feedback_.y-feedback.y/_feedback_.y-_feedback.y)));
                    printf("dcml_coordY: %.2f m\n",dcml_coordY);    
		

					disty_R1 = _feedback_.y - feedback.y;
					printf("disty_R1: %.2f m\n",disty_R1);
            		disty_R2 = _feedback_.y - _feedback.y;
					printf("disty_R2: %.2f m\n",disty_R2);
			
					distx_R1 = _feedback_.x - feedback.x;
					printf("distx_R1: %.2f m\n",distx_R1);
            		distx_R2 = _feedback_.x - _feedback.x;
					printf("distx_R2: %.2f m\n",distx_R2);

					printf("_feedback_.x: %.2f m\n",_feedback_.x);
					printf("_feedback_.y: %.2f m\n",_feedback_.y);
					printf("feedback.x: %.2f m\n",feedback.x);
					
					printf("feedback.y: %.2f m\n",feedback.y);
					printf("_feedback.x: %.2f m\n",_feedback.x);
					
					printf("_feedback.y: %.2f m\n",_feedback.y);
	            //}	
		          //  ros::spinOnce();
            
            //Verifica se há objetos próximos em v1.
			//Se sim, aciona o controle de desvio.
			//Se não, aciona o controle de posição.

            error_dist_robo = sqrt(pow(posdesejada[0]-feedback.x,2)+pow(posdesejada[1]-feedback.y,2));
            _error_dist_robo = sqrt(pow(_posdesejada[0]-_feedback.x,2)+pow(_posdesejada[1]-_feedback.y,2));

            //dcml_coordX = ((_feedback_.x-feedback.x)-(_feedback_.x-_feedback.x)/(log(_feedback_.x-feedback.x/_feedback_.x-_feedback.x)))
            flag_x = isnan(dcml_coordX) ? true : false;
            flag_y = isnan(dcml_coordY) ? true : false;
            
            is_obs = dcml_coordX > 0 ? true : false;
            _is_obs = dcml_coordY > 0 ? true : false;

					//Verifica se há objetos próximos em v2.
			//Se sim, aciona o controle de desvio.
			//Se não, aciona o controle de posição.
			if (_is_obs){

				if(disty_R1 > tolerance_pos){

					msg.linear.x = -4;
                	
                	pub.publish(msg);
                	ros::spinOnce();
					loop_rate.sleep();

				}else{
			
                	msg.linear.x = -2;
                	
                	pub.publish(msg);
                	ros::spinOnce();
					loop_rate.sleep();

				}
			

			if (disty_R2 > tolerance_pos){

				

					_msg.linear.x = -5;

                	_pub.publish(_msg);
					ros::spinOnce();
					loop_rate.sleep();

				}else{
			
                	_msg.linear.x = -3;

                	_pub.publish(_msg);
					ros::spinOnce();
					loop_rate.sleep();

				
				}
			
			}else{
				
				if(disty_R1 > tolerance_pos){

					msg.linear.x = -4;
                	
                	pub.publish(msg);
                	ros::spinOnce();
					loop_rate.sleep();

				}else{
			
                	msg.linear.x = -2;
                	
                	pub.publish(msg);
                	ros::spinOnce();
					loop_rate.sleep();

				}
			

			if (disty_R2 > tolerance_pos){

				

					_msg.linear.x = -5 +(efeito_derivativo);

                	_pub.publish(_msg);
					ros::spinOnce();
					loop_rate.sleep();

				}else{
			
                	_msg.linear.x = -2;

                	_pub.publish(_msg);
					ros::spinOnce();
					loop_rate.sleep();

				
				}
				}


			if (is_obs){

			if(distx_R1 > tolerance_pos){
                
                    //angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
					//Calcula o setpoint da orientação.
				    angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
		  		    
                    //Calcula a diferença entre os ângulos.
		  		    erroorie = angulo - feedback.theta;

				    if (abs(erroorie) > tolerance_orie){

					    msg.angular.z = erroorie/(60/10);
                        pub.publish(msg);

				    
                    }


			}
			

			if (distx_R2 > tolerance_pos){

			
                
                    //angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
					//Calcula o setpoint da orientação.
				    _angulo = atan2(_feedback_.y-_feedback.y,_feedback_.x-_feedback.x);
		  		    
                    //Calcula a diferença entre os ângulos.
		  		    _erroorie = _angulo - _feedback.theta;

				    if (abs(_erroorie) > _tolerance_orie){

					    _msg.angular.z = _erroorie/(60/10);
                        _pub.publish(_msg);

				    
                    }


			
				}
			
			}else{

				if(distx_R1 > tolerance_pos){
                
                    //angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
					//Calcula o setpoint da orientação.
				    angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
		  		    
                    //Calcula a diferença entre os ângulos.
		  		    erroorie = angulo - feedback.theta;

				    if (abs(erroorie) > tolerance_orie){

					    msg.angular.z = erroorie/(60/10);
                        pub.publish(msg);

				    
                    }


				}
			

				if (distx_R2 > tolerance_pos){

			
                
                    //angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
					//Calcula o setpoint da orientação.
				    _angulo = atan2(_feedback_.y-_feedback.y,_feedback_.x-_feedback.x);
		  		    
                    //Calcula a diferença entre os ângulos.
		  		    _erroorie = _angulo - _feedback.theta;

				    if (abs(_erroorie) > _tolerance_orie){

					    _msg.angular.z = _erroorie/(60/10);
                        _pub.publish(_msg);

				    
                    }


			
				}
				

			}


			if (flag_y){
				if(disty_R1 > tolerance_min){

					msg.linear.x = -1;
                	
                	pub.publish(msg);
                	ros::spinOnce();
					loop_rate.sleep();

				}else{
			
                	msg.linear.x = 0;
                	
                	pub.publish(msg);
                	ros::spinOnce();
					loop_rate.sleep();

				}
			

			if (disty_R2 > tolerance_min){

				

					_msg.linear.x = -1;

                	_pub.publish(_msg);
					ros::spinOnce();
					loop_rate.sleep();

				}else{
			
                	_msg.linear.x = 0;

                	_pub.publish(_msg);
					ros::spinOnce();
					loop_rate.sleep();

				
				}
			}



			if (flag_x){


				if(distx_R1 > tolerance_min){
                
                    //angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
					//Calcula o setpoint da orientação.
				    angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
		  		    
                    //Calcula a diferença entre os ângulos.
		  		    erroorie = angulo - feedback.theta;

				    if (abs(erroorie) > tolerance_orie){

					    msg.angular.z = erroorie/(60/10);
                        pub.publish(msg);

				    
                    }


				}else{
						msg.angular.z = 0;
                        pub.publish(msg);
					}
			

			if (distx_R2 > tolerance_min){

			
                
                    //angulo = atan2(_feedback_.y-feedback.y,_feedback_.x-feedback.x);
					//Calcula o setpoint da orientação.
				    _angulo = atan2(_feedback_.y-_feedback.y,_feedback_.x-_feedback.x);
		  		    
                    //Calcula a diferença entre os ângulos.
		  		    _erroorie = _angulo - _feedback.theta;

				    if (abs(_erroorie) > _tolerance_orie){

					    _msg.angular.z = _erroorie/(60/10);
                        _pub.publish(_msg);

				    
                    }


			
				}else{
						_msg.angular.z = 0;
                        _pub.publish(_msg);
					}

				}





			
				pub.publish(msg);
				_pub.publish(_msg);
				ros::spinOnce();
				loop_rate.sleep();
			



		}
			//system("pause");
            return 0;
	}
