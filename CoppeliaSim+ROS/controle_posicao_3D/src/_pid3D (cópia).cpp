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
  

nav_msgs::Odometry _feedback;




//nav_msgs::Odometry feedback;

double _yaw2;

//int threshold_occupied_;
//int threshold_free_;


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
int i,_j,_k;
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


//****************************************************************//
//			          TRATAMENTO DO FEEDBACK - v2                 //
//****************************************************************//

//nav_msgs::Odometry _feedback;
//turtlesim::Pose _feedback;
//nav_msgs::Odometry _odom;

//Callback da Odometria.
void _subCallback(const nav_msgs::Odometry::ConstPtr& _msg)
{

	_feedback.pose.pose.position.x = _msg->pose.pose.position.x;
	_feedback.pose.pose.position.y = _msg->pose.pose.position.y;
	_feedback.pose.pose.orientation.w = _msg->pose.pose.orientation.w;
  	_feedback.pose.pose.orientation.z = _msg->pose.pose.orientation.z;
  
	
  	_feedback.twist.twist.linear.x = _msg->twist.twist.linear.x;
  	_feedback.twist.twist.angular.z = _msg->twist.twist.angular.z;

   	tf::Quaternion q(
        _msg->pose.pose.orientation.x,
        _msg->pose.pose.orientation.y,
        _msg->pose.pose.orientation.z,
        _msg->pose.pose.orientation.w);
  	tf::Matrix3x3 _m(q);
  	double _roll, _pitch, _yaw;
  	_m.getRPY(_roll, _pitch, _yaw);
  	_yaw2 = _yaw;
  
}

//Callback do LIDAR.
void _subCallback_lidar(const sensor_msgs::LaserScan::ConstPtr& _lidar)
{	
	_lidar_size_received = _lidar->ranges.size();
	_lidar_size = floor(_lidar_size_received*(1-_lidar_ignorar_percentage));

	_size_1 = floor((_lidar_size_received - _lidar_size)/2);
	_size_2 = _lidar_size_received - 2*_size_1;

	_dist_obs = 1.5;
	for (i=0;i<_size_2;i++){
		if(_lidar->ranges[i+_size_1] < _dist_obs && _lidar->ranges[i+_size_1] > 0.01){
			_dist_obs = _lidar->ranges[i+_size_1];
			_indice = i;
		}
	}
	
	_range_desvio = _erropos < _range_desvio_close ? _range_desvio_close : _range_desvio_far;
	_range_desvio = _erropos < _range_desvio_very_close ? _range_desvio_very_close : _range_desvio;

	//Verifica em qual percentual do range aceito do Lidar ele está.
	_ang_obs = _indice/_size_2;

	//Se está nos 5% da região frontal, o range de desvio é maior.
	if (_ang_obs > (0.5-0.025) && _ang_obs < (0.5+0.025))
	{
		_range_desvio = _range_desvio*1.2;
	}

	//Verifica os objetos próximos e, se dentro do range escolhido, aciona a flag de desvio.
	_is_obs = _dist_obs < _range_desvio ? true : false;

	//cout << dist_obs << "\n";
	//cout << is_obs << "\n";
	//cout << ang_obs << "\n\n";
}



int main(int argc, char **argv){
  ros::init(argc, argv, "pid_coppelia");
 
  ros::NodeHandle _n;
  ros::Publisher _pub = _n.advertise<geometry_msgs::Twist>("v2_cmd_vel", 1000);
  ros::Subscriber _sub = _n.subscribe("v2_odom", 1000, _subCallback);
  ros::Subscriber _sub_lidar = _n.subscribe("v2_scan", 1000, _subCallback_lidar);


  ros::Time   _last_time, _actual_time;
  
  ros::Rate loop_rate(100);
 
  //system("rosservice call reset");
  //system("rqt_plot /turtle1/pose/x:y &");
  
  double _erroSumL, _lastErrL;
  double _erroSumA, _lastErrA;
  
  double _linearX = 0;
  double _kpL = 1;
  double _kiL = 0;
  double _kdL = 0.5;
  
  double _angularZ = 0;
  double _kpA = 4;
  double _kiA = 0;
  double _kdA = 0.4;
  double _PI = 3.141592654;
  


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

	_posdesejada[0] = 1.8;		
	_posdesejada[1] = 1.5;
    
	//float tolerance = 5;
	double _errorLinear = 99;
    double _errorAngular = 0;
    
    //cout << "Digite a posicao\nX>>";
    //cin >> posX;

    //cout << "Digite a posicao\nY>>";
    //cin >> posY;


		while ((abs(_erroorie) > _tolerance_orie)) {
			
			
			//Verifica se há objetos próximos em v2.
			//Se sim, aciona o controle de desvio.
			//Se não, aciona o controle de posição.
			if(_is_obs){

						
				_msg.angular.z = fc_desvio_angular.make_inference(_ang_obs, _dist_obs);
				_msg.linear.x = fc_desvio_linear.make_inference(_dist_obs)*_max_lin_obs;

				_erropos =  sqrt(pow(_posdesejada[0]-_x,2)+pow(_posdesejada[1]-_y,2));

				_pub.publish(_msg);
				ros::spinOnce();
				loop_rate.sleep();

				system("clear");
				printf("Controle Ativo: DESVIO DE OBSTÁCULO.\n");
				printf("Distância total percorrida R2: %.2fm\n", _travelled);
				printf("Erro de orientação R2: -\n");
				printf("Erro de posição R2: %.2f m\n",_erropos);
				printf("Velocidade angular enviada R2: %.2f rad/s\n",_msg.angular.z);
				printf("Velocidade linear enviada R2: %.2f m/s\n\n",_msg.linear.x);

			}else{
			
				double _yaw = _PI * _feedback.pose.pose.orientation.z;
      
        		_yaw = -(_yaw*0.2)+_yaw;

        		_yaw = _yaw2;

        		_actual_time = ros::Time::now();
        		_last_time = _actual_time;
        		double _timeChange = (_actual_time = _last_time).toSec();

        		//linear PID   
        		_errorLinear = sqrt(pow((_posdesejada[0]-_feedback.pose.pose.position.x),2)+pow((_posdesejada[1]-_feedback.pose.pose.position.y),2));
        		_erroSumL += (_errorLinear * _timeChange);
        		double _dErrL =  (_errorLinear - _lastErrL) / _timeChange;
        		_linearX = _kpL * _errorLinear + _kiL * _erroSumL + _kdL * _dErrL;

        		//linear P
        		//errorLinear = sqrt(pow((posX-feedback.pose.pose.position.x),2)+pow((posY-feedback.pose.pose.position.y),2));
        		//msg.linear.x = fc_linear.make_inference(erropos, erroorie)*max_lin_free;
        		//linearX = msg.linear.x * errorLinear;

        		//angular PID
        		_errorAngular = atan2(_posdesejada[1]-_feedback.pose.pose.position.y, _posdesejada[0]-_feedback.pose.pose.position.x);
        
        		_erroSumA += (_errorAngular * _timeChange);
        		double _dErrA =  (_errorAngular - _lastErrA) / _timeChange;
        		//double err =  (errorAngular - yaw);
        		double _err =  (_errorAngular)-(_yaw);
        
        		//verifica se o erro esteja dentro do intevalo de -PI à PI
        		if (_err > _PI) {
            		_err = _err - (2 * _PI);
        		} 
        		if (_err < -_PI) {
            		_err = _err + (2 * _PI);
        		}    


        		_angularZ = (_kpA * _err) + (_kiA * _erroSumA) + (_kdA * _dErrA);
        
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


        		_msg.linear.x = _linearX;
        		_msg.angular.z = _angularZ;
        
        		//ROS_INFO("X = %f  erroL = %f    Y = %f erroA = %f ",linearX,errorLinear,angularZ, err);
        		ROS_INFO("X2=%f  Y2=%f ",_feedback.pose.pose.position.x,
                               _feedback.pose.pose.position.y);
        
        		ROS_INFO("LIN_X2 = %f     ANG_Z2 = %f", _linearX, _angularZ);
        		ROS_INFO("----------------------");
        
        		// erro = posX-feedback.x;
        		// msg.linear.x = /Kpos*erro/(60/10);
        		// ROS_INFO("TEMPO: %f   X>>%f,Erro>>%f",dt,feedback.x,erro);
        		_pub.publish(_msg);
        		ros::spinOnce();
        		loop_rate.sleep();
        
				//Controle de posição
				_errorLinear = sqrt(pow((_posdesejada[0]-_feedback.pose.pose.position.x),2)+pow((_posdesejada[1]-_feedback.pose.pose.position.y),2));
				if (_errorLinear > _tolerance_pos){
					_pos_ok = false;
					_msg.linear.x = fc_linear.make_inference(_erropos, _erroorie)*_max_lin_free;
				}else{
					_pos_ok = true;
					
          ROS_WARN("...Posicao R2 alcancada...");
					_msg.linear.x = 0;
					_msg.angular.z = 0;
					_pub.publish(_msg);
					ros::spinOnce();
					loop_rate.sleep();
				}

			}
		}
  		

  	}

  return 0;
}