/*
 * global.cpp
 *
 *  Created on: 2014年9月14日
 *      Author: xuf
 */
#include "global.h"
//全局变量

//陀螺仪
 int fd_arm;
 int fd_arm1;
 unsigned char buf_angle[11];

 double time0=get_time();//get时间（用于状态跳变）
 //double DisMin1 = 18;   //追球时最小距离（修改）×××××××××××××××××××××××××××××
 //double DisMin = 19;  //调整球时最小距离（修改）×××××××××××××××××××××××××××××
double DisMin1 = 14;   //追球时最小距离（修改）×××××××××××××××××××××××××××××
double DisMin = 15;  //调整球时最小距离（修改）×××××××××××××××××××××××××××××

 //摄像头角度
 double head_angle[2] = {0, 0};

 double T_lower = 16;		//锯齿波周期
 double T_upper = 4;	//正弦波周期
 double t_lower_offset = 0;
 double t_upper_offset = 0;


 int int_low_kp=65;
 int int_low_ki=30;
 int int_low_kd=10;
 int int_up_kp=65;
 int int_up_ki=30;
 int int_up_kd=10;
 PID pid_lower = {0, 0, 0, 0.0000, 0.0000, 0.0000, 0.03, 5};
 PID pid_upper = {0, 0, 0, 0.0000, 0.0000, 0.0000, 0.04, 5};

char cWait;
int yz_ball[6] 	= {0, 200, 0, 200, 0, 200};

bool send_serial_flag = false;
int serial_flag = 1;
//图像
 int threshold_houghline=80 ;
 int param1=80 ;
 int param2 =10;
int headangle;
unsigned char H,S,V,R,G,B;
unsigned char HSV[64][64][64]={0},RGB[64][64][64]={0};
int white_points_low_2 = 80;
int white_points_up_2 = 300;
int green_points_low_2 = 5;
int green_points_up_2= 40;
int green_circle_low_2 = 40;
int green_circle_up_2= 80;
int max_count_best_2 = 130;
int white_points_low_1 = 20;
int white_points_up_1 = 400;
int green_points_low_1 = 5;
int green_points_up_1= 100;
int white_points_low_3 = 40;
int white_points_up_3 = 400;
int green_points_low_3 = 5;
int green_points_up_3= 105;
//int green_points_up_3= 105;
CvCapture* capture;
IplImage *show_Image,*Image,* pImage,*ball_Image,*ball_Image_gray,*field_Image,*field_Image_gray,*goal_canny_Image,*goal_color_dst,*goal_Image_gray,*goal_Image,*ball_Image_gray1;						//HSVImagegray和white_cvtImage分别是匹配完绿色和白色的灰度图
//whiteImage用来起匹配场地中的白色,全局变量直接在Color_Table()中赋值
//但是whiteImage要在大循环中至零,不然匹配的白点会累积
CvMat* edges;										//第一种方法将轮廓画入这张图,用于第二种方法处理
CvSeq *cont;
CvMat temp;
CvMat* mat;									//用于拷贝的矩阵,传入函数icvHoughCirclesGradient
//double t_begin = 0;
//double t_end = 0;
CvMemStorage *stor,*stor_vertical;
CvSeq* lines,*lines_vertical;




//线程
pthread_t image_t;
pthread_t head_t;
pthread_t body_t;
pthread_t location_t;
pthread_t serial_t;
pthread_t fsm_t;
pthread_t network_t;
pthread_t  game_master_t;

pthread_t mcu_t;


//策略
Robot_info robot_info;

//裁判盒
int sock;
struct sockaddr_in addr_to;
struct sockaddr_in addr_from;

char  get_ref_buf[1023];
//return data buffer
char return_data_buffer[8] = {'R', 'G', 'r', 't', 2, 0, 3, 2};
//massage


//网络
int client_sock;
//PID

//头部细节函数
//锯齿波函数
	double saw(double t, const double T)
	{
		if(t < T/4){
			return t * 4 / T;
		}
		else if(t < 3*T/4){
			return 2 - t * 4 / T;
		}
		else if(t < T){
			return 4 * t / T - 4;
		}
		else{
			return 0;
		}
	}

	//头部20号舵机运动函数
	double lower_head_fun(double t, const double T)
	{
		t += t_lower_offset;
		int t_int = t / T;
		double t_tmp = t - T * t_int;
		return 100.0 * saw(t_tmp, T);
	}

	//头部21号舵机运动函数.正弦波
	double upper_head_fun(double t, const double T)
	{
		t += t_upper_offset;
		int t_int = t/T;
		double t_tmp = t - T * t_int;
		double w = 2 * CV_PI / T;
		return 50.0 * sin(w * t_tmp)+ 50.0;
	}

	//计算开始找球时的时间偏置,有利于动作衔接
	void calc_t_offset(void)
	{
		t_lower_offset = (2 - head_angle[0] / 100.0) * T_lower / 4;
		double w = 2 * CV_PI / T_upper;
		t_upper_offset = acos((head_angle[1] - 50) / 51) / w + T_upper / 4;

		switch(robot_info.lost_ball_states)
		{
		case 1:
			t_lower_offset += 2*(3*T_lower/4 - t_lower_offset);
			break;
		case 2:
			t_lower_offset += 2*(3*T_lower/4 - t_lower_offset);
			t_upper_offset += 2*(3*T_upper/4 - t_upper_offset);
			break;
		case 3:
			break;
		case 4:
			t_upper_offset += 2*(3*T_upper/4 - t_upper_offset);
			break;
		default:
			break;
		}
	}

//状态机
void fsm_attack_body(void)
{
	if(game_state==3)
		{
	switch(robot_info.robot_state)
	{
	case search_ball:
		body_search();
		break;
	case chase_ball:
		body_chase();
		break;
	case round_yaw:
	    body_round_yaw();
		break;
	case search_goal:
		body_search_goal();
		break;
	case adjust_ball:
		body_adjust_ball();
		break;
	case round_goal:
		body_round_goal();
		break;
	case approach_ball:
	body_approach_ball();
	     break;
	case kick_ball:
		body_kick();
		break;
	case  climb_up:
		body_climb_up();
			break;
	default:
		break;
	}
		}
	else{
		body_stop();
		cout<<"game interrupt"<<endl;
	}
	usleep(1);
}

void fsm_attack_head(void)
{
	if(game_state==3)
	{
	switch(robot_info.robot_state)
		{
//	cout<<"   头部"<<robot_info.robot_state<<endl;
		case search_ball:
			head_search();
			break;
		case chase_ball:
			head_chase();
			break;
		case round_yaw:
			head_round_yaw();
			break;
		case search_goal:
			head_search_goal();
			break;
		case adjust_ball:
			head_adjust_ball();
				break;
		case round_goal:
			head_round_goal();
			break;
		case approach_ball:
			head_approach_ball();
			break;
		case kick_ball:
			head_kick();
			break;
		case  climb_up:
				break;
		default:
			break;
		}
	}
	else{
		move_head(0,90);
	}
	usleep(1);
}

void fsm_attack_image(void)
{
    //robot_info.robot_state=search_goal;
	switch(robot_info.robot_state)
		{
//	cout<<"   头部"<<robot_info.robot_state<<endl;
		case search_ball:
			Image_search();
			break;
		case chase_ball:
			Image_chase();
			break;
		case round_yaw:
			Image_round_yaw();
			break;
		case search_goal:
			Image_search_goal();
			break;
		case adjust_ball:
			Image_adjust_ball();
				break;
		case round_goal:
			Image_round_goal();
			break;
		case approach_ball:
			Image_approach_ball();
			break;
		case kick_ball:
			Image_kick();
			break;
		case  climb_up:
			break;
		default:
			break;
		}
}

////球轨迹的预测
//ball_trajectory ballTraject;
//void update_ball_traj_t()
//{
//	{
//		for(int i =0 ;i<2;i++)
//		{
//			ballTraject.ball_time[i] =ballTraject.ball_time[i+1];
//		}
//		ballTraject.ball_time[2] = get_time()-ballTraject.t0;
//	}
//}
//void update_ball_traj_p()
//{
//	{
//		for(int i =0 ;i<2;i++)
//		{
//			ballTraject.pic_x[i] =ballTraject.pic_x[i+1];
//			ballTraject.pic_y[i] =ballTraject.pic_y[i+1];
//		}
//		ballTraject.pic_x[2] =robot_info.ball_info.x;
//		ballTraject.pic_y[2] =robot_info.ball_info.y;
//	}
//}
//Ball_info ball_forecast(ball_trajectory *ball)
//{
//	Ball_info ballForecast;
//
//	double tInterval = get_time() - ball->t0 - ball->ball_time[2];
//
//	double  vx1 = (ball->pic_x[1] - ball->pic_x[0]) / (ball->ball_time[1] - ball->ball_time[0]);
//	double vx2 = (ball->pic_x[2] - ball->pic_x[1]) / (ball->ball_time[2] - ball->ball_time[1]);
//	double ax = (vx2-vx1) / (ball->ball_time[2] - ball->ball_time[0])/2;
//
//	double  vy1 = (ball->pic_y[1] - ball->pic_y[0]) / (ball->ball_time[1] - ball->ball_time[0]);
//	double vy2 = (ball->pic_y[2] - ball->pic_y[1]) / (ball->ball_time[2] - ball->ball_time[1]);
//	double ay = (vy2-vy1) / (ball->ball_time[2] - ball->ball_time[0])/2;
//
//	ballForecast.x = ball->pic_x[2] +vx2*tInterval +0.5 * ax* tInterval*tInterval;
//	ballForecast.y = ball->pic_y[2] +vy2*tInterval +0.5 * ay* tInterval*tInterval;
//	ballForecast.r=ball->pic_r[2];
////	cout<<"   预测   "<<tInterval<<"      "<<ballForecast.x <<endl;
//	return ballForecast;
//}
