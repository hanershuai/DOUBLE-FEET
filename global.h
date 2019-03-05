/*
 * global.h
 *
 *  Created on: 2014年9月14日
 *      Author: xuf
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_
#define byte unsigned char

//#define GONG_KONG

typedef struct{
	double r;
	double  x;
	double y;
}Ball_info;
typedef struct{
	int state;			//0:没有找到,1:找到一根门柱,3:找到两根门柱
	double left_goal;
	double right_goal;
}Goal_info;
typedef struct{
	int ep;
	int ei;
	int ed;
	double kp;
	double ki;
	double kd;
	double t;
	int dz;
}PID;
enum Robot_state{						//机器人状态
	search_ball = 0,
	chase_ball,
	round_yaw,
	search_goal,
	adjust_ball,
	round_goal,
	approach_ball,
	kick_ball,
	climb_up,

};
typedef struct{
	int lower_angle;
	int upper_angle;
	int speed[2];
}Head_ctrl;

//head file
//c++
#include <iostream>
#include <cmath>
#include <string>
#include <iomanip>
#include <fstream>
#include <vector>
using namespace std;

//c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//unix
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <signal.h>
//线程
#include <pthread.h>
//network
#include <sys/socket.h>
#include <netinet/in.h>
#include"tcp.h"
#include <arpa/inet.h>
//信号量
#include <semaphore.h>

//opencv
#include <opencv2/opencv.hpp>
#include "opencv/highgui.h"
#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv/cxmisc.h"




//裁判盒
#include "ref_box.h"

//自定义
#include "pid_debug.h"
//摄像头角度
extern double head_angle[2];
extern PID pid_lower;
extern PID pid_upper;

extern int int_low_kp;
extern int int_low_ki;
extern int int_low_kd;
extern int int_up_kp;
extern int int_up_ki;
extern int int_up_kd;

extern double T_lower;
extern double T_upper;
extern double t_lower_offset;
extern double t_upper_offset;
extern double DisMin1;
extern double DisMin;
extern double time0;
//在线步态
#include "Point.h"
using namespace Robot;
typedef struct{
	int condition;
	CvPoint left_down;
	CvPoint right_down;
	double leftYaw;//左门住对应航向角
	double rightYaw;//右门住对应航向角
}Kuang_info;
#include "Matrix.h"
#include "Vector.h"
#include "OPKinematics.h"
#include "basicwalk.h"
#include "Transform.h"

#include "util.h"
#include "image.h"
#include "pthread.h"
#include "head_actions.h"
#include "body_actions.h"
class Robot_info{
public:
	//机器人自身信息
	int q0Real;//每次图像获取图片时更新此数据
	int q1Real;
	Robot_state robot_state;//状态机
	Robot_state robot_state_last;//状态机   0找球 1追球 2接近球 3绕球 4射门   //进入他的while时置为对应值 否则为0
	Point3D robot_world_pos;	//机器人坐标
	byte fall_info ;			//0:正常  1：向前倒  2：向后倒 	0xaa：读取失败
	double yaw;			//-1读取失败
	double yaw_start;//初始航向角
	bool  fsm_unlocked;//状态机跳转锁。true：可以跳转，false：加锁，不能跳转


	//门框
	double shootYaw;//射门角度，默认为0度
	bool goal_find;
	Kuang_info kuang_info;		//框在视野中的信息

	//球
	Point3D ball_pos4robot;     //球相对于机器人的坐标
	Ball_info ball_info;		//球在视野中的信息
	Goal_info goal_info ;		//0未找到球门　　1找到一根门柱　2找到两根门柱
	//球门在视野中的信息
	bool ball_find;				//是否丢球
	int  lost_ball_states;		//丢球状态，0: 视野中被挡住 1:左上， 2:左下， 3:右上， 4:右下
	double ball_dis4robot;		//球到机器人的距离
	bool start_take_pos_flag;	//开始采点标志

	Robot_info(){
		 q0Real = 0;
		 q1Real = -30;
		robot_state = search_ball;
		robot_state_last = search_ball;
		robot_world_pos = Point3D(0.00001,0.0000001,0);
		ball_pos4robot=Point3D(1,0,0);
		ball_info.r=20;
		ball_info.x=320;
		ball_info.y=240;
		//Todo 门框信息初始化
		shootYaw = 0;
		goal_find=false;
		ball_find = false;
		lost_ball_states = 0;
		ball_dis4robot = 50;
		fall_info=0;
		yaw = 0;
		yaw_start=0;
		fsm_unlocked=false;
		start_take_pos_flag = false;
	}
	Robot_info(Robot_info &robotLast)
	{
		q0Real =robotLast.q0Real;
		 q1Real =robotLast.q1Real;
		robot_state = robotLast.robot_state;
		robot_world_pos = robotLast.robot_world_pos;
		ball_pos4robot=robotLast.ball_pos4robot;

		ball_info.r =robotLast.ball_info.r;
		ball_info.x =robotLast.ball_info.x;
		ball_info.y=robotLast.ball_info.y;

		shootYaw = robotLast.shootYaw;
		goal_find= robotLast.goal_find;
		ball_find = robotLast.ball_find;
		lost_ball_states =robotLast.lost_ball_states;
		ball_dis4robot =robotLast.ball_dis4robot;
		fall_info=robotLast.fall_info;
		yaw =robotLast.yaw;
		yaw_start=robotLast.yaw_start;
		fsm_unlocked= robotLast.	fsm_unlocked;
		start_take_pos_flag = robotLast.start_take_pos_flag ;
	}
void calc_ball_info()
	{
	double r =	ball_info.r;
//	ball_dis4robot  = fabs(-0.00349*r*r*r +0.417*r*r - 18.22*r +320.5);
//if(r<=55)
ball_dis4robot = fabs(0.000604*r*r*r+0.01113*r*r-6.736*r+252);
if(r>=62&&r<=100) { cout<<"R ENOUGH TO KICK"<<endl;ball_dis4robot=10;}
//else
//ball_dis4robot = 14;
    //3
	//ball_dis4robot = fabs(-0.0000671*r*r*r*r*r+0.0111*r*r*r*r-0.7137*r*r*r+22.27*r*r-341.9*r+2198);

	}
void update_robot_info()
	{
		void calc_ball_info();
	}
};

//全局变量声明
extern CvPoint ball_pos;	//球的坐标
extern char cWait;
extern int yz_ball[6];
extern int yz_kuang[6];
extern int yz_court[6];
extern int yz_line[6];
extern int yz_robot[6];
extern int serial_flag;
extern bool send_serial_flag ;
//图像
extern int headangle;
extern unsigned char H,S,V,R,G,B;
extern unsigned char HSV[64][64][64],RGB[64][64][64];
extern int threshold_houghline ;
extern int param1 ;
extern int param2 ;
extern int white_points_low_2 ;
extern int white_points_up_2 ;
extern int green_points_low_2 ;
extern int green_points_up_2;
extern int green_circle_low_2;
extern int green_circle_up_2;
extern int max_count_best_2 ;
extern int white_points_low_1;
extern int white_points_up_1;
extern int green_points_low_1;
extern int green_points_up_1;
extern int white_points_low_3;
extern int white_points_up_3;
extern int green_points_low_3;
extern int green_points_up_3;
extern CvCapture* capture;
extern IplImage *show_Image,*Image,* pImage,*ball_Image,*ball_Image_gray,*field_Image,*field_Image_gray,*goal_canny_Image,*goal_color_dst,*goal_Image_gray,*goal_Image,*ball_Image_gray1;
extern CvSeq *cont;
extern CvMat* edges;										//第一种方法将轮廓画入这张图,用于第二种方法处理
extern CvMat temp;
extern CvMat* mat;									//用于拷贝的矩阵,传入函数icvHoughCirclesGradient
extern Ball_info ball_info;
extern CvSeq* lines,*lines_vertical;
extern CvMemStorage *stor,*stor_vertical;
//extern double t_begin;
//extern double t_end;

//线程
extern pthread_t mcu_t;

extern pthread_t image_t;
extern pthread_t head_t;
extern pthread_t body_t;
extern pthread_t location_t;
extern pthread_t serial_t;
extern pthread_t fsm_t;
extern pthread_t network_t;
extern  pthread_t  game_master_t;


//策略
extern Robot_info robot_info;
extern Robot_info robot_info_last1;
extern Robot_info robot_info_last2;


//陀螺仪
extern int fd_arm;
extern int fd_arm1;
extern unsigned char buf_angle[11];


//裁判盒
typedef struct{
	char penalty;
	char secs_to_unpen;
}PlayerInfo;



//teamInfo
typedef struct{
	char number;
	char color;
	char score;
	char penalty_shot;
	short int single_shots;
	char SPL_coach_message[40];
	PlayerInfo players_info[5];
}TeamInfo;



extern int sock;
extern struct sockaddr_in addr_to;
extern struct sockaddr_in addr_from;
extern char return_data_buffer[8];

extern char  get_ref_buf[1023];



//massage
extern char num_players;
extern char game_state;
extern char frist_half;
extern char kickoff_team;
extern char sec_game_state;
extern char dropin_team;
extern short int dropin_time;
extern short int secs_remaining;
extern short int secondary_time;
extern TeamInfo team1;
extern TeamInfo team2;

//网络
extern int client_sock;
void fsm_attack_body(void);
void fsm_attack_head(void);
void fsm_attack_image(void);
//PID
double saw(double t, const double T);
//头部20号舵机运动函数
double lower_head_fun(double t, const double T);
//头部21号舵机运动函数.正弦波
double upper_head_fun(double t, const double T);
void calc_t_offset(void);
//球轨迹的预测
struct ball_trajectory
{
	double t0;
	int pic_x[3];
	int pic_y[3];
	int pic_r[3];
	int q0[3];
	int q1[3];
	double ball_time[3];

	int robot_x[3];
	int robot_y[3];
};
extern ball_trajectory ballTraject;
Ball_info  ball_forecast(ball_trajectory* ball);
void update_ball_traj_t();
void update_ball_traj_p();
#endif /* GLOBAL_H_ */
