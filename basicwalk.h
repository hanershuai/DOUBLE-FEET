/*
 * basicwalk.h
 *
 *  Created on: Mar 2, 2014
 *      Author: hcn
 */

#ifndef BASICWALK_H_
#define BASICWALK_H_
#include "global.h"
#include <iostream>
#include<cmath>
#include<stdlib.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#include<stdio.h>
#include<termios.h>
#include<errno.h>
#include<sys/time.h>
#include<signal.h>

#include<string.h>
#include <math.h>
#include"Point.h"
#include"Matrix.h"
#include"Vector.h"
#include"OPKinematics.h"
using namespace std;
using namespace Robot;
//
extern int serialFlag;
extern double tStart; //记录时间
extern double worldAngle;
extern Point2D stanceLimitX;
extern Point2D stanceLimitY;
extern Point2D stanceLimitA;
extern Point2D velLimitX;
extern Point2D velLimitY;
extern Point2D velLimitA;
extern Vector3D velDelta;
extern double handSwingAngle;
extern Point2D handSwingAngleLimit;

//Toe/heel overlap checking values
extern Point2D footSizeX;
extern double stanceLimitMarginY;
extern double stanceLimitY2;

//OP default stance width op 默认姿态宽度: 0.0375*2 = 0.075
//Heel overlap At radian 0.15 at each foot = 0.05*sin(0.15)*2=0.015
//Heel overlap At radian 0.30 at each foot = 0.05*sin(0.15)*2=0.030

extern Vector3D qLArm0;
extern Vector3D qRArm0;
extern Vector3D qLArmKick0;
extern Vector3D qRArmKick0;

//Hardness parameters
extern double hardnessSupport;
extern double hardnessSwing;
extern double hardnessArm;

//Head parameters
extern int Turn_qHead[2];
extern int qHead[2];

//Gyro stabilization parameters
extern double gyroFactor;
extern double ankleImuParamX[4];
extern double ankleImuParamY[4];
extern double kneeImuParamX[4];
extern double hipImuParamY[4];
extern double armImuParamX[4];
extern double armImuParamY[4];

//Sidekick parameters
extern Point2D sideKickVel1;
extern Point2D sideKickVel2;
extern Point2D sideKickVel3;
extern double sideKickSupportMod[2][2];
extern double tStepSideKick;

//Support bias parameters to reduce backlash-based instability
extern float supportFront;
extern float supportBack;
extern float supportSideX;
extern float supportSideY;

//----------------------------------------------------------
//-- Walk state variables
//----------------------------------------------------------

extern Vector3D velDiff;

//ZMP exponential coefficients:
extern double aXP, aXN, aYP, aYN;

//Gyro stabilization variables
extern double LankleShift[2];
extern double RankleShift[2];
extern double kneeShift;
extern double armShift[2];

extern bool active;
extern bool started;
extern double iStep;
extern double iStep0;

//double get_time();//若程序运行有问题仍需考虑double太大时用科学计数法表示精度减小的问题
//double t0 =get_time();
extern double tLastStep;
extern int stopRequest;
extern int canWalkKick; //Can we do walkkick with this walk code?
extern int walkKickRequest;
extern int walkKickType;

//重要的步行参量，需经常更改，放此以便
extern Vector3D velCurrent;
extern Vector3D velCommand;
extern double tStep0; //0.6;
extern double tStep; //0.6;
extern double tZmp; //0.165;    此处理论上为Zc/g/tStep......但实际上为理论值的二倍会很理想。另外此值对步态的整体效果有很大影响，可在理论值得二倍左右进行微调。
extern double stepHeight;
extern double ph1Single;
extern double ph2Single;
extern double ph1Zmp;
extern double ph2Zmp;

extern double bodyHeight; //0.300
extern double bodyTilt; //14
extern double footX; //-0.02;
extern double footY; //0.035;
extern double supportX;
extern double supportY;

//WalkKick parameters
extern Point2D walkKickVel;
extern double walkKickSupportMod[2][2];
extern double walkKickHeightFactor;

//Compensation parameters
extern double hipRollCompensation; //左右摇摆可改变此参数
extern double toeTipCompensation;
extern Point2D ankleMod;

extern Vector3D uTorso; //the initial supportX = 0;
extern Vector3D uLeft;
extern Vector3D uRight;

extern double pLLeg[6];
extern double pRLeg[6];
extern double pTorso[6];

extern double tStepWalkKick;  //tStepWalkKick will be modified later to ensure the kicking strength

/********************************************
 自己修订的全局变量（lua中默认为全局变量）
 ********************************************/

extern double phSingle;
extern double ph;
extern Vector3D uSupport;
extern double m1X, m2X, m1Y, m2Y;
extern Vector3D uLeft1;
extern Vector3D uRight1;
extern Vector3D uLeft2;
extern Vector3D uRight2;

//std::vector<double> qLegs(12);//弧度之
extern double yawAngle;
extern int supportLeg;
extern Vector3D uTorsoActual;

extern double gyro_roll;
extern double gyro_pitch;
extern double ankleShiftX;
extern double ankleShiftY;
extern double kneeShiftX;
extern double hipShiftY;
extern double armShiftX;
extern double armShiftY;

extern double hipShift[2];
extern double hipShift3;
extern Vector3D uTorso1;
extern Vector3D uTorso2;
extern Vector3D next_vel;
extern double supportMod[2];
extern double shiftFactor;

extern double T;

extern double xFoot;
extern double zFoot;


extern int initial_step;
//----------------------------------------------------------
//-- End initialization
//----------------------------------------------------------

extern int fd;
extern char device0[20];

extern int angle[12];
extern int ID[12];
extern int ID_hands[6];
extern int ID_head[2];
extern int angleinit[];

extern int angleOfHands[6];
extern int angleOfHead[2];
extern int angleOfHeadInit[2];

extern int angleOfHandsInit[6];

int portInit(char *device);
void serial();
void serial_legs();
void serial_hardness(double hardness, string item);
void serial_hands();
void serial_head();
void serial_all(void);
void serial_head0();
void motion_legs();
void motion_head0();
void motion_head();
void motionInit_head();
void motion_hands();
void set_leg_hardness();
double get_time();
double mod_angle(double a);
Vector3D se2_interpolate(double t, Vector3D u1, Vector3D u2);
Vector3D pose_global(Vector3D pRelative, Vector3D pose);
Vector3D pose_relative(Vector3D pGlobal, Vector3D pose);
Vector3D step_torso(Vector3D uLeft, Vector3D uRight, double shiftFactor);
Vector3D step_right_destination(Vector3D vel, Vector3D uLeft, Vector3D uRight);
Vector3D step_left_destination(Vector3D vel, Vector3D uLeft, Vector3D uRight);
double foot_phase_xf(double ph);
double foot_phase_xf_kick(double ph);
double foot_phase_zf(double ph);
double foot_phase_zf_kick(double ph);
double zmp_solve_aP(double zs, double z1, double z2, double x1, double x2);
double zmp_solve_aN(double zs, double z1, double z2, double x1, double x2);
Vector3D zmp_com(double ph);
double procFunc(double a, double deadband, double maxvalue);
void check_walkkick();
void update_velocity();
void update_still();
void StepReset();
void cacl_next_velCurrent();
void delay(int l);
int update();
int updateOneStep(Vector3D);
int updateOneStep0(Vector3D);
void Calc_qHead(Point2D ball);
void walkkick(Vector3D vel,int flag);
void move_head(double lower, double upper);
#endif /* BASICWALK_H_ */
