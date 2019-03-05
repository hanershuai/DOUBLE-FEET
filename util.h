/***********************************
 *Copyright (C) 2012 HFR(足球机器人团队)
 * 摘	要：与串口操作有关的函数
 * 作	者：赵洪波
 * 修	改：2012.04.30
 * *********************************/
#ifndef __UTIL_H__
#define __UTIL_H__

#include "global.h"

struct data
{
	int angle[21];
	int speed[21];
};

int portInit(const char *device);
int portInit_MCU(const char *device);
int fileOpen(const char* filename);
int fileConvert(const char* filename, struct data* duoji, int* waittime, int num);
void dataConvert(struct data duoji, byte b_data[103]);
void move_duoji(byte id, int position, int speed);
void sendfile(const char *fileName);
void delay_sec(double seconds);
void euler_timer(int time);
void euler_init(void);
void euler_check();
//void euler_check();
bool read_serial(int fd, void *buf, int length);
void find_angle(void);
byte get_fall_info(void);
double get_yaw(void);

void move_duoji(byte id, int position, int speed);
void torque(byte status);
void return_status(byte status);
void set_max_torque(byte id, int max_torque);
void led(byte id, byte action);
void set_power_rang(byte lower, byte upper);
void set_margin_and_slope(byte margin, byte slope);

#endif
