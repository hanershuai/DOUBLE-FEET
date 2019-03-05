/*
 * pid_debug.cpp
 *
 *  Created on: 2014年10月4日
 *      Author: xuf
 */

#include "global.h"


void save_pid(void)
{
	char str[50];
	int file = open("pid.txt", O_RDWR);
	strcpy(str, "pid_lower:\n");
	write(file, str, strlen(str));
	sprintf(str, "kp = %8.5lf\n", pid_lower.kp);
	write(file, str, strlen(str));
	sprintf(str, "ki = %8.5lf\n", pid_lower.ki);
	write(file, str, strlen(str));
	sprintf(str, "kd = %8.5lf\n", pid_lower.kd);
	write(file, str, strlen(str));
	sprintf(str, "t = %8.5lf\n", pid_lower.t);
	write(file, str, strlen(str));
	sprintf(str, "dz = %d\n", pid_lower.dz);
	write(file, str, strlen(str));

	strcpy(str, "\npid_upper:\n");
	write(file, str, strlen(str));
	sprintf(str, "kp = %8.5lf\n", pid_upper.kp);
	write(file, str, strlen(str));
	sprintf(str, "ki = %8.5lf\n", pid_upper.ki);
	write(file, str, strlen(str));
	sprintf(str, "kd = %8.5lf\n", pid_upper.kd);
	write(file, str, strlen(str));
	sprintf(str, "t = %8.5lf\n", pid_upper.t);
	write(file, str, strlen(str));
	sprintf(str, "dz = %d\n", pid_upper.dz);
	write(file, str, strlen(str));

	close(file);

//	cout<<"pid_lower:"<<endl;
//	cout<<"kp="<<pid_lower.kp<<"  ki="<<pid_lower.ki<<"  kd="<<pid_lower.kd<<"  t="<<pid_lower.t<<"  dz="<<pid_lower.dz<<endl;
//	cout<<"pid_upper:"<<endl;
//	cout<<"kp="<<pid_upper.kp<<"  ki="<<pid_upper.ki<<"  kd="<<pid_upper.kd<<"  t="<<pid_upper.t<<"  dz="<<pid_upper.dz<<endl;
//	cout<<endl;
}

void read_pid(void)
{
	FILE *fd;
	fd = fopen("pid.txt", "r");
	fscanf(fd, "pid_lower:\n");
	fscanf(fd, "kp = %lf\n", &pid_lower.kp);
	fscanf(fd, "ki = %lf\n", &pid_lower.ki);
	fscanf(fd, "kd = %lf\n", &pid_lower.kd);
	fscanf(fd, "t = %lf\n", &pid_lower.t);
	fscanf(fd, "dz = %d\n", &pid_lower.dz);

	fscanf(fd, "pid_upper:\n");
	fscanf(fd, "kp = %lf\n", &pid_upper.kp);
	fscanf(fd, "ki = %lf\n", &pid_upper.ki);
	fscanf(fd, "kd = %lf\n", &pid_upper.kd);
	fscanf(fd, "t = %lf\n", &pid_upper.t);
	fscanf(fd, "dz = %d\n", &pid_upper.dz);

	fclose(fd);

	cout<<"pid_lower:"<<endl;
	cout<<"kp="<<pid_lower.kp<<"  ki="<<pid_lower.ki<<"  kd="<<pid_lower.kd<<"  t="<<pid_lower.t<<"  dz="<<pid_lower.dz<<endl;
	cout<<"pid_upper:"<<endl;
	cout<<"kp="<<pid_upper.kp<<"  ki="<<pid_upper.ki<<"  kd="<<pid_upper.kd<<"  t="<<pid_upper.t<<"  dz="<<pid_upper.dz<<endl;
	cout<<endl;
}

double pid_calc_lower(PID *pid)
{
	const int err_yz = pid->dz;	//误差阈值，误差小于该值不用调节
	const double out_limit = 15.0;
	pid->ep = ( robot_info.ball_info.x-160);					//因为时根据原图像调节的pid参数,现在图像缩小了一倍,所以将误差乘以2
	if(abs(pid->ep) < err_yz)
		return 0;
	pid->kp = (double)int_low_kp/1000;
	pid->ki = (double)(int_low_ki-100)/1000;
	pid->kd = (double)int_low_kd/1000;

	double u0 = pid->kp * pid->ep + pid->ki * pid->ei + pid->kd * pid->ed;
	pid->ed = pid->ei;
	pid->ei = pid->ep;
	u0 = min(max(-out_limit, u0), out_limit);
	return u0;
}

double pid_calc_upper(PID *pid)
{
	const int err_yz = pid->dz;	//误差阈值，误差小于该值不用调节
	const double out_limit = 5.0;
	pid->ep = (robot_info.ball_info.y - 120);
	if(abs(pid->ep) < err_yz)
		return 0;
	pid->kp = (double)int_up_kp/1000;
	pid->ki = (double)(int_up_ki-100)/1000;
	pid->kd = (double)int_up_kd/1000;

	double u0 = pid->kp * pid->ep + pid->ki * pid->ei + pid->kd * pid->ed;
	pid->ed = pid->ei;
	pid->ei = pid->ep;
	u0 = min(max(-out_limit, u0), out_limit);
	return u0;
}

void pid_reset(void)
{
	pid_lower.ep = 0;
	pid_lower.ei = 0;
	pid_lower.ed = 0;

	pid_upper.ep = 0;
	pid_upper.ei = 0;
	pid_upper.ed = 0;
}

