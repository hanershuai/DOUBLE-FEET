/***********************************
 *Copyright (C) 2012 HFR(足球机器人团队)
 * 摘	要：与串口操作有关的函数
 * 作	者：赵洪波
 * 修	改：2018.01.30
 * *********************************/
#include "global.h"
//int k=1,fall_angle_start=0;
int mcu_x_middle_1=-91;//(forward=+;backward=-);
int mcu_x_middle_2=-87;
int mcu_x_middle_3=-94;
int mcu_x_middle_4=-94;
int mcu_x_middle_5=-102;//paud_rate=115200;
int mcu_x_middle_6=-95;

int mcu_x_middle=mcu_x_middle_3;

//int average_yaw_start=0,k=1;
//串口初始化
int portInit(const char *device)
{
	int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY); //| O_NOCTTY | O_NDELAY
	if (fd == -1)
	{
		perror("Can't Open Serial Port");
		return -1;
	}
	struct termios options;
	if (tcgetattr(fd, &options) != 0)
		perror("SetupSerial 1");
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;
	if (tcsetattr(fd, TCSANOW, &options) != 0)
		perror("SetupSerial 3");
	return fd;
}
int portInit_MCU(const char *device)
{
	int fd2 ;//= open(device, O_RDWR | O_NOCTTY | O_NDELAY); //| O_NOCTTY | O_NDELAY
	fd2=open(device,O_RDWR | O_NOCTTY |O_NONBLOCK);
	if (fd2 == -1)
	{
		perror("Can't Open Serial Port");
		return -1;
	}
	struct termios options;
	if (tcgetattr(fd2, &options) == 0)
		//perror("SetupSerial 1");
	//options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;
	if (tcsetattr(fd2, TCSANOW, &options) == 0)
		//perror("SetupSerial 3");
	return fd2;
}

//打开文件，返回帧数
int fileOpen(const char *filename)
{
	int num = 0, pfile;
	char tmp = 0;
	pfile = open(filename, O_RDONLY);
	if (pfile == -1)
	{
		perror("Can't Open File");
		return -1;
	}
	while (read(pfile, &tmp, 1))
		if (tmp == '\n')
			num++;
	num++;
	num = num / 3;
	close(pfile);
	return num;
}

//加载文件
int fileConvert(const char *filename, struct data* duoji, int* waittime, int num)
{
	int fp, tmp1 = 0, waitN = 0, dataN = 0;  //tmp缓存 tmp1行数
	int SPACE = 0, tmp4 = 0;  //tmp4舵机字符缓存
	char tmp = 0;
	char tmp2[10], tmp3[10]; //tmp2是waittime的缓存 tmp3是data的缓存
	fp = open(filename, O_RDONLY);
	while (read(fp, &tmp, 1))
	{
		if (tmp == '\n')
			tmp1++;
		if (tmp1 == 0)
			continue;
		else
			switch((tmp1 - 1) % 3)
			{
			case 0:
				if (tmp == 32)
					SPACE++;
				if (SPACE == 2)
				{
					tmp2[waitN] = tmp;
					//cout<<tmp<<"\n";
					waitN++;
				}
				break;
			case 1:
				if (tmp == '\n')
				{
//					cout<<waitN<<"\n";
					tmp2[waitN] = '\0';
					SPACE = 0;
					waitN = 0;
					waittime[(tmp1 - 1) / 3] = atoi(tmp2);
				}
				if (tmp != ',' && tmp != '\n' && tmp != '\\')
				{
					tmp3[tmp4] = tmp;
					tmp4++;
				}
				else if (tmp == ',')
				{
					dataN++;
				if (dataN % 2 == 1)
					{
						tmp3[tmp4] = '\0';
						duoji[(tmp1 - 1) / 3].angle[dataN / 2] = atoi(tmp3);
						tmp4 = 0;
					}
					else
					{
						tmp3[tmp4] = '\0';
						duoji[(tmp1 - 1) / 3].speed[dataN / 2 - 1] = atoi(tmp3);
						tmp4 = 0;
					}
					if (dataN / 2 == 21)
						dataN = 0;
				}
				break;
			case 2:
				if (tmp != ',' && tmp != '\n' && tmp != '\\')
				{
					tmp3[tmp4] = tmp;
					tmp4++;
				}
				else if (tmp == ',')
				{
					dataN++;
					if (dataN % 2 == 1)
					{
						tmp3[tmp4] = '\0';
						duoji[(tmp1 - 1) / 3].angle[dataN / 2] = atoi(tmp3);
						tmp4 = 0;
					}
					else
					{
						tmp3[tmp4] = '\0';
						duoji[(tmp1 - 1) / 3].speed[dataN / 2 - 1] = atoi(tmp3);
						tmp4 = 0;
					}
					if (dataN / 2 == 21)
						dataN = 0;
				}
				break;
			default:
				break;
			}
	}
	close(fp);
	return (0);
}



//装载数据包
//void dataConvert(struct data duoji, byte c_data[103])


void dataConvert(struct data duoji, byte b_data[103])
{


	b_data[0] = 0xff;
	b_data[1] = 0xff;
	b_data[2] = 0xfe;
	b_data[3] = 0x6d; 	//Length=109
	b_data[4] = 0x83; 	//SYNC WRITE
	b_data[5] = 0x1e; 	//Goal Position(L) Adress
	b_data[6] = 0x04; 	//The length of the data to be written

	for (int i = 0; i < 21; i++)
	{
			b_data[i * 5 + 7] = (byte) ((i + 1) & 0x00ff); 					//ID
			b_data[i * 5 + 8] = (byte) (duoji.angle[i] & 0x00ff); 			//Goal Position(L)
			b_data[i * 5 + 9] = (byte) ((duoji.angle[i] >> 8) & 0x00ff); 	//Goal Position(H)
			b_data[i * 5 + 10] = (byte) (duoji.speed[i] & 0x00ff); 			//Moving Speed(L)
			b_data[i * 5 + 11] = (byte) ((duoji.speed[i] >> 8) & 0x00ff); 	//Moving Speed(H)

			cout<<"ID   "<<i+1<<"    "<<duoji.angle[i]<<  "      "<<duoji.speed[i] <<endl;

	}
	byte crc = 0;
	for (int i = 2; i < 112; i++)
	{
		crc += b_data[i];
	}
	crc = (byte) (~crc);

	//???怎么是102

//	b_data[102] = (crc);
	b_data[112] = (crc);


//直接在这里发送
	//怎么没返回

}

//发送关键帧
void sendfile(const char *fileName)
{
	struct data *duoji = NULL;
	int *waittime = NULL, num;
	byte b_data[113] ;
	for (int t = 0 ;t<113;t++)
	{
		b_data[t] = 0 ;
	}

	num = fileOpen(fileName);
	cout<<"num = "<<num<<endl;
	duoji = (struct data *) calloc(num, sizeof(struct data));
	waittime = (int *) calloc(num, sizeof(int));
	fileConvert(fileName, duoji, waittime, num);


	for (int i = 0; i < num; i++)
	{
		cout<<"  行数  "<<i<<"\n";
		dataConvert(duoji[i], b_data);

		for(int j=0; j<2; j++){
			cout<<"  行数  ，列数 "<<i <<"    "<<j<<"\n";
		//	write(fd, "012345678", 9*sizeof(byte));
			write(fd, b_data, 113*sizeof(byte));

			cout<<" 一遍发送完"<<"\n";
		}
//		usleep(waittime[i] * 1000);


		cout<<"DEALYTIME"<<waittime[i]<<"\n";
		delay_sec((double)waittime[i] / 100.0);

		//delay_sec((double)waittime[i] / 100.0);
	}
	free(duoji);
	free(waittime);
}

//阻塞线程seconds秒
void delay_sec(double seconds)
{
	double time0=get_time();
	while(get_time()-time0<seconds)usleep(1);
}

//定时检测机器人欧拉角，每隔time毫秒
void euler_timer(int time)
{
	struct itimerval value;
	value.it_value.tv_sec = time / 1000;
	value.it_value.tv_usec = (time % 1000 * 1000);
	value.it_interval.tv_sec = 0;
	value.it_interval.tv_usec = 0;
	setitimer(ITIMER_REAL, &value, NULL);
}
//欧拉角初始化
void euler_init(void)
{
#ifdef GONG_KONG
	fd_arm = portInit("/dev/ttyUSB0");//工控
#else
	fd_arm= portInit("/dev/ttyUSB1");//电子罗盘
#endif
	robot_info.yaw_start = get_yaw();
	//cout<<"aaaaaa"<<endl;
	//signal(SIGALRM, &euler_check);
	//cout<<"aaa"<<endl;
	euler_timer(150);
}

/*
void fall_check()
{
	robot_info.fall_info=get_fall_info();
	//cout<<"fall="<<robot_info.fall_info<<endl;
	euler_timer(150);
	return ;
}*/
//获取欧拉角信息，包括航向角和摔倒信息
void euler_check()
{
//	find_angle();
	//robot_info.yaw=get_yaw()-robot_info.yaw_start;
			//if(robot_info.yaw>180)
				//robot_info.yaw=robot_info.yaw-360;
			//if(robot_info.yaw<-180)
				//robot_info.yaw=robot_info.yaw+360;
	    	robot_info.fall_info=get_fall_info();
		//	cout<<"fall="<<(int)robot_info.fall_info<<endl;
		//	cout<<"yaw = "<<robot_info.yaw<<endl;
	euler_timer(150);
}

//读串口， 成功返回true,失败返回false
bool read_serial(int fd, void *buf, int length)
{
	void *buf_tmp = buf;
	int bytes_remain = length;
	int bytes_send = 0;
	int count = 0;
	fcntl(fd_arm,F_SETFL,FNDELAY);
	do
	{
		bytes_send = read(fd_arm, buf_tmp, bytes_remain);
		if(bytes_send > 0){
			count = 0;
			buf_tmp += bytes_send;
			bytes_remain -=bytes_send;
		}
		else
		{
			count++;
			if(count > 20000){
//				cout<<"error int read serial: timeout!"<<endl;
				return false;
			}
		}
	}while(bytes_remain > 0);
	return true;
}

//获得摔倒信息
byte get_fall_info(void)
{
     	find_angle();
		static int fallAngle ;
		int angle_fall[3];
		byte result;
			    angle_fall[1] = (short(buf_angle[3] << 8 | buf_angle[2])) / 32768.0 * 180; //X轴滚转角（x轴）
		//		angle_fall[1] = (short(buf_angle[5] << 8 | buf_angle[4])) / 32768.0 * 180; //Y轴俯仰角（y 轴）
		//		cout<<"buf_angle[5]="<<(int)buf_angle[5]<<endl;
//				angle[2] =(short(buf_angle[7] << 8 |buf_angle[6])) / 32768.0 * 180; //Z轴偏航角（z 轴）
				//if(k==1) {fall_angle_start=angle_fall[1];k--;}
				fallAngle=angle_fall[1]-mcu_x_middle;//;-fall_angle_start;
				//cout<<"fall_angle="<<fallAngle<<endl;
		 			if (fallAngle>60)
		 				{cout<<"fall_forward="<<fallAngle<<endl;result=1;}
		 			else
		 			{
		 				if(fallAngle<-60)
		 					{cout<<"fall_backward="<<fallAngle<<endl;result=2;}
		 			    else
		 			    	result=0;
		 			}
	return result;
}

void find_angle(void)
{
	unsigned char find;
	int count=0;
	while (1)
	{
		read_serial(fd_arm, &find, 1);
		buf_angle[count] = find;
		if (buf_angle[0] != 0x55 && count == 0)
		{
			continue;
		}
		count++;
		if (count == 2 && buf_angle[1] != 0x53 )
		{
			count = 0;
			continue;
		}
		if (count == 11)
		{
			count = 0;
			break;
		}
	}
}

//获取航向角，失败返回-1
double get_yaw(void)
{
	 int i=0;
	 int max,min;
	 int average_yaw=0;
	 int  compare[5];
	 while(1)
	 {
		find_angle();
	double angle_yaw[3];
//	    angle_yaw[2] = (short(buf_angle[3] << 8 | buf_angle[2])) / 32768.0 * 180; //X轴滚转角（x轴）
//		angle[1] = (short(buf_angle[5] << 8 | buf_angle[4])) / 32768.0 * 180; //Y轴俯仰角（y 轴）
		angle_yaw[2] =(short(buf_angle[7] << 8 |buf_angle[6])) / 32768.0 * 180; //Z轴偏航角（z 轴）
		 if(i==0)
			 {
			 compare[0]=int(angle_yaw[2]);
			 max=min=compare[0];
			 }
			 compare[i]=int(angle_yaw[2]);
			 if (max<compare[i]) max=compare[i] ;
			 if (min>compare[i]) min=compare[i];
			 i++;
			 if ((max-min)>20&&i>4)
			 {
				 i=0;
				average_yaw=0;
				 continue;
			 }
			 else if((max-min)<20&&i>4)
			 {
				 break;
			 }
	}

	 average_yaw=compare[2]-robot_info.yaw_start;
	 if(average_yaw>180) average_yaw-=360;
	 if(average_yaw<-180) average_yaw+=360;
	 if(average_yaw>20||average_yaw<-20)
	 //cout<<"average_yaw="<<(int)average_yaw<<endl;
	return average_yaw;
}

//移动一个舵机
void move_duoji(byte id, int position, int speed)
{
	byte databuf[9];
	databuf[0] = 0xff;
	databuf[1] = 0xff;
	databuf[2] = id;
	databuf[3] = 0x07; //length
	databuf[4] = 0x03; //command
	databuf[5] = 0x1e; //address
	databuf[6] = (byte)(position & 0x00ff);
	databuf[7] = (byte)((position >> 8) & 0x00ff);
	databuf[8] = (byte)(speed & 0x00ff);
	databuf[9] = (byte)((speed >> 8) & 0x00ff);
	databuf[10] = 0;
	for(int i=2; i<10; i++)
		databuf[10] += databuf[i];
	databuf[10] = (byte)(~databuf[10]);
	write(fd, databuf, 11*sizeof(byte));
	write(fd, databuf, 11*sizeof(byte));
}

//舵机扭矩状态：0->卸力；1->激活扭矩
void torque(byte status)
{
	byte databuf[8];
	databuf[0] = 0xff;
	databuf[1] = 0xff;
	databuf[2] = 0xfe; //id broadcast
	databuf[3] = 0x04; //length
	databuf[4] = 0x03; //command
	databuf[5] = 0x18; //address
	databuf[6] = status;
	databuf[7] = 0;
	for(int i=2; i<=6; i++)
		databuf[7] += databuf[i];
	databuf[7] = (byte)(~databuf[7]);
	write(fd, databuf, 8*sizeof(byte));
	write(fd, databuf, 8*sizeof(byte));
}

//设置反馈包返回级别: 0->不返回；1->只返回read_data指令；2->全部返回
void return_status(byte status)
{
	byte databuf[8];
	databuf[0] = 0xff;
	databuf[1] = 0xff;
	databuf[2] = 0xfe; //id broadcast
	databuf[3] = 0x04; //length
	databuf[4] = 0x03; //command
	databuf[5] = 0x10; //address
	databuf[6] = status;
	databuf[7] = 0;
	for(int i=2; i<=6; i++)
		databuf[7] += databuf[i];
	databuf[7] = (byte)(~databuf[7]);
	write(fd, databuf, 8*sizeof(byte));
	write(fd, databuf, 8*sizeof(byte));
}

//设置最大扭矩：0x3ff(1023)->最大
void set_max_torque(byte id, int max_torque)
{
	byte databuf[9];
	databuf[0] = 0xff;
	databuf[1] = 0xff;
	databuf[2] = id; //id broadcast
	databuf[3] = 0x05; //length
	databuf[4] = 0x03; //command
	databuf[5] = 0x22;//0x0e; //address
	databuf[6] = (byte)(max_torque & 0x00ff);
	databuf[7] = (byte)((max_torque >> 8) & 0x00ff);
	databuf[8] = 0;
	for(int i=2; i<=7; i++)
		databuf[8] += databuf[i];
	databuf[8] = (byte)(~databuf[8]);
	write(fd, databuf, 9*sizeof(byte));
	write(fd, databuf, 9*sizeof(byte));
}

//LED亮灭 action: 0->灭 1->亮
void led(byte id, byte action)
{
	byte databuf[8];
	databuf[0] = 0xff;
	databuf[1] = 0xff;
	databuf[2] = id;
	databuf[3] = 0x04;
	databuf[4] = 0x03;
	databuf[5] = 0x19;
	databuf[6] = action;
	databuf[7] = 0;
	for(int i=2; i<=6; i++)
		databuf[7] += databuf[i];
	databuf[7] = (byte)(~databuf[7]);
	write(fd, databuf, 8*sizeof(byte));
	write(fd, databuf, 8*sizeof(byte));
}

//设置允许电压范围, 推荐：100～220
void set_power_rang(byte lower, byte upper)
{
	byte databuf[9];
	databuf[0] = 0xff;
	databuf[1] = 0xff;
	databuf[2] = 0xfe; //id broadcast
	databuf[3] = 0x05; //length
	databuf[4] = 0x03; //command
	databuf[5] = 0x0c; //address
	databuf[6] = lower;
	databuf[7] = upper;
	databuf[8] = 0;
	for(int i=2; i<=7; i++)
		databuf[8] += databuf[i];
	databuf[8] = (byte)(~databuf[8]);
	write(fd, databuf, 9*sizeof(byte));
	write(fd, databuf, 9*sizeof(byte));
}

//设置旋转余量和斜率, 推荐margin=1, slope=32
void set_margin_and_slope(byte margin, byte slope)
{
	byte databuf[9];
	databuf[0] = 0xff;
	databuf[1] = 0xff;
	databuf[2] = 0xfe; //id broadcast
	databuf[3] = 0x05; //length
	databuf[4] = 0x03; //command
	databuf[5] = 0x1a; //address
	databuf[6] = margin;
	databuf[7] = slope;
	databuf[8] = 0;
	for(int i=2; i<=7; i++)
		databuf[8] += databuf[i];
	databuf[8] = (byte)(~databuf[8]);
	write(fd, databuf, 9*sizeof(byte));
	write(fd, databuf, 9*sizeof(byte));
}
