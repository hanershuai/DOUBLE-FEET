/*
 * basicwalk.cpp
 *
 *  Created on: 2013-5-28
 *      Author: lixinwei
 */
#include "global.h"



using namespace Robot;
int serialFlag = 0;//XXXXXXXXXXXXXXX
/*
 * Walk Parameters
 * Stance and velocity limit values
 */
Point2D stanceLimitX = Point2D(-0.15, 0.30);
Point2D stanceLimitY = Point2D(0.06, 0.20);
Point2D stanceLimitA = Point2D(-0 * PI / 180, 40 * PI / 180);
Point2D velLimitX = Point2D(-0.07, 0.15);//XXXXXXXXXXXXXX
Point2D velLimitY = Point2D(-0.06, 0.06);//XXXXXXXXXXXXXXXXX
Point2D velLimitA = Point2D(-0.4, 0.4);//XXXXXXXXXXXXXX
Vector3D velDelta = Vector3D(0.01, 0.015, 0.15);/*速度误差值*/

/*
 * Toe/heel overlap checking values
 */
Point2D footSizeX = Point2D(-0.05, 0.05);//xxxxxxxxxxxxxxxxxxx
double stanceLimitMarginY = 0.027;//xxxxxxxxxxxxxxxx
double stanceLimitY2 = 2 * 0.035 - stanceLimitMarginY;//xxxxxxxxxxxxxxxxxx

/*
 * Stance parameters
 */
double bodyHeight = 0.31;//重心高度
double bodyTilt = 10* PI / 180;
//double bodyTilt = 12* PI / 180;/*倾斜*/
double footX = -0.004;
double footY = 0.045;
double supportX = 0.015;
double supportY = -0.005;
Vector3D qLArm0 = Vector3D(90, 8, -40) * PI / 180;
Vector3D qRArm0 = Vector3D(90, -8, -40) * PI / 180;
Vector3D qLArmKick0 = Vector3D(90, 30, -60) * PI / 180;
Vector3D qRArmKick0 = Vector3D(90, -30, -60) * PI / 180;


//Hardness parameters
double hardnessSupport = 1;
double hardnessSwing = 1;
double hardnessArm = 1;
//Gait parameters布态参量
//重要的步行参量，需经常更改，放此以便
double tStep0 = 0.27;//机器人踏一步的周期
double tStep = 0.27;//机器人踏一步的周期
double tZmp = 0.193; //0.193
//double tZmp = 0.171; //0.1466;//0.165-tStep.6;0.245?-tStep.3;    此处理论上为sqrt(Zc/g)......但实际上为理论值的二倍会很理想。另外此值对步态的整体效果有很大影响，可在理论值得二倍左右进行微调。
////the less tZmp, the more bias
double stepHeight = 0.035;//抬脚高度
double  ph1Single =0.04;//单脚站立的时间
	double  ph2Single =0.96 ;//双脚站立的时间
double ph1Zmp = 0.1;
double ph2Zmp = 0.9;//一般不更改

//Compensation parameters修正参数
double hipRollCompensation = 1* PI / 180; //左右摇摆可改变此参数
double toeTipCompensation = 	0 * PI / 180;
Point2D ankleMod = Point2D(-1, 0) * 0 * PI / 180;//ankle脚踝


//Gyro stabilization parameters陀螺仪稳定参数
double gyroFactor = 0.273 * PI / 180 * 300 / 1024;//xxxxxxxxxxxxxxxxxxxxxxx
double ankleImuParamX[4] = { 0.9, 0.3 * gyroFactor, 0, 25 * PI / 180 };//xxxxxxxxxx
double ankleImuParamY[4] = { 0.9, 0.7 * gyroFactor, 0, 25 * PI / 180 };//xxxxxxxxxxx
double kneeImuParamX[4] = { 0.9, 1.2 * gyroFactor, 0, 25 * PI / 180 };//xxxxxxxxxxx
double hipImuParamY[4] = { 0.9, 0.3 * gyroFactor, 0, 25 * PI / 180 };//xxxxxxxxxxxxx
double armImuParamX[4] = { 0.3, 10 * gyroFactor, 20 * PI / 180, 45 * PI / 180 };//xxxxxxxxx
double armImuParamY[4] = { 0.3, 10 * gyroFactor, 20 * PI / 180, 45 * PI / 180 };//xxxxxxxxx
//WalkKick parameters
Point2D walkKickVel = Point2D(0.05, 0.08);
double walkKickSupportMod[2][2] = { { 0.03, 0 }, { 0.03, 0 } };
double walkKickHeightFactor = 1.5;
double  tStepWalkKick = tStep;//Config.walk.tStepWalkKick or tStep;
//

//Sidekick parameters侧踢参数（未使用）
/*Point2D sideKickVel1 = Point2D(0.04, 0.04);//Config.walk.sideKickVel1 or {0.04,0.04};
Point2D sideKickVel2 = Point2D(0.09, 0.05);// Config.walk.sideKickVel2 or {0.09,0.05};
Point2D sideKickVel3 = Point2D(0.09, -0.02);//Config.walk.sideKickVel3 or {0.09,-0.02};侧踢速率
double sideKickSupportMod[2][2] = { { 0, 0 }, { 0, 0 } };
double tStepSideKick = 0.70;//侧踢周期0.7*/
//
//Support bias parameters to reduce backlash-based instability
float supportFront = 0.0;
float supportBack = -0;
float supportSideX = -0;
float supportSideY = 0;
//改变支撑保证稳定
//----------------------------------------------------------
//-- Walk state variables-//
//----------------------------------------------------------
Vector3D uTorso = Vector3D(supportX, 0, 0); //the initial supportX = 0;
Vector3D uLeft = Vector3D(0, footY, 0);
Vector3D uRight = Vector3D(0, -footY, 0);
double pLLeg[6] = { 0, footY, 0, 0, 0, 0 };
double pRLeg[6] = { 0, -footY, 0, 0, 0, 0 };
double pTorso[6] = { supportX, 0, bodyHeight, 0, bodyTilt, 0 };

Vector3D velCurrent =Vector3D(0, 0, 0);//速度当前值
Vector3D velCommand = Vector3D(0, 0, 0);//速度理论值
Vector3D velDiff = Vector3D(0, 0, 0);//
//
//ZMP exponential coefficients:
double aXP = 0, aXN = 0, aYP = 0, aYN = 0;

//Gyro stabilization variables
double LankleShift[2] = { 0.18 * PI / 180, -1.5 * PI / 180 };
double RankleShift[2] = { 0.18* PI /180, -1 * PI / 180 };
double kneeShift = 0.13;
double armShift[2] = { 0, 0 };

bool active = true;
bool started = true;
double iStep0 = -1;
double iStep = 0;
//

double phSingle;
double ph;
Vector3D uSupport;
double m1X, m2X, m1Y, m2Y;
Vector3D uLeft1= Vector3D(0, footY, 0);
Vector3D uRight1= Vector3D(0, -footY, 0);
Vector3D uLeft2 = Vector3D(0, footY, 0);
Vector3D uRight2 = Vector3D(0, -footY, 0);
double get_time();//若程序运行有问题仍需考虑double太大时用科学计数法表示精度减小的问题
//double t0 =get_time();
double tLastStep = 0.00;
//
int stopRequest = 2;
int canWalkKick = 1; //Can we do walkkick with this walk code?
int walkKickRequest = 0;
int walkKickType = 0;
std::vector<double> qLegs(12);  //弧度之
double yawAngle = 0;//xxxxxxxxxxxxxxxxxxxxxxx
int supportLeg = 1;
Vector3D uTorsoActual = Vector3D(0, 0, 0);
//
double gyro_roll = 0;//xxxxxxxxxxxxxxxxxxxxxx
double gyro_pitch = 0;//xxxxxxxxxxxxxxxxxxxxxx
double ankleShiftX = 0;
double ankleShiftY = 0;
double kneeShiftX = 0;
double hipShiftY = 0;
double armShiftX = 0;
double armShiftY = 0;

double hipShift[2] = { 0.3*PI/180,0.45*PI/180 };  //0控制前后+前-后；1控制左右 -左+右
double hipShift3 = 0*PI/180;    //控制左右转动     +右转且平移 -左转且平移//TODO 不 合理
//double hipShift4=-0.8*PI/180;
Vector3D uTorso1 = Vector3D(supportX, 0, 0);
Vector3D uTorso2 = Vector3D(supportX, 0, 0);
double supportMod[2] = { 0, 0 };
double shiftFactor = 0.5;

double T;

double xFoot = 0.1;
double zFoot = 0.9;//抬脚高度的因数
                                    //(×stepHeight是真正的抬起高度)

int pauseCommand = 0;//xxxxxxxxxxxxxx
int goOnCommand = 0;//xxxxxxxxxxxxxxxxx

int initial_step = 2;
//----------------------------------------------------------
//-- End initialization
//----------------------------------------------------------

int fd;
char device0[20];
int angle[12] = {
2197,2034,2042,1736,2231,2048,
1885,2070,2053,1872,1713,2058
//2048,2048,2043,1728,2168,1921,
//2048,2048,2017,1877,1911,2048
		};
int ID_head[2] = { 20, 21 };
int angleOfHead[2] = { 486, 731};
int angleOfHeadInit[2] = { 486,145};
int ID[] = { 8, 9, 10, 11, 12, 13, 2, 3, 4, 5, 6, 7 };  //No.3 and 9 motor was exchanged
int ID_hands[6] = { 14, 15, 16, 17, 18, 19 };
int angleinit[] = {
2197,2034,2042,1736,2231,2048,
1885,2070,2053,1872,1713,2058
//2197,2034,1987,1809,2237,1935,
//1910,2044,2094,1785,1827,2058
//2048,2048,2043,1728,2168,1921,
//2048,2048,2017,1877,1911,2048
};//初始角
int angleOfHands[6] = {
		2048,486,512,
		2048,372,512
		};
int angleOfHandsInit[6] = {

		2048,486,512,
		2048,372,512
		};//初始角
const string lleg("lleg"), rleg("rleg"), legleg("legleg"), lhand("lhand"), rhand("rhand"), handhand(
		"handhand");
int newStepFlag = 0;
int standStill = 0;
char device1[20];
int fd1;
int DecodeIMUData(unsigned char *);
//void fileInit(char*,char*,char*);
double aa[3], w[3], Angle[3], Temperature;
double handSwingAngle = 0;
Point2D handSwingAngleLimit = Point2D(0, 45);

//Head parameters
int Turn_qHead[2] = { 0, 0 };
int qHead[2] = { 0, 0 };
int DecodeIMUData(unsigned char chrTemp[])
{
	switch(chrTemp[1])
	{
	case 0x51:
		aa[0] = (short(chrTemp[3] << 8 | chrTemp[2])) / 32768.0 * 16;
		aa[1] = (short(chrTemp[5] << 8 | chrTemp[4])) / 32768.0 * 16;
		aa[2] = (short(chrTemp[7] << 8 | chrTemp[6])) / 32768.0 * 16;
		Temperature = (short(chrTemp[9] << 8 | chrTemp[8])) / 340.0 + 36.25;
		printf("a = %4.3f\t%4.3f\t%4.3f\t\r\n", aa[0], aa[1], aa[2]);
		return 0;
	case 0x52:
		w[0] = (short(chrTemp[3] << 8 | chrTemp[2])) / 32768.0 * 2000;
		w[1] = (short(chrTemp[5] << 8 | chrTemp[4])) / 32768.0 * 2000;
		w[2] = (short(chrTemp[7] << 8 | chrTemp[6])) / 32768.0 * 2000;
		Temperature = (short(chrTemp[9] << 8 | chrTemp[8])) / 340.0 + 36.25;
		printf("w = %4.3f\t%4.3f\t%4.3f\t\r\n", w[0], w[1], w[2]);
		return 1;
	case 0x53:
		Angle[0] = (short(chrTemp[3] << 8 | chrTemp[2])) / 32768.0 * 180;
		Angle[1] = (short(chrTemp[5] << 8 | chrTemp[4])) / 32768.0 * 180;
		Angle[2] = (short(chrTemp[7] << 8 | chrTemp[6])) / 32768.0 * 180;
		Temperature = (short(chrTemp[9] << 8 | chrTemp[8])) / 340.0 + 36.25;
		printf("Angle = %4.2f\t%4.2f\t%4.2f\tT=%4.2f\r\n", Angle[0], Angle[1], Angle[2], T);
		return 2;
	}
	return 0;
}

int portInit(char *device)
{
	int fd_ = open(device, O_RDWR | O_NOCTTY | O_NDELAY);    //| O_NOCTTY | O_NDELAY
	if (fd_ == -1)
	{
		cerr << "Can't Open Serial Port  " << device << endl;
		exit(EXIT_FAILURE);
//		fd_ = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
//		if (fd_ == -1)
//		{
//			cerr << "Can't Open Serial Port  " << "/dev/ttyUSB1" << endl;
////			exit(EXIT_FAILURE);
//		}
	}//打开串口
	struct termios options;
	if (tcgetattr(fd_, &options) != 0)
		perror("SetupSerial 1");
	options.c_cflag = B115200| CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;
	if (tcsetattr(fd_, TCSANOW, &options) != 0)
		perror("SetupSerial 3");
	return fd_;
}

void serial_legs()
{
		//int speed[12] = { 0x0ff, 0x0ff, 0x0ff, 0x0ff, 0x0ff, 0x0ff, 0x0ff, 0x0ff, 0x0ff, 0x0ff, 0x0ff, 0x0ff };
		int speed[12]={0x1ff,0x1ff,0x1ff,0x1ff,0x1ff,0x1ff,0x1ff,0x1ff,0x1ff,0x1ff,0x1ff,0x1ff};
		//int speed[12]={0x0ef,0x0ef,0x0ef,0x0ef,0x0ef,0x0ef,0x0ef,0x0ef,0x0ef,0x0ef,0x0ef,0x0ef};
		unsigned char data[68];
		int i;
		data[0] = 0xff;
		data[1] = 0xff;
		data[2] = 0xfe;
		data[3] = 0x40;
		data[4] = 0x83;
		data[5] = 0x1e;
		data[6] = 0x04;
		for (i = 0; i < 12; i++)
		{
			data[7 + 5 * i] = (unsigned char) (ID[i] & 0x00ff);
			data[8 + 5 * i] = (unsigned char) (angle[i] & 0x00ff);
			data[9 + 5 * i] = (unsigned char) ((angle[i] >> 8) & 0x00ff);
			data[10 + 5 * i] = (unsigned char) (speed[i] & 0x00ff);
			data[11 + 5 * i] = (unsigned char) ((speed[i] >> 8) & 0x00ff);
		}
		unsigned char crc = 0;
		for (i = 2; i < 67; i++)
		{
			crc += data[i];
		}
		data[67] = ~crc;
		while (write(fd, data, 68 * sizeof(unsigned char)) < 68)
			;
}
void serial_hardness(double hardness, string item)
{
		hardness = (hardness <= 1) ? hardness : 1.0;
		hardness = (hardness >= 0) ? hardness : 0.0;
		int Hardness = hardness * 1023;
		unsigned char HardnessL = Hardness & 0x00ff;
		unsigned char HardnessH = (Hardness >> 8) & 0x00ff;
		unsigned char data[68];
		if (item == legleg)
		{
			int i;
			data[0] = 0xff;
			data[1] = 0xff;
			data[2] = 0xfe;
			data[3] = 0x28;
			data[4] = 0x83;
			data[5] = 0x22;
			data[6] = 0x02;
			for (i = 0; i < 12; i++)
			{
				data[7 + 3 * i] = (unsigned char) (ID[i] & 0x00ff);
				data[8 + 3 * i] = HardnessL;
				data[9 + 3 * i] = HardnessH;
			}
			unsigned char crc = 0;
			for (i = 2; i < 43; i++)
			{
				crc += data[i];
			}
			data[43] = ~crc;
			write(fd, data, 44 * sizeof(unsigned char));
		}
		if (item == handhand)
		{
			int i;
			data[0] = 0xff;
			data[1] = 0xff;
			data[2] = 0xfe;
			data[3] = 0x16;
			data[4] = 0x83;
			data[5] = 0x22;
			data[6] = 0x02;
			for (i = 0; i < 6; i++)
			{
				data[7 + 3 * i] = (unsigned char) (ID_hands[i] & 0x00ff);
				data[8 + 3 * i] = HardnessL;
				data[9 + 3 * i] = HardnessH;
			}
			unsigned char crc = 0;
			for (i = 2; i < 25; i++)
			{
				crc += data[i];
			}
			data[25] = ~crc;
			write(fd, data, 26 * sizeof(unsigned char));
		}
		if (item == lleg)
		{
			int i;
			data[0] = 0xff;
			data[1] = 0xff;
			data[2] = 0xfe;
			data[3] = 0x16;
			data[4] = 0x83;
			data[5] = 0x22;
			data[6] = 0x02;
			for (i = 0; i < 6; i++)
			{
				data[7 + 3 * i] = (unsigned char) (ID[i] & 0x00ff);
				data[8 + 3 * i] = HardnessL;
				data[9 + 3 * i] = HardnessH;
			}
			unsigned char crc = 0;
			for (i = 2; i < 25; i++)
			{
				crc += data[i];
			}
			data[25] = ~crc;
			write(fd, data, 26 * sizeof(unsigned char));
		}
		if (item == rleg)
		{
			int i;
			data[0] = 0xff;
			data[1] = 0xff;
			data[2] = 0xfe;
			data[3] = 0x16;
			data[4] = 0x83;
			data[5] = 0x22;
			data[6] = 0x02;
			for (i = 0; i < 6; i++)
			{
				data[7 + 3 * i] = (unsigned char) (ID[i + 6] & 0x00ff);
				data[8 + 3 * i] = HardnessL;
				data[9 + 3 * i] = HardnessH;
			}
			unsigned char crc = 0;
			for (i = 2; i < 25; i++)
			{
				crc += data[i];
			}
			data[25] = ~crc;
			write(fd, data, 26 * sizeof(unsigned char));
		}
}

void serial_head()
{
	int speed[2] = { 0, 0 };
	unsigned char data[68];
	int i;
	data[0] = 0xff;
	data[1] = 0xff;
	data[2] = 0xfe;
	data[3] = 0x0e;
	data[4] = 0x83;
	data[5] = 0x1e;
	data[6] = 0x04;
	for (i = 0; i < 2; i++)
	{
		data[7 + 5 * i] = (unsigned char) (ID_head[i] & 0x00ff);
		data[8 + 5 * i] = (unsigned char) (angleOfHead[i] & 0x00ff);
		data[9 + 5 * i] = (unsigned char) ((angleOfHead[i] >> 8) & 0x00ff);
		data[10 + 5 * i] = (unsigned char) (speed[i] & 0x00ff);
		data[11 + 5 * i] = (unsigned char) ((speed[i] >> 8) & 0x00ff);
	}
	unsigned char crc = 0;
	for (i = 2; i < 17; i++)
	{
		crc += data[i];
	}
	data[17] = ~crc;
	while (write(fd, data, 18 * sizeof(unsigned char)) < 18);
}
void motion_legs()
{
	double phComp = 0;
	if(ph<ph1Single)
		phComp = ph/ph1Single;
	else
		phComp = 1-(ph - ph1Single)/(ph2Single);
		if (supportLeg == 0)    //leftsupport
		{
			qLegs[0] = qLegs[0] +hipShift3;
			qLegs[1] = qLegs[1] + hipShift[1];    //--Hip roll stabilization
			qLegs[2] = qLegs[2] + hipShift[0];
			qLegs[3] = qLegs[3] + kneeShift;    //--Knee pitch stabilization
			qLegs[4] = qLegs[4] + LankleShift[0];    //--Ankle pitch stabilization

			qLegs[5] = qLegs[5] + LankleShift[1]- hipRollCompensation * phComp;    //--Ankle roll stabilization
			qLegs[11] = qLegs[11] + RankleShift[1] - hipRollCompensation * phComp;      //--Ankle roll stabilization
			qLegs[1] = qLegs[1] + hipRollCompensation * phComp;    // --Hip roll compensation
			qLegs[7] = qLegs[7] + hipRollCompensation * phComp;    //-Hip roll compensation

			qLegs[10] = qLegs[10] + toeTipCompensation * phComp;    //--Lifting toetip
		}
		else
		{
			qLegs[6] = qLegs[6] +hipShift3;
			qLegs[7] = qLegs[7] + hipShift[1];    //--Hip roll stabilization
			qLegs[8] = qLegs[8] + hipShift[0];
			qLegs[9] = qLegs[9] + kneeShift;    //--Knee pitch stabilization
			qLegs[10] = qLegs[10] + RankleShift[0];    //--Ankle pitch stabilization

			qLegs[5] = qLegs[5] + LankleShift[1] + hipRollCompensation * phComp;    //--Ankle roll stabilization
			qLegs[11] = qLegs[11] + RankleShift[1] + hipRollCompensation * phComp;      //--Ankle roll stabilization

			qLegs[4] = qLegs[4] + toeTipCompensation * phComp;    //--Lifting toetip
			qLegs[1] = qLegs[1] + hipRollCompensation * phComp;    // --Hip roll compensation
			qLegs[7] = qLegs[7] - hipRollCompensation * phComp;    //-Hip roll compensation
		}

		angle[0] = (int) (angleinit[0] - qLegs[0] * 180 / PI * 4095 / 360);
		angle[1] = (int) (angleinit[1] - qLegs[1] * 180 / PI * 4095 / 360);
		angle[2] = (int) (angleinit[2] + qLegs[2] * 180 / PI * 4095 / 360);
		angle[3] = (int) (angleinit[3] + qLegs[3] * 180 / PI * 4095 / 360);
		angle[4] = (int) (angleinit[4] - qLegs[4] * 180 / PI * 4095 / 360);
		angle[5] = (int) (angleinit[5] + qLegs[5] * 180 / PI * 4095 / 360);

		//////////////////////////////////////////////////////////////上为zuo脚，，下为you脚
		angle[6] = (int) (angleinit[6] - qLegs[6] * 180 / PI * 4095 / 360);
		angle[7] = (int) (angleinit[7] - qLegs[7] * 180 / PI * 4095 / 360);
		angle[8] = (int) (angleinit[8] - qLegs[8] * 180 / PI * 4095 / 360);
		angle[9] = (int) (angleinit[9] - qLegs[9] * 180 / PI * 4095 / 360);
		angle[10] = (int) (angleinit[10] + qLegs[10] * 180 / PI * 4095 / 360);
		angle[11] = (int) (angleinit[11] + qLegs[11] * 180 / PI * 4095 / 360);

		/*angle[0] = (int) (angleinit[0] + qLegs[0] * 180 / PI * 4095 / 360);
		angle[1] = (int) (angleinit[1] + qLegs[1] * 180 / PI * 4095 / 360);
		angle[2] = (int) (angleinit[2] - qLegs[2] * 180 / PI * 4095 / 360);
		angle[3] = (int) (angleinit[3] - qLegs[3] * 180 / PI * 4095 / 360);
		angle[4] = (int) (angleinit[4] + qLegs[4] * 180 / PI * 4095 / 360);
		angle[5] = (int) (angleinit[5] - qLegs[5] * 180 / PI * 4095 / 360);
		//////////////////////////////////////////////////////////////上为zuo脚，，下为you脚
		angle[6] = (int) (angleinit[6] + qLegs[6] * 180 / PI * 4095 / 360);
		angle[7] = (int) (angleinit[7] + qLegs[7] * 180 / PI * 4095 / 360);
		angle[8] = (int) (angleinit[8] + qLegs[8] * 180 / PI * 4095 / 360);
		angle[9] = (int) (angleinit[9] +qLegs[9] * 180 / PI * 4095 / 360);
		angle[10] = (int) (angleinit[10] - qLegs[10] * 180 / PI * 4095 / 360);
		angle[11] = (int) (angleinit[11] - qLegs[11] * 180 / PI * 4095 / 360);*/
//serial_legs();
}
/*void motion_head()    //得出头部angle，！=q
{
	qHead[0] = min(max((qHead[0]), -90), 90);
	qHead[1] = min(max((qHead[1]), -90), 30);
	angleOfHead[0] = 512 - qHead[0] * 1023 / 300;
	angleOfHead[1] = 512-20*1023/300 - qHead[1] * 1023 / 300;
//	serial_head();
}
void motion_head0()    //得出头部angle，！=q
{
	angleOfHead[0] = 512 - qHead[0] * 1023 / 300;
	angleOfHead[1] = 544 - qHead[1] * 1023 / 300;
//	serial_head();
}
void motionInit_head()    //得出头部angle，！=q
{
	qHead[0] = 0;
	qHead[1] = 0;
	Turn_qHead[0] = 0;
	Turn_qHead[1] = 0;
	angleOfHead[0] = 512;
	angleOfHead[1] = 544;
//	serial_head();
}*/
void serial_hands()
{
		int speed[6] = { 0, 0, 0, 0, 0, 0 };
		unsigned char data[68];
		int i;
		data[0] = 0xff;
		data[1] = 0xff;
		data[2] = 0xfe;
		data[3] = 0x22;
		data[4] = 0x83;
		data[5] = 0x1e;
		data[6] = 0x04;
		for (i = 0; i < 6; i++)
		{
			data[7 + 5 * i] = (unsigned char) (ID_hands[i] & 0x00ff);
			data[8 + 5 * i] = (unsigned char) (angleOfHands[i] & 0x00ff);
			data[9 + 5 * i] = (unsigned char) ((angleOfHands[i] >> 8) & 0x00ff);
			data[10 + 5 * i] = (unsigned char) (speed[i] & 0x00ff);
			data[11 + 5 * i] = (unsigned char) ((speed[i] >> 8) & 0x00ff);
		}
		unsigned char crc = 0;
		for (i = 2; i < 37; i++)
		{
			crc += data[i];
		}
		data[37] = ~crc;
		while (write(fd, data, 38 * sizeof(unsigned char)) < 38)
			;
}

//发送一帧
void serial_all(void)
{
	byte id_encode[20] = {8, 9, 10, 11, 12, 13, 2, 3, 4, 5, 6, 7, 14, 15, 16, 17, 18, 19, 20, 21};
	int angle_tmp[20];
	byte data_buf[108];
	data_buf[0] = 0xff;
	data_buf[1] = 0xff;
	data_buf[2] = 0xfe;
	data_buf[3] = 104;	//length
	data_buf[4] = 0x83;
	data_buf[5] = 0x1e;
	data_buf[6] = 0x04;
	memcpy(angle_tmp, angle, 12*sizeof(int));
	memcpy(&angle_tmp[12], angleOfHands, 6*sizeof(int));
	memcpy(&angle_tmp[18], angleOfHead, 2*sizeof(int));
	for (int i = 0; i < 20; i++)
	{
		data_buf[i * 5 + 7] = id_encode[i];				//ID
		data_buf[i * 5 + 8] = (byte) (angle_tmp[i] & 0x00ff); 			//Goal Position(L)
		data_buf[i * 5 + 9] = (byte) ((angle_tmp[i] >> 8) & 0x00ff); 	//Goal Position(H)
		data_buf[i * 5 + 10] = 0;										//Moving Speed(L)
		data_buf[i * 5 + 11] = 0; 										//Moving Speed(H)
	}
	byte crc = 0;
	for (int i = 2; i < 107; i++)
	{
		crc += data_buf[i];
	}
	crc = (byte) (~crc);
	data_buf[107] = (crc);
	write(fd, data_buf, 108);
}
void motion_hands()
{
	double swingAngle = min(max(handSwingAngleLimit.X, handSwingAngle), handSwingAngleLimit.Y);
	unsigned char swingAnglePh = 1023.0 * (0.5 * (1 - cos((1 - ph) * PI)) + 1 - ph) * 0.5
			* swingAngle / 300.0;
	if (supportLeg == 0)
	{
		angleOfHands[0] = angleOfHandsInit[0] - swingAnglePh;
		angleOfHands[3] = angleOfHandsInit[3] - swingAnglePh;
	}
	if (supportLeg == 1)
	{
		angleOfHands[0] = angleOfHandsInit[0] + swingAnglePh;
		angleOfHands[3] = angleOfHandsInit[3] + swingAnglePh;
	}
	if (ph == 0)
	{
		angleOfHands[0] = angleOfHandsInit[0];
		angleOfHands[3] = angleOfHandsInit[3];
	}
	//serial_hands();
}

double get_time()//获取时间
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	double t;
	t = tv.tv_sec + tv.tv_usec * pow(10, -6);
	return t;
}

double mod_angle(double a)
{
	//Reduce angle to [-pi, pi)
	while (a >= PI)
		a = a - 2 * PI;
	return a;
}

Vector3D se2_interpolate(double t, Vector3D u1, Vector3D u2)    //调节运动参数（缓和）t为u2的权重
{
	//helps smooth out the motions using a weighted average
	Vector3D global;
	global.X = u1.X + t * (u2.X - u1.X),
			global.Y = u1.Y + t * (u2.Y - u1.Y),
			global.Z = u1.Z + t * mod_angle(u2.Z - u1.Z);
	return global;
}

Vector3D pose_global(Vector3D pRelative, Vector3D pose)
{
	Vector3D global;
	double ca = cos(pose.Z);
	double sa = sin(pose.Z);
	global.X = pose.X + ca * pRelative.X - sa * pRelative.Y,
			global.Y = pose.Y + sa * pRelative.X + ca * pRelative.Y,
			global.Z = pose.Z + pRelative.Z;
	return global;
}

Vector3D pose_relative(Vector3D pGlobal, Vector3D pose)//世界坐标与单步参数的关系
{
	Vector3D global;
	double ca = cos(pose.Z);
	double sa = sin(pose.Z);
	double px = pGlobal.X - pose.X;
	double py = pGlobal.Y - pose.Y;
	double pa = pGlobal.Z - pose.Z;
	global.X = ca * px + sa * py;
	global.Y = -sa * px + ca * py;
	global.Z = mod_angle(pa);
	return global;
}

Vector3D step_torso(Vector3D uLeft, Vector3D uRight, double shiftFactor)//计算torso
{
	Vector3D global;
	Vector3D u0 = se2_interpolate(0.5, uLeft, uRight);    //u0什么用？
	Vector3D uLeftSupport = pose_global(Vector3D(supportX, supportY, 0), uLeft);
	Vector3D uRightSupport = pose_global(Vector3D(supportX, -supportY, 0), uRight);
	global = se2_interpolate(shiftFactor, uLeftSupport, uRightSupport);
	return global;
}

Vector3D step_right_destination(Vector3D vel, Vector3D uLeft, Vector3D uRight)//计算右腿支撑时坐标参数
{
	Vector3D global;
	Vector3D u0 = se2_interpolate(0.5, uLeft, uRight);
	//Determine nominal midpoint position 1.5 steps in future
	Vector3D u1 = pose_global(vel, u0);
	Vector3D u2 = pose_global(vel * 0.5, u1);
	Vector3D uRightPredict = pose_global(Vector3D(0, -footY, 0), u2);
	Vector3D uRightLeft = pose_relative(uRightPredict, uLeft);
	//Do not pidgeon toe, cross feet:

	//Check toe and heel overlap
	/*double toeOverlap= footSizeX.X*uRightLeft.Z;
	 double heelOverlap= footSizeX.Y*uRightLeft.Z;
	 double limitY = max(stanceLimitY.X,stanceLimitY2+max(toeOverlap,heelOverlap));
	 cout<<"Toeoverlap Heeloverlap"<<toeOverlap<<heelOverlap<<limitY<<endl;*/

	uRightLeft.X = min(max(uRightLeft.X, -stanceLimitX.Y), -stanceLimitX.X);
	uRightLeft.Y = min(max(uRightLeft.Y, -stanceLimitY.Y), -stanceLimitY.X);
	uRightLeft.Z = min(max(uRightLeft.Z, -stanceLimitA.Y), -stanceLimitA.X);

	global = pose_global(uRightLeft, uLeft);
	return global;
}

Vector3D step_left_destination(Vector3D vel, Vector3D uLeft, Vector3D uRight)//计算左腿支撑时坐标参数
{
	Vector3D global;
	Vector3D u0 = se2_interpolate(0.5, uLeft, uRight);
	//Determine nominal midpoint position 1.5 steps in future
	Vector3D u1 = pose_global(vel, u0);
	Vector3D u2 = pose_global(vel * 0.5, u1);

	Vector3D uLeftPredict = pose_global(Vector3D(0, footY, 0), u2);
	Vector3D uLeftRight = pose_relative(uLeftPredict, uRight);

	uLeftRight.X = min(max(uLeftRight.X, stanceLimitX.X), stanceLimitX.Y);
	uLeftRight.Y = min(max(uLeftRight.Y, stanceLimitY.X), stanceLimitY.Y);
	uLeftRight.Z = min(max(uLeftRight.Z, stanceLimitA.X), stanceLimitA.Y);

	global = pose_global(uLeftRight, uRight);

	return global;
}

double foot_phase_xf(double ph)
{
	//Computes relative x,z motion of foot during single support phase
	//phsingle=0: x=0,z=0  phsingle=1:x=1,z=0
	phSingle = min(max(ph - ph1Single, 0.0) / (ph2Single - ph1Single), 1.0);//{ph,ph1single,ph2single}->phsingle
//	double phSingleSkew = pow(phSingle, 0.80) - 0.17 * phSingle * (1 - phSingle);//0.8    0.17
		double phSingleSkew = pow(phSingle, 0.80) - 0.1 * phSingle * (1 - phSingle);//0.8    0.17
	double xf = 0.5 * (1 - cos(PI * phSingleSkew));

	/*float factor1=0;
	 float factor2=0;
	 double phSingleSkew2 = max(min(1.0,(phSingleSkew-factor1)/(1-factor1-factor2)), 0.0);
	 xf = 0.5*(1-cos(PI*phSingleSkew2));*/
	//check for walkkick step
	if (walkKickRequest == 4)
	{
		if (walkKickType < 2)
		{
			float kickN = 1.5;
			if (phSingle < 0.5)
				xf = kickN * phSingle;
			else
				xf = (1 - kickN) * (2 * phSingle - 1) + kickN;
		}
	}
	return xf;
}

double foot_phase_xf_kick(double ph)
{
	//Computes relative x,z motion of foot during single support phase
	//phsingle=0: x=0,z=0  phsingle=1:x=1,z=0
	phSingle = min(max(ph - ph1Single, 0.0) / (ph2Single - ph1Single), 1.0);
	double xf =0;
			float kickN = 2;
			double  t1=0.4;
			if (phSingle < t1)
				xf = kickN * phSingle;
			else
				xf = (1 - kickN) * (1/t1 * phSingle - 1) + kickN;
	return xf;
}
double foot_phase_zf_kick(double ph)
{
	//Computes relative x,z motion of foot during single support phase
	//phsingle=0: x=0,z=0  phsingle=1:x=1,z=0
	phSingle = min(max(ph - ph1Single, 0.0) / (ph2Single - ph1Single), 1.0);
	double phSingleSkew = pow(phSingle, 0.8) - 0.1 * phSingle * (1 - phSingle);
//	phSingleSkew =phSingle;
	double zf = 0.5 * (1 - cos(2 * PI * phSingleSkew));
		zf = zf * walkKickHeightFactor;	//increase step height *1.5
	return zf;
}
double foot_phase_zf(double ph)
{
	//Computes relative x,z motion of foot during single support phase
	//phsingle=0: x=0,z=0  phsingle=1:x=1,z=0

	phSingle = min(max(ph - ph1Single, 0.0) / (ph2Single - ph1Single), 1.0);
	double phSingleSkew = pow(phSingle, 0.8) - 0.1 * phSingle * (1 - phSingle);
//	phSingleSkew =phSingle;
	double zf = 0.5 * (1 - cos(2 * PI * phSingleSkew));

	//check for walkkick step
	if (walkKickRequest == 4)
	{
		zf = zf * walkKickHeightFactor;	//increase step height *1.5
	}
	return zf;
}

double zmp_solve_aP(double zs, double z1, double z2, double x1, double x2)
{
	/*
	 Solves ZMP equation:
	 x(t) = z(t) + aP*exp(t/tZmp) + aN*exp(-t/tZmp) - tZmp*mi*sinh((t-Ti)/tZmp)
	 where the ZMP point is piecewise linear:
	 z(0) = z1, z(T1 < t < T2) = zs, z(tStep) = z2
	 */
	double T1 = tStep * ph1Zmp;
	double T2 = tStep * ph2Zmp;
	double m1 = (zs - z1) / T1;
	double m2 = -(zs - z2) / (tStep - T2);

	double c1 = x1 - z1 + tZmp * m1 * sinh(-T1 / tZmp);
	double c2 = x2 - z2 + tZmp * m2 * sinh((tStep - T2) / tZmp);
	double expTStep = exp(tStep / tZmp);
	double aP = (c2 - c1 / expTStep) / (expTStep - 1 / expTStep);
	return aP;
}
double zmp_solve_aN(double zs, double z1, double z2, double x1, double x2)
{
	/*
	 Solves ZMP equation:
	 x(t) = z(t) + aP*exp(t/tZmp) + aN*exp(-t/tZmp) - tZmp*mi*sinh((t-Ti)/tZmp)
	 where the ZMP point is piecewise linear:
	 z(0) = z1, z(T1 < t < T2) = zs, z(tStep) = z2
	 */
	double T1 = tStep * ph1Zmp;
	double T2 = tStep * ph2Zmp;
	double m1 = (zs - z1) / T1;
	double m2 = -(zs - z2) / (tStep - T2);

	double c1 = x1 - z1 + tZmp * m1 * sinh(-T1 / tZmp);
	double c2 = x2 - z2 + tZmp * m2 * sinh((tStep - T2) / tZmp);
	double expTStep = exp(tStep / tZmp);
	double aN = (c1 * expTStep - c2) / (expTStep - 1 / expTStep);
	return aN;
}

Vector3D zmp_com(double ph)
{
	Vector3D com;
	double expT = exp(tStep * ph / tZmp);
	com.X = uSupport.X + aXP * expT + aXN / expT;
	com.Y = uSupport.Y + aYP * expT + aYN / expT;
	if (ph < ph1Zmp)
	{
		com.X = com.X + m1X * tStep * (ph - ph1Zmp)
				- tZmp * m1X * sinh(tStep * (ph - ph1Zmp) / tZmp);
		com.Y = com.Y + m1Y * tStep * (ph - ph1Zmp)
				- tZmp * m1Y * sinh(tStep * (ph - ph1Zmp) / tZmp);
	}
	else if (ph > ph2Zmp)
	{
		com.X = com.X + m2X * tStep * (ph - ph2Zmp)
				- tZmp * m2X * sinh(tStep * (ph - ph2Zmp) / tZmp);
		com.Y = com.Y + m2Y * tStep * (ph - ph2Zmp)
				- tZmp * m2Y * sinh(tStep * (ph - ph2Zmp) / tZmp);
	}
	com.Z = ph * (uLeft2.Z + uRight2.Z) / 2 + (1 - ph) * (uLeft1.Z + uRight1.Z) / 2;
	return com;

}

double procFunc(double a, double deadband, double maxvalue)
{
	//--Piecewise linear function for IMU feedback
	double b = 0;
	if (a > 0)
		b = min(max(0.0, abs(a) - deadband), maxvalue);
	else
		b = -min(max(0.0, abs(a) - deadband), maxvalue);
	return b;
}

void check_walkkick()
{
	//--Check walking kick phases
	if (walkKickType > 1)
		return;

	if (walkKickRequest == 1)  //--If support foot is right, skip 1st step
	{
		if (supportLeg == walkKickType)
			walkKickRequest = 2;
	}

	if (walkKickRequest == 1)
	{  //-- Feet together
		if (supportLeg == 0)
			uRight2 = pose_global(Vector3D(0, -2 * footY, 0), uLeft1);
		else
			uLeft2 = pose_global(Vector3D(0, 2 * footY, 0), uRight1);
		walkKickRequest = walkKickRequest + 1;
	}

	else if (walkKickRequest == 2)
	{  //-- Support step forward
		if (supportLeg == 0)
		{
			uRight2 = pose_global(Vector3D(walkKickVel.X, -2 * footY, 0), uLeft1);
			shiftFactor = 0.7;  //--shift final torso to right foot
		}
		else
		{
			uLeft2 = pose_global(Vector3D(walkKickVel.X, 2 * footY, 0), uRight1);
			shiftFactor = 0.3; //--shift final torso to left foot
		}
		supportMod[0] = walkKickSupportMod[0][0];
		supportMod[1] = walkKickSupportMod[0][1];
		walkKickRequest = walkKickRequest + 1;

		//--Slow down tStep for two kick step
		tStep = tStepWalkKick;
	}

	else if (walkKickRequest == 3)
	{  //-- Kicking step forward
		if (supportLeg == 0)
			uRight2 = pose_global(Vector3D(walkKickVel.Y, -2 * footY, 0), uLeft1);
		else
			uLeft2 = pose_global(Vector3D(walkKickVel.Y, 2 * footY, 0), uRight1);  //--RS

		supportMod[0] = walkKickSupportMod[1][0];
		supportMod[1] = walkKickSupportMod[1][1];
		walkKickRequest = walkKickRequest + 1;
	}

	else if (walkKickRequest == 4)
	{  //-- Feet together
		if (supportLeg == 0)
			uRight2 = pose_global(Vector3D(0, -2 * footY, 0), uLeft1);
		else
			uLeft2 = pose_global(Vector3D(0, 2 * footY, 0), uRight1);

		walkKickRequest = 0;
		tStep = tStep0;
	}
}
void update_velocity()
{
	velDiff.X= min(max(velCommand.X-velCurrent.X,
	  -velDelta.X),velDelta.X);
	velDiff.Y= min(max(velCommand.Y-velCurrent.Y,
	  -velDelta.Y),velDelta.Y);
	velDiff.Z= min(max(velCommand.Z-velCurrent.Z,
	  -velDelta.Z),velDelta.Z);
	supportMod[1] = 0.0;
	if(velDiff.X>=0.05)
			supportMod[0] = 0.02;
	else if(velDiff.X>=0.01)
		supportMod[0] = 0.01;
	else if(velDiff.X<=-0.01)
		supportMod[0] =-0.02;
	else supportMod[0] = -0.01;

	if(velDiff.Y>=0.02)
		supportMod[1] = 0.02;
	else if(velDiff.Y>=0.01)
		supportMod[1] = 0.01;
	else if(velDiff.Y<-0.02)
	{
		supportMod[1] = -0.02;
		velDiff.Z = -4*PI/180+velDiff.Z;
	}
	else if(velDiff.Y<-0.01)
	{
		supportMod[1] = -0.01;
		velDiff.Z = -4*PI/180+velDiff.Z;
	}
	else supportMod[1] = 0;

velCurrent = velCurrent+velDiff;
if(abs(velCurrent.Z)>10*PI/180&&velCurrent.X>=0.04)
  velCurrent.Z = (velCurrent.Z>0)?(5*PI/180):(-5*PI/180);
if( initial_step>0)
velCurrent=Vector3D(0,0,0);
initial_step=initial_step-1;
}
void update_still()
{
	uTorso = step_torso(uLeft, uRight,0.5);
	uTorsoActual = pose_global(Vector3D(-footX, 0, 0), uTorso);
		pLLeg[0] = uLeft.X;
		pLLeg[1] = uLeft.Y;
		pLLeg[5] = uLeft.Z;
		pRLeg[0] = uRight.X;
		pRLeg[1] = uRight.Y;
		pRLeg[5] = uRight.Z;
		pTorso[0] = uTorsoActual.X;
		pTorso[1] = uTorsoActual.Y;
		pTorso[5] = uTorsoActual.Z;
		qLegs = darwinop_kinematics_inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);//得到瞬时舵机角度改变量
	  motion_legs();
//	serial_legs();
}
void StepReset()//初始化
{
	uLeft = Vector3D(0,footY,0);
	uRight = Vector3D(0,-footY,0);
	uTorso = step_torso(uLeft, uRight,0.5);
	uTorsoActual = pose_global(Vector3D(-footX, 0, 0), uTorso);
		pLLeg[0] = uLeft.X;
		pLLeg[1] = uLeft.Y;
		pLLeg[5] = uLeft.Z;
		pRLeg[0] = uRight.X;
		pRLeg[1] = uRight.Y;
		pRLeg[5] = uRight.Z;
		pTorso[0] = uTorsoActual.X;
		pTorso[1] = uTorsoActual.Y;
		pTorso[5] = uTorsoActual.Z;
		qLegs = darwinop_kinematics_inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);
	  motion_legs();
//		serial_legs();
}
void set_leg_hardness()
{
	double phComp = min(min(1.0, phSingle / 0.1), (1 - phSingle) / 0.1);
	double a = 0;
	if (supportLeg == 0)  //left leg
	{
		a = (phComp + 1.0) / 2.0;
	}
	if (supportLeg == 1)  //left leg
	{
		a = 1.0 - (phComp + 1.0) / 2.0;
	}
	serial_hardness(a * hardnessSupport + (1 - a) * hardnessSwing, lleg);
	serial_hardness((1 - a) * hardnessSupport + a * hardnessSwing, rleg);
}
Vector3D checkVel(Vector3D vel0, Vector3D vel1)  //vel0为当前速度，vel1为下一步命令速度
{
	if (vel1.X - vel0.X > 0.02)
	{
		vel0 = vel0 + Vector3D(0.02, 0, 0);
		vel0.Y = vel1.Y;
		vel0.Z = vel1.Z;
	}
	else if (vel1.X - vel0.X < -0.02)
	{
		vel0 = vel0 - Vector3D(0.02, 0, 0);

		vel0.Y = vel1.Y;
		vel0.Z = vel1.Z;
	}
	else
	{
		vel0 = vel1;
	}
	return vel0;
}

int update()
{
	double t = get_time();
	  if (not active)
	  {
	    update_still();
	    return 0;
	  }
	  if (not started)
	  {
		  started=true;
		  t = get_time();
	    return 0;
	  }
	ph = (t - tLastStep) / tStep;
	if (ph > 1)
	{
//		walkKickRequest=1;
		iStep=iStep+1;
		ph = ph - floor(ph);
//		tLastStep = tLastStep + tStep;
		tLastStep = get_time();
	}
	// --Stop when stopping sequence is done
	if ((iStep >  iStep0)&&(stopRequest==2))
	{
			 stopRequest = 0;
			    active = false;
			return 0;
	}
	//  -- New step
	  if (iStep > iStep0)
	  {
	 update_velocity();
		iStep0 = iStep;
		supportLeg = int(iStep) % 2; //-- 0 for left support, 1 for right support
		uLeft1 = uLeft2;
		uRight1 = uRight2;
		uTorso1 = uTorso2;

		supportMod[0] = 0; //--Support Point modulation for walkkick
		supportMod[1] = 0;
		shiftFactor = 0.5; //--How much should we shift final Torso pose?
		if (walkKickRequest > 0)	//--If stop signal sent, put two feet together
			check_walkkick();
		else if(stopRequest==1)	//--Final step
		{
			stopRequest=2;
			velCurrent=Vector3D(0.01,0,0);
			velCommand=Vector3D(00.01,0,0);
			if (supportLeg == 0) // Left support
//				uRight2 = step_right_destination(velCurrent, uLeft1, uRight1);
				uRight2 = pose_global(Vector3D(0,-2*footY,0), uLeft1);
			else//-- Right support
//				uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1);
				uLeft2 = pose_global(Vector3D(0,2*footY,0), uRight1);
		}
		else//--Normal walk, advance steps
		{
			tStep=tStep0;
		      if (supportLeg == 0 )//-- Left support
		        uRight2 = step_right_destination(velCurrent, uLeft1, uRight1);
		      else//-- Right support
		        uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1);

		      //  --Velocity-based support point modulation
		      if (velCurrent.X > 0.06)
				supportMod[0] = supportFront;
			else if (velCurrent.X < 0)
				supportMod[0] = supportBack;

			if (velCurrent.Y > 0.015)
			{
				supportMod[0] = supportSideX;
				supportMod[1] = supportSideY;
			}
			else if (velCurrent.Y < -0.015)
			{
				supportMod[0] = supportSideX;
				supportMod[1] = -supportSideY;
			}
		}

		uTorso2 = step_torso(uLeft2, uRight2, shiftFactor);

		//--Apply velocity-based support point modulation for uSupport
		if (supportLeg == 0)  //--LS
		{
			Vector3D uLeftTorso = pose_relative(uLeft1, uTorso1);

			Vector3D uTorsoModded = pose_global(Vector3D(supportMod[0], supportMod[1], 0), uTorso);
			Vector3D uLeftModded = pose_global(uLeftTorso, uTorsoModded);
			uSupport = pose_global(Vector3D(supportX, supportY, 0), uLeftModded);  //,uLeft1);//
//			set_leg_hardness();
		}
		else //--RS
		{
			Vector3D uRightTorso = pose_relative(uRight1, uTorso1);
			Vector3D uTorsoModded = pose_global(Vector3D(supportMod[0], supportMod[1], 0), uTorso);
			Vector3D uRightModded = pose_global(uRightTorso, uTorsoModded);
			uSupport = pose_global(Vector3D(supportX, -supportY, 0), uRightModded); //;//uRight1)
//			set_leg_hardness();
		}
//cout<<supportLeg<<"    "<<uLeft.X<<"   "<<uRight.X<<"   "<<uTorso.X<<"      istep"<<iStep<<"    "<<iStep0<<endl;
		//--Compute ZMP coefficients
		m1X = (uSupport.X - uTorso1.X) / (tStep * ph1Zmp);
		m2X = (uTorso2.X - uSupport.X) / (tStep * (1 - ph2Zmp));
		m1Y = (uSupport.Y - uTorso1.Y) / (tStep * ph1Zmp);
		m2Y = (uTorso2.Y - uSupport.Y) / (tStep * (1 - ph2Zmp));
		aXP = zmp_solve_aP(uSupport.X, uTorso1.X, uTorso2.X,
				uTorso1.X, uTorso2.X);
		aXN = zmp_solve_aN(uSupport.X, uTorso1.X, uTorso2.X,
				uTorso1.X, uTorso2.X);
		aYP = zmp_solve_aP(uSupport.Y, uTorso1.Y, uTorso2.Y,
				uTorso1.Y, uTorso2.Y);
		aYN = zmp_solve_aN(uSupport.Y, uTorso1.Y, uTorso2.Y,
				uTorso1.Y, uTorso2.Y);
//		cout<<"m1X		m2X		m1Y		m2Y		aXP			aXN		aYP			aYN"<<endl;
//		cout<<m1X<<"	"<<m2X<<"	"<<m1Y<<"	"<<m2Y<<"	"<<aXP<<"	"<<aXN<<"	"<<aYP<<"	"<<aYN<<endl<<endl;
	}
	  //--End new step
	xFoot = foot_phase_xf(ph);
	zFoot = foot_phase_zf(ph);
	if(initial_step>0) zFoot=0;  // --Don't lift foot at initial step
	 pLLeg[2]=0;  pRLeg[2] = 0;
	if (supportLeg == 0)   //-- Left support
	{
		uRight = se2_interpolate(xFoot, uRight1, uRight2);
		pRLeg[2] = stepHeight * zFoot;
	}
	else    //-- Right support
	{
		uLeft = se2_interpolate(xFoot, uLeft1, uLeft2);
		pLLeg[2] = stepHeight * zFoot;
	}
	 if( supportLeg == 0 )//    -- Left support
	 {
	    if (walkKickRequest == 4&&walkKickType>1)// --Side kick
	    {
	      if (xFoot<0.5 )
	      {
	    	  uRight = se2_interpolate(xFoot*2, uRight1, (uRight1+uRight2)*0.5);
	      }
	      else uRight = se2_interpolate(xFoot*2-1, (uRight1+uRight2)*0.5, uRight2);
	    }
	      else
	      uRight = se2_interpolate(xFoot, uRight1, uRight2);
	    pRLeg[2] = stepHeight*zFoot;
	    }
		 if( supportLeg == 1 )//    -- Right  support
		 {
		    if (walkKickRequest == 4&&walkKickType>1)// --Side kick
		    {
		      if (xFoot<0.5 )
		      {
		    	  uLeft = se2_interpolate(xFoot*2, uLeft1,(uLeft1+uLeft2)*0.5);
		      }
		      else uLeft = se2_interpolate(xFoot*2-1,(uLeft1+uLeft2)*0.5, uLeft2);
		    }
		      else
		    	  uLeft = se2_interpolate(xFoot, uLeft1, uLeft2);
		    pLLeg[2] = stepHeight*zFoot;
		    }
	uTorso = zmp_com(ph);
	uTorsoActual = pose_global(Vector3D(-footX, 0, 0), uTorso);
	pLLeg[0] = uLeft.X;
	pLLeg[1] = uLeft.Y;
	pLLeg[5] = uLeft.Z;
	pRLeg[0] = uRight.X;
	pRLeg[1] = uRight.Y;
	pRLeg[5] = uRight.Z;
	pTorso[0] = uTorsoActual.X;
	pTorso[1] = uTorsoActual.Y;
	pTorso[5] = uTorsoActual.Z;
	qLegs = darwinop_kinematics_inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);
  motion_legs();
//	serial_legs();
//  motion_hands();
//  serial_head();
	return 0;
}

int updateOneStep(Vector3D vel)
{
	tLastStep = get_time();
	double t = get_time();
	bool go = true;
	int stepNum = 1;
	velCommand= vel;
	update_velocity();
	while (go)
	{
	//cout<<velCurrent.X<<"//"<<velCurrent.Y<<"//"<<velCurrent.Z<<endl;
		t = get_time();
		T = t - tLastStep;
		ph = (t - tLastStep) / tStep;
		if (ph > 1)
		{
			iStep++;
			ph = ph - 1;
			stepNum = stepNum + 1;
			tLastStep = get_time();
		}
		if (stepNum == 2)
			break;
		  if (iStep > iStep0)
	  {
		iStep0 = iStep;
		supportLeg = int(iStep) % 2; //-- 0 for left support, 1 for right support
		uLeft1 = uLeft2;
		uRight1 = uRight2;
		uTorso1 = uTorso2;

//		supportMod[0] = 0; //--Support Point modulation for walkkick
//		supportMod[1] = 0;
		shiftFactor = 0.5; //--How much should we shift final Torso pose?
		if (walkKickRequest > 0)	//--If stop signal sent, put two feet together
			check_walkkick();
		else if(stopRequest==1)	//--Final step
		{
			stopRequest=2;
			if (supportLeg == 0) // Left support
//				uRight2 = step_right_destination(velCurrent, uLeft1, uRight1);
				uRight2 = pose_global(Vector3D(0,-2*footY,0), uLeft1);
			else//-- Right support
//				uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1);
				uLeft2 = pose_global(Vector3D(0,2*footY,0), uRight1);
		}
		else//--Normal walk, advance steps
		{
			tStep=tStep0;
		      if (supportLeg == 0)// Left support
		        uRight2 = step_right_destination(velCurrent, uLeft1, uRight1);
		      else  //-- Right support
		        uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1);

//		      //  --Velocity-based support point modulation
//		      if (velCurrent.X > 0.06)
//				supportMod[0] = supportFront;
//			else if (velCurrent.X < 0)
//				supportMod[0] = supportBack;
//
//			if (velCurrent.Y > 0.015)
//			{
//				supportMod[0] = supportSideX;
//				supportMod[1] = supportSideY;
//			}
//			else if (velCurrent.Y < -0.015)
//			{
//				supportMod[0] = supportSideX;
//				supportMod[1] = -supportSideY;
//			}
		}

		uTorso2 = step_torso(uLeft2, uRight2, shiftFactor);

		//--Apply velocity-based support point modulation for uSupport
		if (supportLeg == 0)  //--LS
		{
			Vector3D uLeftTorso =pose_relative(uLeft1,uTorso1);

			      Vector3D uTorsoModded =pose_global(Vector3D(supportMod[0],supportMod[1],0),uTorso);
				  Vector3D uLeftModded =pose_global(uLeftTorso,uTorsoModded);

			      uSupport =pose_global(Vector3D(supportX, supportY, 0),uLeftModded);//uLeft1);
//			set_leg_hardness();
		}
		else //--RS
		{
			Vector3D uRightTorso = pose_relative(uRight1, uTorso1);
			Vector3D uTorsoModded = pose_global(Vector3D(supportMod[0], supportMod[1], 0), uTorso);
			Vector3D uRightModded = pose_global(uRightTorso, uTorsoModded);
			uSupport = pose_global(Vector3D(supportX, -supportY, 0), uRightModded); //;//uRight1)
//			set_leg_hardness();
		}
		//--Compute ZMP coefficients
		m1X = (uSupport.X - uTorso1.X) / (tStep * ph1Zmp);
		m2X = (uTorso2.X - uSupport.X) / (tStep * (1 - ph2Zmp));
		m1Y = (uSupport.Y - uTorso1.Y) / (tStep * ph1Zmp);
		m2Y = (uTorso2.Y - uSupport.Y) / (tStep * (1 - ph2Zmp));
		aXP = zmp_solve_aP(uSupport.X, uTorso1.X, uTorso2.X,
				uTorso1.X, uTorso2.X);
		aXN = zmp_solve_aN(uSupport.X, uTorso1.X, uTorso2.X,
				uTorso1.X, uTorso2.X);
		aYP = zmp_solve_aP(uSupport.Y, uTorso1.Y, uTorso2.Y,
				uTorso1.Y, uTorso2.Y);
		aYN = zmp_solve_aN(uSupport.Y, uTorso1.Y, uTorso2.Y,
				uTorso1.Y, uTorso2.Y);
//		cout<<"m1X		m2X		m1Y		m2Y		aXP			aXN		aYP			aYN"<<endl;
//		cout<<m1X<<"	"<<m2X<<"	"<<m1Y<<"	"<<m2Y<<"	"<<aXP<<"	"<<aXN<<"	"<<aYP<<"	"<<aYN<<endl<<endl;
	}
	  //--End new step
		if (velCurrent.X == 0 && velCurrent.Y == 0 && velCurrent.Z == 0)
		{
			ph = 0;
		}
		xFoot = foot_phase_xf(ph);
		zFoot = foot_phase_zf(ph);
		/*if(initial_step>0) zFoot=0;  // --Don't lift foot at initial step
		 pLLeg[2]=0;  pRLeg[2] = 0;*/
		if (supportLeg == 0)   //-- Left support
		{
			uRight = se2_interpolate(xFoot, uRight1, uRight2);
			pRLeg[2] = stepHeight * zFoot;
		}
		else    //-- Right support
		{

			uLeft = se2_interpolate(xFoot, uLeft1, uLeft2);
			pLLeg[2] = stepHeight * zFoot;
		}

		uTorso = zmp_com(ph);

		uTorsoActual = pose_global(Vector3D(-footX, 0, 0), uTorso);

		pLLeg[0] = uLeft.X;
		pLLeg[1] = uLeft.Y;
		pLLeg[5] = uLeft.Z;
		pRLeg[0] = uRight.X;
		pRLeg[1] = uRight.Y;
		pRLeg[5] = uRight.Z;
		pTorso[0] = uTorsoActual.X;
		pTorso[1] = uTorsoActual.Y;
		pTorso[5] = uTorsoActual.Z;
//cout<<uLeft.X<<"          "<<uLeft.Y<<"       "<<uLeft.Z<<endl;
//cout<<uRight.X<<"          "<<uRight.Y<<"       "<<uRight.Z<<endl<<endl;;
		qLegs = darwinop_kinematics_inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);
		{
//	  serial_head();
//  waitTime(0.01);
			motion_legs();
//			serial_legs();
motion_hands();
//serial_hands();
		}
#ifndef GONG_KONG
		usleep(5000);
#else
		usleep(50000);
#endif
	}
	//cout<<"当		前	 速	度	"<<velCurrent.X<<"   "<<velCurrent.Y<<"      "<<velCurrent.Z*180/PI<<endl;
	return 0;
}



int updateOneStep0(Vector3D vel)
{

	tLastStep = get_time();
	double t = get_time();
	bool go = true;
	int stepNum = 0;
	velCurrent = vel;
	while (go)
	{
		t = get_time();
		T = t - tLastStep;
		ph = (t - tLastStep) / tStep;
		if (ph > 1)
		{
			iStep++;
			ph = 0;
			stepNum = stepNum + 1;
			tLastStep = get_time();
		}
		if (stepNum == 0)
		{
			velCurrent.X = vel.X;
		}
		if (stepNum == 1)
		{
			velCurrent.X = vel.X * 0.5;
		}
		if (stepNum == 2)
			break;
		if (iStep != iStep0)
		{
			iStep0 = iStep;
			supportLeg = int(iStep) % 2; //-- 0 for left support, 1 for right support
			set_leg_hardness();
			uLeft1 = uLeft2;
			uRight1 = uRight2;
			uTorso1 = uTorso2;

			supportMod[0] = 0; //--Support Point modulation for walkkick
			supportMod[1] = 0;
			shiftFactor = 0.5; //--How much should we shift final Torso pose?
			if (walkKickRequest > 0)
				check_walkkick();
			else
			{
				tStep = tStep0;

				if (supportLeg == 0) // Left support
					uRight2 = step_right_destination(velCurrent, uLeft1, uRight1);
				else
					//-- Right support
					uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1);
				if (velCurrent.X > 0.06)
					supportMod[0] = supportFront;
				else if (velCurrent.X < 0)
					supportMod[0] = supportBack;

				if (velCurrent.Y > 0.015)
				{
					supportMod[0] = supportSideX;
					supportMod[1] = supportSideY;
				}
				else if (velCurrent.Y < -0.015)
				{
					supportMod[0] = supportSideX;
					supportMod[1] = -supportSideY;
				}
			}

			uTorso2 = step_torso(uLeft2, uRight2, shiftFactor);

			//--Apply velocity-based support point modulation for uSupport
			if (supportLeg == 0)  //--LS
			{
				Vector3D uLeftTorso = pose_relative(uLeft1, uTorso1);

				Vector3D uTorsoModded = pose_global(Vector3D(supportMod[0], supportMod[1], 0),
						uTorso);
				Vector3D uLeftModded = pose_global(uLeftTorso, uTorsoModded);

				uSupport = pose_global(Vector3D(supportX, supportY, 0), uLeft1);  //uLeftModded);
				//
				// Body.set_rleg_hardness(hardnessSwing);
			}
			else //--RS
			{
				Vector3D uRightTorso = pose_relative(uRight1, uTorso1);
				Vector3D uTorsoModded = pose_global(Vector3D(supportMod[0], supportMod[1], 0),
						uTorso);
				Vector3D uRightModded = pose_global(uRightTorso, uTorsoModded);
				uSupport = pose_global(Vector3D(supportX, -supportY, 0), uRight1); //uRightModded);

				//Body.set_rleg_hardness(hardnessSupport);
			}

			//--Compute ZMP coefficients
			m1X = (uSupport.X - uTorso1.X) / (tStep * ph1Zmp);
			m2X = (uTorso2.X - uSupport.X) / (tStep * (1 - ph2Zmp));
			m1Y = (uSupport.Y - uTorso1.Y) / (tStep * ph1Zmp);
			m2Y = (uTorso2.Y - uSupport.Y) / (tStep * (1 - ph2Zmp));
			aXP = zmp_solve_aP(uSupport.X, uTorso1.X, uTorso2.X,
					uTorso1.X, uTorso2.X);
			aXN = zmp_solve_aN(uSupport.X, uTorso1.X, uTorso2.X,
					uTorso1.X, uTorso2.X);
			aYP = zmp_solve_aP(uSupport.Y, uTorso1.Y, uTorso2.Y,
					uTorso1.Y, uTorso2.Y);
			aYN = zmp_solve_aN(uSupport.Y, uTorso1.Y, uTorso2.Y,
					uTorso1.Y, uTorso2.Y);
		}
		if (velCurrent.X == 0 && velCurrent.Y == 0 && velCurrent.Z == 0)
		{
			ph = 0;
		}
		xFoot = foot_phase_xf(ph);
		zFoot = foot_phase_zf(ph);
		/*if(initial_step>0) zFoot=0;  // --Don't lift foot at initial step
		 pLLeg[2]=0;  pRLeg[2] = 0;*/
		if (supportLeg == 0)   //-- Left support
		{
			uRight = se2_interpolate(xFoot, uRight1, uRight2);
			pRLeg[2] = stepHeight * zFoot;
		}
		else    //-- Right support
		{

			uLeft = se2_interpolate(xFoot, uLeft1, uLeft2);
			pLLeg[2] = stepHeight * zFoot;
		}

		uTorso = zmp_com(ph);

		uTorsoActual = pose_global(Vector3D(-footX, 0, 0), uTorso);

		pLLeg[0] = uLeft.X;
		pLLeg[1] = uLeft.Y;
		pLLeg[5] = uLeft.Z;
		pRLeg[0] = uRight.X;
		pRLeg[1] = uRight.Y;
		pRLeg[5] = uRight.Z;
		pTorso[0] = uTorsoActual.X;
		pTorso[1] = uTorsoActual.Y;
		pTorso[5] = uTorsoActual.Z;
		qLegs = darwinop_kinematics_inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);
		motion_legs();
		motion_hands();
#ifndef GONG_KONG
		usleep(5000);
#else
		usleep(50000);
#endif
	}
//	cout<<"当		前	 速	度	"<<velCurrent.X<<"   "<<velCurrent.Y<<"      "<<velCurrent.Z<<endl;
	return 0;
}
void delay(int l)
{
	int i, j;
	for (i = l; i > 0; i--)
		for (j = 150; j > 0; j--);
}
void walkkick(Vector3D vel,int flag_leg)//主动改：0左脚1右脚////-1被动机器人自己改
{
	tLastStep = get_time();
	double t = get_time();
	bool go = true;
	int stepNum = 1;
	velCurrent= vel;
//	update_velocity();
	while (go)
	{
		t = get_time();
		T = t - tLastStep;
		ph = (t - tLastStep) / tStep;
		if (ph > 1)
		{
			iStep++;
			ph = ph - 1;
			stepNum = stepNum + 1;
			tLastStep = get_time();
		}
		if (stepNum == 2)
			break;
		  if (iStep > iStep0)
	  {
		iStep0 = iStep;
		if(flag_leg== -1)
		supportLeg =abs(supportLeg-1); //-- 0 for left support, 1 for right support
		else supportLeg =flag_leg;
		uLeft1 = uLeft2;
		uRight1 = uRight2;
		uTorso1 = uTorso2;

//		supportMod[0] = 0; //--Support Point modulation for walkkick
//		supportMod[1] = 0;
		shiftFactor = 0.5; //--How much should we shift final Torso pose?
		if (walkKickRequest > 0)	//--If stop signal sent, put two feet together
			check_walkkick();
		else if(stopRequest==1)	//--Final step
		{
			stopRequest=2;
			if (supportLeg == 0) // Left support
//				uRight2 = step_right_destination(velCurrent, uLeft1, uRight1);
				uRight2 = pose_global(Vector3D(0,-2*footY,0), uLeft1);
			else//-- Right support
//				uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1);
				uLeft2 = pose_global(Vector3D(0,2*footY,0), uRight1);
		}
		else//--Normal walk, advance steps
		{
			tStep=tStep0;
		      if (supportLeg == 0)// Left support
		        uRight2 = step_right_destination(velCurrent, uLeft1, uRight1);
		      else  //-- Right support
		        uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1);

//		      //  --Velocity-based support point modulation
//		      if (velCurrent.X > 0.06)
//				supportMod[0] = supportFront;
//			else if (velCurrent.X < 0)
//				supportMod[0] = supportBack;
//
//			if (velCurrent.Y > 0.015)
//			{
//				supportMod[0] = supportSideX;
//				supportMod[1] = supportSideY;
//			}
//			else if (velCurrent.Y < -0.015)
//			{
//				supportMod[0] = supportSideX;
//				supportMod[1] = -supportSideY;
//			}
		}

		uTorso2 = step_torso(uLeft2, uRight2, shiftFactor);

		//--Apply velocity-based support point modulation for uSupport
		if (supportLeg == 0)  //--LS
		{
			Vector3D uLeftTorso =pose_relative(uLeft1,uTorso1);

			      Vector3D uTorsoModded =pose_global(Vector3D(supportMod[0],supportMod[1],0),uTorso);
				  Vector3D uLeftModded =pose_global(uLeftTorso,uTorsoModded);

			      uSupport =pose_global(Vector3D(supportX, supportY, 0),uLeftModded);//uLeft1);
//			set_leg_hardness();
		}
		else //--RS
		{
			Vector3D uRightTorso = pose_relative(uRight1, uTorso1);
			Vector3D uTorsoModded = pose_global(Vector3D(supportMod[0], supportMod[1], 0), uTorso);
			Vector3D uRightModded = pose_global(uRightTorso, uTorsoModded);
			uSupport = pose_global(Vector3D(supportX, -supportY, 0), uRightModded); //;//uRight1)
//			set_leg_hardness();
		}
		//--Compute ZMP coefficients
		m1X = (uSupport.X - uTorso1.X) / (tStep * ph1Zmp);
		m2X = (uTorso2.X - uSupport.X) / (tStep * (1 - ph2Zmp));
		m1Y = (uSupport.Y - uTorso1.Y) / (tStep * ph1Zmp);
		m2Y = (uTorso2.Y - uSupport.Y) / (tStep * (1 - ph2Zmp));
		aXP = zmp_solve_aP(uSupport.X, uTorso1.X, uTorso2.X,
				uTorso1.X, uTorso2.X);
		aXN = zmp_solve_aN(uSupport.X, uTorso1.X, uTorso2.X,
				uTorso1.X, uTorso2.X);
		aYP = zmp_solve_aP(uSupport.Y, uTorso1.Y, uTorso2.Y,
				uTorso1.Y, uTorso2.Y);
		aYN = zmp_solve_aN(uSupport.Y, uTorso1.Y, uTorso2.Y,
				uTorso1.Y, uTorso2.Y);
//		cout<<"m1X		m2X		m1Y		m2Y		aXP			aXN		aYP			aYN"<<endl;
//		cout<<m1X<<"	"<<m2X<<"	"<<m1Y<<"	"<<m2Y<<"	"<<aXP<<"	"<<aXN<<"	"<<aYP<<"	"<<aYN<<endl<<endl;
	}
	  //--End new step
		if (velCurrent.X == 0 && velCurrent.Y == 0 && velCurrent.Z == 0)
		{
			ph = 0;
		}
		xFoot = foot_phase_xf_kick(ph);
		zFoot = foot_phase_zf_kick(ph);
		/*if(initial_step>0) zFoot=0;  // --Don't lift foot at initial step
		 pLLeg[2]=0;  pRLeg[2] = 0;*/
		if (supportLeg == 0)   //-- Left support
		{
			uRight = se2_interpolate(xFoot, uRight1, uRight2);
			pRLeg[2] = stepHeight * zFoot;
		}
		else    //-- Right support
		{

			uLeft = se2_interpolate(xFoot, uLeft1, uLeft2);
			pLLeg[2] = stepHeight * zFoot;
		}

		uTorso = zmp_com(ph);

		uTorsoActual = pose_global(Vector3D(-footX, 0, 0), uTorso);

		pLLeg[0] = uLeft.X;
		pLLeg[1] = uLeft.Y;
		pLLeg[5] = uLeft.Z;
		pRLeg[0] = uRight.X;
		pRLeg[1] = uRight.Y;
		pRLeg[5] = uRight.Z;
		pTorso[0] = uTorsoActual.X;
		pTorso[1] = uTorsoActual.Y;
		pTorso[5] = uTorsoActual.Z;
//cout<<uLeft.X<<"          "<<uLeft.Y<<"       "<<uLeft.Z<<endl;
//cout<<uRight.X<<"          "<<uRight.Y<<"       "<<uRight.Z<<endl<<endl;;
		qLegs = darwinop_kinematics_inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);
		{
//	  serial_head();
//  waitTime(0.01);
			motion_legs();
//			serial_legs();
//motion_hands();
		}
#ifndef GONG_KONG
		usleep(5000);
#else
		usleep(50000);
#endif
	}
//	cout<<"踢球     当		前	 速	度	"<<velCurrent.X<<"   "<<velCurrent.Y<<"      "<<velCurrent.Z<<endl;
}

//动头, 参数：角度。 下：-90～90, 上：0～90
void move_head(double lower, double upper)
{
	head_angle[0] = min(max(-100.0, lower), 100.0);
	head_angle[1] = min(max(0.0, upper), 100.0);

	robot_info.q0Real = head_angle[0];
    //cout<<"q0Realz"<<robot_info.q0Real<<endl;
	angleOfHead[0] = (int)((head_angle[0] + 100.0) * 424*4 / 300.0 -80 + 0.5);
	angleOfHead[1] =  (int)((100-head_angle[1]) * 600/ 100.0 + 700 + 0.5);
	//serial_head();
}
