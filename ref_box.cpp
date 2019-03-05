/*ref_box.cpp
 *
 *  Created on: 2014年9月10日
 *      Author: xuf
 */
#include "global.h"





//massage
char num_players;
char game_state = 3;
char frist_half;
char kickoff_team;
char sec_game_state;
char dropin_team;
short int dropin_time;
short int secs_remaining;
short int secondary_time;
TeamInfo team1;
TeamInfo team2;

//udp广播通讯初始化，参数：端口号
bool udp_broadcast_client_init(uint16_t port)
{
	//建立一个数据报套接字
	if((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1){
		cerr<<"failed to create server sock!";
		return false;
	}
	int en = 1;
	if(setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *)&en, sizeof(en)) < 0){
		cerr<<"failed to set sock";
		return false;
	}
	memset(&addr_to, 0, sizeof(addr_to));//将地址结构清0
	addr_to.sin_family=AF_INET;    //AF_INET===2表示TCP/IP协议
	addr_to.sin_port=htons(port);  //设置端口
//	addr_to.sin_port=port;
	addr_to.sin_addr.s_addr=INADDR_ANY;//任意地址

	//将IP地址与端口号绑定到套接字上,表示服务器用此端口号进行数据的接收与发送
	if((bind(sock, (struct sockaddr*)&addr_to, sizeof(addr_to))) <0){
		cerr<<"failed to bind sock!";
		return false;
	}
//	printf ("Hello  world5555\r\n");
	return true;
}

//解析数据包
void get_info(char *buf_ref)
{
	//num_players = buf_ref[6];
	game_state = buf_ref[7];
/*	frist_half = buf_ref[8];//表明上下半场的变量
	kickoff_team = buf_ref[9];//开球队伍
	sec_game_state = buf_ref[10]; //
	dropin_team = buf_ref[11];
	dropin_time = (buf_ref[13]<<8) + buf_ref[12];//下场时间。只看到往上加 没看到上限
	if ( buf_ref[14]<0)  secs_remaining = ((buf_ref[15]+1)<<8) + buf_ref[14];//剩余时间
	else   secs_remaining = (buf_ref[15]<<8)+ buf_ref[14];
	//cout<<"buf_ref[15]= "<<(int)buf_ref[15]<<endl;
	//cout<<"buf_ref[14]= "<<(int)buf_ref[14]<<endl;


	team1.number = buf_ref[18];
	team1.color = buf_ref[19];
	team1.score = buf_ref[20];
	team1.penalty_shot = buf_ref[21];//点球
	team1.single_shots = (buf_ref[23]<<8) + buf_ref[22];//

	//
	memcpy(team1.SPL_coach_message, &buf_ref[24], 40);//



	team2.number = buf_ref[88];
	team2.color = buf_ref[89];
	team2.score = buf_ref[90];
	team2.penalty_shot = buf_ref[91];
	team2.single_shots = (buf_ref[93]<<8) + buf_ref[92];
	memcpy(team2.SPL_coach_message, &buf_ref[94], 40);


	for(int i=0; i<5; ++i){
		team1.players_info[i].penalty = buf_ref[64+2*i];
		team2.players_info[i].penalty = buf_ref[134+2*i];


		team1.players_info[i].secs_to_unpen = buf_ref[64+2*i+1];
		team2.players_info[i].secs_to_unpen = buf_ref[134+2*i+1];
	}*/
}

//接收gameControlData
int recv_game_control_data(char *buf)
{
	int rec;
	unsigned int client_len;
	client_len = sizeof(addr_from);
	rec = recvfrom(sock, buf, 1023, 0, (struct sockaddr *)&addr_from, &client_len);

//	{
//		if(ret==EWOULDBLOCK||ret==EAGAIN)
//			printf("recvfrom timeout\n");
//		else
//			printf("recvfrom err:%d\n",rec);
//	}
	return rec;
}

//发送RobotReturnData
bool send_return_data_to_gamemaster(char player_num, char message)
{
	return_data_buffer[6] = player_num;
	return_data_buffer[7] = message;
	if(sendto(sock, return_data_buffer, 8, 0, (struct sockaddr *)&addr_from, sizeof(addr_from)) < 0){
		cerr<<"failed to send return data!";
		return false;
	}
	return true;
}

//打印信息,调试用
void info_output(void)
{
	cout<<"num_players = "<<(int)num_players<<endl;
	cout<<"frist_half = "<<(int)frist_half<<endl;
	cout<<"kickoff_team = "<<(int)kickoff_team<<endl;
	cout<<"dropin_team = "<<(int)dropin_team<<endl;
	cout<<"dropin_time = "<<(int)dropin_time<<endl;
	cout<<"secs_remaining = "<<(int)secs_remaining<<endl<<endl;
	cout<<"team1.number = "<<(int)team1.number<<"\t\t"<<"team2.number = "<<(int)team2.number<<endl;
	cout<<"team1.color = "<<(int)team1.color<<"\t\t\t"<<"team2.color = "<<(int)team2.color<<endl;
	cout<<"team1.score = "<<(int)team1.score<<"\t\t\t"<<"team2.score = "<<(int)team2.score<<endl;
	cout<<"team1.penalty_shot = "<<(int)team1.penalty_shot<<"\t\t"<<"team2.penalty_shot = "<<(int)team2.penalty_shot<<endl;
	cout<<"team1.single_shots = "<<(int)team1.single_shots<<"\t\t"<<"team2.single_shots = "<<(int)team2.single_shots<<endl<<endl;
	cout<<"team1.SPL_coach_message = "<<team1.SPL_coach_message<<endl;
	cout<<"team2.SPL_coach_message = "<<team2.SPL_coach_message<<endl;
	//	team1.players_info[i].penalty = buf[64+2*i];
	//team2.players_info[i].penalty = buf[134+2*i];


	//team1.players_info[i].secs_to_unpen = buf[64+2*i+1];
	//team2.players_info[i].secs_to_unpen = buf[134+2*i+1];
	//
	//
	for(int i=0; i<5; ++i)
	{
				cout<<"team1.players_info["<<i<<  "].penalty  = "<<(int)team1.players_info[i].penalty <<endl;
				cout<<"team2.players_info["<<i<<  "].penalty = "<<(int)team2.players_info[i].penalty<<endl;
				cout<<"team2.players_info["<<i<<  "].secs_to_unpen = "<<(int)team1.players_info[i].secs_to_unpen <<endl;
				cout<<"team2.players_info["<<i<<  "].secs_to_unpen = "<<(int)team2.players_info[i].secs_to_unpen <<endl;
	}


	cout<<endl<<endl;

}



//
//int ref_box_init()
//{
//	 udp_broadcast_client_init( 3838);
//	 return 0;
//	}
//





