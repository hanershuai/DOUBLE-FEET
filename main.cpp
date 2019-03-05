/*
 * main.cpp
 *
 *  Created on: 2014年6月23日
 *      Author: xuf
 */
#include <iostream>
#include "global.h"
#include <stdlib.h>
#include "basicwalk.h"

int main()
{


//	robot_info.yaw_start=-70;//初始电子罗盘0角与场地0`角的差值  南球门（对着11出口）
		//robot_info.yaw_start=162;//北球门
	//euler_init();
	Image_Init();
//udp_broadcast_client_init(3838);
//server_sock_init(&client_sock, 8888, "192.168.1.189");
#ifdef GONG_KONG
	fd = portInit("/dev/ttyS0");
#else
    //fd = portInit("/dev/ttyUSB0");					//调试时注意更改初始化的串口
    //fd_arm=portInit_MCU("/dev/ttyUSB1");
    //robot_info.yaw_start=get_yaw();
#endif

    update_still();
    delay_sec(1);


        //if(pthread_create(&mcu_t, NULL, mcu, NULL) != 0)
		cout<<"mcu pthread failed"<<endl;
		cout<<"mcu pthread succeeded"<<endl;

    	/*if(pthread_create(&game_master_t , NULL, process_game_master, NULL) != 0)
    	cout<<"process game_master pthread failed"<<endl;
    	cout<<"process game_master pthread succeeded"<<endl;*/
        if(pthread_create(&image_t, NULL, image, NULL) != 0)
		cout<<"image pthread failed"<<endl;
		cout<<"image pthread succeeded"<<endl;

  	    //if(pthread_create(&serial_t, NULL, serial, NULL) != 0)
		cout<<"serial pthread failed"<<endl;
		cout<<"serial pthread succeeded"<<endl;

    	//if(pthread_create(&fsm_t, NULL, fsm_change, NULL) != 0)
    	cout<<"serial pthread failed"<<endl;
    	cout<<"serial pthread succeeded"<<endl;

		//if(pthread_create(&head_t, NULL, process_head, NULL) != 0)
		cout<<"process head pthread failed"<<endl;
		cout<<"process head pthread succeeded"<<endl;

		//if(pthread_create(&body_t, NULL, process_body, NULL) != 0)
		cout<<"process body pthread failed"<<endl;
		cout<<"process body pthread succeeded"<<endl;




		void *pthread_result;
		//pthread_join(mcu_t,&pthread_result);
        pthread_join(image_t, &pthread_result);
		//pthread_join(serial_t, &pthread_result);
		//pthread_join(body_t, &pthread_result);
		//pthread_join(fsm_t, &pthread_result);
		//pthread_join(head_t, &pthread_result);
	//	pthread_join(game_master_t, &pthread_result);



	return 0;
}



