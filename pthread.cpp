/*
 * pthread.cpp
 *
 *  Created on: 2014年9月19日
 *      Author: xuf
 */

#include "global.h"
/************************************线程*********************************************/
//mcu ptread
void *mcu(void *arg)
{
    while(1)
    {
        robot_info.fall_info=get_fall_info();
        robot_info.yaw=get_yaw();
        usleep(1);
    }
    return (void *)"mcu pthread return!";
}
//图像线程
void *image(void *arg)
{
	while(1)
	{
		fsm_attack_image();

		usleep(1);
		char c = cvWaitKey(1);
		if(c == 27 )
		{
			Image_Clear();
			exit(0);
		}
	}
	return (void *)"image pthread return!";
}

//身体动作线程
void *process_body(void *arg)
{
	while(1)
	{
		fsm_attack_body();
		usleep(1);
	}
	return (void *)"body process pthread finish.";
}
//头部动作线程
void *process_head(void *arg)
{

	while(1)
	{
	fsm_attack_head();
	usleep(1);
	}
	return (void *)"head process pthread finish.";
}

//裁判盒线程
void * process_game_master(void *arg)
{
		while (1)
		{
			 recv_game_control_data(get_ref_buf );
			 get_info(get_ref_buf );

			 usleep(1);
		}


	return (void *)"game master pthread finish.";
}

//串口线程 发送步态数据
void *serial(void *arg)
{
	while(1){
	if(send_serial_flag ==false)
	{
		serial_head();
		delay_sec(0.005);
		serial_legs();
		delay_sec(0.005);
		serial_hands();
		delay_sec(0.005);

	}
//	image_update();

		usleep(1);
	}

	return (void *)"serial pthread finish.";
}








void *fsm_change(void *arg)
{


	while(1)
		{

			if(robot_info.fall_info==1||robot_info.fall_info==2)
			{
				delay_sec(0.5);

				if(robot_info.fall_info==1||robot_info.fall_info==2)
						{
				robot_info.robot_state = climb_up;
						}
			}
			else
			{
                    //robot_info.robot_state=round_yaw;
					   switch(robot_info.robot_state)
						{
								case search_ball:
								    //robot_info.robot_state=chase_ball;
												 if(robot_info.ball_find == true)
														 {
															robot_info.robot_state = chase_ball;
															robot_info.fsm_unlocked = false;
//															  cout<<"robot_info.fsm_unlocked 1= "<<robot_info.fsm_unlocked<<endl;

															}
											  break;
								case chase_ball:
								    //robot_info.robot_state=round_yaw;
                                    //cout<<"Chase_ball"<<endl;
													if(!robot_info.ball_find)
															{

																  robot_info.robot_state = search_ball;
	//															  cout<<"robot_info.fsm_unlocked 2= "<<robot_info.fsm_unlocked<<endl;
						//							  	  	  	  	 robot_info.ball_dis4robot = 50;

															  }
													  if(robot_info.ball_find&&(robot_info.ball_dis4robot <= DisMin1))
															  {
									  	  	  	  		            time0=get_time();
																	robot_info.robot_state =round_yaw;

	//																  cout<<"robot_info.fsm_unlocked 3= "<<robot_info.fsm_unlocked<<endl;

															  }
											break;
							   case round_yaw:
                                                    //robot_info.robot_state =search_goal;
								                    //cout<<"robot_info.yaw="<<robot_info.yaw<<endl;
													if((robot_info.yaw<=20.00&&robot_info.yaw>=-20.00))
															{

																robot_info.robot_state =search_goal;
		//														  cout<<"robot_info.fsm_unlocked 4= "<<robot_info.fsm_unlocked<<endl;
                                                                //cout<<"break and enter search goal"<<endl;
															 }
													else if((get_time()-time0)>30)
													{
														robot_info.robot_state =search_goal;
													}



											 break;
							  case search_goal:
							      //cout<<"Search_goal"<<endl;
	//`							  robot_info.fsm_unlocked = false;
	//							  cout<<"robot_info.fsm_unlocked 5= "<<robot_info.fsm_unlocked<<endl;
													  if(robot_info.fsm_unlocked==true)
															 {
														        time0=get_time();
																 robot_info.robot_state = adjust_ball;
															//	 cout<<"robot_info.robot_state="<<robot_info.robot_state<<endl;
																 robot_info.fsm_unlocked=false;
																 robot_info.ball_find=false;
															   }
											break;
							  case adjust_ball:
							      //cout<<"Adjust_ball"<<endl;
                                                //robot_info.robot_state=round_goal;
								  	  	  	  	  if(robot_info.ball_find==true)
								  	  	  	 {
								  	  	  	  		  time0=get_time();
								  	  	  		 robot_info.robot_state = round_goal;

								  	  	  	 }
								  	  	  	  	  else if((get_time()-time0)>8)
								  	  	  	  	  {

						//		  	  	  	  	 robot_info.ball_dis4robot = 50;
								  	  	  	  	robot_info.robot_state =search_ball;
								  	  	  	  	  }


								  	  	  break;
							  case round_goal:
                                                        time0=get_time();
                                                        robot_info.robot_state =approach_ball;
                                    //cout<<"Round_goal"<<endl;
							  	  	  	  	  	  	  {

														double Z= 6* PI / 180;
//														cout<<"angle="<<fabs((get_time()-time0)*Z-fabs(double(headangle))*PI/180)<<endl;
													  if(((get_time()-time0)*Z-fabs(double(headangle))*PI/180)>=0*PI/180)
															   {
					//									         robot_info.ball_dis4robot = 50;
																  robot_info.robot_state = approach_ball;
																  time0=get_time();

																 }

							  	  	  	  	  	  	  	  }
											break;
							  case  approach_ball:
							  {
	//							  cout<<"time="<<get_time() - time0<<endl;
	                              //cout<<"Approach_ball"<<endl;
												  if( (get_time() - time0)>10)
													 {

													  	  	  	  	  	  	  if(robot_info.ball_find!=true)
																	 	 	 {
								//												   robot_info.ball_dis4robot = 50;
																				   robot_info.robot_state = search_ball;
																				 }
													  	  	  	  	  	  	  else
																			 {
																				   robot_info.robot_state = kick_ball;
																			 }
													 }


												  else  if(abs(int(robot_info.q0Real))<=20&&robot_info.ball_dis4robot <=DisMin)
													 {
																							 robot_info.robot_state = kick_ball;
													 }

							  }
										   break;

							  case  kick_ball:
								  	  	  	  	  	  	  	  	  	  if(robot_info.fsm_unlocked==true)
								  	  	  	  	  	  	  	  	  	  {
                                                                         robot_info.robot_state = search_ball;
																		 robot_info.fsm_unlocked=false;
																		 robot_info.ball_find =false;
																		 robot_info.ball_dis4robot = 50;
																		 delay_sec(1);

								  	  	  	  	  	  	  	  	  	  }
								  	  	  	  	  	  	  	  	  	  //if(robot_info.fsm_unlocked==true)
								  	  	  	  	  	  	  	  	  	  //body_stop();
								  	  	  break;

							  case climb_up:

								                                if(robot_info.fall_info==0)
								                                {
								                                	delay_sec(0.5);
								                                	if(robot_info.fall_info==0)
								                                	{
								                                	robot_info.robot_state = search_ball;
                                                                    robot_info.ball_dis4robot = 50;

								                               }
								                                }
								           break;



							  default:
										   break;
							}

					}

				usleep(1);
		}
return (void *)"fsm_change pthread finish.";
}
