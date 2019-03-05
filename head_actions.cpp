/*
 * head_actions.cpp
 *
 *  Created on: 2014年9月19日
 *      Author: xuf
 */
#include"global.h"


//找球
void head_search(void)
{
	    double t;
		calc_t_offset();
        cout<<"head_search"<<endl;
		double t0 = get_time();
		while(robot_info.robot_state == search_ball)
		{
			     t = get_time() - t0;
			    if(t > T_lower)
			         {

				           return;
			         }

			move_head(lower_head_fun(t, T_lower), upper_head_fun(t, T_upper));
			delay_sec(0.005);

	   }
}





/*bool search_ball_should_break(void)
{
	if(robot_info.fall_info != 0){
		robot_info.robot_state = stand_up;
		return true;
	}
	cout<<"search robot_info.ball_find = "<<robot_info.ball_find<<endl;
	if(robot_info.ball_find){
		cout<<"search------------------->>>>>>chase"<<endl;
		robot_info.robot_state = chase_ball;
		return true;
	}
	else return false;
}*/







//追球
void head_chase(void)
{

	double angle_change[2];
	int start_count = 0;
	cout<<"head_chase"<<endl;
	while(robot_info.robot_state == chase_ball)
                                        //while(robot_info.ball_dis4robot >= DisMin &&robot_info.ball_find == true&&robot_info.fall_info==0)
	            {

		/*cout<<"robot_info.ball_dis4robot"<<robot_info.ball_dis4robot<<endl;*/
		robot_info.calc_ball_info();
		if(start_count < 2)
		{
			start_count++;
			angle_change[0] = 25 * (robot_info.ball_info.x - 160) / 160 * 0.3;
			angle_change[1] = 21 * (robot_info.ball_info.y - 120) / 120 * 0.2;
			move_head(head_angle[0] - angle_change[0], head_angle[1] - 2*angle_change[1]);
			delay_sec(0.1);
			if(robot_info.ball_find){
				delay_sec(0.2);
			}
			else{
				continue;
			}
		}
		else{
			angle_change[0] = pid_calc_lower(&pid_lower);
			angle_change[1] = pid_calc_upper(&pid_upper);
			}
		move_head(head_angle[0] - angle_change[0], head_angle[1] - 2*angle_change[1]);

			delay_sec(pid_upper.t);
//cout<<"head_upper"<<head_angle[1]-2*angle_change[1]<<endl;
		}
 cout<<"robot_info.ball_dis4robot"<<robot_info.ball_dis4robot<<endl;

        if(!robot_info.ball_find)
                  {
		                  int lost_count = 0;
		                  while( lost_count < 2)
		                                  {
			                                      lost_count++;
			                                      angle_change[0] = pid_calc_lower(&pid_lower) * 1;
			                                      angle_change[1] = pid_calc_upper(&pid_upper) * 1;
			                                      move_head(head_angle[0] - angle_change[0], head_angle[1] - angle_change[1]);
			                                      delay_sec(pid_upper.t);
		                                  }


	             }

	}







int  get_kuang_fangxiang_unknow(int x,int qhead,int angle_court)
{
	double angle_t_1=angle_court-qhead;
	double angle_t_2=1.0*(320-x)*21/320;

	int a_t= (int)(angle_t_1+angle_t_2+5);
	if(a_t>360) a_t-=360;
	else if(a_t<0) a_t=360+a_t;
	return a_t;
}






//检查pid追球状态跳出条件
/*bool chase_ball_should_break(void)
{
	if(robot_info.fall_info != 0){
		robot_info.robot_state = stand_up;
		return true;
	}
	cout<<"chase robot_info.ball_find = "<<robot_info.ball_find<<endl;
	if(!robot_info.ball_find){
		cout<<"chase----------------------->>>>>search"<<endl;
		robot_info.robot_state = search_ball;
		return true;
	}
	if(robot_info.catch_ball_flag != 0){
		robot_info.robot_state = catch_ball;
		return true;
	}
	else return false;
}*/






//根据航向角绕球
void head_round_yaw(void)
{
	     cout<<"head_round_yaw"<<endl;
	      /*cout<<"yaw = "<<robot_info.yaw<<endl;*/
	 	double angle_change[2];
	 		int start_count = 0;
	 		while(robot_info.robot_state == round_yaw)
	 	                                        //while(robot_info.ball_dis4robot >= DisMin &&robot_info.ball_find == true&&robot_info.fall_info==0)
	 		            {

	 			/*cout<<"robot_info.ball_dis4robot"<<robot_info.ball_dis4robot<<endl;*/
	 			robot_info.calc_ball_info();
	 			if(start_count < 2)
	 			{
	 				start_count++;
	 				angle_change[0] = 25 * (robot_info.ball_info.x - 160) / 160 * 0.3;
	 				angle_change[1] = 21 * (robot_info.ball_info.y - 120) / 120 * 0.2;
	 				move_head(head_angle[0] - angle_change[0], head_angle[1] - angle_change[1]);
	 				delay_sec(0.1);
	 				if(robot_info.ball_find){
	 					delay_sec(0.2);
	 				}
	 				else{
	 					continue;
	 				}
	 			}
	 			else{
	 				angle_change[0] = pid_calc_lower(&pid_lower);
	 				angle_change[1] = pid_calc_upper(&pid_upper);
	 				}
	 			move_head(head_angle[0] - angle_change[0], head_angle[1] - angle_change[1]);

	 				delay_sec(pid_upper.t);

	 			}
	 // cout<<"robot_info.ball_dis4robot"<<robot_info.ball_dis4robot<<endl;

	 	        if(!robot_info.ball_find)
	 	                  {
	 			                  int lost_count = 0;
	 			                  while( lost_count < 2)
	 			                                  {
	 				                                      lost_count++;
	 				                                      angle_change[0] = pid_calc_lower(&pid_lower) * 1;
	 				                                      angle_change[1] = pid_calc_upper(&pid_upper) * 1;
	 				                                      move_head(head_angle[0] - angle_change[0], head_angle[1] - angle_change[1]);
	 				                                      delay_sec(pid_upper.t);
	 			                                  }


	 		             }

	 	//move_head(0,0);
	}









//根据球门绕球
//只有同时识别两根门柱的情况，返回２后选择绕门柱或是直接向前踢，
//当前是身体静止，头部匀速两个周期，若需要，可以让头部更慢，一个周期
void head_search_goal(void)
{

	             if(robot_info.fsm_unlocked==true) return;
    	           cout<<"head_search_goal"<<endl;
    				headangle=0;
    				double  angle_of_head[3]={0,0,0};//记录三个视野头部的角度
    				calc_t_offset();
    				robot_info.goal_info.state=0;
    				robot_info.goal_find=0;
    				int count=0;


    				delay_sec(0.5);
    				//头部右摆，当找到两个球门时跳出，找到一个时continue

    				move_head(-60,100);

    			//	cout<<"右摆q0real = "<<robot_info.q0Real<<endl<<endl<<endl;
    					for(int i=0;i<10;i++)
    						{
    						delay_sec(0.1);
    				if(robot_info.goal_info.state>=2)
    				{
    					cout<<"right_find"<<endl;
    					headangle=head_angle[0];
    					robot_info.fsm_unlocked=true;
    					return;

    					}
    				else if(robot_info.goal_info.state==1)
    				{
    					count++;
    					angle_of_head[0]=head_angle[0];
    					break;
    				}

                   }

    				//头部中间摆

    				move_head(0,100);
    				//cout<<"中间q0real = "<<robot_info.q0Real<<endl<<endl<<endl;

    				for(int i=0;i<10;i++)
    			 {
    			    		delay_sec(0.1);

    				if(robot_info.goal_info.state>=2)
    			    				{
    									cout<<"middle_find"<<endl;
    						            headangle=head_angle[1];
    			    					robot_info.fsm_unlocked=true;
    			    					return;

    			    					}
    			    				else if(robot_info.goal_info.state==1)
    			    				{
    			    					count++;
    			    					angle_of_head[1]=head_angle[0];
    			    					break;
    			    				}
    			 }


    				//头部左摆

    				move_head(60,100);
    			//	cout<<"左摆q0real = "<<robot_info.q0Real<<endl<<endl<<endl;

    				for(int i=0;i<10;i++)
    				{
    			    	delay_sec(0.1);

    				if(robot_info.goal_info.state>=2)
    			    				{
    					cout<<"left_find"<<endl;
    									headangle=head_angle[2];
    			    					robot_info.fsm_unlocked=true;
    			    					return;

    			    					}
    			    				else if(robot_info.goal_info.state==1)
    			    				{
    			    					count++;
    			    					angle_of_head[2]=head_angle[0];
    			    					break;
    			    				}
    				}

              	cout<<"count="<<count<<endl;
              	cout<<angle_of_head[0]<<"    "<<angle_of_head[1]<<"    "<<angle_of_head[2]<<endl;
    				//如果左右两个视野中均找到一根门柱，取中间为球门位置
    			if(angle_of_head[2]!=0&&angle_of_head[0]!=0)
    			{

    				headangle=angle_of_head[1];
    				robot_info.fsm_unlocked=true;
    			}

    			//如果只有左边+中间视野或右边+中间视野找到一根门柱，取两个视野角度平均值
    			else if(count==2)
    			{
    				headangle=(angle_of_head[0]+angle_of_head[1]+angle_of_head[2])/2;
    				robot_info.fsm_unlocked=true;
    			}

    			//如果只有一个视野看到一根球门，取该角度
    			else if(count==1)
    			{
    				headangle=angle_of_head[0]+angle_of_head[1]+angle_of_head[2];
    				robot_info.fsm_unlocked=true;
    			}

    			//没看到球门，取当前角度
    			else if(count==0)
    			{
    				cout<<"count=0"<<endl;
    				headangle=0;
    				robot_info.fsm_unlocked=true;
    			}





    		/*		while(robot_info.goal_info.state!=2)//正对球门
    				              {
    					                     t = get_time() - time0;
    					                     if(t > T_lower)
    					                                   {
    					                    	                   cout<<"超时"<<endl;
    					                                            	headangle=0;
    					                                               	robot_info.goal_info.state=2;
    					                                               	robot_info.fsm_unlocked=true;
    						                                            break;
    				                                     	}
    					                     cout<<"head_round_search"<<endl;
    					                      move_head(lower_head_fun(t, T_lower),90 );
    					                      if(robot_info.goal_info.state==2)
												{
															   headangle=head_angle[0];
																robot_info.fsm_unlocked=true;
																break;
												}
    				              }*/

//    					//找到，门柱记录当前位置，找到第二跟门柱，头部保持静止，身体绕球
//    							if(robot_info.goal_info.state==2)
//    						{
////    							 cout<<" goal find  2"<<endl;
//    							headangle=head_angle[0];
//    							break;
//    							}


//    					//绕球完成，状态跳转
//
//    	//			cvCircle(ball_Image,cvPoint(firstgoal_left,200),5,CV_RGB(0,150,50),8);
//    				if(robot_info.goal_info.state==2)
//    				{
//    					robot_info.robot_state = round_goal;
//    				}
////    				else
////    				{
////    					robot_info.robot_state = round_goal;
////    				}
//    					delay_sec(0.005);
    }

 void head_adjust_ball(void)
 {
	 cout<<"head_adjust_ball"<<endl;
	 time0=get_time();
	 while(robot_info.robot_state ==adjust_ball)
		 {

			double	 t=get_time()-time0;
		                    move_head(lower_head_fun(t, 4),8);
	              }

 }


void head_round_goal(void)
{
	cout<<"head_round_goal"<<endl;
	double angle_change[2];
		int start_count = 0;
		while(robot_info.robot_state == round_goal)
	                                        //while(robot_info.ball_dis4robot >= DisMin &&robot_info.ball_find == true&&robot_info.fall_info==0)
		            {

			/*cout<<"robot_info.ball_dis4robot"<<robot_info.ball_dis4robot<<endl;*/
			robot_info.calc_ball_info();
			if(start_count < 2)
			{
				start_count++;
				angle_change[0] = 25 * (robot_info.ball_info.x - 160) / 160 * 0.3;
				angle_change[1] = 21 * (robot_info.ball_info.y - 120) / 120 * 0.2;
				move_head(head_angle[0] - angle_change[0], head_angle[1] - angle_change[1]);
				delay_sec(0.1);
				if(robot_info.ball_find){
					delay_sec(0.2);
				}
				else{
					continue;
				}
			}
			else{
				angle_change[0] = pid_calc_lower(&pid_lower);
				angle_change[1] = pid_calc_upper(&pid_upper);
				}
			move_head(head_angle[0] - angle_change[0], head_angle[1] - angle_change[1]);

				delay_sec(pid_upper.t);

			}
	 cout<<"robot_info.ball_dis4robot"<<robot_info.ball_dis4robot<<endl;

	        if(!robot_info.ball_find)
	                  {
			                  int lost_count = 0;
			                  while( lost_count < 2)
			                                  {
				                                      lost_count++;
				                                      angle_change[0] = pid_calc_lower(&pid_lower) * 1;
				                                      angle_change[1] = pid_calc_upper(&pid_upper) * 1;
				                                      move_head(head_angle[0] - angle_change[0], head_angle[1] - angle_change[1]);
				                                      delay_sec(pid_upper.t);
			                                  }


		             }

	//move_head(0,0);
	}


void head_approach_ball(void)
{
	cout<<" approach_ball_ head"<<endl;
	int start_count = 0;
	double t;
	double angle_change[2];
	double t0=get_time();
	move_head(0,10);
	 while(robot_info.robot_state ==approach_ball )
	 {
		 if(robot_info.ball_find!=true)

              {
			  t = get_time() - t0;
						    if(t >8)
						         {

							           return;
						         }

						move_head(lower_head_fun(t, 8), 5);
						delay_sec(0.005);

              }
		 else
               {
			 if(start_count < 2)
					                       {
						                                 start_count++;
						                                 angle_change[0] = 25 * (robot_info.ball_info.x - 160) / 160 * 0.3;
						                                 angle_change[1] = 21 * (robot_info.ball_info.y - 120) / 120 * 0.2;
					                         	         move_head(head_angle[0] - angle_change[0], head_angle[1] - angle_change[1]);
						                                 delay_sec(0.1);
					        	                         if(robot_info.ball_find)
					        	                                 {
							                                                  delay_sec(0.2);
						                                          }
						                               else
						                                         {
							                                                        continue;
						                                          }
				                         	}
					             else
					                       {
				    		                              angle_change[0] = pid_calc_lower(&pid_lower);
					    	                              angle_change[1] = pid_calc_upper(&pid_upper);
					                     move_head(head_angle[0] - angle_change[0], head_angle[1] - angle_change[1]);
					                       }

					           delay_sec(pid_upper.t);
				            }
	          }

}


void head_kick(void)
{
	//cout<<"head_kick"<<endl;
move_head(robot_info.q0Real,90);
}






