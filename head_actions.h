/*
 * head_actions.h
 *
 *  Created on: 2014年9月19日
 *      Author: xu
 */

#ifndef HEAD_ACTIONS_H_
#define HEAD_ACTIONS_H_

void body_stop(void);
void head_search(void);
void head_chase(void);
void head_round_yaw(void);
void head_search_goal(void);
void head_adjust_ball(void);
void head_round_goal(void);
void head_kick(void);
void head_approach_ball(void);
bool search_ball_should_break(void);
bool chase_ball_should_break(void);

#endif /* HEAD_ACTIONS_H_ */
