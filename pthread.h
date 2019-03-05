/*
 * pthread.h
 *
 *  Created on: 2014年9月19日
 *      Author: xu
 */

#ifndef PTHREAD_H_
#define PTHREAD_H_

void image_search(void);
void image_chase(void);
void image_approach(void);
void image_round(void);
void image_kick(void);
void image_stand_up(void);

extern int flag_begin;

void *mcu(void *arg);
void *image(void *arg);
void *network(void *arg);
void *process_body(void *arg);
void *process_head(void *arg);
void *process_game_master(void *arg);
void *serial(void *arg);
void *fsm_change(void *arg);
void *location(void *arg);

#endif /* PTHREAD_H_ */
