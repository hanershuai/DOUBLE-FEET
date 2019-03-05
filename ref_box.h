/*ref_box.h
 *
 *  Created on: 2014年9月10日
 *      Author: xuf
 */

#ifndef REF_BOX_H_
#define REF_BOX_H_

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

#include "global.h"


using namespace std;



bool udp_broadcast_client_init(uint16_t port);
void get_info(char *buf);
void info_output(void);
int recv_game_control_data(char *buf);
bool send_return_data_to_gamemaster(char player_num, char message);
//int ref_box_init()
#endif /* UDP_H_ */
