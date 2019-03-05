/*
 * tcp.h
 *
 *  Created on: 2015年7月5日
 *      Author: xu
 */

#ifndef __TCP_H__
#define __TCP_H__

#include <stddef.h>

extern int client_sock;

bool server_sock_init(int * client_sock, int port, const char *ip_address = NULL);

#endif


