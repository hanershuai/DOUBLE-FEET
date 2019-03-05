/*
 * tcp.cpp
 *
 *  Created on: 2015年7月5日
 *      Author: xu
 */

#include "tcp.h"
#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

//套接字初始化， client_sock:输出参数，用于通信的客户端套接字，  prot:端口号，  ip_address:ip地址，默认为NULL表示接收任意网络的连接
//成功则返回true, 失败则返回false
bool server_sock_init(int * client_sock, int port, const char *ip_address)
{
	int server_sock_fd;
	struct sockaddr_in server_sock_address;
	int en = 1;
	//创建未命名套接字
	if((server_sock_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
		std::cout<<"failed to create server socket."<<std::endl;
		return false;
	}

	if (setsockopt(server_sock_fd, SOL_SOCKET, SO_REUSEADDR, &en, sizeof (int)) == -1)
		std::cout<<"failed to set reused address."<<std::endl;

	server_sock_address.sin_family = AF_INET;
	if(ip_address == NULL)
		server_sock_address.sin_addr.s_addr = INADDR_ANY;		//允许来自任何网络接口的链接
	else
		server_sock_address.sin_addr.s_addr = inet_addr(ip_address);
	server_sock_address.sin_port = htons (port);		//将主机字节序转化为网络字节序，保证端口号正确
	memset (&(server_sock_address.sin_zero), '\0', 8);	//填充无意义的字节

	//命名套接字
	if (bind(server_sock_fd, (struct sockaddr *) &server_sock_address, sizeof (struct sockaddr)) == -1){
		std::cout<<"failed to bind server socket."<<std::endl;
		return false;
	}

	//创建监听队列
	if (listen (server_sock_fd, 5) == -1){
		std::cout<<"failed to listen server socket."<<std::endl;
		return false;
	}

	//等待一个客户端连接
	struct sockaddr_in client_address;
	socklen_t client_length = sizeof(struct sockaddr_in);
	std::cout<<"waiting connect..."<<std::endl;
	if((*client_sock = accept(server_sock_fd, (struct sockaddr *)&client_address, &client_length)) == -1){
		std::cout<<"failed to accept a connect."<<std::endl;
		return false;
	}
	std::cout<<"connected from "<<inet_ntoa(client_address.sin_addr)<<std::endl;
	return true;
}


