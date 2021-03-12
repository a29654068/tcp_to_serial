/*
* \brief
* a simple tcp server and serial port read/write
*
* \copyright
* Copyright (C) MOXA Inc. All rights reserved.
* This software is distributed under the terms of the
* MOXA License. See the file COPYING-MOXA for details.
*
* \date 2021/03/12
* First release
* \author Alan Lan
*/

/*****************************************************************************
* Include files
****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>   
#include <errno.h>   
#include <termios.h> 
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <time.h>

/*****************************************************************************
* Private types/enumerations/variables/define
****************************************************************************/
#define SERVER_PORT 1234
#define BUFSIZE 6000 
#define BACKLOG 10
#define PROTOCOL 0

/* define for level */
#define LOWLEVEL 2600 
#define HIGHLEVEL 4500 
#define ADDLEVEL 1850

/*****************************************************************************
 * Private function declaration
 ****************************************************************************/
static int serial_open_port(void);
static int serial_set_port(int serial_socket);
static int tcp_server_setting();
/*****************************************************************************
 * Public variables
 ****************************************************************************/
unsigned int g_level = 0;

/*****************************************************************************
 * Private functions
 ****************************************************************************/
/**
 *	\brief	        open the serial port.
 *	\param[in]	None	
 *
 *	\return         socket number
 *
 *	This function open th serial port and set some options.
 */
static int serial_open_port(void)
{
    int serial_socket;

    serial_socket = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY); /* open port and set socket non-blocking */

    if (serial_socket == -1)
    {
        perror("open_port: Unable to open /dev/ttyf1 - ");
    }
    else
    {
        fcntl(serial_socket, F_SETFL, 0); 
    }

    return serial_socket;
}

/**
 *	\brief	        set the serial port option.
 *	\param[in]	serialsocket	
 *
 *	\return 0       set successfully
 *  \return -1      set failed
 *
 *	This function set th serial port baud rate, flow control, and other parameter configuration about serial.
 */
static int serial_set_port(int serial_socket)
{
    struct termios options;

    /* get current socket option */
    if (tcgetattr(serial_socket, &options) == -1)
    {
        perror("tcgetattr");
        return -1;
    }

    /* set baud rate 921600 */
    cfsetspeed(&options, B921600);

    /* set 8N1 and no parity */
    options.c_cflag &= ~PARENB;  /* disable parity */
    options.c_cflag &= ~CSTOPB;  /* set 1 stop bit */
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      /* set 8 data bit */

    /* Enable RTS/CTS hardware flow control */
    options.c_cflag |= CRTSCTS;

    /* serial socket set raw I/O */
    options.c_cflag |= (CLOCAL | CREAD);  /* Enable the receiver and set local mode */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* choose raw input */
    options.c_oflag &= ~OPOST;  /* choose raw output */

    options.c_cc[VMIN] = 0;  /* read doesn't block */
    options.c_cc[VTIME] = 10; /* 0.1s read timeout */

    /* Set the new options for the port */
    if (tcsetattr(serial_socket, TCSANOW, &options) == -1)
    {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}

/**
 *	\brief	            set the tcp server.
 *	\param[in]	None	
 *
 *	\return listener    set successfully and return listener socket number
 *  \return -1          set failed
 *
 *	This function set the tcp server.
 */
static int tcp_server_setting()
{
    int listener, optval = 1;
    struct sockaddr_in server_info;

    memset(&server_info, 0, sizeof(server_info));
    server_info.sin_family = AF_INET;
    server_info.sin_addr.s_addr = INADDR_ANY;
    server_info.sin_port = htons(SERVER_PORT);

    if ((listener = socket(AF_INET, SOCK_STREAM, PROTOCOL)) == -1)
    {
        perror("socket");
        return -1;
    }

    if (setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1)
    {
        perror("set socket");
        return -1;
    }

    if (bind(listener, (struct sockaddr *)&server_info, sizeof(server_info)) == -1)
    {
        perror("bind");
        return -1;
    }

    return listener;
}
/*****************************************************************************
* Public functions
****************************************************************************/


int main()
{
    clock_t start, end, t1, t2;
    double cpu_time;
    int listener, new_fd, uport_fd, fd_max, recv_bytes, send_bytes;
    fd_set master, read_fds, write_fds;
    struct sockaddr_in client_info;
    socklen_t client_info_size = sizeof(client_info);

    char client_ip[INET_ADDRSTRLEN], before_serial[BUFSIZE];
    char after_serial[BUFSIZE];
    
    /* clear the fds in master, write_fds read_fds set */
    FD_ZERO(&master);
    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);

    /* setting the tcp server */
    if ((listener = tcp_server_setting()) == -1)
    {
        printf("server setting fail\n");
        return -1;
    }

    if (listen(listener, BACKLOG) == -1)
    {
        perror("listen");
        return -1;
    }

    printf("Server is listening\n");

    /* open and set serial port */
    if ((uport_fd = serial_open_port()) == -1)
    {
        return -1;
    }

    if (serial_set_port(uport_fd) == -1)
    {
        return -1;
    }

    /* update master set and fd_max */
    FD_SET(listener, &master);
    FD_SET(uport_fd, &master);

    fd_max = (listener > uport_fd) ? listener : uport_fd;

    while (1)
    {
        /* write_fds = master; */
        read_fds = master; /* update read_fds */

        if (select(fd_max + 1, &read_fds, &write_fds, NULL, NULL) == -1) /* listen connection whether is ready */
        {
            perror("select");
            return -1;
        }


        if (FD_ISSET(listener, &read_fds)) /* if the socket in read_fd */
        {
            /* if the socket is listener, handle new connection */
            
            if ((new_fd = accept(listener, (struct sockaddr *)&client_info, &client_info_size)) == -1) /* accept new connection */
            {
                perror("accept");
                return -1;
            }
            else
            {
                FD_SET(new_fd, &master); /* update master set */

                if (new_fd > fd_max)
                {
                    fd_max = new_fd;
                }

                printf("new connection from %s on socket %d\n", inet_ntop(AF_INET,
                        (struct sockaddr *)&client_info.sin_addr, client_ip, sizeof(client_ip))
                    , new_fd);
            }
            
        }

        if (FD_ISSET(new_fd, &read_fds)) 
        {
            if(g_level < HIGHLEVEL) /* if level is lower than HIGHLEVEL, starting read data from client */
            {
                /* start = clock();  
                /* start, end is used to test the API execution time */

                recv_bytes = read(new_fd, before_serial, ADDLEVEL);

                /* end = clock();
                /* printf("read from client time = %f\n", cpu_time = ((double)(end-start)) / CLOCKS_PER_SEC); */

                if (recv_bytes <= 0)
                {
                    perror("recv");
                    close(new_fd);
                    FD_CLR(new_fd, &master);
                }
                else
                {
                    /* printf("recv %d from client\n", recv_bytes);     test how many data read from client 
                    /* start = clock(); if(tt == 0){t1=clock(); tt =1;} */

                    send_bytes = write(uport_fd, before_serial, recv_bytes);

                    /* end = clock(); 
                    /* printf("write to serial time = %f\n", cpu_time = ((double)(end-start)) / CLOCKS_PER_SEC); */

                    if (send_bytes > 0)  /* add the water level */
                    {
                        g_level += send_bytes; 

                        /* printf("send %d to serial\n", send_bytes);   test how many data send to serial
                        /* printf("glevel = %d\n", g_level); */
                    }
                }
            }
        }

        if (FD_ISSET(uport_fd, &read_fds)) 
        {
            if(g_level >= LOWLEVEL) /* if level is higher than LOWLEVEL, starting read from serial */
            {
                /*start = clock(); */

                recv_bytes = read(uport_fd, after_serial, sizeof(after_serial));

                /* end = clock(); t2 = clock();tt = 0;
                /* printf("read from serial time = %f\n", cpu_time = ((double)(end-start)) / CLOCKS_PER_SEC);
                /* printf("from serial write to serial read = %f\n", cpu_time = ((double)(t2 - t1)) / CLOCKS_PER_SEC); */

                if (recv_bytes > 0)
                {
                    /* printf("recv %d from serial\n", recv_bytes);      test how many data read from serial
                    /* start = clock(); */

                    send_bytes = write(new_fd, after_serial, recv_bytes);

                    /* end = clock();
                    /* printf("write to client time = %f\n\n", cpu_time = ((double)(end-start)) / CLOCKS_PER_SEC); */

                    if (send_bytes > 0) /* reduce the water level */
                    {
                        g_level -= send_bytes; 

                        /* printf("send %d to client\n\n", send_bytes);  test how many data send to client
                        /* printf("glevel = %d\n", g_level); */
                    }
                    /* blocking the processing to analyze/debug 
                    /* int tt;printf("tt");scanf("%d",&tt); */
                }
            }
        }
    }

    close(uport_fd);
    close(listener);
    return 0;
}