/*
* \brief
* a simple tcp server and serial port read/write
*
* \copyright
* Copyright (C) MOXA Inc. All rights reserved.
* This software is distributed under the terms of the
* MOXA License. See the file COPYING-MOXA for details.
*
* \date 2021/03/03
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
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
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
#define FLAGS    0

#define LOWLEVEL 2000
#define HIGHLEVEL 4500
#define ADDLEVEL 1500
/*****************************************************************************
 * Private function declaration
 ****************************************************************************/

/*****************************************************************************
 * Public variables
 ****************************************************************************/
unsigned int g_level = 0;
/*****************************************************************************
* Public functions
****************************************************************************/
int write_all(int socket, char *buf, int len)
{
    int total = 0, send_bytes;
    int bytesleft = len;
    while (bytesleft > 0)
    {
        send_bytes = write(socket, buf + total, bytesleft);
        total += send_bytes;
        bytesleft -= send_bytes;
    }
    
    return total > 0 ? total:-1;
}

int read_all(int socket, char *buf, int len)
{
    int total = 0, recv_bytes;
    int bytesleft = len;
    while (bytesleft > 0)
    {
        recv_bytes =read(socket, buf + total, bytesleft);
        total += recv_bytes;
        bytesleft -= recv_bytes;
        printf("recv_bytes %d total %d in readall\n", recv_bytes, total);
    }
    
    return total > 0 ? total:-1;
}


int serial_open_port(void)
{
    int serial_socket;

    serial_socket = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY); /* open port and set socket non-blocking */

    //serial_socket = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY );

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

int serial_set_port(int serial_socket)
{
    struct termios options;

    /* get current socket option */
    if (tcgetattr(serial_socket, &options) == -1)
    {
        perror("tcgetattr");
        return -1;
    }

    /* set baud rate 19200 */
    cfsetspeed(&options, B921600); //B921600 B115200

    /* set 8N1 and no parity */
    options.c_cflag &= ~PARENB;  /* disable parity */
    options.c_cflag &= ~CSTOPB;  /* set 1 stop bit */
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      /* set 8 data bit */

    /* Enable RTS/CTS hardware flow control */
    options.c_cflag |= CRTSCTS;
    //options.c_iflag |= CRTSCTS;  
    //options.c_oflag |= CRTSCTS;
    //options.c_iflag |= (IXON | IXOFF | IXANY);
    //options.c_oflag |= (IXON | IXOFF | IXANY);

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

int tcp_server_setting()
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
    /* clear the fds in master and read_fds set */
    FD_ZERO(&master);
    FD_ZERO(&read_fds);

    //memset(&before_serial, 0 sizeof(before_serial));

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

    /* open serial port */
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
        read_fds = master; /* update read_fds */

        if (select(fd_max + 1, &read_fds, NULL, NULL, NULL) == -1) /* listen connection whether is ready */
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
            if(g_level < HIGHLEVEL)
            {
                start = clock();
                recv_bytes = read(new_fd, before_serial, ADDLEVEL);
                end = clock();
                //printf("read from client time = %f\n", cpu_time = ((double)(end-start)) / CLOCKS_PER_SEC);
                if (recv_bytes <= 0)
                {
                    perror("recv");
                    close(new_fd);
                    FD_CLR(new_fd, &master);
                }
                else
                {
                    printf("recv %d from client\n", recv_bytes);
                    start = clock(); t1 = clock();
                    send_bytes = write(uport_fd, before_serial, ADDLEVEL);
                    end = clock();
                    //printf("write serial time = %f\n", cpu_time = ((double)(end-start)) / CLOCKS_PER_SEC);
                    if (send_bytes > 0)  
                    {
                        g_level += send_bytes; 
                        printf("send %d to serial\n", send_bytes);
                        printf("glevel = %d\n", g_level);
                    }
                }
            }
        }

        if (FD_ISSET(uport_fd, &read_fds)) 
        {
            if(g_level >= LOWLEVEL)
            {
                memset(after_serial, 0, sizeof(after_serial));
                start = clock();
                recv_bytes = read(uport_fd, after_serial, sizeof(after_serial));
                end = clock(); t2 = clock();
                //printf("read from serial time = %f\n", cpu_time = ((double)(end-start)) / CLOCKS_PER_SEC);
                //printf("from serial write to serial read = %f\n", cpu_time = ((double)(t2 - t1)) / CLOCKS_PER_SEC);
                if (recv_bytes > 0)
                {
                    printf("recv %d from serial\n", recv_bytes);
                    start = clock();
                    send_bytes = write(new_fd, after_serial, strlen(after_serial));
                    end = clock();
                    //printf("write to client time = %f\n\n", cpu_time = ((double)(end-start)) / CLOCKS_PER_SEC);
                    if (send_bytes > 0)
                    {
                        g_level -= send_bytes; 
                        printf("send %d to client\n\n", send_bytes);
                        printf("glevel = %d\n", g_level);
                    }
                    int tt;printf("tt");scanf("%d",&tt);
                }
            }
        }
    }

    close(uport_fd);
    close(listener);
    return 0;
}