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

/*****************************************************************************
* Private types/enumerations/variables/define
****************************************************************************/
#define SERVER_PORT 1234
#define BUFSIZE 1024
#define BACKLOG 10
#define PROTOCOL 0
#define FLAGS    0
/*****************************************************************************
 * Private function declaration
 ****************************************************************************/

/*****************************************************************************
 * Public variables
 ****************************************************************************/

/*****************************************************************************
* Public functions
****************************************************************************/

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
    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);
//921600
    /* set 8N1 and no parity */
    options.c_cflag &= ~PARENB;  /* disable parity */
    options.c_cflag &= ~CSTOPB;  /* set 1 stop bit */
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      /* set 8 data bit */

    /* serial socket set raw I/O */
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;

    /* set current socket option */
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
    int listener, new_fd, uport_fd, fd_max, recv_bytes;
    fd_set master, read_fds;
    struct sockaddr_in client_info;
    socklen_t client_info_size = sizeof(client_info);

    char client_ip[INET_ADDRSTRLEN], recv_buf[BUFSIZE];
    char recv_buf_serial[BUFSIZE];
    /* clear the fds in master and read_fds set */
    FD_ZERO(&master);
    FD_ZERO(&read_fds);

    //memset(&recv_buf, 0 sizeof(recv_buf));

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

        for (int i = 0; i <= fd_max; i++)
        {
            if (FD_ISSET(i, &read_fds)) /* if the socket in read_fd */
            {
                /* if the socket is listener, handle new connection */
                if (i == listener)
                {
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
                else if (i == uport_fd)
                {
                    /* read data from serial */
                    memset(recv_buf_serial, '\0', sizeof(recv_buf_serial));
                    sleep(1);

                    if ((recv_bytes = read(uport_fd, recv_buf_serial, sizeof(recv_buf_serial))) > 0)
                    {
                        printf("server received '%s' from serial socket %d\n", recv_buf_serial, i); /* server recv data from serial */
                        printf("server send '%s' to client\n\n", recv_buf_serial);

                        if (send(new_fd, recv_buf_serial, sizeof(recv_buf_serial), FLAGS) <= 0)
                        {
                            perror("send");
                        }
                    }
                }
                else
                {
                    /* if the socket is client connection, handle recv from client */
                    if ((recv_bytes = recv(i, recv_buf, sizeof(recv_buf), FLAGS)) <= 0)
                    {
                        if (recv_bytes == 0)  /* handle recv == 0, client close the connection */
                        {
                            printf("server: client socket %d close the connection\n", i);
                        }
                        else
                        {
                            perror("recv");
                        }

                        close(i);
                        FD_CLR(i, &master);
                    }
                    else
                    {
                        printf("server received '%s' from client socket %d\n", recv_buf, i);

                        if (write(uport_fd, recv_buf, sizeof(recv_buf)) == -1)  /* server send data from client to serial */
                        {
                            perror("send");
                        }
                        else
                        {
                            printf("server send '%s' to serial\n\n", recv_buf);
                        }
                    }
                }
            }
        }
    }

    close(uport_fd);
    close(listener);
    return 0;
}