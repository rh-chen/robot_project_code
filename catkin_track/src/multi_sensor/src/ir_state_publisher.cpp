
#ifndef _SERIPORT_H
#define _SERIPORT_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h> //set baud rate
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>

#include <errno.h>

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/Range.h"
#include <tf/transform_broadcaster.h>



#define buffLen 1024
#define rcvTimeOut 2

#endif
/*
 * Created by Kalman on 16/12/9
 */

/* A imprecise delay funtion */
void delay(int ms)
{
    while(ms--)
    {
        int i=10000;
        while(i--);
    }
}

/* Open the serial port */

int openPort(int fd, int comport)
{

    if (comport == 1)
    {
        fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ttyS0 .....\n");
        }
    }
    else if (comport == 2)
    {
        fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ttyS1 .....\n");
        }
    }
    else if (comport == 3)
    {
        fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ttyS2 .....\n");
        }
    }
    /*************************************************/
    else if (comport == 4)
    {
        fd = open("/dev/ttyUSB2", O_RDWR | O_NOCTTY | O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ttyUSB0 .....\n");
        }
    }

    if (fcntl(fd, F_SETFL, 0)<0)
    {
        printf("fcntl failed!\n");
    }
    else
    {
        printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
    }
    if (isatty(STDIN_FILENO) == 0)
    {
        printf("standard input is not a terminal device\n");
    }
    else
    {
        printf("is a tty success!\n");
    }
    printf("fd-open=%d\n", fd);
    return fd;
}

int setOpt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (nBits)
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }

    switch (nEvent)
    {
        case 'O':                     //奇校验
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':                     //偶校验
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':                    //无校验
            newtio.c_cflag &= ~PARENB;
            break;
    }

    switch (nSpeed)
    {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }
    if (nStop == 1)
    {
        newtio.c_cflag &= ~CSTOPB;
    }
    else if (nStop == 2)
    {
        newtio.c_cflag |= CSTOPB;
    }
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}

int readDataTty(int fd, char *rcv_buf, int TimeOut, int Len)
{
    int retval;
    fd_set rfds;
    struct timeval tv;
    int ret, pos;
    tv.tv_sec = TimeOut / 1000;  //set the rcv wait time
    tv.tv_usec = TimeOut % 1000 * 1000;  //100000us = 0.1s

    pos = 0;
    while (1)
    {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        retval = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (retval == -1)
        {
            perror("select()");
            break;
        }
        else if (retval)
        {
//            tcflush(fd,TCIFLUSH);
            ret = read(fd, rcv_buf + pos, 1);
            if (-1 == ret)
            {
                break;
            }

            pos++;
            if (Len <= pos)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    return pos;
}

/* Serial port send Data */

int sendDataTty(int fd, char *send_buf, int Len)
{
    ssize_t ret;

    ret = write(fd, send_buf, Len);

    if (ret == -1)
    {
        printf("write device error\n");
        return -1;
    }

    //    delay(100);

    return 1;
}

/* This function is used to initialize serial port.The return is very important.Sending and receiving data can not lack of it */

int seriportInit(void)
{
    int iSetOpt = 0;
    int fdSerial = 0;

    if((fdSerial = openPort(fdSerial, 4))<0)
    {
        perror("open_port error");
        return -1;
    }
    if((iSetOpt = setOpt(fdSerial,115200,8,'N',1))<0)
    {
        perror("set_opt error");
        return -1;
    }
    printf("Serial fdSerial=%d\n",fdSerial);
    tcflush(fdSerial,TCIOFLUSH);
    fcntl(fdSerial,F_SETFL,0);
    return fdSerial;
}
int main(int argc, char**argv)
{
    int fdSerial=seriportInit();
    char rcv_buf[1024];


    ros::init(argc, argv, "range_msg");



        ros::Rate loop_rate(10);

        while (ros::ok())
        {


            readDataTty(fdSerial,rcv_buf,2,1024);



            if( (rcv_buf[0] & 0XFF) == 0X5A  && (rcv_buf[2] & 0XFF) == 0XCC )
            {
                std::cout <<(int)rcv_buf[1] <<std::endl;
                rcv_buf[0] = 0;


            }



            delay(50);






            ros::spinOnce();
            loop_rate.sleep();
            }

        return 0;


}
