
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
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#define buffLen 1024
#define rcvTimeOut 2

#endif
const double PI = 3.1415926;
const double GRAVITY = 9.8015;
/*
 * Created by Kalman on 16/12/9
 */

/* A imprecise delay funtion */
using namespace std;
int hex_char_value(char c)
{
    if(c >= '0' && c <= '9')
        return c - '0';
    else if(c >= 'a' && c <= 'f')
        return (c - 'a' + 10);
    else if(c >= 'A' && c <= 'F')
        return (c - 'A' + 10);
    assert(0);
    return 0;
}
int hex_to_decimal(const char* szHex, int len)
{
    int result = 0;
    for(int i = 0; i < len; i++)
    {
        result += (int)pow((float)16, (int)len-i-1) * hex_char_value(szHex[i]);
    }
    return result;
}
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
        fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ttyUSB1 .....\n");
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
    ros::init(argc, argv, "publisher_imu");

    ros::NodeHandle n;

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imupro", 1000);

    ros::Rate loop_rate(200);

    int count = 0;

    char rcv_buf[1024];

    sensor_msgs::Imu msg;

    int fdSerial = seriportInit();

    msg.header.frame_id = "imu_pro_link";

    msg.linear_acceleration_covariance[0] = 0;
    msg.linear_acceleration_covariance[1] = 0;
    msg.linear_acceleration_covariance[2] = 0;

    msg.linear_acceleration_covariance[3] = 0;
    msg.linear_acceleration_covariance[4] = 0;
    msg.linear_acceleration_covariance[5] = 0;

    msg.linear_acceleration_covariance[6] = 0;
    msg.linear_acceleration_covariance[7] = 0;
    msg.linear_acceleration_covariance[8] = 0;

    msg.angular_velocity_covariance[0] = 0;
    msg.angular_velocity_covariance[1] = 0;
    msg.angular_velocity_covariance[2] = 0;

    msg.angular_velocity_covariance[3] = 0;
    msg.angular_velocity_covariance[4] = 0;
    msg.angular_velocity_covariance[5] = 0;

    msg.angular_velocity_covariance[6] = 0;
    msg.angular_velocity_covariance[7] = 0;
    msg.angular_velocity_covariance[8] = 0;




        while (ros::ok())
        {


            readDataTty(fdSerial,rcv_buf,2,1024);
/*

            if( (rcv_buf[0] & 0XFF) == 0Xaa && (rcv_buf[1] & 0XFF) == 0Xaa )
            {
                         //   std::cout << ((short)((rcv_buf[2] & 0xff) << 8))+((short)( rcv_buf[3]&0xff))   << std::endl;


                            msg.header.stamp = ros::Time::now();
                            msg.linear_acceleration.x = (float) (((short)((rcv_buf[8] & 0xff) << 8))+((short)( rcv_buf[9]&0xff))) /32768*4*9.8;
                            msg.linear_acceleration.y = (float)(((short)((rcv_buf[10] & 0xff) << 8))+((short)( rcv_buf[11]&0xff)))/32768 *4 *9.8;
                            msg.linear_acceleration.z = (float) (((short)((rcv_buf[12] & 0xff) << 8))+((short)( rcv_buf[13]&0xff)))/32768*4*9.8;
                            msg.angular_velocity.x = (float)(((short)((rcv_buf[2] & 0xff) << 8))+((short)( rcv_buf[3]&0xff))) /32768*1000 -(-1.13261) ;
                            msg.angular_velocity.y = (float)(((short)((rcv_buf[4] & 0xff) << 8))+((short)( rcv_buf[5]&0xff))) /32768*1000 - (0.642275);
                            msg.angular_velocity.z = (float)(((short)((rcv_buf[6] & 0xff) << 8))+((short)( rcv_buf[7]&0xff))) /32768*1000 - (-1.73559);

                            imu_pub.publish(msg);

                            ros::spinOnce();

                            loop_rate.sleep();

            sum_angular_x = sum_angular_x + msg.angular_velocity.x;
            sum_angular_y = sum_angular_y + msg.angular_velocity.y;
            sum_angular_z = sum_angular_z + msg.angular_velocity.z;

               count++;

            }
            std::cout<<sum_angular_x /count << " "<< sum_angular_y /count << " " <<sum_angular_z /count <<std::endl;
*/












        short int accel_x_temp,accel_y_temp,accel_z_temp;
        short int gyro_x_temp, gyro_y_temp, gyro_z_temp;
        double accel_x_real,accel_y_real,accel_z_real;
        double gyro_x_real,gyro_y_real,gyro_z_real;

        if((rcv_buf[0] & 0XFF) == 0XAA &&(rcv_buf[1] & 0XFF) == 0XAA) {
            accel_x_temp = (rcv_buf[2]&0xFF) | (rcv_buf[3] << 8);
            accel_y_temp = (rcv_buf[4]&0xFF) | (rcv_buf[5] << 8);
            accel_z_temp = (rcv_buf[6]&0xFF) | (rcv_buf[7] << 8);
            gyro_x_temp = (rcv_buf[8]&0xFF) | (rcv_buf[9] << 8);
            gyro_y_temp = (rcv_buf[10]&0xFF) | (rcv_buf[11] << 8);
            gyro_z_temp = (rcv_buf[12]&0xFF) | (rcv_buf[13] << 8);

            accel_x_real = (double)(accel_x_temp) * 12 / 0x10000 * GRAVITY;
            accel_y_real = (double)(accel_y_temp) * 12 / 0x10000 * GRAVITY;
            accel_z_real = (double)(accel_z_temp) * 12 / 0x10000 * GRAVITY;
            gyro_x_real = (double)(gyro_x_temp) * 2000 / 0x10000 * PI /180;
            gyro_y_real = (double)(gyro_y_temp) * 2000 / 0x10000 * PI /180;
            gyro_z_real = (double)(gyro_z_temp) * 2000 / 0x10000 * PI /180;


            msg.header.stamp = ros::Time::now();
            msg.linear_acceleration.x = accel_x_real;
            msg.linear_acceleration.y = accel_y_real;
            msg.linear_acceleration.z = accel_z_real;
            msg.angular_velocity.x =gyro_x_real ;
            msg.angular_velocity.y = gyro_y_real;
            msg.angular_velocity.z = gyro_z_real;

            imu_pub.publish(msg);



            std::cout << "accelx: " << accel_x_real << "accely: " << accel_y_real << "accelz: " << accel_z_real << "gyrox: " << gyro_x_real << "gyroy: " << gyro_y_real << "gyroz: " << gyro_z_real << std::endl;

        }


            //delay(5);






            ros::spinOnce();
            loop_rate.sleep();
            }

        return 0;

























}
