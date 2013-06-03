#ifndef QGC_ROS_BRIDGE_H
#define QGC_ROS_BRIDGE_H

/* These headers are for QNX, but should all be standard on unix/linux */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#endif

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

#include "mavlink/v1.0/ardupilotmega/mavlink.h"
#include "mavlink_msgs/Mavlink.h"

#include "ros/ros.h"


using namespace mav_msgs;


class qgc_ros_bridge {

public:
    qgc_ros_bridge();

    void receiveFromQGC();
    void sendToQGC(mavlink_message_t msg);

    void mavMessageCallback(const Mavlink &mav_msg);
    void openUDPPort();

    mavlink_message_t msg_out;
//    mav_msgs::Mavlink mav_ros_msg;

private:



   void initializeUDPPort();


   char target_ip[100];

   float position[6];
   int sock;
   struct sockaddr_in gcAddr;
   struct sockaddr_in locAddr;
   //struct sockaddr_in fromAddr;

   ssize_t recsize;
   socklen_t fromlen;
   int bytes_sent;

   //uint16_t len;

   ros::NodeHandle nh;
   ros::Publisher comPub;
   ros::Subscriber comSub;

};




#endif //QGC_ROS_BRIDGE_H
