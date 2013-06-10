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
//#include "mavlink_msgs/Mavlink.h"
#include "qgc_ros_bridge/Mavlink.h"

#include "ros/ros.h"


//using namespace mav_msgs;
using namespace qgc_ros_bridge;


class QGCROSBridge {

public:
    QGCROSBridge();

    void receiveFromQGC();
    void sendToQGC(mavlink_message_t msg);

    void mavMessageCallback(const Mavlink &mav_msg);
    void openUDPPort();

    mavlink_message_t msg_out_;
//    mav_msgs::Mavlink mav_ros_msg;

private:



   void initializeUDPPort();


   char target_ip_[100];

   float position_[6];
   int sock_;
   struct sockaddr_in gcAddr_;
   struct sockaddr_in locAddr_;
   //struct sockaddr_in fromAddr;

   ssize_t recsize_;
   socklen_t fromlen_;
   int bytes_sent_;

   //uint16_t len;

   ros::NodeHandle nh_;
   ros::Publisher comPub_;
   ros::Subscriber comSub1_;
   ros::Subscriber comSub2_;
   ros::Subscriber comSub3_;
   ros::Subscriber comSub4_;
   ros::Subscriber comSub5_;
   ros::Subscriber comSub6_;
};




#endif //QGC_ROS_BRIDGE_H
