#include "qgc_ros_bridge.h"

/**
 * Takes a ROS-Mavlink-message (mavlink_ros_msg) and converts it into a Mavlink-Message (msg)
 */
static inline void createMavlinkFromROS(const Mavlink *mavlink_ros_msg, mavlink_message_t *msg)
{

    msg->checksum = mavlink_ros_msg->checksum;
    msg->magic = mavlink_ros_msg->magic;
    msg->len = mavlink_ros_msg->len;
    msg->seq = mavlink_ros_msg->seq;
    msg->sysid = mavlink_ros_msg->sysid;
    msg->compid = mavlink_ros_msg->compid;
    msg->msgid = mavlink_ros_msg->msgid;

    for(int i = 0;i<((MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8);i++){
        msg->payload64[i] = mavlink_ros_msg->payload[i];

    }

}

/**
 * Takes a Mavlink-message (mavlink_msg) and converts it into a ROS-Mavlink-Message (mavlink_ros_msg)
 */
static inline void createROSFromMavlink(const mavlink_message_t* mavlink_msg, Mavlink* mavlink_ros_msg)
{
    mavlink_ros_msg->checksum = mavlink_msg->checksum;
    mavlink_ros_msg->magic = mavlink_msg->magic;
    mavlink_ros_msg->len = mavlink_msg->len;
    mavlink_ros_msg->seq = mavlink_msg->seq;
    mavlink_ros_msg->sysid = mavlink_msg->sysid;
    mavlink_ros_msg->compid = mavlink_msg->compid;
    mavlink_ros_msg->msgid = mavlink_msg->msgid;

    for (int i = 0; i < ((MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8); i++){
    mavlink_ros_msg->payload[i] = mavlink_msg->payload64[i];
//       (mavlink_ros_msg->payload).push_back(mavlink_msg->payload64[i]);
    }

}


QGCROSBridge::QGCROSBridge(ros::NodeHandle nh){

nh_ = nh;

if(nh_.getParam("target_ip",target_ip_)){
    ROS_INFO("Got Param: %s",target_ip_.c_str());
}
else{
    target_ip_ = "192.168.1.6";
    //target_ip_ = "127.0.0.1";
    ROS_WARN("Failed to get target ip address from Launch File, defaulting to local ip %s \n",target_ip_.c_str());
}



comPub_ = nh_.advertise<Mavlink>("/ArduPilot/mav_qgc",1000);
comSub_ = nh_.subscribe("/ArduPilot/mav_data",1000,&QGCROSBridge::mavMessageCallback,this);

/*comSub1_ = nh_.subscribe("/ArduPilot_1/mav_data",1000,&QGCROSBridge::mavMessageCallback,this);
comSub2_ = nh_.subscribe("/ArduPilot_2/mav_data",1000,&QGCROSBridge::mavMessageCallback,this);
comSub3_ = nh_.subscribe("/ArduPilot_3/mav_data",1000,&QGCROSBridge::mavMessageCallback,this);
comSub4_ = nh_.subscribe("/ArduPilot_4/mav_data",1000,&QGCROSBridge::mavMessageCallback,this);
comSub5_ = nh_.subscribe("/ArduPilot_5/mav_data",1000,&QGCROSBridge::mavMessageCallback,this);
comSub6_ = nh_.subscribe("/ArduPilot_6/mav_data",1000,&QGCROSBridge::mavMessageCallback,this);*/


    sock_ = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    initializeUDPPort();

}

void QGCROSBridge::initializeUDPPort(){

    memset(&locAddr_, 0, sizeof(locAddr_));
    locAddr_.sin_family = AF_INET;
    locAddr_.sin_addr.s_addr = INADDR_ANY;
    locAddr_.sin_port = htons(14551);

    memset(&gcAddr_, 0, sizeof(gcAddr_));
    gcAddr_.sin_family = AF_INET;
    gcAddr_.sin_addr.s_addr = inet_addr(target_ip_.c_str());
    gcAddr_.sin_port = htons(14550);


}

void QGCROSBridge::openUDPPort(){


    /* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */
    if (-1 == bind(sock_,(struct sockaddr *)&locAddr_, sizeof(struct sockaddr)))
    {
        perror("error bind failed");
        close(sock_);
        exit(EXIT_FAILURE);
    }

    /* Attempt to make it non blocking */
    if (fcntl(sock_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sock_);
        exit(EXIT_FAILURE);
    }


}
void QGCROSBridge::sendToQGC(mavlink_message_t msg){

    uint8_t buf[BUFFER_LENGTH];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

   /* printf("Message to QGC: ");
    for (int i=0;i<len;i++){

        uint8_t temp = buf[i];
        printf("%02x ", (unsigned char)temp);
    }
    printf("\n");*/
    bytes_sent_ = sendto(sock_, buf, len, 0, (struct sockaddr*)&gcAddr_, sizeof(struct sockaddr_in));

}

void QGCROSBridge::receiveFromQGC(){

    uint8_t buf[BUFFER_LENGTH];

    memset(buf, 0, BUFFER_LENGTH);
    recsize_ = recvfrom(sock_, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr_, &fromlen_);

    if (recsize_ > 0)
    {
        // Something received - print out all bytes and parse packet
        mavlink_message_t msg;
        mavlink_status_t status;
        mavlink_msgs::Mavlink mav_ros_msg;

      //  printf("Bytes Received: %d\nDatagram: ", (int)recsize);
        for (int i = 0; i < recsize_; ++i)
        {

            uint8_t temp = buf[i];
            //printf("%02x ", (unsigned char)temp);
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
            {
                // Packet received
               // printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
                createROSFromMavlink(&msg,&mav_ros_msg);
                comPub_.publish(mav_ros_msg);
            }
        }
       // printf("\n");
    }


}

void QGCROSBridge::mavMessageCallback(const Mavlink &mav_msg){

   mavlink_message_t message;
   createMavlinkFromROS(&mav_msg,&message);
//   printf("New Message Sent to QGC\n\r");
   sendToQGC(message);

//   uint8_t buf[BUFFER_LENGTH];
//   mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//   bytes_sent = sendto(sock_, buf, len, 0, (struct sock_addr*)&gcAddr_, sizeof(struct sock_addr_in));

}


