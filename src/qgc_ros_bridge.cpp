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

       (mavlink_ros_msg->payload).push_back(mavlink_msg->payload64[i]);
    }

}


qgc_ros_bridge::qgc_ros_bridge(){

comPub = nh.advertise<Mavlink>("/mav_qgc",1000);
comSub = nh.subscribe("/mav_data",1000,&qgc_ros_bridge::mavMessageCallback,this);


    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    initializeUDPPort();

}

void qgc_ros_bridge::initializeUDPPort(){

    strcpy(target_ip, "127.0.0.1");

    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(14551);

    memset(&gcAddr, 0, sizeof(gcAddr));
    gcAddr.sin_family = AF_INET;
    gcAddr.sin_addr.s_addr = inet_addr(target_ip);
    gcAddr.sin_port = htons(14550);


}

void qgc_ros_bridge::openUDPPort(){


    /* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */
    if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
    {
        perror("error bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    }

    /* Attempt to make it non blocking */
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sock);
        exit(EXIT_FAILURE);
    }


}
void qgc_ros_bridge::sendToQGC(mavlink_message_t msg){

    uint8_t buf[BUFFER_LENGTH];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

}

void qgc_ros_bridge::receiveFromQGC(){

    uint8_t buf[BUFFER_LENGTH];

    memset(buf, 0, BUFFER_LENGTH);
    recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);

    if (recsize > 0)
    {
        // Something received - print out all bytes and parse packet
        mavlink_message_t msg;
        mavlink_status_t status;
        mav_msgs::Mavlink mav_ros_msg;

        printf("Bytes Received: %d\nDatagram: ", (int)recsize);
        for (int i = 0; i < recsize; ++i)
        {
            uint8_t temp = buf[i];
            printf("%02x ", (unsigned char)temp);
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
            {
                // Packet received
                printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
                createROSFromMavlink(&msg,&mav_ros_msg);
                comPub.publish(mav_ros_msg);
            }
        }
        printf("\n");
    }


}

void qgc_ros_bridge::mavMessageCallback(const Mavlink &mav_msg){

   mavlink_message_t message;
   createMavlinkFromROS(&mav_msg,&message);
//   printf("New Message Sent to QGC\n\r");
   sendToQGC(message);

//   uint8_t buf[BUFFER_LENGTH];
//   mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//   bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

}


