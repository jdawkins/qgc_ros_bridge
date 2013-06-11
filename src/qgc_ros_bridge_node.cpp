//#include "ros/ros.h"
#include "qgc_ros_bridge.h"



int main(int argc, char* argv[])
{

    ros::init(argc,argv,"qgc_bridge");
    ros::NodeHandle nh = ros::NodeHandle("~");
    QGCROSBridge* qgc = new QGCROSBridge(nh);
   // qgc_ros_bridge qgc;

    qgc->openUDPPort();
  //  qgc.openUDPPort();



    while (ros::ok()){


        qgc->receiveFromQGC(); // Read new data from QGC and Post to ROS on /mav_qgc topic

   //     comPub.publish(qgc.mav_ros_msg); // Publish the

        ros::spinOnce();    //Allow ROS to run Callback function if new /mav_data message is posted
    }

}
