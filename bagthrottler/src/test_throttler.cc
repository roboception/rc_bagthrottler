#include <ros/ros.h>
#include "BagThrottler.h"

using namespace sensor_msgs;
using namespace rc;
using namespace std;


void callback(JointStateConstPtr msg)
{ static int lastSeq = -1;
  if (lastSeq>0 && lastSeq+1!=msg->header.seq) {
    throw runtime_error("!!!callback missed event!!!");
  }
  lastSeq = msg->header.seq;
  ROS_INFO_STREAM("callback with heavy computation, stamp: " <<
       msg->header.stamp << " seq: " << msg->header.seq);
  sleep(1);
}

/*******************************************************************************
*******************************************************************************/

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "throttler_test");
  ros::NodeHandle nh;


  // configure bag throttler to a specific rosbag topic
  BagThrottler::throttle<sensor_msgs::JointState>("/iiwa/joint_states");
  BagThrottler::throttle<sensor_msgs::JointState>("/output/from/other/node", "/rosbag/topic/to/be/throttled");

  ros::Subscriber sub = nh.subscribe("/iiwa/joint_states", 2, &callback);

  ros::spin();
  return 0;
}
