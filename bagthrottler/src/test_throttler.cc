#include <ros/ros.h>
#include "BagThrottler.h"

using namespace sensor_msgs;
using namespace rc;
using namespace std;

void fastCallback(JointStateConstPtr msg)
{ static int lastSeq = -1;
  if (lastSeq>0 && lastSeq+1!=msg->header.seq) {
    throw runtime_error("!!!fastCallback missed event!!!");
  }
  lastSeq = msg->header.seq;
  ROS_INFO_STREAM("fastCallback with low computation, stamp: " <<
       msg->header.stamp << " seq: " << msg->header.seq);
}

void slowCallback(JointStateConstPtr msg)
{ static int lastSeq = -1;
  if (lastSeq>0 && lastSeq+1!=msg->header.seq) {
    throw runtime_error("!!!slowCallback missed event!!!");
  }
  lastSeq = msg->header.seq;
  ROS_INFO_STREAM("slowCallback with heavy computation, stamp: " <<
       msg->header.stamp << " seq: " << msg->header.seq);
  sleep(1);
}

/*******************************************************************************
*******************************************************************************/

int main(int argc, char *argv[])
{
  // first we need to initialize ros node
  ros::init(argc, argv, "throttler_test");

  // second, we configure bag throttler to a specific rosbag topic
  BagThrottler::throttle<sensor_msgs::JointState>("/iiwa/joint_states");
  BagThrottler::throttle<sensor_msgs::JointState>("/output/from/other/node", "/rosbag/topic/to/be/throttled");

  // finally, we can register our actual subscribers
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/iiwa/joint_states", 5, &fastCallback);
  ros::Subscriber sub2 = nh.subscribe("/iiwa/joint_states", 2, &slowCallback);

  ros::spin();
  return 0;
}
