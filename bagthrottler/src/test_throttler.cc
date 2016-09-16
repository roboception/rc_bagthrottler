#include <stdio.h>
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


  BagThrottler::throttle("/iiwa/joint_states");
  ros::Subscriber sub = nh.subscribe("/iiwa/joint_states", 2, &callback);

  ros::spin();
  return 0;
}
