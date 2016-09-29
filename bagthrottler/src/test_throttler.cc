#include <ros/ros.h>
#include "BagThrottler.h"

#include <sensor_msgs/JointState.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

using namespace sensor_msgs;
using namespace rc;
using namespace std;

void fastCallback(JointStateConstPtr msg)
{
  static int lastSeq = -1;
  if (lastSeq>0 && lastSeq+1!=msg->header.seq) {
    throw runtime_error("!!!fastCallback missed event!!!");
  }
  lastSeq = msg->header.seq;
  ROS_INFO_STREAM("fastCallback with low computation, stamp: " <<
       msg->header.stamp << " seq: " << msg->header.seq);
}

ros::Publisher pub;
void slowCallback(JointStateConstPtr msg)
{
  static int lastSeq = -1;
  if (lastSeq>0 && lastSeq+1!=msg->header.seq) {
    throw runtime_error("!!!slowCallback missed event!!!");
  }
  lastSeq = msg->header.seq;
  ROS_INFO_STREAM("slowCallback with heavy computation, stamp: " <<
       msg->header.stamp << " seq: " << msg->header.seq);
  sleep(1);

  JointStatePtr outmsg(new JointState(*msg));
  pub.publish(outmsg);
}

void syncCallback(JointStateConstPtr msg)
{
  ROS_INFO_STREAM("syncCallback with heavy computation, stamp: " <<
                                                                 msg->header.stamp << " seq: " << msg->header.seq);
  static int lastSeq = -1;
  if (lastSeq>0 && lastSeq+1!=msg->header.seq) {
    throw runtime_error("!!!syncCallback missed event!!!");
  }
  lastSeq = msg->header.seq;
  sleep(1);

  JointStatePtr outmsg(new JointState(*msg));
//  pub.publish(outmsg);
}

/*******************************************************************************
*******************************************************************************/

using namespace message_filters;
int main(int argc, char *argv[])
{
  // first we need to initialize ros node
  ros::init(argc, argv, "first_node");

  // second, we configure bag throttler to a specific rosbag topic
//  BagThrottler::throttle<JointState>("/iiwa/joint_states");

  // finally, we can register our actual subscribers
  ros::NodeHandle nh;
//  pub = nh.advertise<JointState>("/joint_states", 1000);
//  ros::Subscriber sub = nh.subscribe("/iiwa/joint_states", 5, &fastCallback);
//  ros::Subscriber sub2 = nh.subscribe("/iiwa/joint_states", 2, &slowCallback);

  message_filters::Subscriber<JointState> joint_sub(nh, "/iiwa/joint_states", 2);
  message_filters::Subscriber<JointState> joint_sub2(nh, "/iiwa/joint_states", 10);
  typedef sync_policies::ExactTime<JointState, JointState> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), joint_sub, joint_sub2);
  BagThrottler::throttle<JointState>(sync, "/iiwa/joint_states");
  sync.registerCallback(boost::bind(&syncCallback, _1));

  ros::spin();
  return 0;
}
