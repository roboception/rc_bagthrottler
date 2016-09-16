//
// Created by emmerich on 16.09.16.
//

#ifndef RC_BAGTHROTTLER_BAGTHROTTLER_H
#define RC_BAGTHROTTLER_BAGTHROTTLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <rc_msgs/ThrottleBag.h>

namespace rc
{

class BagThrottler
{
  public:
    typedef boost::shared_ptr<BagThrottler> Ptr;

    static Ptr throttle(const std::string &topic)
    {

      // create throttler object and subscribe to topic
      Ptr throttler(new BagThrottler(topic));
      ros::NodeHandle nh;

      ros::Subscriber sub = nh.subscribe(topic, 100,
                                         &BagThrottler::throttlingCallback,
                                         throttler);

      /// store subscriber, otherwise subscription is lost
      throttler->storeSubscriber(sub);

      /// store throttler object, othewise subscription is lost
      AllThrottlers.push_back(throttler);

      return throttler;
    }

    void throttlingCallback(sensor_msgs::JointStateConstPtr msg)
    {
      ROS_INFO_STREAM("throttling                     , stamp: " <<
           msg->header.stamp << " seq: " << msg->header.seq);
      client.call(srvCall);
    }

  private:
    static std::vector<Ptr> AllThrottlers;

    BagThrottler(const std::string &topic) : topic(topic)
    {
      ros::NodeHandle nh;
      client = nh.serviceClient<rc_msgs::ThrottleBag>("/bagControl");
      srvCall.request.topic = topic;
      srvCall.request.id = ros::this_node::getName();
      srvCall.request.qsize = 1;

      // throttle bag already before it starts publishing
      client.call(srvCall);
    }

    void storeSubscriber(ros::Subscriber s)
    {
      sub = s;
    }

    std::string topic;
    ros::ServiceClient client;
    rc_msgs::ThrottleBag srvCall;
    ros::Subscriber sub;
};

std::vector<BagThrottler::Ptr> BagThrottler::AllThrottlers = std::vector<BagThrottler::Ptr>();

}

#endif //RC_BAGTHROTTLER_BAGTHROTTLER_H
