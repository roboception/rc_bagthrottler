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

/**
 * Class that implements functionality to throttle rosbag via its remote control
 * interface.
 */
class BagThrottler
{
  public:
    typedef boost::shared_ptr<BagThrottler> Ptr;

    /**
     * Directly connected rosbag throttling.
     *
     * Configures the bag throttler to throttle the \c topic based on messages
     * that are received by this node. Use this version, if this node is
     * directly fed by a rosbag topic.
     *
     * @see static void throttle(const std::string &triggerTopic, const std::string &throttledTopic)
     *
     * @param topic the rosbag topic that should be throttled
     */
    template<class M>
    static void throttle(const std::string &topic)
    {
      throttle<M>(topic, topic);
    }

    /**
     * Transitive rosbag throttling (for longer topic chains)
     *
     * Configures the bag throttler to throttle the \c throttledTopic
     * based on messages that are received by this node on \c triggerTopic.
     *
     * Use this version, if this node is not directly fed by rosbag but by an
     * intermediate node.
     *
     * @see static void throttle(const std::string &topic)
     *
     * @param triggerTopic this node's topic that triggers/determines the throttling of rosbag
     * @param throttledTopic the rosbag topic that is throttled
     */
    template<class M>
    static void
    throttle(const std::string &triggerTopic, const std::string &throttledTopic)
    {

      // create throttler object and subscribe to triggerTopic
      ros::NodeHandle nh;
      Ptr throttler(new BagThrottler(triggerTopic, throttledTopic));
      ros::Subscriber sub = nh.subscribe(throttledTopic, 100,
                                         &BagThrottler::throttlingCallback<M>,
                                         throttler.get());

      /// store subscriber, otherwise subscription is lost
      throttler->storeSubscriber(sub);

      /// store throttler object, othewise subscription is lost
      AllThrottlers.push_back(throttler);
    }

    template<class M>
    void throttlingCallback(boost::shared_ptr<M const>)
    {
      client.call(srvCall);
    }

  private:

    BagThrottler(const std::string &triggerTopic,
                 const std::string &throttledTopic) : throttledTopic(
            throttledTopic)
    {
      ros::NodeHandle nh;
      client = nh.serviceClient<rc_msgs::ThrottleBag>("/bagControl");
      srvCall.request.topic = throttledTopic;
      srvCall.request.id = ros::this_node::getName();
      srvCall.request.qsize = 1;

      // throttle bag already before it starts publishing
      client.call(srvCall);
    }

    void storeSubscriber(ros::Subscriber s)
    {
      sub = s;
    }

    std::string throttledTopic;   //< the rosbag output topic which should be throttled
    std::string subscribedTopic;  //< messages on this topic trigger/determine the throttling

    ros::ServiceClient client;    //< the service client that implements the throttling
    rc_msgs::ThrottleBag srvCall; //< the service call that implements the throttling

    ros::Subscriber sub;          //< subscriber to be stored for that it is not getting lost
    static std::vector<Ptr> AllThrottlers;
};

std::vector<BagThrottler::Ptr> BagThrottler::AllThrottlers = std::vector<BagThrottler::Ptr>();

}

#endif //RC_BAGTHROTTLER_BAGTHROTTLER_H
