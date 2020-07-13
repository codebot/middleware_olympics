#include "ros/ros.h"
#include "ros1_contestant/StampedBlob.h"
using std::string;


class SubscriberNode
{
public:
  ros::NodeHandle nh, nh_private;
  ros::Subscriber sub;
  uint64_t expected_counter = 1;
  bool tcp_nodelay = false;

  SubscriberNode()
  : nh(), nh_private("~")
  {
  }

  int run(int argc, char **argv);
  void callback(const ros1_contestant::StampedBlob::ConstPtr& msg);
};

void SubscriberNode::callback(
  const ros1_contestant::StampedBlob::ConstPtr& msg)
{
  ros::Duration latency = ros::Time::now() - msg->stamp;
  if (expected_counter != msg->counter)
  {
    ROS_ERROR("message skipped! %lu != %lu", msg->counter, expected_counter);
  }
  expected_counter = msg->counter + 1;
  printf("latency = %.6f\n", latency.toSec());
}

int SubscriberNode::run(int argc, char **argv)
{
  nh_private.param<bool>("tcp_nodelay", tcp_nodelay, false);
  ros::TransportHints hints;
  if (tcp_nodelay)
    hints = ros::TransportHints().tcpNoDelay();

  sub = nh.subscribe(
    "blob",
    10,
    &SubscriberNode::callback,
    this,
    hints);

  ROS_INFO("entering spin()...");
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swabber_follower");
  SubscriberNode node;
  return node.run(argc, argv);
}
