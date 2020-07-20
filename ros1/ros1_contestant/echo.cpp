#include <algorithm>
#include <vector>
#include "ros/ros.h"
#include "ros1_contestant/StampedBlob.h"
using std::string;


class EchoNode
{
public:
  ros::NodeHandle nh, nh_private;
  ros::Publisher pub;
  ros::Subscriber sub;

  bool tcp_nodelay = false;
  bool verbose = false;

  EchoNode()
  : nh(), nh_private("~")
  {
  }

  int run(int argc, char **argv);
  void callback(const ros1_contestant::StampedBlob::ConstPtr& msg);
};

void EchoNode::callback(
  const ros1_contestant::StampedBlob::ConstPtr& msg)
{
  pub.publish(msg);

  if (verbose)
    ROS_INFO("echo %d\n", static_cast<int>(msg->counter));
}

int EchoNode::run(int argc, char **argv)
{
  nh_private.param<bool>("tcp_nodelay", tcp_nodelay, false);
  ros::TransportHints hints;
  if (tcp_nodelay)
    hints = ros::TransportHints().tcpNoDelay();

  nh_private.param<bool>("verbose", verbose, false);

  pub = nh.advertise<ros1_contestant::StampedBlob>("echo_out", 10);

  sub = nh.subscribe("echo_in", 10, &EchoNode::callback, this, hints);

  if (verbose)
    ROS_INFO("entering spin()...");

  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "echo");
  EchoNode node;
  return node.run(argc, argv);
}
