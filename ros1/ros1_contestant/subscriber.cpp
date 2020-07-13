#include "ros/ros.h"
#include "ros1_contestant/StampedBlob.h"
using std::string;


class SubscriberNode
{
public:
  ros::NodeHandle nh, nh_private;
  ros::Subscriber sub;

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
  printf("latency = %.6f\n", latency.toSec());
}

int SubscriberNode::run(int argc, char **argv)
{
  ROS_INFO("entering spin()...");
  sub = nh.subscribe("blob", 10, &SubscriberNode::callback, this);
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swabber_follower");
  SubscriberNode node;
  return node.run(argc, argv);
}
