#include "ros/ros.h"
#include "ros1_contestant/StampedBlob.h"
using std::string;


class PublisherNode
{
public:
  ros::NodeHandle nh, nh_private;
  ros1_contestant::StampedBlob msg;
  ros::Publisher pub;

  PublisherNode()
  : nh(), nh_private("~")
  {
  }

  int run(int argc, char **argv);
};

int PublisherNode::run(int argc, char **argv)
{
  ROS_INFO("hello");
  pub = nh.advertise<ros1_contestant::StampedBlob>("blob", 10);

  ros::Rate pub_rate(1);
  while (ros::ok())
  {
    msg.stamp = ros::Time::now();
    pub.publish(msg);
    pub_rate.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
  PublisherNode node;
  return node.run(argc, argv);
}
