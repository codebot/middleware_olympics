#include <cstdlib>
#include "ros/ros.h"
#include "ros1_contestant/StampedBlob.h"
using std::string;


class PublisherNode
{
public:
  ros::NodeHandle nh, nh_private;
  ros1_contestant::StampedBlob msg;
  ros::Publisher pub;

  int blob_size = 0;
  double publish_rate = 10.0;

  PublisherNode()
  : nh(), nh_private("~")
  {
  }

  int run(int argc, char **argv);
};

int PublisherNode::run(int argc, char **argv)
{
  // stuff the blob with random data so it's not compressible
  nh_private.param<int>("blob_size", blob_size, 0);
  msg.blob.reserve(blob_size);
  srandom(0);
  for (int i = 0; i < blob_size; i++)
    msg.blob.push_back(random() % 256);
  ROS_INFO("blob_size = %zu", msg.blob.size());

  nh_private.param<double>("publish_rate", publish_rate, 10.0);

  pub = nh.advertise<ros1_contestant::StampedBlob>("blob", 10);

  ros::Rate pub_rate(publish_rate);
  while (ros::ok())
  {
    msg.stamp = ros::Time::now();
    pub.publish(msg);
    ros::spinOnce();
    pub_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
  PublisherNode node;
  return node.run(argc, argv);
}
