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
  int max_message_count = 0;

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

  nh_private.param<double>("publish_rate", publish_rate, 10.0);

  nh_private.param<int>("max_message_count", max_message_count, 0);
  // ROS_INFO("pub max message_count: %d", max_message_count);

  pub = nh.advertise<ros1_contestant::StampedBlob>("blob", 100);

  msg.counter = 0;
  ros::Rate pub_rate(publish_rate);
  while (ros::ok())
  {
    msg.counter++;
    msg.stamp = ros::Time::now();
    pub.publish(msg);
    ros::spinOnce();
    pub_rate.sleep();

    if (max_message_count && msg.counter > max_message_count)
    {
      ROS_INFO("publisher complete");
      ros::spinOnce();
      break;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
  PublisherNode node;
  return node.run(argc, argv);
}
