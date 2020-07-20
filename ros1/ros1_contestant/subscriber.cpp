#include <algorithm>
#include <vector>
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
  int max_message_count = 0;
  int skips = 0;
  bool verbose = false;
  bool stats_printed = false;

  std::vector<double> latency_measurements;

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
  if (stats_printed)
    return;

  ros::Duration latency = ros::Time::now() - msg->stamp;
  if (expected_counter != msg->counter)
  {
    skips++;
    if (verbose)
      ROS_ERROR(
        "message skipped! %lu != %lu",
        msg->counter,
        expected_counter);
  }
  expected_counter = msg->counter + 1;

  if (max_message_count && msg->counter > max_message_count)
  {
    std::sort(latency_measurements.begin(), latency_measurements.end());
    double median = latency_measurements[latency_measurements.size()/2];

    printf(
      "%.6f %.6f %.6f %d\n",
      median,
      latency_measurements.front(),
      latency_measurements.back(),
      skips);

    ros::shutdown();
    stats_printed = true;
  }

  const double l = latency.toSec();
  latency_measurements.push_back(l);

  if (verbose)
    printf("latency = %.6f\n", latency.toSec());
}

int SubscriberNode::run(int argc, char **argv)
{
  nh_private.param<bool>("tcp_nodelay", tcp_nodelay, false);
  ros::TransportHints hints;
  if (tcp_nodelay)
    hints = ros::TransportHints().tcpNoDelay();

  nh_private.param<bool>("verbose", verbose, false);

  nh_private.param<int>("max_message_count", max_message_count, 0);
  latency_measurements.reserve(max_message_count);

  sub = nh.subscribe(
    "blob",
    100,
    &SubscriberNode::callback,
    this,
    hints);

  if (verbose)
    ROS_INFO("entering spin()...");
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");
  SubscriberNode node;
  return node.run(argc, argv);
}
