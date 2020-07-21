#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "ros1_contestant/StampedBlob.h"

using std::string;


class PingNode
{
public:
  ros::NodeHandle nh, nh_private;
  ros::Publisher pub;
  ros::Subscriber sub;
  uint64_t last_rx_counter = 0, last_tx_counter = 0;
  bool tcp_nodelay = false;
  int max_message_count = 0;
  int num_skips = 0;
  bool verbose = false;
  int blob_size = 0;
  double publish_rate = 10.0;
  double echo_connect_time = 1.0;  // give other side some time to connect

  std::vector<double> latency_measurements;

  PingNode()
  : nh(), nh_private("~")
  {
  }

  int run(int argc, char **argv);
  void callback(const ros1_contestant::StampedBlob::ConstPtr& msg);
};

void PingNode::callback(
  const ros1_contestant::StampedBlob::ConstPtr& rx_msg)
{
  ros::Duration latency = ros::Time::now() - rx_msg->stamp;

  if (rx_msg->counter == last_tx_counter)
  {
    latency_measurements.push_back(latency.toSec());

    //if (verbose)
    //  printf("latency = %.6f\n", latency.toSec());
  }
  else
  {
    if (verbose)
      ROS_ERROR(
        "received unexpected message counter! %lu != %lu",
        rx_msg->counter,
        last_tx_counter);
  }

  last_rx_counter = rx_msg->counter;
}

int PingNode::run(int argc, char **argv)
{
  nh_private.param<bool>("verbose", verbose, false);

  nh_private.param<int>("max_message_count", max_message_count, 0);
  latency_measurements.reserve(max_message_count);

  nh_private.param<bool>("tcp_nodelay", tcp_nodelay, false);
  ros::TransportHints hints;
  if (tcp_nodelay)
    hints = ros::TransportHints().tcpNoDelay();
  sub = nh.subscribe("echo_out", 10, &PingNode::callback, this, hints);

  pub = nh.advertise<ros1_contestant::StampedBlob>("echo_in", 10);

  ros1_contestant::StampedBlob tx_msg;
  nh_private.param<int>("blob_size", blob_size, 0);
  tx_msg.blob.reserve(blob_size);
  srandom(0);
  for (int i = 0; i < blob_size; i++)
    tx_msg.blob.push_back(random() % 256);

  nh_private.param<double>("publish_rate", publish_rate, 10.0);

  nh_private.param<double>("startup_delay", echo_connect_time, 1.0);

  ros::Duration(echo_connect_time).sleep();

  // ROS_INFO("sending messages...");

  const double publish_interval = 1.0 / publish_rate;
  ros::Time t_last_publish(ros::Time::now());

  while (ros::ok())
  {
    if ((ros::Time::now() - t_last_publish).toSec() > publish_interval)
    {
      // It's time to publish our next message. First, let's see if
      // we received the expected message echo by now
      if (last_rx_counter != tx_msg.counter)
      {
        num_skips++;

        if (verbose)
          ROS_ERROR(
            "last_rx_counter != tx_msg.counter: %lu != %lu  (%d)",
            tx_msg.counter,
            last_rx_counter,
            num_skips);
      }

      // now let's send our next message
      tx_msg.counter++;
      tx_msg.stamp = ros::Time::now();
      pub.publish(tx_msg);

      last_tx_counter = tx_msg.counter;
      t_last_publish = tx_msg.stamp;
    }

    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001));
    //printf("num skips: %d\n", num_skips);

    if (max_message_count && tx_msg.counter > max_message_count)
      break;
  }
  ros::shutdown();

  std::sort(latency_measurements.begin(), latency_measurements.end());
  double median = latency_measurements[latency_measurements.size()/2];

  printf(
    "%.6f %.6f %.6f %d %d\n",
    median,
    latency_measurements.front(),
    latency_measurements.back(),
    num_skips,
    static_cast<int>(tx_msg.counter));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping");
  PingNode node;
  return node.run(argc, argv);
}
