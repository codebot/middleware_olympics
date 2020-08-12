#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <signal.h>
#include <time.h>
#include <unistd.h>

extern "C" {
#include "zenoh-ffi.h"
}
#include "stamped_blob.h"

static bool g_done = false;
static ZNSession *g_session = nullptr;

void sigint_handler(int)
{
  g_done = true;
}

void callback(const zn_sample * /*sample*/)
{
  //stamped_blob_t *msg = (stamped_blob_t *)sample->value.val;
  //printf("rx time: (%u, %u)\n", msg->seconds, msg->nanoseconds);
  printf("rx callback\n");
}

int main(int argc, char **argv)
{
  signal(SIGINT, sigint_handler);

  printf("opening session...\n");
  g_session = zn_open(PEER, NULL, NULL);
  if (!g_session)
  {
    printf("unable to open session :(\n");
    return 1;
  }
  printf("opened session\n");

  int num_topics = 1;

  if (argc > 1)
  {
    num_topics = atoi(argv[1]);
  }
  printf("subscribing to %d topics\n", num_topics);

  std::vector<ZNSubscriber *> subs;
  for (int i = 0; i < num_topics; i++)
  {
    std::string topic_name = "/topic_" + std::to_string(i);
    subs.push_back(
      zn_declare_subscriber(
        g_session,
        topic_name.c_str(),
        zn_subinfo_default(),
        callback));
  }

  /*
  ZNSubscriber *sub = zn_declare_subscriber(
    g_session,
    "/hello",
    zn_subinfo_default(),
    callback);
  */

  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  const double t_start = spec.tv_sec + 1.0e-9 * spec.tv_nsec;
  const double MAX_SECONDS = 5.5;

  while (!g_done)
  {
    clock_gettime(CLOCK_REALTIME, &spec);
    const double t = spec.tv_sec + 1.0e-9 * spec.tv_nsec;
    if (t - t_start > MAX_SECONDS)
      break;

    usleep(1000);
    //sleep(1);
  }

  for (size_t i = 0; i < subs.size(); i++)
  {
    zn_undeclare_subscriber(subs[i]);
  }

  zn_close(g_session);
  printf("closed session\n");
  return 0;
}
