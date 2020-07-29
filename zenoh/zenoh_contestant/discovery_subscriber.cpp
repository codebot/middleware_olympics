#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <signal.h>
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
}

int main(int /*argc*/, char ** /*argv*/)
{
  signal(SIGINT, sigint_handler);

  printf("opening session...\n");
  g_session = zn_open(PEER_MODE, NULL, NULL);
  if (!g_session)
  {
    printf("unable to open session :(\n");
    return 1;
  }
  printf("opened session\n");

  ZNSubscriber *sub = zn_declare_subscriber(
    g_session,
    "/foo",
    callback);

  while (!g_done)
  {
    usleep(1000);
    //sleep(1);
  }

  zn_undeclare_subscriber(sub);

  zn_close(g_session);
  printf("closed session\n");
  return 0;
}
