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

std::vector<uint8_t> echo_buf;
bool echo_buf_sent = true;
int g_msg_length = 0;

void callback(const zn_sample *sample)
{
  //stamped_blob_t *msg = (stamped_blob_t *)sample->value.val;
  //printf("rx time: (%u, %u)\n", msg->seconds, msg->nanoseconds);

  if (sample->value.len > echo_buf.capacity())
    echo_buf.resize(sample->value.len);

  memcpy(&echo_buf[0], sample->value.val, sample->value.len);
  g_msg_length = sample->value.len;

  echo_buf_sent = false;

  /*
  zn_write(
    g_session,
    "/echo_out",
    (const char *)sample->value.val,
    sample->value.len);
  */
}

int main(int /*argc*/, char ** /*argv*/)
{
  echo_buf.resize(11000000);

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
    "/ball_serve",
    callback);

  while (!g_done)
  {
    //printf("sleeping\n");
    if (!echo_buf_sent)
    {
      zn_write(
        g_session,
        "/ball_return",
        (const char *)&echo_buf[0],
        g_msg_length);

      echo_buf_sent = true;
    }
    usleep(10);
    //sleep(1);
  }

  zn_undeclare_subscriber(sub);

  zn_close(g_session);
  printf("closed session\n");
  return 0;
}