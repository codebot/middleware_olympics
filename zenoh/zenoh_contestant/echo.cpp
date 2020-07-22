#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <signal.h>
#include <unistd.h>

extern "C" {
#include "zenoh-ffi.h"
}

static bool g_done = false;
static ZNSession *g_session = nullptr;

void sigint_handler(int)
{
  g_done = true;
}

std::vector<uint8_t> echo_buf;

void callback(const zn_sample *sample)
{
  if (echo_buf.size() < sample->value.len)
    echo_buf.resize(sample->value.len);
  memcpy(&echo_buf[0], sample->value.val, sample->value.len);

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
    "/echo_in",
    callback);

  while (!g_done)
  {
    //printf("sleeping\n");
    if (!echo_buf.empty())
    {
      zn_write(
        g_session,
        "/echo_out",
        (const char *)&echo_buf[0],
        echo_buf.size());

      echo_buf.clear();
    }
    usleep(1000);
    //sleep(1);
  }

  zn_undeclare_subscriber(sub);

  zn_close(g_session);
  printf("closed session\n");
  return 0;
}
