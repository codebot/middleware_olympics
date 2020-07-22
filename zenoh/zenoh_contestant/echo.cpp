#include <cstdio>
#include <cstdlib>
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

void callback(const zn_sample *sample)
{
  if (g_done)
    return;

  zn_write(
    g_session,
    "/echo_out",
    (const char *)sample->value.val,
    sample->value.len);

  /*
  printf(
    "callback: [%d bytes]\n",
    static_cast<int>(sample->value.len));
  */
}

int main(int /*argc*/, char ** /*argv*/)
{
  signal(SIGINT, sigint_handler);

  printf("opening session...\n");
  g_session = zn_open(PEER_MODE, NULL, NULL);  //"tcp/192.168.50.212:7447", NULL);
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
    printf("sleeping\n");
    sleep(1);
  }

  zn_undeclare_subscriber(sub);

  zn_close(g_session);
  printf("closed session\n");
  return 0;
}
