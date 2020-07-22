#include <cstdio>
#include <cstdlib>
#include <vector>

#include <unistd.h>

extern "C" {
#include "zenoh-ffi.h"
}

void callback(const zn_sample *sample)
{
  printf(
    "callback: [%s] [%d bytes] [%s]\n",
    sample->key.val,
    static_cast<int>(sample->value.len),
    sample->value.val);
}

int main(int /*argc*/, char ** /*argv*/)
{
  printf("opening session...\n");
  ZNSession *session = zn_open(CLIENT_MODE, "tcp/127.0.0.1:7447", NULL);
  if (!session)
  {
    printf("unable to open session :(\n");
    return 1;
  }
  printf("opened session\n");

  ZNSubscriber *sub = zn_declare_subscriber(
    session,
    "/blob",
    callback);

  while (1)
  {
    printf("sleeping\n");
    sleep(5);
  }

  zn_undeclare_subscriber(sub);

  zn_close(session);
  printf("closed session\n");
  return 0;
}
