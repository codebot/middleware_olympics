#include <cstdio>
#include <cstdlib>
#include <vector>

#include <unistd.h>

extern "C" {
#include "zenoh-ffi.h"
}

void callback(const zn_sample *sample)
{
  printf("callback (%u bytes)\n", sample->value.len);
}

int main(int /*argc*/, char ** /*argv*/)
{
  printf("opening session...\n");
  ZNSession *session = zn_open(PEER, NULL, NULL);
  if (!session)
  {
    printf("unable to open session :(\n");
    return 1;
  }
  printf("opened session\n");

  ZNSubscriber *sub = zn_declare_subscriber(
    session,
    "/blob",
    zn_subinfo_default(),
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
