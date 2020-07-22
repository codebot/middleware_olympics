#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <unistd.h>
#include <signal.h>

extern "C" {
#include "zenoh-ffi.h"
}


static bool g_done = false;
void sigint_handler(int)
{
  g_done = true;
}

int main(int /*argc*/, char ** /*argv*/)
{
  signal(SIGINT, sigint_handler);

  printf("opening session...\n");
  ZNSession *session = zn_open(PEER_MODE, NULL, NULL);
  if (!session)
  {
    printf("unable to open session :(\n");
    return 1;
  }
  printf("opened session\n");

  /*
  unsigned long resource_id = zn_declare_resource(session, "/blob");
  printf("resource id: %zu\n", resource_id);
  */

  srandom(0);
  const int BLOB_SIZE = 10000000;
  std::vector<uint8_t> blob;
  blob.reserve(BLOB_SIZE);
  for (int i = 0; i < BLOB_SIZE; i++)
    blob.push_back(random() % 256);

  for (int count = 0; !g_done; count++)
  {
    /*char msg_buf[100] = {0};
    snprintf(msg_buf, sizeof(msg_buf), "Hello, world! %d", count);*/
    zn_write(
      session,
      "/blob",
      (char *)&blob[0],
      BLOB_SIZE);
      // strlen(msg_buf));
    usleep(500000);
  }

  zn_close(session);
  printf("closed session\n");
  return 0;
}
