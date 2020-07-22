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
  ZNSession *session = zn_open(CLIENT_MODE, "tcp/127.0.0.1:7447", NULL);
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

  /*
  srandom(0);
  const int BLOB_SIZE = 1000;
  std::vector<uint8_t> blob;
  blob.reserve(BLOB_SIZE);
  for (int i = 0; i < BLOB_SIZE; i++)
    blob.push_back(random() % 256);
  const void *blob_ptr = (void *)&blob[0];
  */


  for (int count = 0; !g_done; count++)
  {
    char msg_buf[100] = {0};
    snprintf(msg_buf, sizeof(msg_buf), "Hello, world! %d", count);
    zn_write(
      session,
      "/blob",
      msg_buf,
      strlen(msg_buf));
    usleep(1000);
  }

  zn_close(session);
  printf("closed session\n");
  return 0;
}
