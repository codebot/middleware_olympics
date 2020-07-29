#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <time.h>
#include <unistd.h>
#include <signal.h>

#include "stamped_blob.h"

extern "C" {
#include "zenoh-ffi.h"
}

static bool g_done = false;

///////////////////////////////////////////////////

void sigint_handler(int)
{
  g_done = true;
}

int main(int argc, char ** argv)
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

  const int MSG_SIZE = 100;
  uint8_t *buffer = (uint8_t *)malloc(sizeof(stamped_blob_t) + MSG_SIZE);
  stamped_blob_t *tx_msg = (stamped_blob_t *)buffer;
  for (int i = 0; i < MSG_SIZE; i++)
    tx_msg->blob_bytes[i] = i;

  for (int count = 0; !g_done; count++)
  {
    usleep(1000);
    /*
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    tx_msg->counter = count;
    tx_msg->seconds = (uint32_t)spec.tv_sec;
    tx_msg->nanoseconds = (uint32_t)spec.tv_nsec;
    tx_msg->blob_length = blob_size;

    // printf("serving blob %d\n", blob_size);

    zn_write(
      session,
      "/ball_serve",
      (const char *)tx_msg,
      sizeof(stamped_blob_t) + blob_size);

    g_ball_returned = false;

    // wait until we hear back or until a timeout happens
    for (int attempt = 0; !g_ball_returned && attempt < 1000; attempt++)
    {
      usleep(1000);
    }

    usleep(1000);

    if (count % 100 == 0)
      printf("%d samples\n", count);
    */
  }

  zn_close(session);
  printf("closed session\n");
  free(buffer);
  return 0;
}
