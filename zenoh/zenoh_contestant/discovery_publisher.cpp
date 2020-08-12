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
  ZNSession *session = zn_open(PEER, NULL, NULL);
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

  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  const double t_start = spec.tv_sec + 1.0e-9 * spec.tv_nsec;
  const double MAX_SECONDS = 5.5;

  for (int count = 0; !g_done; count++)
  {
    clock_gettime(CLOCK_REALTIME, &spec);
    const double t = spec.tv_sec + 1.0e-9 * spec.tv_nsec;
    if (t - t_start > MAX_SECONDS)
      break;

    usleep(100000);

    /*
    tx_msg->counter = count;
    tx_msg->seconds = (uint32_t)spec.tv_sec;
    tx_msg->nanoseconds = (uint32_t)spec.tv_nsec;

    const int blob_size = 0;
    tx_msg->blob_length = blob_size;

    printf("sending blob (%d)\n", blob_size);
    zn_write(
      session,
      "/world",
      (const char *)tx_msg,
      sizeof(stamped_blob_t) + blob_size);
    */
  }

  zn_close(session);
  printf("closed session\n");
  free(buffer);
  return 0;
}
