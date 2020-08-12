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
void sigint_handler(int)
{
  g_done = true;
}

void callback(const zn_sample *sample)
{
  printf(
    "callback: [%d bytes]\n",
    static_cast<int>(sample->value.len));

  stamped_blob_t *rx_msg = (stamped_blob_t *)sample->value.val;

  printf("rx time: (%u, %u)\n", rx_msg->seconds, rx_msg->nanoseconds);

  struct timespec now;
  clock_gettime(CLOCK_REALTIME, &now);

  uint32_t sec_diff = (uint32_t)now.tv_sec - rx_msg->seconds;
  uint32_t nsec_diff = (uint32_t)now.tv_nsec - rx_msg->nanoseconds;

  if (nsec_diff > 1000000000)
  {
    nsec_diff -= 1000000000;
    sec_diff++;
  }

  double dt = sec_diff + 1.0e-9 * (double)nsec_diff;
  printf("dt = %.6f\n", dt);
}

int main(int /*argc*/, char ** /*argv*/)
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

  /*
  unsigned long resource_id = zn_declare_resource(session, "/blob");
  printf("resource id: %zu\n", resource_id);
  */
  const int BLOB_SIZE = 1000000;
  uint8_t *buffer = (uint8_t *)malloc(sizeof(stamped_blob_t) + BLOB_SIZE);
  stamped_blob_t *tx_msg = (stamped_blob_t *)buffer;
//    (stamped_blob_t *)malloc(sizeof(stamped_blob_t) + BLOB_SIZE);
  tx_msg->blob_length = BLOB_SIZE;

  srandom(0);
  for (int i = 0; i < BLOB_SIZE; i++)
    tx_msg->blob_bytes[i] = random() % 256;

  ZNSubscriber *sub = zn_declare_subscriber(
    session,
    "/echo_out",
    zn_subinfo_default(),
    callback);

  for (int count = 0; !g_done; count++)
  {
    usleep(500000);

    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    tx_msg->counter = count;
    tx_msg->seconds = (uint32_t)spec.tv_sec;
    tx_msg->nanoseconds = (uint32_t)spec.tv_nsec;

    zn_write(
      session,
      "/echo_in",
      (const char *)tx_msg,
      sizeof(stamped_blob_t) + BLOB_SIZE);
  }

  zn_undeclare_subscriber(sub);

  zn_close(session);
  printf("closed session\n");
  free(buffer);
  return 0;
}
