#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

#include <time.h>
#include <unistd.h>
#include <signal.h>

#include "stamped_blob.h"

extern "C" {
#include "zenoh-ffi.h"
}

static bool g_done = false;
static bool g_ball_returned = false;
static FILE *g_log = nullptr;

///////////////////////////////////////////////////

void sigint_handler(int)
{
  g_done = true;
}

void callback(const zn_sample *sample)
{
  /*
  printf(
    "callback: [%d bytes]\n\n",
    static_cast<int>(sample->value.len));
  */

  stamped_blob_t *rx_msg = (stamped_blob_t *)sample->value.val;

  struct timespec now;
  clock_gettime(CLOCK_REALTIME, &now);

  int32_t sec_diff = (int32_t)now.tv_sec - rx_msg->seconds;
  int32_t nsec_diff = (int32_t)now.tv_nsec - rx_msg->nanoseconds;

  if (nsec_diff < 0)
  {
    nsec_diff += 1000000000;
    sec_diff--;
  }

  double dt = sec_diff + 1.0e-9 * (double)nsec_diff;

  /*
  if (dt > 0.1)
  {
    printf("tx time: (%u, %u)\n", rx_msg->seconds, rx_msg->nanoseconds);
    printf("rx time: (%u, %zu)\n", (unsigned)now.tv_sec, now.tv_nsec);
    printf("diff: (%u, %u)\n", sec_diff, nsec_diff);
    printf("  %d dt = %.6f\n", (int)sample->value.len, dt);
    printf("\n\n\n");
  }
  */


  fprintf(g_log, "%d,%.6f\n", (int)sample->value.len, dt);

  g_ball_returned = true;
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

  g_log = fopen("log.txt", "w");

  const int MAX_BLOB_SIZE = 10000000;
  uint8_t *buffer = (uint8_t *)malloc(sizeof(stamped_blob_t) + MAX_BLOB_SIZE);
  stamped_blob_t *tx_msg = (stamped_blob_t *)buffer;
  srandom(0);
  for (int i = 0; i < MAX_BLOB_SIZE; i++)
    tx_msg->blob_bytes[i] = random() % 256;

//    (stamped_blob_t *)malloc(sizeof(stamped_blob_t) + BLOB_SIZE);

  std::mt19937 gen(1234);  // constant seed!
  std::uniform_real_distribution<double>
    magnitude(1.0, log(MAX_BLOB_SIZE) / log(10));

  /*
  unsigned long resource_id = zn_declare_resource(session, "/blob");
  printf("resource id: %zu\n", resource_id);
  */

  ZNSubscriber *sub = zn_declare_subscriber(
    session,
    "/ball_return",
    zn_subinfo_default(),
    callback);

  for (int count = 0; !g_done; count++)
  {
    const int blob_size = static_cast<int>(pow(10.0, magnitude(gen)));

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
  }

  zn_undeclare_subscriber(sub);

  zn_close(session);
  printf("closed session\n");
  free(buffer);
  return 0;
}
