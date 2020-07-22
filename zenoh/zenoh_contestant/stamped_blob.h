#ifndef STAMPED_BLOB_H
#define STAMPED_BLOB_H

#include <stdint.h>

typedef struct {
  uint64_t counter;
  uint32_t seconds;
  uint32_t nanoseconds;
  uint32_t blob_length;
  uint8_t blob_bytes[];
} stamped_blob_t;

#endif
