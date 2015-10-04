#ifndef _FGFDM_H
#define _FGFDM_H

#include "rtapi.h"
#include "hal.h"

#include "fgfdm_rtapi.h"

#include "net_fdm.h"

#define FGFDM_MODULE_NAME "fgfdm"
#define FGFDM_SHMEM_KEY 0xed3e3f4a
#define FGFDM_BUFFER_COUNT 2

#define FGFDM_LISTENER_TIMEOUT 3000

typedef struct {
  int data_valid;
  uint32_t timestamp;
  uint32_t msgno;
  FGNetFDM data;
} FGFDM_BUFFER_T;

typedef struct {
  int wr_pos;
  int rd_pos;
  FGFDM_BUFFER_T buffer[FGFDM_BUFFER_COUNT];
} FGFDM_SHMEM_T;

#endif
