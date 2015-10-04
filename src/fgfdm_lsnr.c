#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "fgfdm.h"

typedef struct {
  hal_bit_t *data_valid;
  hal_u32_t *timestamp;
  hal_u32_t *msgno;
} FGFDM_LSNR_HAL_T;

static const char *modname = FGFDM_MODULE_NAME "_lsnr";
static int hal_comp_id;
static FGFDM_LSNR_HAL_T *hal_data;

static int lsnr_sock = -1;

static int shmem_id;

static void exitHandler(int sig) {
  if (lsnr_sock > 0) {
    close(lsnr_sock);
  }
}

int main(int argc, char **argv) {
  int ret = 1;
  struct sockaddr_in lsnr_addr;
  FGFDM_SHMEM_T *shmem;
  struct timeval tv;
  ssize_t n;
  FGNetFDM msg;
  long ts;
  int warn_shown;
  int wr_next;
  FGFDM_BUFFER_T *buffer;

  // initialize component
  hal_comp_id = hal_init(modname);
  if (hal_comp_id < 1) {
    fprintf(stderr, "%s: ERROR: hal_init failed\n", modname);
    goto fail0;
  }

  // allocate hal memory
  hal_data = hal_malloc(sizeof(FGFDM_LSNR_HAL_T));
  if (hal_data == NULL) {
    fprintf(stderr, "%s: ERROR: unable to allocate HAL shared memory\n", modname);
    goto fail1;
  }

  // register pins
  if (hal_pin_bit_newf(HAL_OUT, &(hal_data->data_valid), hal_comp_id, "%s.lsnr.data-valid", FGFDM_MODULE_NAME) != 0) {
    fprintf(stderr, "%s: ERROR: unable to register pin %s.lsnr.data-valid\n", modname, FGFDM_MODULE_NAME);
    goto fail1;
  }
  *(hal_data->data_valid) = 0;

  if (hal_pin_u32_newf(HAL_OUT, &(hal_data->timestamp), hal_comp_id, "%s.lsnr.timestamp", FGFDM_MODULE_NAME) != 0) {
    fprintf(stderr, "%s: ERROR: unable to register pin %s.lsnr.timestamp\n", modname, FGFDM_MODULE_NAME);
    goto fail1;
  }
  *(hal_data->timestamp) = 0;

  if (hal_pin_u32_newf(HAL_OUT, &(hal_data->msgno), hal_comp_id, "%s.lsnr.msgno", FGFDM_MODULE_NAME) != 0) {
    fprintf(stderr, "%s: ERROR: unable to register pin %s.lsnr.msgno\n", modname, FGFDM_MODULE_NAME);
    goto fail1;
  }
  *(hal_data->msgno) = 0;

  // initialize signal handling
  signal(SIGINT, exitHandler);
  signal(SIGTERM, exitHandler);

  // get port number
  if (argc != 2) {
    fprintf(stderr, "%s: ERROR: invalid arguments\n", modname);
    goto fail1;
  }
  bzero(&lsnr_addr, sizeof(lsnr_addr));
  lsnr_addr.sin_family = AF_INET;
  lsnr_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  lsnr_addr.sin_port = htons(atoi(argv[1]));

  // setup shared mem for double buffer
  shmem_id = rtapi_shmem_new(FGFDM_SHMEM_KEY, hal_comp_id, sizeof(FGFDM_SHMEM_T));
  if ( shmem_id < 0 ) {
    fprintf(stderr, "%s: ERROR: couldn't allocate user/RT shared memory\n", modname);
    goto fail1;
  }
  if (fgfdm_rtapi_shmem_getptr(shmem_id, (void **) &shmem)) {
    fprintf(stderr, "%s: ERROR: couldn't map user/RT shared memory\n", modname);
    goto fail3;
  }
  bzero(shmem, sizeof(FGFDM_SHMEM_T));

  // create socket
  lsnr_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (lsnr_sock < 0) {
    fprintf(stderr, "%s: ERROR: unable to create UDP socket\n", modname);
    goto fail3;
  }

  // set timeout
  tv.tv_sec = FGFDM_LISTENER_TIMEOUT / 1000;
  tv.tv_usec = (FGFDM_LISTENER_TIMEOUT % 1000) * 1000;
  if (setsockopt(lsnr_sock, SOL_SOCKET, SO_RCVTIMEO, (void *) &tv, sizeof(tv))) {
    fprintf(stderr, "%s: ERROR: unable to set socket timeout\n", modname);
    goto fail4;
  }

  // bind to udp port
  if (bind(lsnr_sock, (struct sockaddr *)&lsnr_addr, sizeof(lsnr_addr))) {
    fprintf(stderr, "%s: ERROR: unable bind listening socket\n", modname);
    goto fail4;
  }

  // everything is fine
  ret = 0;
  hal_ready(hal_comp_id);

  warn_shown = 0;
  for (;;) {
    // read data from flightgear
    n = recv(lsnr_sock, &msg, sizeof(FGNetFDM), 0);
    ts = fgfdm_get_ticks();
    if (n < 0) {
      // kill
      if (errno == EINTR) {
        break;
      }

      // timeout
      if (errno == EAGAIN) {
        *(hal_data->data_valid) = 0;
        continue;
      }

      // other error
      fprintf(stderr, "%s: ERROR: unable to read from socket\n", modname);
      break;
    }

    // get next write buffer
    wr_next = shmem->wr_pos + 1;
    if (wr_next >= FGFDM_BUFFER_COUNT) {
      wr_next = 0;
    }
    
    // check for overflow
    if (wr_next == shmem->rd_pos) {
      if (!warn_shown) {
        warn_shown = 1;
        fprintf(stderr, "%s: WARNING: fifo overflow\n", modname);
      }
      continue;
    }

    // set timestamp
    buffer = &shmem->buffer[wr_next];
    buffer->data_valid = 0;
    buffer->timestamp = ts;
    buffer->msgno = *(hal_data->msgno);

    // check data size
    if (n != sizeof(FGNetFDM)) {
      shmem->wr_pos = wr_next;
      *(hal_data->data_valid) = 0;
      if (!warn_shown) {
        warn_shown = 1;
        fprintf(stderr, "%s: WARNING: invalid data length (is: %ld sould be: %ld)\n", modname, n, sizeof(FGNetFDM));
      }
      continue;
    }

    // convert to host byte order
    ntohfdm(&msg);

    // check version
    if (msg.version != FG_NET_FDM_VERSION) {
      shmem->wr_pos = wr_next;
      *(hal_data->data_valid) = 0;
      if (!warn_shown) {
        warn_shown = 1;
        fprintf(stderr, "%s: WARNING: invalid data version (is: %u sould be: %u)\n", modname, msg.version, FG_NET_FDM_VERSION);
      }
      continue;
    }

    // now data is valid
    memcpy(&buffer->data, &msg, sizeof(FGNetFDM));
    buffer->data_valid = 1;
    shmem->wr_pos = wr_next;
    warn_shown = 0;

    // update status pins
    *(hal_data->data_valid) = 1;
    *(hal_data->timestamp) = ts;
    (*(hal_data->msgno))++;
  }

fail4:
  close(lsnr_sock);
fail3:
  rtapi_shmem_delete(shmem_id, hal_comp_id);
fail1:
  hal_exit(hal_comp_id);
fail0:
  return ret;
}

