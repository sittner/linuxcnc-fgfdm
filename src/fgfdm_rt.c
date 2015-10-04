#include "fgfdm.h"

#include "rtapi_app.h"
#include "rtapi_math.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sascha Ittner <sascha.ittner@modusoft.de>");
MODULE_DESCRIPTION("FlightGear NetFDM to HAL interface");

#define RAD2DEG(a) (a * (180.0 / M_PI))

typedef struct {
    // statistic data
    hal_bit_t *data_valid;
    hal_u32_t *timestamp;
    hal_u32_t *msgno;

    // Positions
    hal_float_t *longitude;
    hal_float_t *latitude;
    hal_float_t *altitude;
    hal_float_t *agl;
    hal_float_t *phi;
    hal_float_t *theta;
    hal_float_t *psi;
    hal_float_t *alpha;
    hal_float_t *beta;

    // Velocities
    hal_float_t *phidot;
    hal_float_t *thetadot;
    hal_float_t *psidot;
    hal_float_t *vcas;
    hal_float_t *climb_rate;
    hal_float_t *v_north;
    hal_float_t *v_east;
    hal_float_t *v_down;
    hal_float_t *v_body_u;
    hal_float_t *v_body_v;
    hal_float_t *v_body_w;
    
    // Accelerations
    hal_float_t *A_X_pilot;
    hal_float_t *A_Y_pilot;
    hal_float_t *A_Z_pilot;

    // Stall
    hal_float_t *stall_warning;
    hal_float_t *slip_deg;

    // Pressure
    
    // Engine status
    hal_u32_t *num_engines;
    hal_u32_t *eng_state[FG_MAX_ENGINES];
    hal_float_t *rpm[FG_MAX_ENGINES];
    hal_float_t *fuel_flow[FG_MAX_ENGINES];
    hal_float_t *fuel_px[FG_MAX_ENGINES];
    hal_float_t *egt[FG_MAX_ENGINES];
    hal_float_t *cht[FG_MAX_ENGINES];
    hal_float_t *mp_osi[FG_MAX_ENGINES];
    hal_float_t *tit[FG_MAX_ENGINES];
    hal_float_t *oil_temp[FG_MAX_ENGINES];
    hal_float_t *oil_px[FG_MAX_ENGINES];

    // Consumables
    hal_u32_t *num_tanks;
    hal_float_t *fuel_quantity[FG_MAX_TANKS];

    // Gear status
    hal_u32_t *num_wheels;
    hal_u32_t *wow[FG_MAX_WHEELS];
    hal_float_t *gear_pos[FG_MAX_WHEELS];
    hal_float_t *gear_steer[FG_MAX_WHEELS];
    hal_float_t *gear_compression[FG_MAX_WHEELS];

    // Environment
    hal_u32_t *cur_time;
    hal_s32_t *warp;
    hal_float_t *visibility;

    // Control surface positions (normalized values)
    hal_float_t *elevator;
    hal_float_t *elevator_trim_tab;
    hal_float_t *left_flap;
    hal_float_t *right_flap;
    hal_float_t *left_aileron;
    hal_float_t *right_aileron;
    hal_float_t *rudder;
    hal_float_t *nose_wheel;
    hal_float_t *speedbrake;
    hal_float_t *spoilers;

    long long timeout;
} FGFDM_HAL_T;

static int comp_id = -1;
static int shmem_id = -1;
static FGFDM_SHMEM_T *shmem;
static FGFDM_HAL_T *hal_data;

void fgfdm_read(void *arg, long period) {
  FGFDM_BUFFER_T *buffer;
  FGNetFDM *data;
  int i, rd_pos;

  // check if data available
  rd_pos = shmem->rd_pos;
  if (rd_pos == shmem->wr_pos) {
    if (hal_data->timeout > 0) {
      hal_data->timeout -= period;
    } else {
      *(hal_data->data_valid) = 0;
    }
    return;
  }

  // get current read buffer
  buffer = &shmem->buffer[rd_pos];

  // set statistics data
  hal_data->timeout = FGFDM_LISTENER_TIMEOUT * 1000000LL;
  *(hal_data->data_valid) = buffer->data_valid;
  *(hal_data->timestamp) = buffer->timestamp;
  *(hal_data->msgno) = buffer->msgno;

  // update flightgear data
  data = &buffer->data;

  *(hal_data->longitude) = RAD2DEG(data->longitude);
  *(hal_data->latitude) = RAD2DEG(data->latitude);
  *(hal_data->altitude) = data->altitude;
  *(hal_data->agl) = data->agl;
  *(hal_data->phi) = RAD2DEG(data->phi);
  *(hal_data->theta) = RAD2DEG(data->theta);
  *(hal_data->psi) = RAD2DEG(data->psi);
  *(hal_data->alpha) = RAD2DEG(data->alpha);
  *(hal_data->beta) = RAD2DEG(data->beta);

  *(hal_data->phidot) = RAD2DEG(data->phidot);
  *(hal_data->thetadot) = RAD2DEG(data->thetadot);
  *(hal_data->psidot) = RAD2DEG(data->psidot);
  *(hal_data->vcas) = data->vcas;
  *(hal_data->climb_rate) = data->climb_rate;
  *(hal_data->v_north) = data->v_north;
  *(hal_data->v_east) = data->v_east;
  *(hal_data->v_down) = data->v_down;
  *(hal_data->v_body_u) = data->v_body_u;
  *(hal_data->v_body_v) = data->v_body_v;
  *(hal_data->v_body_w) = data->v_body_w;

  *(hal_data->A_X_pilot) = data->A_X_pilot;
  *(hal_data->A_Y_pilot) = data->A_Y_pilot;
  *(hal_data->A_Z_pilot) = data->A_Z_pilot;

  *(hal_data->stall_warning) = data->stall_warning;
  *(hal_data->slip_deg) = data->slip_deg;

  *(hal_data->num_engines) = data->num_engines;
  for (i=0; i<FG_MAX_ENGINES; i++) {
    *(hal_data->eng_state[i]) = data->eng_state[i];
    *(hal_data->rpm[i]) = data->rpm[i];
    *(hal_data->fuel_flow[i]) = data->fuel_flow[i];
    *(hal_data->fuel_px[i]) = data->fuel_px[i];
    *(hal_data->egt[i]) = data->egt[i];
    *(hal_data->cht[i]) = data->cht[i];
    *(hal_data->mp_osi[i]) = data->mp_osi[i];
    *(hal_data->tit[i]) = data->tit[i];
    *(hal_data->oil_temp[i]) = data->oil_temp[i];
    *(hal_data->oil_px[i]) = data->oil_px[i];
  }

  *(hal_data->num_tanks) = data->num_tanks;
  for (i=0; i<FG_MAX_TANKS; i++) {
    *(hal_data->fuel_quantity[i]) = data->fuel_quantity[i];
  }

  *(hal_data->num_wheels) = data->num_wheels;
  for (i=0; i<FG_MAX_WHEELS; i++) {
    *(hal_data->wow[i]) = data->wow[i];
    *(hal_data->gear_pos[i]) = data->gear_pos[i];
    *(hal_data->gear_steer[i]) = data->gear_steer[i];
    *(hal_data->gear_compression[i]) = data->gear_compression[i];
  }

  *(hal_data->cur_time) = data->cur_time;
  *(hal_data->warp) = data->warp;
  *(hal_data->visibility) = data->visibility;
  *(hal_data->elevator) = data->elevator;
  *(hal_data->elevator_trim_tab) = data->elevator_trim_tab;
  *(hal_data->left_flap) = data->left_flap;
  *(hal_data->right_flap) = data->right_flap;
  *(hal_data->left_aileron) = data->left_aileron;
  *(hal_data->right_aileron) = data->right_aileron;
  *(hal_data->rudder) = data->rudder;
  *(hal_data->nose_wheel) = data->nose_wheel;
  *(hal_data->speedbrake) = data->speedbrake;
  *(hal_data->spoilers) = data->spoilers;

  // update read pointer
  rd_pos++;
  if (rd_pos >= FGFDM_BUFFER_COUNT) {
    rd_pos = 0;
  }
  shmem->rd_pos = rd_pos;
}

int rtapi_app_main(void) {
  char name[HAL_NAME_LEN + 1];
  int i;

  // connect to the HAL
  if ((comp_id = hal_init (FGFDM_MODULE_NAME)) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: hal_init() failed\n");
    goto fail0;
  }

  // open shmem segment
  shmem_id = rtapi_shmem_new(FGFDM_SHMEM_KEY, comp_id, sizeof(FGFDM_SHMEM_T));
  if (shmem_id < 0) {
    rtapi_print_msg (RTAPI_MSG_ERR, "FGFDM: couldn't allocate user/RT shared memory\n");
    goto fail1;
  }
  if (fgfdm_rtapi_shmem_getptr(shmem_id, (void **) &shmem) < 0 ) {
    rtapi_print_msg (RTAPI_MSG_ERR, "FGFDM: couldn't map user/RT shared memory\n");
    goto fail2;
  }

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(FGFDM_HAL_T))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: hal_malloc() failed\n");
    goto fail2;
  }

  // export pins
  if (hal_pin_bit_newf(HAL_OUT, &(hal_data->data_valid), comp_id, "%s.data-valid", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.data-valid failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->data_valid) = 0;

  if (hal_pin_u32_newf(HAL_OUT, &(hal_data->timestamp), comp_id, "%s.timestamp", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.timestamp failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->timestamp) = 0;

  if (hal_pin_u32_newf(HAL_OUT, &(hal_data->msgno), comp_id, "%s.msgno", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.msgno failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->msgno) = 0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->longitude), comp_id, "%s.pos.longitude", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.pos.longitude failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->longitude) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->latitude), comp_id, "%s.pos.latitude", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.pos.latitude failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->latitude) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->altitude), comp_id, "%s.pos.altitude", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.pos.altitude failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->altitude) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->agl), comp_id, "%s.pos.agl", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.pos.agl failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->agl) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->phi), comp_id, "%s.pos.phi", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.pos.phi failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->phi) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->theta), comp_id, "%s.pos.theta", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.pos.theta failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->theta) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->psi), comp_id, "%s.pos.psi", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.pos.psi failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->psi) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->alpha), comp_id, "%s.pos.alpha", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.pos.alpha failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->alpha) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->beta), comp_id, "%s.pos.beta", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.pos.beta failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->beta) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->phidot), comp_id, "%s.velo.phidot", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.velo.phidot failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->phidot) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->thetadot), comp_id, "%s.velo.thetadot", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.velo.thetadot failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->thetadot) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->psidot), comp_id, "%s.velo.psidot", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.velo.psidot failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->psidot) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->vcas), comp_id, "%s.velo.vcas", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.velo.vcas failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->vcas) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->climb_rate), comp_id, "%s.velo.climb_rate", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.velo.climb_rate failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->climb_rate) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->v_north), comp_id, "%s.velo.v_north", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.velo.v_north failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->v_north) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->v_east), comp_id, "%s.velo.v_east", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.velo.v_east failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->v_east) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->v_down), comp_id, "%s.velo.v_down", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.velo.v_down failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->v_down) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->v_body_u), comp_id, "%s.velo.v_body_u", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.velo.v_body_u failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->v_body_u) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->v_body_v), comp_id, "%s.velo.v_body_v", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.velo.v_body_v failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->v_body_v) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->v_body_w), comp_id, "%s.velo.v_body_w", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.velo.v_body_w failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->v_body_w) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->A_X_pilot), comp_id, "%s.accel.A_X_pilot", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.accel.A_X_pilot failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->A_X_pilot) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->A_Y_pilot), comp_id, "%s.accel.A_Y_pilot", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.accel.A_Y_pilot failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->A_Y_pilot) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->A_Z_pilot), comp_id, "%s.accel.A_Z_pilot", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.accel.A_Z_pilot failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->A_Z_pilot) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->stall_warning), comp_id, "%s.stall.stall_warning", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.stall.stall_warning failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->stall_warning) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->slip_deg), comp_id, "%s.stall.slip_deg", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.stall.slip_deg failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->slip_deg) = 0.0;

  if (hal_pin_u32_newf(HAL_OUT, &(hal_data->num_engines), comp_id, "%s.engine.num_engines", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.engine.num_engines failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->num_engines) = 0;

  for (i=0; i<FG_MAX_ENGINES; i++) {
    if (hal_pin_u32_newf(HAL_OUT, &(hal_data->eng_state[i]), comp_id, "%s.engine.%d.eng_state", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.engine.%d.eng_state failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->eng_state[i]) = 0;

    if (hal_pin_float_newf(HAL_OUT, &(hal_data->rpm[i]), comp_id, "%s.engine.%d.rpm", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.engine.%d.rpm failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->rpm[i]) = 0.0;

    if (hal_pin_float_newf(HAL_OUT, &(hal_data->fuel_flow[i]), comp_id, "%s.engine.%d.fuel_flow", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.engine.%d.fuel_flow failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->fuel_flow[i]) = 0.0;

    if (hal_pin_float_newf(HAL_OUT, &(hal_data->fuel_px[i]), comp_id, "%s.engine.%d.fuel_px", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.engine.%d.fuel_px failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->fuel_px[i]) = 0.0;

    if (hal_pin_float_newf(HAL_OUT, &(hal_data->egt[i]), comp_id, "%s.engine.%d.egt", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.engine.%d.egt failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->egt[i]) = 0.0;

    if (hal_pin_float_newf(HAL_OUT, &(hal_data->cht[i]), comp_id, "%s.engine.%d.cht", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.engine.%d.cht failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->cht[i]) = 0.0;

    if (hal_pin_float_newf(HAL_OUT, &(hal_data->mp_osi[i]), comp_id, "%s.engine.%d.mp_osi", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.engine.%d.mp_osi failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->mp_osi[i]) = 0.0;

    if (hal_pin_float_newf(HAL_OUT, &(hal_data->tit[i]), comp_id, "%s.engine.%d.tit", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.engine.%d.tit failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->tit[i]) = 0.0;

    if (hal_pin_float_newf(HAL_OUT, &(hal_data->oil_temp[i]), comp_id, "%s.engine.%d.oil_temp", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.engine.%d.oil_temp failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->oil_temp[i]) = 0.0;

    if (hal_pin_float_newf(HAL_OUT, &(hal_data->oil_px[i]), comp_id, "%s.engine.%d.oil_px", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.engine.%d.oil_px failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->oil_px[i]) = 0.0;
  }

  if (hal_pin_u32_newf(HAL_OUT, &(hal_data->num_tanks), comp_id, "%s.cons.num_tanks", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.cons.num_tanks failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->num_tanks) = 0;

  for (i=0; i<FG_MAX_TANKS; i++) {
    if (hal_pin_float_newf(HAL_OUT, &(hal_data->fuel_quantity[i]), comp_id, "%s.cons.%d.fuel_quantity", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.cons.%d.fuel_quantity failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->fuel_quantity[i]) = 0.0;
  }

  if (hal_pin_u32_newf(HAL_OUT, &(hal_data->num_wheels), comp_id, "%s.gear.num_wheels", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.gear.num_wheels failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->num_wheels) = 0;

  for (i=0; i<FG_MAX_WHEELS; i++) {
    if (hal_pin_u32_newf(HAL_OUT, &(hal_data->wow[i]), comp_id, "%s.gear.%d.wow", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.gear.%d.wow failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->wow[i]) = 0;

    if (hal_pin_float_newf(HAL_OUT, &(hal_data->gear_pos[i]), comp_id, "%s.gear.%d.gear_pos", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.gear.%d.gear_pos failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->gear_pos[i]) = 0.0;

    if (hal_pin_float_newf(HAL_OUT, &(hal_data->gear_steer[i]), comp_id, "%s.gear.%d.gear_steer", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.gear.%d.gear_steer failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->gear_steer[i]) = 0.0;

    if (hal_pin_float_newf(HAL_OUT, &(hal_data->gear_compression[i]), comp_id, "%s.gear.%d.gear_compression", FGFDM_MODULE_NAME, i)) {
      rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.gear.%d.gear_compression failed\n", FGFDM_MODULE_NAME, i);
      goto fail2;
    }
    *(hal_data->gear_compression[i]) = 0.0;
  }

  if (hal_pin_u32_newf(HAL_OUT, &(hal_data->cur_time), comp_id, "%s.env.cur_time", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.env.cur_time failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->cur_time) = 0;

  if (hal_pin_s32_newf(HAL_OUT, &(hal_data->warp), comp_id, "%s.env.warp", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.env.warp failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->warp) = 0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->visibility), comp_id, "%s.env.visibility", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.env.visibility failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->visibility) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->elevator), comp_id, "%s.ctrl.elevator", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.ctrl.elevator failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->elevator) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->elevator_trim_tab), comp_id, "%s.ctrl.elevator_trim_tab", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.ctrl.elevator_trim_tab failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->elevator_trim_tab) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->left_flap), comp_id, "%s.ctrl.left_flap", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.ctrl.left_flap failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->left_flap) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->right_flap), comp_id, "%s.ctrl.right_flap", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.ctrl.right_flap failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->right_flap) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->left_aileron), comp_id, "%s.ctrl.left_aileron", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.ctrl.left_aileron failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->left_aileron) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->right_aileron), comp_id, "%s.ctrl.right_aileron", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.ctrl.right_aileron failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->right_aileron) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->rudder), comp_id, "%s.ctrl.rudder", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.ctrl.rudder failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->rudder) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->nose_wheel), comp_id, "%s.ctrl.nose_wheel", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.ctrl.nose_wheel failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->nose_wheel) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->speedbrake), comp_id, "%s.ctrl.speedbrake", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.ctrl.speedbrake failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->speedbrake) = 0.0;

  if (hal_pin_float_newf(HAL_OUT, &(hal_data->spoilers), comp_id, "%s.ctrl.spoilers", FGFDM_MODULE_NAME)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "FGFDM: exporting pin %s.ctrl.spoilers failed\n", FGFDM_MODULE_NAME);
    goto fail2;
  }
  *(hal_data->spoilers) = 0.0;

  // initialize internal values
  hal_data->timeout = 0;

  // export read function
  rtapi_snprintf(name, HAL_NAME_LEN, "%s.read", FGFDM_MODULE_NAME);
  if (hal_export_funct(name, fgfdm_read, NULL, 0, 0, comp_id)) {
    rtapi_print_msg (RTAPI_MSG_ERR, "FGFDM: read funct export failed\n");
    goto fail2;
  }

  hal_ready (comp_id);
  return 0;

fail2:
  rtapi_shmem_delete(shmem_id, comp_id);
fail1:
  hal_exit(comp_id);
fail0:
  return -EINVAL;
}

void rtapi_app_exit(void) {
  rtapi_shmem_delete(shmem_id, comp_id);
  hal_exit(comp_id);
}

