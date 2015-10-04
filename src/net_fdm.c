#include <arpa/inet.h>

#include "net_fdm.h"

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__

// The function ntohd is defined this way due to the way some
// processors and OSes treat floating point values.  Some will raise
// an exception whenever a "bad" floating point value is loaded into a
// floating point register.  Solaris is notorious for this, but then
// so is LynxOS on the PowerPC.  By translating the data in place,
// there is no need to load a FP register with the "corruped" floating
// point value.  By doing the BIG_ENDIAN test, I can optimize the
// routine for big-endian processors so it can be as efficient as
// possible
void ntohd(double *x) {
    uint32_t *ovl = (uint32_t *) x;
    uint32_t tmp;

    tmp = ovl[0];
    ovl[0] = ntohl(ovl[1]);
    ovl[1] = ntohl(tmp);
}

// float version
void ntohf(float *x)	{
    uint32_t *ovl = (uint32_t *) x;
    *ovl = ntohl(*ovl);
}

void ntohfdm(FGNetFDM *net) {
    int i;

    // Convert to the net buffer from network format
    net->version = ntohl(net->version);

    ntohd(&net->longitude);
    ntohd(&net->latitude);
    ntohd(&net->altitude);
    ntohf(&net->agl);
    ntohf(&net->phi);
    ntohf(&net->theta);
    ntohf(&net->psi);
    ntohf(&net->alpha);
    ntohf(&net->beta);

    ntohf(&net->phidot);
    ntohf(&net->thetadot);
    ntohf(&net->psidot);
    ntohf(&net->vcas);
    ntohf(&net->climb_rate);
    ntohf(&net->v_north);
    ntohf(&net->v_east);
    ntohf(&net->v_down);
    ntohf(&net->v_body_u);
    ntohf(&net->v_body_v);
    ntohf(&net->v_body_w);

    ntohf(&net->A_X_pilot);
    ntohf(&net->A_Y_pilot);
    ntohf(&net->A_Z_pilot);

    ntohf(&net->stall_warning);
    ntohf(&net->slip_deg);

    net->num_engines = ntohl(net->num_engines);
    for ( i = 0; i < net->num_engines; ++i ) {
        net->eng_state[i] = ntohl(net->eng_state[i]);
        ntohf(&net->rpm[i]);
        ntohf(&net->fuel_flow[i]);
        ntohf(&net->fuel_px[i]);
        ntohf(&net->egt[i]);
        ntohf(&net->cht[i]);
        ntohf(&net->mp_osi[i]);
        ntohf(&net->tit[i]);
        ntohf(&net->oil_temp[i]);
        ntohf(&net->oil_px[i]);
    }

    net->num_tanks = ntohl(net->num_tanks);
    for ( i = 0; i < net->num_tanks; ++i ) {
        ntohf(&net->fuel_quantity[i]);
    }

    net->num_wheels = ntohl(net->num_wheels);
    for ( i = 0; i < net->num_wheels; ++i ) {
        net->wow[i] = ntohl(net->wow[i]);
        ntohf(&net->gear_pos[i]);
        ntohf(&net->gear_steer[i]);
        ntohf(&net->gear_compression[i]);
    }

    net->cur_time = ntohl(net->cur_time);
    net->warp = ntohl(net->warp);
    ntohf(&net->visibility);

    ntohf(&net->elevator);
    ntohf(&net->elevator_trim_tab);
    ntohf(&net->left_flap);
    ntohf(&net->right_flap);
    ntohf(&net->left_aileron);
    ntohf(&net->right_aileron);
    ntohf(&net->rudder);
    ntohf(&net->nose_wheel);
    ntohf(&net->speedbrake);
    ntohf(&net->spoilers);
}

#else

void ntohfdm(FGNetFDM *net) {
}

#endif

