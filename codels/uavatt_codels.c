/*
 * Copyright (c) 2018 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                           Anthony Mallet on Wed May 30 2018
 */
#include "acuavatt.h"

#include <sys/time.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "uavatt_c_types.h"
#include "codels.h"


/* --- Attribute set_geom ----------------------------------------------- */

/** Validation codel uavatt_set_geom of attribute set_geom.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
uavatt_set_geom(const double G[48], uavatt_ids_body_s *body,
                const genom_context self)
{
  (void)self; /* -Wunused-parameter */
  double f[6];
  int i;

  uavatt_invert_G(G, body->iG);

  uavatt_Gw2(G, body->wmin, f);
  for(i = 0; i < 3; i++) body->thrust_min[0] = f[0];

  uavatt_Gw2(G, body->wmax, f);
  for(i = 0; i < 3; i++) body->thrust_max[0] = f[0];

  return genom_ok;
}


/* --- Attribute set_wlimit --------------------------------------------- */

/** Validation codel uavatt_set_wlimit of attribute set_wlimit.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
uavatt_set_wlimit(double wmin, double wmax, uavatt_ids_body_s *body,
                  const genom_context self)
{
  (void)self; /* -Wunused-parameter */
  double f[6];
  int i;

  uavatt_Gw2(body->G, wmin, f);
  for(i = 0; i < 3; i++) body->thrust_min[i] = f[i];

  uavatt_Gw2(body->G, wmax, f);
  for(i = 0; i < 3; i++) body->thrust_max[i] = f[i];

  return genom_ok;
}


/* --- Attribute set_emerg ---------------------------------------------- */

/** Validation codel uavatt_set_emerg of attribute set_emerg.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
uavatt_set_emerg(uavatt_ids_servo_s_emerg_s *emerg,
                 const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  emerg->dq = emerg->dq * emerg->dq / 9.;
  emerg->dw = emerg->dw * emerg->dw / 9.;
  return genom_ok;
}


/* --- Function set_state ----------------------------------------------- */

/** Codel uavatt_set_state of function set_state.
 *
 * Returns genom_ok.
 */
genom_event
uavatt_set_state(const or_rb3d_force *thrust, const or_t3d_att *att,
                 const or_t3d_avel *avel, const or_t3d_aacc *aacc,
                 or_uav_input *reference, const genom_context self)
{
  (void)self; /* -Wunused-parameter */
  struct timeval tv;

  gettimeofday(&tv, NULL);
  *reference = (or_uav_input){
    .ts = { .sec = tv.tv_sec, .nsec = tv.tv_usec * 1000. },
    .intrinsic = false,

    .thrust = { ._present = true, ._value = *thrust },
    .att = { ._present = true, ._value = *att },
    .avel = { ._present = true, ._value = *avel },
    .aacc = { ._present = true, ._value = *aacc }
  };

  return genom_ok;
}


/* --- Function stop ---------------------------------------------------- */

/** Codel uavatt_servo_stop of function stop.
 *
 * Returns genom_ok.
 */
genom_event
uavatt_servo_stop(or_uav_input *reference, const genom_context self)
{
  (void)self; /* -Wunused-parameter */
  struct timeval tv;

  gettimeofday(&tv, NULL);
  reference->ts.sec = tv.tv_sec;
  reference->ts.nsec = tv.tv_usec * 1000.;

  reference->thrust._present = false;
  reference->att._present = false;
  reference->avel._present = false;
  reference->aacc._present = false;

  return genom_ok;
}


/* --- Function log ----------------------------------------------------- */

/** Codel uavatt_log of function log.
 *
 * Returns genom_ok.
 * Throws uavatt_e_sys.
 */
genom_event
uavatt_log(const char path[64], uint32_t decimation,
           uavatt_log_s **log, const genom_context self)
{
  int fd;

  fd = open(path, O_WRONLY|O_APPEND|O_CREAT|O_TRUNC, 0666);
  if (fd < 0) return uavatt_e_sys_error(path, self);

  if (write(fd, uavatt_log_header_fmt "\n", sizeof(uavatt_log_header_fmt)) < 0)
    return uavatt_e_sys_error(path, self);

  if ((*log)->req.aio_fildes >= 0) {
    close((*log)->req.aio_fildes);

    if ((*log)->pending)
      while (aio_error(&(*log)->req) == EINPROGRESS)
        /* empty body */;
  }
  (*log)->req.aio_fildes = fd;
  (*log)->pending = false;
  (*log)->skipped = false;
  (*log)->decimation = decimation < 1 ? 1 : decimation;
  (*log)->missed = 0;
  (*log)->total = 0;

  return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel uavatt_log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
uavatt_log_stop(uavatt_log_s **log, const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  if (*log && (*log)->req.aio_fildes >= 0)
    close((*log)->req.aio_fildes);
  (*log)->req.aio_fildes = -1;

  return genom_ok;
}


/* --- Function log_info ------------------------------------------------ */

/** Codel uavatt_log_info of function log_info.
 *
 * Returns genom_ok.
 */
genom_event
uavatt_log_info(const uavatt_log_s *log, uint32_t *miss,
                uint32_t *total, const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  *miss = *total = 0;
  if (log) {
    *miss = log->missed;
    *total = log->total;
  }
  return genom_ok;
}
