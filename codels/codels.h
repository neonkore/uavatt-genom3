/*
 * Copyright (c) 2016-2018 LAAS/CNRS
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
 *                                           Anthony Mallet on Thu May 31 2018
 */
#ifndef H_UAVATT_CODELS
#define H_UAVATT_CODELS

#include <aio.h>

#include "uavatt_c_types.h"

#ifdef __cplusplus
extern "C" {
#endif

  int	uavatt_controller(const uavatt_ids_body_s *body,
                          const uavatt_ids_servo_s *servo,
                          const or_pose_estimator_state *state,
                          const or_uav_input *desired,
                          uavatt_log_s *log,
                          or_rotorcraft_input *wprop);
  int	uavatt_wrench(const uavatt_ids_body_s *body,
                      const or_pose_estimator_state *state,
                      const double wprop[or_rotorcraft_max_rotors],
                      double wrench[6]);

  void	uavatt_invert_G(const double G[6 * or_rotorcraft_max_rotors],
                        double iG[or_rotorcraft_max_rotors * 6]);
  void	uavatt_Gw2(const double G[6 * or_rotorcraft_max_rotors], const double w,
                   double f[6]);

#ifdef __cplusplus
}
#endif

static inline genom_event
uavatt_e_sys_error(const char *s, genom_context self)
{
  uavatt_e_sys_detail d;
  size_t l = 0;

  d.code = errno;
  if (s) {
    strncpy(d.what, s, sizeof(d.what) - 3);
    l = strlen(s);
    strcpy(d.what + l, ": ");
    l += 2;
  }
  if (strerror_r(d.code, d.what + l, sizeof(d.what) - l)) {
    /* ignore error*/;
  }
  return uavatt_e_sys(&d, self);
}

struct uavatt_log_s {
  struct aiocb req;
  char buffer[4096];
  bool pending, skipped;
  uint32_t decimation;
  size_t missed, total;

# define uavatt_g	" %g "
# define uavatt_log_header_fmt                                          \
  "ts delay "                                                           \
  "fz tx ty tz "                                                        \
  "roll pitch yaw "                                                     \
  "wx wy wz awx awy awz "                                               \
  "e_rx e_ry e_rz e_wx e_wy e_wz"
# define uavatt_log_fmt                                                 \
  "%d.%09d " uavatt_g                                                   \
  uavatt_g uavatt_g uavatt_g uavatt_g                                   \
  uavatt_g uavatt_g uavatt_g                                            \
  uavatt_g uavatt_g uavatt_g uavatt_g uavatt_g uavatt_g                 \
  uavatt_g uavatt_g uavatt_g uavatt_g uavatt_g uavatt_g
};

#endif /* H_UAVATT_CODELS */
