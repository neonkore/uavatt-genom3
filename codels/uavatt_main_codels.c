/*
 * Copyright (c) 2018-2019 LAAS/CNRS
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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "uavatt_c_types.h"
#include "codels.h"


/* --- Task main -------------------------------------------------------- */


/** Codel uavatt_main_start of task main.
 *
 * Triggered by uavatt_start.
 * Yields to uavatt_control.
 */
genom_event
uavatt_main_start(uavatt_ids *ids, const genom_context self)
{
  static const double kf = 6.5e-4;
  static const double c = 0.0154;
  static const double d = 0.23;

  ids->body = (uavatt_ids_body_s){
    /* mikrokopter quadrotors defaults */
    .G = {
         0.,    0.,   0.,    0.,   0., 0., 0., 0.,
         0.,    0.,   0.,    0.,   0., 0., 0., 0.,
         kf,    kf,   kf,    kf,   0., 0., 0., 0.,

         0.,  d*kf,   0., -d*kf,   0., 0., 0., 0.,
      -d*kf,    0., d*kf,    0.,   0., 0., 0., 0.,
       c*kf, -c*kf, c*kf, -c*kf,   0., 0., 0., 0.
    },

    .J = {
      0.015,    0.,    0.,
      0.,    0.015,    0.,
      0.,       0., 0.015
    },

    .mass = 1.0,

    .wmax = 90., .wmin = 16.
  };
  uavatt_set_geom(ids->body.G, &ids->body, self);
  uavatt_set_wlimit(ids->body.wmin, ids->body.wmax, &ids->body, self);

  ids->servo = (uavatt_ids_servo_s){
    .gain = {
      .Kqxy = 2.3, .Kwxy = .23, .Kqz = .2, .Kwz = .02,
    },

    .ramp = 3,
    .scale = 0.,

    .emerg = {
      .descent = .1,
      .dq = 5. * 5. * M_PI*M_PI/180./180./9.,
      .dw = 20. * 20. * M_PI*M_PI/180./180./9.
    }
  };

  ids->reference = (or_uav_input){
    .ts = { .sec = 0, .nsec = 0 },
    .intrinsic = false,
    .thrust._present = false,
    .att._present = false,
    .avel._present = false,
    .aacc._present = false,
  };


  /* init logging */
  ids->log = malloc(sizeof(*ids->log));
  if (!ids->log) abort();
  *ids->log = (uavatt_log_s){
    .req = {
      .aio_fildes = -1,
      .aio_offset = 0,
      .aio_buf = ids->log->buffer,
      .aio_nbytes = 0,
      .aio_reqprio = 0,
      .aio_sigevent = { .sigev_notify = SIGEV_NONE },
      .aio_lio_opcode = LIO_NOP
    },
    .pending = false, .skipped = false,
    .decimation = 1, .missed = 0, .total = 0
  };

  return uavatt_control;
}


/** Codel uavatt_main_control of task main.
 *
 * Triggered by uavatt_control.
 * Yields to uavatt_measure.
 */
genom_event
uavatt_main_control(const uavatt_ids_body_s *body,
                    uavatt_ids_servo_s *servo,
                    const uavatt_state *state, or_uav_input *reference,
                    uavatt_log_s **log,
                    const uavatt_rotor_input *rotor_input,
                    const genom_context self)
{
  const or_pose_estimator_state *state_data = NULL;
  or_rotorcraft_input *wprop;
  struct timeval tv;
  size_t i;
  int s;

  wprop = rotor_input->data(self);
  if (!wprop) return uavatt_measure;

  gettimeofday(&tv, NULL);

  /* reset propeller velocities by default - updated later by the controller */
  wprop->ts.sec = tv.tv_sec;
  wprop->ts.nsec = tv.tv_usec * 1000;
  wprop->desired._length = body->rotors;
  for(i = 0; i < wprop->desired._length; i++)
    wprop->desired._buffer[i] = 0.;

  /* current state */
  if (state->read(self) || !(state_data = state->data(self)))
    goto output;
  if (tv.tv_sec + 1e-6 * tv.tv_usec >
      0.5 + state_data->ts.sec + 1e-9 * state_data->ts.nsec)
    goto output;

  /* wait for reception of the first valid input */
  if (reference->ts.sec == 0)
    goto output;

  /* deal with obsolete reference */
  if (tv.tv_sec + 1e-6 * tv.tv_usec >
      0.5 + reference->ts.sec + 1e-9 * reference->ts.nsec) {
    reference->thrust._present = false;
    reference->att._present = false;
    reference->avel._present = false;
    reference->aacc._present = false;
  }

  /* SO(3) controller */
  s = uavatt_controller(body, servo, state_data, reference, *log, wprop);
  if (!s) {
    if (servo->scale < 1.) {
      for(i = 0; i < wprop->desired._length; i++)
        wprop->desired._buffer[i] *= servo->scale;

      servo->scale += 1e-3 * uavatt_control_period_ms / servo->ramp;
    }
  }

  /* output */
output:
  rotor_input->write(self);

  return uavatt_measure;
}


/** Codel uavatt_main_measure of task main.
 *
 * Triggered by uavatt_measure.
 * Yields to uavatt_pause_control.
 */
genom_event
uavatt_main_measure(const uavatt_ids_body_s *body,
                    const uavatt_state *state,
                    const uavatt_rotor_measure *rotor_measure,
                    const uavatt_wrench_measure *wrench_measure,
                    const genom_context self)
{
  const or_pose_estimator_state *state_data;
  const or_rotorcraft_output *rotor_data;
  or_wrench_estimator_state *wrench_data;
  double wprop[or_rotorcraft_max_rotors];
  double wrench[6];
  struct timeval tv;
  double now;
  size_t i;

  wrench_data = wrench_measure->data(self);
  if (!wrench_data) return uavatt_pause_control;

  gettimeofday(&tv, NULL);
  now = tv.tv_sec + 1e-6 * tv.tv_usec;

  *wrench_data = (or_wrench_estimator_state){
    .ts = { .sec = tv.tv_sec, .nsec = 1000 * tv.tv_usec },
    .intrinsic = false,
    .force = { ._present = false },
    .force_cov = { ._present = false },
    .torque = { ._present = false },
    .torque_cov = { ._present = false }
  };

  /* current state (already read by control codel) */
  if (!(state_data = state->data(self)))
    goto output;

  /* current propeller speed */
  if (rotor_measure->read(self) || !(rotor_data = rotor_measure->data(self)))
    goto output;

  for(i = 0; i < rotor_data->rotor._length; i++) {
    if (now > 0.1 +
        rotor_data->rotor._buffer[i].ts.sec +
        1e-9 * rotor_data->rotor._buffer[i].ts.nsec)
      goto output;

    if (rotor_data->rotor._buffer[i].spinning)
      wprop[i] = rotor_data->rotor._buffer[i].velocity;
    else
      wprop[i] = 0.;
  }

  /* wrench */
  if (uavatt_wrench(body, state_data, wprop, wrench))
    goto output;

  *wrench_data = (or_wrench_estimator_state){
    .ts = { .sec = tv.tv_sec, .nsec = 1000 * tv.tv_usec },
    .intrinsic = false,
    .force = {
      ._present = true,
      ._value = { .x = wrench[0], .y = wrench[1], .z = wrench[2] }
    },
    .force_cov = { ._present = false },
    .torque = {
      ._present = true,
      ._value = { .x = wrench[3], .y = wrench[4], .z = wrench[5] }
    },
    .torque_cov = { ._present = false }
  };

output:
  wrench_measure->write(self);

  return uavatt_pause_control;
}


/** Codel uavatt_main_stop of task main.
 *
 * Triggered by uavatt_stop.
 * Yields to uavatt_ether.
 */
genom_event
uavatt_main_stop(const uavatt_rotor_input *rotor_input,
                 const genom_context self)
{
  or_rotorcraft_input *wprop;
  struct timeval tv;
  size_t i;

  wprop = rotor_input->data(self);
  if (!wprop) return uavatt_ether;

  gettimeofday(&tv, NULL);
  wprop->ts.sec = tv.tv_sec;
  wprop->ts.nsec = tv.tv_usec * 1000;
  wprop->control = or_rotorcraft_velocity;

  for(i = 0; i < wprop->desired._length; i++)
    wprop->desired._buffer[i] = 0.;

  rotor_input->write(self);
  return uavatt_ether;
}


/* --- Activity servo --------------------------------------------------- */

/** Codel uavatt_servo_loop of activity servo.
 *
 * Triggered by uavatt_start.
 * Yields to uavatt_pause_start, uavatt_ether.
 * Throws uavatt_e_input.
 */
genom_event
uavatt_servo_loop(const uavatt_uav_input *uav_input,
                  or_uav_input *reference, const genom_context self)
{
  const or_uav_input *input_data;

  if (uav_input->read(self)) return uavatt_e_input(self);
  input_data = uav_input->data(self);
  if (!input_data) return uavatt_e_input(self);

  /* check if timestamps have changed */
  if (reference->ts.nsec != input_data->ts.nsec ||
      reference->ts.sec != input_data->ts.sec)
    *reference = *input_data;

  return uavatt_pause_start;
}

/** Codel uavatt_servo_stop of activity servo.
 *
 * Triggered by uavatt_stop.
 * Yields to uavatt_ether.
 * Throws uavatt_e_input.
 */
genom_event
uavatt_servo_stop(or_uav_input *reference, const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  reference->thrust._present = false;
  reference->att._present = false;
  reference->avel._present = false;
  reference->aacc._present = false;

  return uavatt_ether;
}
