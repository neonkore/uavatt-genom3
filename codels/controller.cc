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
 *                                           Anthony Mallet on Thu May 31 2018
 */
#include "acuavatt.h"

#include <sys/time.h>
#include <aio.h>
#include <err.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "codels.h"


/*
 * --- uavatt_controller ---------------------------------------------------
 *
 * Implements the attitude controller bits described in:
 *
 * T. Lee, M. Leoky and N. H. McClamroch, "Geometric tracking control of a
 * quadrotor UAV on SE(3)", 49th IEEE Conference on Decision and Control
 * (CDC), Atlanta, GA, 2010, pp. 5420-5425.
 */

int
uavatt_controller(const uavatt_ids_body_s *body,
                  const uavatt_ids_servo_s *servo,
                  const or_pose_estimator_state *state,
                  const or_uav_input *desired,
                  uavatt_log_s *log,
                  or_rotorcraft_rotor_control *wprop)
{
  using namespace Eigen;

  Vector3d fd;
  Matrix3d Rd;
  Quaternion<double> qd;
  Vector3d wd, awd;

  Matrix3d R;
  Quaternion<double> q;
  Vector3d w;

  Matrix3d E;
  Vector3d eR, ew;

  Matrix<double, 6, 1> wrench;
  Map< Array<double, or_rotorcraft_max_rotors, 1> > wprop_(wprop->_buffer);

  static bool emerg_q, emerg_w;

  /* geometry */
  Map<
    const Matrix<double, or_rotorcraft_max_rotors, 6, RowMajor>
    > iG_(body->iG);

  /* gains */
  const Array3d Kq(servo->gain.Kqxy, servo->gain.Kqxy, servo->gain.Kqz);
  const Array3d Kw(servo->gain.Kwxy, servo->gain.Kwxy, servo->gain.Kwz);


  /* desired state */
  if (desired->thrust._present)
    fd <<
      desired->thrust._value.x,
      desired->thrust._value.y,
      desired->thrust._value.z;
  else
    fd << 0., 0., body->mass * 9.81;

  if (desired->att._present)
    qd.coeffs() <<
      desired->att._value.qx, desired->att._value.qy, desired->att._value.qz,
      desired->att._value.qw;
  else
    qd = Quaternion<double>::Identity();
  Rd = qd.matrix();

  if (desired->avel._present)
    wd <<
      desired->avel._value.wx,
      desired->avel._value.wy,
      desired->avel._value.wz;
  else
    wd << 0., 0., 0.;

  if (desired->aacc._present)
    awd <<
      desired->aacc._value.awx,
      desired->aacc._value.awy,
      desired->aacc._value.awz;
  else
    awd << 0., 0., 0.;


  /* current state */
  if (state->att._present && !std::isnan(state->att._value.qw) &&
      state->att_cov._present &&
      state->att_cov._value.cov[0] < servo->emerg.dq &&
      state->att_cov._value.cov[2] < servo->emerg.dq &&
      state->att_cov._value.cov[5] < servo->emerg.dq &&
      state->att_cov._value.cov[9] < servo->emerg.dq) {
    q.coeffs() <<
      state->att._value.qx, state->att._value.qy, state->att._value.qz,
      state->att._value.qw;
    if (emerg_q)
      warnx("recovered accurate orientation estimation");
    emerg_q = false;
  } else {
    if (!emerg_q)
      warnx("emergency: inaccurate orientation estimation (stddev %g)",
        state->att._present ?
            std::sqrt(std::max(
                        std::max(
                          std::max(
                            state->avel_cov._value.cov[0],
                            state->avel_cov._value.cov[2]),
                          state->avel_cov._value.cov[5]),
                        state->avel_cov._value.cov[9])) :
            nan(""));
    emerg_q = true;
    q = qd;
  }
  R = q.matrix();

  if (state->avel._present && !std::isnan(state->avel._value.wx) &&
      state->avel_cov._present &&
      state->avel_cov._value.cov[0] < servo->emerg.dw &&
      state->avel_cov._value.cov[2] < servo->emerg.dw &&
      state->avel_cov._value.cov[5] < servo->emerg.dw) {
    w << state->avel._value.wx, state->avel._value.wy, state->avel._value.wz;
    if (emerg_w)
      warnx("recovered accurate angular velocity estimation");
    emerg_w = false;
  } else {
    if (!emerg_w)
      warnx("emergency: inaccurate angular velocity estimation (stddev %g)",
        state->avel._present ?
            std::sqrt(std::max(std::max(state->avel_cov._value.cov[0],
                                        state->avel_cov._value.cov[2]),
                               state->avel_cov._value.cov[5])) :
            nan(""));
    emerg_w = true;
    w = wd;
  }


  /* orientation error */
  E = 0.5 * (Rd.transpose()*R - R.transpose()*Rd);
  eR <<
    (E(2, 1) - E(1, 2))/2.,
    (E(0, 2) - E(2, 0))/2.,
    (E(1, 0) - E(0, 1))/2.;

  if (!desired->att._present) eR(2) = 0.;


  /* angular velocity error */
  ew = R.transpose() * (w - wd);


  /* wrench in body frame - XXX assumes vertical thrust in body frame */
  wrench.block<3, 1>(0, 0) << 0., 0., fd.dot(R.col(2));
  wrench.block<3, 1>(3, 0) = - Kq * eR.array() - Kw * ew.array();


  /* thrust limitation */
  if (wrench(2) < body->thrust_min[2]) wrench(2) = body->thrust_min[2];
  if (wrench(2) > body->thrust_max[2]) wrench(2) = body->thrust_max[2];


  /* output */
  wprop_ = (iG_ * wrench).array().sqrt();


  /* torque limitation */
  if ((wprop_.block(0, 0, wprop->_length, 1) < body->wmin).any() ||
      (wprop_.block(0, 0, wprop->_length, 1) > body->wmax).any()) {
    Array<double, 6, 1> k, kmin, kmax;

    kmin << 1., 1., 1., 0., 0., 0.;
    kmax << 1., 1., 1., 1., 1., 1.;

    do {
      k = (kmin + kmax)/2.;

      wprop_ = (iG_ * (wrench.array() * k).matrix()).array().sqrt();

      if ((wprop_.block(0, 0, wprop->_length, 1) < body->wmin).any() ||
          (wprop_.block(0, 0, wprop->_length, 1) > body->wmax).any())
        kmax = k;
      else
        kmin = k;
    } while(((kmax - kmin) > 1e-2).any());

    wrench = wrench.array() * k;
  }


  /* logging */
  if (log->req.aio_fildes >= 0) {
    log->total++;
    if (log->total % log->decimation == 0) {
      if (log->pending) {
        if (aio_error(&log->req) != EINPROGRESS) {
          log->pending = false;
          if (aio_return(&log->req) <= 0) {
            warn("log");
            close(log->req.aio_fildes);
            log->req.aio_fildes = -1;
          }
        } else {
          log->skipped = true;
          log->missed++;
        }
      }
    }

    if (log->req.aio_fildes >= 0 && !log->pending) {
      double d;
      double roll, pitch, yaw;
      struct timeval tv;

      d = hypot(Rd(0,0), Rd(1,0));
      if (fabs(d) > 1e-10) {
        yaw = atan2(Rd(1,0), Rd(0,0));
        roll = atan2(Rd(2,1), Rd(2,2));
      } else {
        yaw = atan2(-Rd(0,1), Rd(1,1));
        roll = 0.;
      }
      pitch = atan2(-Rd(2,0), d);

      gettimeofday(&tv, NULL);
      d =
        tv.tv_sec - state->ts.sec + (tv.tv_usec * 1e3 - state->ts.nsec)*1e-9;

      log->req.aio_nbytes = snprintf(
        log->buffer, sizeof(log->buffer),
        "%s" uavatt_log_fmt "\n",
        log->skipped ? "\n" : "",
        state->ts.sec, state->ts.nsec, d,
        wrench(2), wrench(3), wrench(4), wrench(5),
        roll, pitch, yaw, wd(0), wd(1), wd(2), awd(0), awd(1), awd(2),
        eR(0), eR(1), eR(2), ew(0), ew(1), ew(2));

      if (aio_write(&log->req)) {
        warn("log");
        close(log->req.aio_fildes);
        log->req.aio_fildes = -1;
      } else
        log->pending = true;

      log->skipped = false;
    }
  }

  return 0;
}


/*
 * --- uavatt_wrench -------------------------------------------------------
 *
 * Compute measured total wrench
 */

int
uavatt_wrench(const uavatt_ids_body_s *body,
              const or_pose_estimator_state *state,
              const double wprop[or_rotorcraft_max_rotors],
              double wrench[6])
{
  using namespace Eigen;

  Quaternion<double> q;
  Map< const Array<double, or_rotorcraft_max_rotors, 1> >wprop_(wprop);
  Map< Matrix<double, 6, 1> >wrench_(wrench);

  Map< const Matrix<double,
                    6, or_rotorcraft_max_rotors, RowMajor> > G(body->G);

  /* current state - XXX do something if state not present / uncertain */
  if (state->att._present && !std::isnan(state->att._value.qw))
    q.coeffs() <<
      state->att._value.qx, state->att._value.qy, state->att._value.qz,
      state->att._value.qw;
  else
    q = Quaternion<double>::Identity();

  wrench_ = G * wprop_.square().matrix();
  wrench_.block<3, 1>(0, 0) = q * wrench_.block<3, 1>(0, 0);
  wrench_.block<3, 1>(3, 0) = q * wrench_.block<3, 1>(3, 0);

  return 0;
}


/* --- uavatt_invert_G ----------------------------------------------------- */

void
uavatt_invert_G(const double G[6 * or_rotorcraft_max_rotors],
                double iG[or_rotorcraft_max_rotors * 6])
{
  using namespace Eigen;

  Map< const Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G_(G);
  Map< Matrix<double, or_rotorcraft_max_rotors, 6, RowMajor> > iG_(iG);

  iG_ = G_.
    jacobiSvd(ComputeFullU | ComputeFullV).
    solve(Matrix<double, 6, 6>::Identity());
}


/* --- uavatt_Gw2 ---------------------------------------------------------- */

void
uavatt_Gw2(const double G[6 * or_rotorcraft_max_rotors], const double w,
           double f[6])
{
  using namespace Eigen;

  Map< const Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G_(G);
  Map< Matrix<double, 6, 1> > f_(f);

  f_ = G_ * Matrix<double, or_rotorcraft_max_rotors, 1>::Constant(w * w);
}
