/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file adrc_rate_control.cpp
 */

#include "ADRCRateControl.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

bool ADRCRateControl::setParameters(float r0, float h0, const Vector3f &b0,
                                   const Vector3f &omega_o, const Vector3f &k1,
                                   float alpha1, float delta1)
{
    if (!PX4_ISFINITE(r0) || r0 <= 0.f || !PX4_ISFINITE(h0) || h0 <= 0.f
        || !b0.isAllFinite() || !omega_o.isAllFinite() || !k1.isAllFinite()
        || !PX4_ISFINITE(alpha1) || alpha1 <= 0.f || alpha1 > 1.f
        || !PX4_ISFINITE(delta1) || delta1 <= 0.f) {
        return false;
    }

    for (int i = 0; i < 3; i++) {
        if (fabsf(b0(i)) <= 1e-6f || omega_o(i) <= 0.f || omega_o(i) > 10.f || k1(i) < 0.f) {
            return false;
        }
    }

    _r0 = r0;
    _h0 = h0;
    _b0 = b0;
    _omega_o = omega_o;
    _k1 = k1;
    _alpha1 = alpha1;
    _delta1 = delta1;
    _initialized = false;
    return true;
}

void ADRCRateControl::updateTrackingDifferentiator(const Vector3f &rate_sp, float dt)
{
    Vector3f x2_dot{};

    for (int i = 0; i < 3; i++) {
        x2_dot(i) = fst(
            _x1(i) - rate_sp(i),
            _x2(i),
            _r0,
            _h0
        );
    }

    const Vector3f x1_next = _x1 + dt * _x2;
    const Vector3f x2_next = _x2 + dt * x2_dot;

    _x1 = x1_next;
    _x2 = x2_next;
}

    void ADRCRateControl::updateESO(const matrix::Vector3f &rate, const matrix::Vector3f &last_u, float dt)
    {
        //Update the Linear Extended State Observer (ESO).

        //Args:
        //  y (float): Output of the plant (current measurement).
        //   u (float): Control signal.
        const Vector3f e = _z1 - rate;

	const Vector3f beta1 = 2.f * _omega_o;
    	const Vector3f beta2 = _omega_o.emult(_omega_o);

   	const Vector3f z1_dot = _z2+ _b0.emult(last_u)- beta1.emult(e);
    	const Vector3f z2_dot = -beta2.emult(e);

	const Vector3f z1_next = _z1 + dt * z1_dot;
	const Vector3f z2_next = _z2 + dt * z2_dot;

	_z1 = z1_next;
	_z2 = z2_next;
    }


  Vector3f ADRCRateControl::computeNLSEF()
	{
	// Compute the Nonlinear State Error Feedback (NLSEF).

	// Returns:
	//  float: Control signal after nonlinear state error feedback.

	const Vector3f e1 = _x1 - _z1;
	 Vector3f fal_e1{};

	for (int i = 0; i < 3; i++) {
		fal_e1(i) = fal(e1(i), _alpha1, _delta1);
	}

    	const Vector3f u0 =
        _x2
        + _k1.emult(fal_e1);

	Vector3f u{};

	for (int i = 0; i < 3; i++) {
		u(i) = (u0(i) - _z2(i)) / _b0(i);
	}

	return u;
	}



float ADRCRateControl::fst(float x1, float x2, float r0, float h0)
{
    const float d = r0 * h0;
    const float d0 = d * h0;

    const float y = x1 + h0 * x2;
    const float alpha0 = sqrtf(d * d + 8.f * r0 * fabsf(y));

    const float sign_y = (y > 0.f) ? 1.f : ((y < 0.f) ? -1.f : 0.f);

    float alpha{0.f};

    if (fabsf(y) > d0) {
        alpha = x2 + 0.5f * (alpha0 - d) * sign_y;

    } else {
        alpha = x2 + y / h0;
    }

    const float sign_alpha =
        (alpha > 0.f) ? 1.f : ((alpha < 0.f) ? -1.f : 0.f);

    if (fabsf(alpha) <= d) {
        return -r0 * alpha / d;

    } else {
        return -r0 * sign_alpha;
    }
}

float ADRCRateControl::fal(float e, float alpha, float delta)
{
    const float sign_e =
        (e > 0.f) ? 1.f : ((e < 0.f) ? -1.f : 0.f);

    if (fabsf(e) > delta) {
        return powf(fabsf(e), alpha) * sign_e;

    } else {
        return e / powf(delta, 1.f - alpha);
    }
}

void ADRCRateControl::reset(
    const Vector3f &rate,
    const Vector3f &rate_sp)
{
    _x1 = rate_sp;
    _x2.setZero();

    _z1 = rate;
    _z2.setZero();

    _last_u.setZero();

    _initialized = true;
}

Vector3f ADRCRateControl::update(const Vector3f &rate, const Vector3f &rate_sp, float dt, bool landed)
{
    if (!rate.isAllFinite() || !rate_sp.isAllFinite() || !PX4_ISFINITE(dt) || dt <= 0.f) {
        _initialized = false;
        return Vector3f{};
    }

    if (!_initialized) {
        reset(rate, rate_sp);
    }

    // TD仍然更新目标
    updateTrackingDifferentiator(rate_sp, dt);

    if (!landed) {
        // 飞行中正常运行ESO
        updateESO(rate, _last_u, dt);

    } else {
        // 地面时不让扰动估计继续累积
        _z1 = rate;
        _z2.setZero();
        _last_u.setZero();
    }

    const Vector3f control_signal = computeNLSEF();

    return control_signal;
}
