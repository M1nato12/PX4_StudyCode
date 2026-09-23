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
 * @file adrc_rate_control.hpp
 *
 * ADRC 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <cmath>

class ADRCRateControl
{
public:
	ADRCRateControl() = default;
	~ADRCRateControl() = default;

    bool setParameters(float r0, float h0, const matrix::Vector3f &b0,
                       const matrix::Vector3f &omega_o, const matrix::Vector3f &k1,
                       float alpha1, float delta1);

    matrix::Vector3f update(const matrix::Vector3f &rate,
                            const matrix::Vector3f &rate_sp,
                            float dt, bool landed);

    // Feed the command that actually passed the fixed-wing output limits to the ESO.
    void setAppliedControl(const matrix::Vector3f &control) { _last_u = control; }

	const matrix::Vector3f &getZ1() const { return _z1; }
	const matrix::Vector3f &getZ2() const { return _z2; }
	const matrix::Vector3f &getLastControl() const { return _last_u; }

	void reset(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp);


private:
    void updateESO(const matrix::Vector3f &rate, const matrix::Vector3f &last_u, float dt);

    matrix::Vector3f computeNLSEF();

    void updateTrackingDifferentiator(const matrix::Vector3f &rate_sp,
                                      float dt);

    static float fst(float x1, float x2, float r0, float h0);
    static float fal(float e, float alpha, float delta);
    bool _initialized{false};

    	// TD parameters
	float _r0{1.f};
	float _h0{0.01f};

	// TD states
	matrix::Vector3f _x1{};
	matrix::Vector3f _x2{};

	// NLSEF parameters
	matrix::Vector3f _k1{1.f, 1.f, 1.f};
	float _alpha1{0.5f};
    	float _delta1{0.01f};

	// ESO states
	matrix::Vector3f _z1{};
	matrix::Vector3f _z2{};
	// Previous control
	matrix::Vector3f _last_u{};

	// ADRC parameters
	matrix::Vector3f _b0{};
	matrix::Vector3f _omega_o{};
};
