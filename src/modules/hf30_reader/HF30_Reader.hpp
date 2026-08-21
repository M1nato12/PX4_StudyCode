/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file HF30_Reader.hpp
 */

#include <float.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/uORB.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <slew_rate/SlewRate.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/hf30_monitor_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/fixed_wing_lateral_setpoint.h>
#include <uORB/topics/fixed_wing_longitudinal_setpoint.h>
#include <uORB/topics/vehicle_status_processed.h>
#include <uORB/topics/vehicle_attitude_processed.h>

using namespace time_literals;

static constexpr fixed_wing_lateral_setpoint_s empty_lateral_control_setpoint = {.timestamp = 0, .course = NAN, .airspeed_direction = NAN, .lateral_acceleration = NAN};
static constexpr fixed_wing_longitudinal_setpoint_s empty_longitudinal_control_setpoint = {.timestamp = 0, .altitude = NAN, .height_rate = NAN, .equivalent_airspeed = NAN, .pitch_direct = NAN, .throttle_direct = NAN};

class HF30_Reader final : public ModuleBase<HF30_Reader>, public ModuleParams,
	public px4::WorkItem
{
public:
	HF30_Reader();

	~HF30_Reader() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	uORB::Subscription _fw_lateral_ctrl_sub{ORB_ID(fixed_wing_lateral_setpoint)};
	uORB::Subscription _fw_longitudinal_ctrl_sub{ORB_ID(fixed_wing_longitudinal_setpoint)};

	vehicle_local_position_s _local_pos{};
	parameter_update_s _parameter_update{};
	vehicle_status_s _vehicle_status{};
	vehicle_attitude_s _vehicle_attitude{};

	uORB::Publication <vehicle_status_processed_s> _vehicle_status_pub{ORB_ID(vehicle_status_processed)};
	uORB::Publication <vehicle_attitude_processed_s> _vehicle_attitude_pub{ORB_ID(vehicle_attitude_processed)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FW_STA_DOU>) _param_sta_dou,
		(ParamInt<px4::params::FW_ATT_DOU>) _param_att_dou
	)

	SlewRate<float> _value_slew_rate;

		// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	void vehicle_status_update();

	void vehicle_attitude_update();

};
