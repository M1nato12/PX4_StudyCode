/****************************************************************************
 *
 * Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "HF30_Reader.hpp"

HF30_Reader::HF30_Reader() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")){}

HF30_Reader::~HF30_Reader()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void HF30_Reader::vehicle_status_update()
{
	if (_vehicle_status_sub.update()) {
		_vehicle_status = _vehicle_status_sub.get();
		vehicle_status_processed_s status_processed{};

		status_processed.timestamp = hrt_absolute_time();
		status_processed.nav_state= _vehicle_status.nav_state;
		status_processed.processed_value = _vehicle_status.nav_state * _param_sta_dou.get();

		_vehicle_status_pub.publish(status_processed);
	}
}

void HF30_Reader::vehicle_attitude_update()
{
	if (_vehicle_attitude_sub.update(&_vehicle_attitude)) {

		vehicle_attitude_processed_s attitude_processed{};

		attitude_processed.timestamp = hrt_absolute_time();
		for (int i = 0; i < 4; i++) {
			attitude_processed.q0[i] = _vehicle_attitude.q[i];
			attitude_processed.processed_value[i]
				= _vehicle_attitude.q[i] * _param_att_dou.get();
		}
		_vehicle_attitude_pub.publish(attitude_processed);
	}
}


void HF30_Reader::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_local_pos_sub.update(&_local_pos)) {
		vehicle_attitude_update();
		vehicle_status_update();
	}

	perf_end(_loop_perf);
}


int HF30_Reader::task_spawn(int argc, char *argv[])
{

	HF30_Reader *instance = new HF30_Reader();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool
HF30_Reader::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int HF30_Reader::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HF30_Reader::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
HF30 Reader is a module that reads the vehicle status and attitude, and publishes the processed data to the vehicle_status_processed and vehicle_attitude_processed topics.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hf30_reader", "system");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int hf30_reader_main(int argc, char *argv[])
{
	return HF30_Reader::main(argc, argv);
}
