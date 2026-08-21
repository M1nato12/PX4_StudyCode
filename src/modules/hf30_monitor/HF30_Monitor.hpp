/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

// PX4 平台通用基础
#include <px4_platform_common/defines.h>                                // 通用宏定义 (PX4_OK / PX4_ERROR / PX4_ERR 等)
#include <px4_platform_common/module.h>                                 // 模块基类 ModuleBase 及模块注册辅助宏
#include <px4_platform_common/module_params.h>                          // 参数基类 ModuleParams (配合 DEFINE_PARAMETERS 使用)
#include <px4_platform_common/posix.h>                                  // POSIX 兼容层 (提供跨平台的标准库封装)
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>     // 工作队列调度基类 ScheduledWorkItem (Run() 调度)

// 底层驱动与性能计数
#include <drivers/drv_hrt.h>                                            // 高精度定时器 (hrt_absolute_time() 等)
#include <lib/perf/perf_counter.h>                                      // 性能计数器 (perf_alloc / perf_begin / perf_end)

// uORB 消息通信
#include <uORB/Publication.hpp>                                         // uORB 发布者模板 (Publication)
#include <uORB/Subscription.hpp>                                        // uORB 订阅者 (Subscription)
#include <uORB/SubscriptionCallback.hpp>                                // uORB 回调订阅 (SubscriptionCallbackWorkItem, 消息更新时调度 Run)
#include <uORB/topics/parameter_update.h>                               // parameter_update 参数更新主题消息
#include <uORB/topics/vehicle_status.h>                                 // vehicle_status 飞行器状态主题消息 (含解锁状态)
#include <uORB/topics/hf30_monitor_status.h>                            // hf30_monitor_status 自定义主题消息 (用于发布模块状态)

using namespace time_literals;

class HF30_monitor : public ModuleBase<HF30_monitor>,
public ModuleParams, public px4::ScheduledWorkItem
{
public:
	HF30_monitor();
	~HF30_monitor() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);
	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Publications
	uORB::Publication<hf30_monitor_status_s> _status_pub{ORB_ID(hf30_monitor_status)};

	// Subscriptions
	// uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::HF30_BLINK_MS>) _param_hf30_blink_ms   /**< param_hf30_blink_ms */
	)

	bool _led_state{false};
	uint32_t _run_count{0};
	bool _armed{false};
};
