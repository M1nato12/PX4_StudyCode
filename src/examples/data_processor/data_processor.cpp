#include <px4_platform_common/log.h>          // PX4日志
#include <px4_platform_common/posix.h>        // px4_usleep
#include <px4_platform_common/module.h>                      // 模块启动/停止/状态
#include <px4_platform_common/module_params.h>               // 模块参数基础
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp> // 定时任务

#include <uORB/Subscription.hpp>              // uORB订阅
#include <uORB/Publication.hpp>               // uORB发表

#include <uORB/topics/vehicle_attitude.h>     // 姿态话题
#include <uORB/topics/vehicle_angular_velocity.h> // 角速度话题
#include <uORB/topics/processed_data.h>

#include <cstring>

// // 允许使用100_ms这种时间写法
// using namespace time_literals;

class DataProcessor : public ModuleBase<DataProcessor>, public ModuleParams,
		      public px4::ScheduledWorkItem
{
private:
 /* 输入接口*/
    // 姿态话题订阅器
    uORB::Subscription _attitude_sub{
        ORB_ID(vehicle_attitude)
    };
    // 角速度话题订阅器
    uORB::Subscription _angular_velocity_sub{
        ORB_ID(vehicle_angular_velocity)
    };
/*输出接口*/
    // processed_data话题发布器
    uORB::Publication<processed_data_s> _processed_pub{
        ORB_ID(processed_data)
    };

vehicle_attitude_s _attitude{};
vehicle_angular_velocity_s _angular_velocity{};
processed_data_s _output{};

public:
    DataProcessor() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers) {}
    ~DataProcessor() override = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[])
{
DataProcessor *instance = new DataProcessor();

	if(instance==nullptr){
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);

	_task_id = task_id_is_work_queue;

	instance->ScheduleNow();

	return PX4_OK;
}
    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}


    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr)
{
	if(reason!=nullptr){
		PX4_WARN("%s\n",reason);
	}
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	Subscribe attitude and angular velocity,
	process the data and publish processed_data.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("data_processor", "example");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");

  return 0;
}

void Run() override
{
	if(should_exit())
	{
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// 读取姿态
	bool attitude_updated = _attitude_sub.copy(&_attitude);

	// 读取角速度
	bool angular_velocity_updated = _angular_velocity_sub.copy(&_angular_velocity);

	// 两个话题都读取成功
	if (attitude_updated && angular_velocity_updated) {
	    // 时间戳
	    _output.timestamp = hrt_absolute_time();
	    _output.timestamp_sample = _angular_velocity.timestamp_sample;

	//     PX4_INFO("attitude q: %.3f %.3f %.3f %.3f",
	// 	     (double)_attitude.q[0],
	// 	     (double)_attitude.q[1],
	// 	     (double)_attitude.q[2],
	// 	     (double)_attitude.q[3]);

	//     PX4_INFO("angular velocity: %.3f %.3f %.3f",
	// 	     (double)_angular_velocity.xyz[0],
	// 	     (double)_angular_velocity.xyz[1],
	// 	     (double)_angular_velocity.xyz[2]);

	    // 姿态简单处理
	    for (int i = 0; i < 4; ++i) {
		_output.attitude_q[i] = _attitude.q[i] * 2.0f;
	    }

	    // 角速度简单处理
	    for (int i = 0; i < 3; ++i) {
		_output.angular_velocity[i] = _angular_velocity.xyz[i] * 2.0f;
	    }

	    // 发布处理后的数据
	    _processed_pub.publish(_output);
	}

	// 每100ms执行一次
	ScheduleDelayed(100000);

}
};

extern "C" __EXPORT int data_processor_main(int argc, char *argv[])
{
    /*
     * 将终端命令交给ModuleBase处理：
     *
     * data_processor start  → task_spawn()
     * data_processor stop   → 设置停止标志
     * data_processor status → 查看模块状态
     */
	return DataProcessor::main(argc, argv);
}
