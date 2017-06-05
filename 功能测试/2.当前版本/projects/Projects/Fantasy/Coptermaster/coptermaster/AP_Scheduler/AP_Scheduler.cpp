#include "../AP_HAL/AP_HAL.h"
#include "AP_Scheduler.h"
#include "../AP_Param/AP_Param.h"
#include "../AP_Progmem/AP_Progmem_Identity.h"
#include "../ArduCopter/Copter.h"

extern const AP_HAL::HAL& hal;

int8_t AP_Scheduler::current_task = -1;

//const AP_Param::GroupInfo AP_Scheduler::var_info[]  = {
//    // @Param: DEBUG
//    // @DisplayName: Scheduler debug level
//    // @Description: Set to non-zero to enable scheduler debug messages. When set to show "Slips" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table.
//    // @Values: 0:Disabled,2:ShowSlips,3:ShowOverruns
//    // @User: Advanced
//    AP_GROUPINFO("DEBUG",    0, AP_Scheduler, _debug, 0),
//    AP_GROUPEND
//};

// initialise the scheduler
void AP_Scheduler::init(const AP_Scheduler::Task *tasks, uint8_t num_tasks) 
{
     _tasks = tasks;
    _num_tasks = num_tasks; 
    _last_run = new uint16_t[_num_tasks];
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
    _tick_counter = 0;
}

// one tick has passed
void AP_Scheduler::tick(void)
{
    _tick_counter++;
	 
}
//FixÐÞ¸Ä2.1
int AP_Scheduler::getTick()
{
	return (int)_tick_counter;
}
/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void AP_Scheduler::run(uint16_t time_available)
{ 
	//FixÐÞ¸ÄV1.5
	long start, end;
	// ------------------------  ²å×®µã ---------------------------------
	start = clock();
	this->supt->setCurProcessResult("micros", start, 1);

	// ------------------------  ²å×®¼¤Àø ---------------------------------
	uint32_t run_started_usec = hal.scheduler->micros();

	end = clock();
	this->supt->setCurProcessResult("micros", end, 2);
	this->supt->setCurProcessResult("micros", (end - start), 3);

	uint32_t now = run_started_usec;
	for (uint8_t i = 0; i<_num_tasks; i++) {
		/*cout << "##################" << endl
			<< "\ttime_available :" << time_available << endl
			<< "\t_num_tasks :" << (int)_num_tasks << endl
			<< "\tname:" << _tasks[i].name << endl
			<< "\tinterval_ticks:" << pgm_read_word(&_tasks[i].interval_ticks) << endl
			<< "\tmax_time_micros:" << pgm_read_word(&_tasks[i].max_time_micros) << endl
			<< "##################" << endl;*/
		uint16_t dt = _tick_counter - _last_run[i];
		uint16_t interval_ticks = pgm_read_word(&_tasks[i].interval_ticks);
		if (dt >= interval_ticks) {
			// this task is due to run. Do we have enough time to run it?
			_task_time_allowed = pgm_read_word(&_tasks[i].max_time_micros);
			uint16_t interval_ticks_product_two = interval_ticks * 2;
			
			//FixÐÞ¸Ä1.7
			supt->setCurProcessResult("output",0,3);
			if (dt >= interval_ticks_product_two) {
			//if (dt >= interval_ticks * 2) {
				// we've slipped a whole run of this task!
				if (_debug > 1) {
					/*hal.console->printf_P(PSTR("Scheduler slip task[%u-%s] (%u/%u/%u)\n"),
						(unsigned)i,
						_tasks[i].name,
						(unsigned)dt,
						(unsigned)interval_ticks,
						(unsigned)_task_time_allowed);*/
				}
			}

			if (_task_time_allowed <= time_available) {
				// run it
				_task_time_started = now;
				task_fn_t func;
				pgm_read_block(&_tasks[i].function, &func, sizeof(func));
				current_task = i;
				string name = _tasks[i].name;
				//FixÐÞ¸Ä2.1
				// ------------------------  ²å×®µã ---------------------------------
				start = clock();
				this->supt->setCurProcessResult(name, start, 1);

				// ------------------------  ²å×®¼¤Àø ---------------------------------
				func();

				end = clock();
				this->supt->setCurProcessResult(name, end, 2);
				this->supt->setCurProcessResult(name, (end - start), 3);

				current_task = -1;

				// record the tick counter when we ran. This drives
				// when we next run the event
				_last_run[i] = _tick_counter;

				// work out how long the event actually took

				//FixÐÞ¸Ä1.7
				// ------------------------  ²å×®µã ---------------------------------
				start = clock();
				this->supt->setCurProcessResult("micros", start, 1);

				// ------------------------  ²å×®¼¤Àø ---------------------------------
				now = hal.scheduler->micros();
				end = clock();
				this->supt->setCurProcessResult("micros", end, 2);
				this->supt->setCurProcessResult("micros", (end - start), 3);

				uint32_t time_taken = now - _task_time_started;

				//FixÐÞ¸Ä1.7
				
				string str[] = {"read_aux_switches",
								"ten_hz_logging_loop",
								"rpm_update",
								"full_rate_logging_loop",
								"compass_accumulate",
								"update_thr_average",
								"throttle_loop",
								"gcs_send_heartbeat",
								"dataflash_periodic",
								"landinggear_update",
								"ekf_check",
								"arm_motors_check",
								"fifty_hz_logging_loop",
								"read_receiver_rssi",
								"update_mount",
								"auto_trim",
								"update_notify",
								"run_nav_updates",
								"lost_vehicle_check", 
								"output",
								"gcs_send_deferred",
								"perf_update",
								"compass_cal_update",
								"angle_ef_roll_pitch_rate_ef_yaw",
								"init_arm_motors",
								"set_failsafe_radio",
								"init_disarm_motors"
						};
				int size = sizeof(str) / sizeof(str[0]);
				for (int i = 0; i < size;i++)
					supt->setCurProcessResult(str[i], 0, 3);
				if (time_taken > _task_time_allowed) {
					// the event overran!
					if (_debug > 2) {
						/*hal.console->printf_P(PSTR("Scheduler overrun task[%u-%s] (%u/%u)\n"),
							(unsigned)i,
							_tasks[i].name,
							(unsigned)time_taken,
							(unsigned)_task_time_allowed);*/
					}
				}
				if (time_taken >= time_available) {
					goto update_spare_ticks;
				}
				time_available -= time_taken;
				/*cout << "time_taken:" << time_taken << endl
					<< "\ttime_available :" << time_available << endl;*/
			}
		}
	}

	// update number of spare microseconds
	_spare_micros += time_available;

update_spare_ticks:
	_spare_ticks++;
	if (_spare_ticks == 32) {
		_spare_ticks /= 2;
		_spare_micros /= 2;
	}
}

/*
  return number of micros until the current task reaches its deadline
 */
uint16_t AP_Scheduler::time_available_usec(void)
{
    uint32_t dt = hal.scheduler->micros() - _task_time_started;
    if (dt > _task_time_allowed) {
        return 0;
    }
    return _task_time_allowed - dt;
}

/*
  calculate load average as a number from 0 to 1
 */
float AP_Scheduler::load_average(uint32_t tick_time_usec) const
{
    if (_spare_ticks == 0) {
        return 0.0f;
    }
    uint32_t used_time = tick_time_usec - (_spare_micros/_spare_ticks);
    return used_time / (float)tick_time_usec;
}

extern Copter copter;