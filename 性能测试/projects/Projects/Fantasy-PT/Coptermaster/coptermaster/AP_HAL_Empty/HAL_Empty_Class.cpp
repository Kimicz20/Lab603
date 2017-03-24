
#include "../AP_HAL/AP_HAL.h"
//#if CONFIG_HAL_BOARD == HAL_BOARD_EMPTY

#include <assert.h>

#include "HAL_Empty_Class.h"
#include "AP_HAL_Empty_Private.h" 
using namespace Empty;

//static EmptyUARTDriver uartADriver;
//static EmptyUARTDriver uartBDriver;
//static EmptyUARTDriver uartCDriver;
static EmptySemaphore  i2cSemaphore;
static EmptyI2CDriver  i2cDriver(&i2cSemaphore);
static EmptySPIDeviceManager spiDeviceManager;
static EmptyAnalogIn analogIn;
static EmptyStorage storageDriver;
static EmptyGPIO gpioDriver;
static EmptyRCInput rcinDriver;
static EmptyRCOutput rcoutDriver;
static EmptyScheduler schedulerInstance;
static EmptyUtil utilInstance;


HAL_Empty::HAL_Empty() :
    AP_HAL::HAL(
        //&uartADriver,
        //&uartBDriver,
        //&uartCDriver,
		NULL,
		NULL,
		NULL,
        NULL,            /* no uartD */
        NULL,            /* no uartE */
        &i2cDriver,
        NULL, /* only one i2c */
        NULL, /* only one i2c */
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        //&uartADriver, 
		NULL,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance),
    _member(new EmptyPrivateMember(123))
{}

void HAL_Empty::run(int argc, char* const argv[], Callbacks* callbacks) const
{
  	assert(callbacks);
	/* 被测程序预处理过程 */
	long start, end;
	// 1.创建所需的对象
	copter.supt = new SupportClass();

	//2.系统初始化
    scheduler->init(NULL);  
    _member->init();
    scheduler->system_initialized();

    
    //3.程序执行所有测试用例
	for (;;) { 
		if (copter.supt->getCurrentIndex() > copter.supt->getTestCaseCount()){
			cout << "Break here!" << endl;
			break;
		}
		//3.1 设置时钟以及处理
		copter.supt->setHandler();
		copter.supt->setAlarm(1);
		copter.supt->setCurrentTestCase();
		
		start = clock();
		copter.supt->setCurProcessResult("setup", start, 1);
		copter.setup();
		end = clock();
		copter.supt->setCurProcessResult("setup", end, 2);
		copter.supt->setCurProcessResult("setup", (end - start), 3);
	 	
		start = clock();
		copter.supt->setCurProcessResult("loop", start, 1);
		copter.loop();
		end = clock();
		copter.supt->setCurProcessResult("loop", end, 2);
		copter.supt->setCurProcessResult("loop", (end - start), 3);

		/*
		cout
			<< (int)copter.channel_roll->control_in << endl
			<< (int)copter.channel_yaw->control_in << endl
			<< (int)copter.channel_pitch->control_in << endl 
			<< (int)copter.channel_throttle->control_in << endl; 
		
		if (hal.scheduler->millis()>50000 || copter.supt->getCurrentIndex() > copter.supt->getTestCaseCount()) 
			break;
			*/
		/*
	 	int num = copter.supt->getCurrentTestCase()->getProcessNum();
		for(int i = 1;i <= num ;i++){
			long start = clock() * 1000;
			copter.supt->setCurProcessResult(copter.supt->getCurrentTestCase()->getProcessNameWithId(i), start, 1);
			long end = clock() * 1000;
			copter.supt->setCurProcessResult(copter.supt->getCurrentTestCase()->getProcessNameWithId(i), end, 2);
			copter.supt->setCurProcessResult(copter.supt->getCurrentTestCase()->getProcessNameWithId(i), (end - start), 3);
		}*/
		//4.测试结果存放在内存区中
		copter.supt->putTestCasesInMem();
	}
	copter.supt->pullMem();
	return;
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_Empty hal;
    return hal;
}

//#endif
