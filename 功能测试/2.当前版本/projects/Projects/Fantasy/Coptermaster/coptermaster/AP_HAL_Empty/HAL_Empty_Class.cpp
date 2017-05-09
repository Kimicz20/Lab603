
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
    
    //3.程序执行所有测试用例
	for (;;) { 
		if (copter.supt->getCurrentIndex() > copter.supt->getTestCaseCount()){
			cout << "Break here!" << endl;
			break;
		}
		copter.supt->setHandlerAndAlarm(5);
		copter.supt->setCurrentTestCase();
		cout << "用例ID：" << copter.supt->getCurrentTestCase()->getTestCaseID() << endl;

		//2.系统初始化
		scheduler->init(NULL);
		_member->init();
		scheduler->system_initialized();

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

		copter.supt->setCurTestCaseResult("OK");

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
