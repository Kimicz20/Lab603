#include<iostream>
#include"timer.h"

int main() {

	// 1.创建所需的对象
	SupportClass *supt = new SupportClass();

	//3.程序执行所有测试用例
	for (;;) {
		if (supt->getCurrentIndex() > supt->getTestCaseCount()){
			cout << "Break here!" << endl;
			break;
		}
		supt->setCurrentTestCase();
		cout << "testCaseID:" << supt->getCurrentTestCase()->getTestCaseID() << endl;

		//2.系统初始化并运行
		Timer timer(300);
		timer.supt = supt;
		timer.init();
		timer.run();

		//3.设置执行结果
		supt->setCurTestCaseResult("OK");

		//4.测试结果存放在内存区中
		supt->putTestCasesInMem();
	}
	supt->pullMem();


	return 0;
}