#include<iostream>
#include"timer.h"

int main() {

	// 1.��������Ķ���
	SupportClass *supt = new SupportClass();

	//3.����ִ�����в�������
	for (;;) {
		if (supt->getCurrentIndex() > supt->getTestCaseCount()){
			cout << "Break here!" << endl;
			break;
		}
		supt->setCurrentTestCase();
		cout << "testCaseID:" << supt->getCurrentTestCase()->getTestCaseID() << endl;

		//2.ϵͳ��ʼ��������
		Timer timer(300);
		timer.supt = supt;
		timer.init();
		timer.run();

		//3.����ִ�н��
		supt->setCurTestCaseResult("OK");

		//4.���Խ��������ڴ�����
		supt->putTestCasesInMem();
	}
	supt->pullMem();


	return 0;
}