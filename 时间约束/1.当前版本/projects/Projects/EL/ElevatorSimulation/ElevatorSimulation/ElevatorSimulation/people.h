#ifndef PEOPLE_H_
#define PEOPLE_H_

#include"common.h"

class People {

private:

	//���˽����Ĳ�¥
	int InFloor;

	//����Ҫȥ�Ĳ�¥
	int OutFloor;

	//�����ܹ����̵ĵȺ�ʱ��
	int GiveupTime;

	//�������ݵ�����ʱ��
	int consumeTime;

public:
	
	//������Ա�����ʱ����Ҫ��ʽָ��InFloor,OutFloor,GiveupTime������Ϣ��
	People(int InFloor,int OutFloor,int GiveupTime);

	int getInFloor() {
		return InFloor;
	}

	int getOutFloor() {
		return OutFloor;
	}

	int getGiveupTime() {
		return GiveupTime;
	}

	~People() {};

};

#endif // !PEOPLE_H_

