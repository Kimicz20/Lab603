#ifndef PEOPLE_H_
#define PEOPLE_H_

#include"common.h"

class People {

private:

	//该人进入哪层楼
	int InFloor;

	//该人要去哪层楼
	int OutFloor;

	//此人能够容忍的等候时间
	int GiveupTime;

	//进出电梯的消耗时间
	int consumeTime;

public:
	
	//产生人员对象的时候需要显式指定InFloor,OutFloor,GiveupTime属性信息。
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

