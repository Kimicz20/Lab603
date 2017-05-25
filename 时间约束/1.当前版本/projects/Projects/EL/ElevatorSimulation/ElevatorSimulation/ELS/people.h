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

	//该人所处的状态
	int currentState;

	//该人是什么时刻产生的
	int StartTime;

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

	int getStartTime() {
		return StartTime;
	}

	void setStartTime(int time){
		StartTime = time;
	}
	
	void setCurrentState(PeopleState ps){
		currentState = ps;
	}

	int getCurrentState(){
		return currentState;
	}

	~People() {};

};

#endif // !PEOPLE_H_

