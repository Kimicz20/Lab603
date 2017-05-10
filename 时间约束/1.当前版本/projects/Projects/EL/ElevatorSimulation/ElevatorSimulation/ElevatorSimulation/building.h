#ifndef BUILDING_H_
#define BUILDING_H_

#include<random>
#include"elevator.h"
#include"circularqueue.h"

class Building {

private:
	
	//下个人出现的时间间隔。
	int InterTime;

	//该栋楼对应的电梯。
	Elevator * elevator;

	//产生人员对象的楼层对列，每层楼的人员对象产生后需要排队等待。
	CircularQueue<People *, MAXPEOPLENUMBER> * buildingQueue[FLOORSNUMBER];

	//定时器指针，用来向定时器注册事件。
	ITimer * timer;

public:

	Building(ITimer *);

	~Building();

	//随机产生一个人员对象，需要设定的值包含电梯的callUp,callDown属性。
	void newPeople();

	//该楼层电梯里面的人先出来，等待的人再进去。
	void peopleOutIn();
	 
	Elevator * getElevator() {
		return elevator;
	}

};

struct tetrad
{
	int InFloor;
	int OutFloor;
	int GiveupTime;
	int InterTime;
};

tetrad * createRandomSequence();

#endif // !BUILDING_H_
