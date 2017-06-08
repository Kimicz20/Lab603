#include"building.h"

Building::Building(ITimer * t)
{
	for (int i = 0; i < FLOORSNUMBER; i++)
	{
		this->buildingQueue[i] = new std::queue<People *>;
	}
	this->timer = t;
	elevator = new Elevator(t);
}

Building::~Building()
{
	//析构每个楼层队列里面的人员对象。
	for (int i = 0; i < FLOORSNUMBER; i++)
	{
		//析构楼层队列数据结构。	
		delete this->buildingQueue[i];
	}

	//析构电梯对象。
	delete elevator;
}


void Building::peopleComing()
{
	
	//产生人员对象。
	supt->timeHandle("newPeople", START, "peopleComing");
	People * singlePeople = pf.newPeople();
	supt->timeHandle("newPeople", END);

	singlePeople->setStartTime(timer->getTime());

	std::default_random_engine dre;

	std::uniform_int_distribution<int> di2(100, 200);

	this->InterTime = di2(dre) / 10 * 10;

	//为下一个人员对象的产生做准备。即注册产生人员对象事件。
	this->timer->addEventList(peopleevent1, this->InterTime);

	//产生下一个人员对象后，需要直接注册那个时刻的电梯响应事件。
	this->timer->addEventList(elevatorevent1, this->InterTime);

	//加入对应楼层队列。
	this->buildingQueue[singlePeople->getInFloor()]->push(singlePeople);

	//按下电钮并且等候。
	if (singlePeople->getInFloor() > singlePeople->getOutFloor())
	{
		supt->timeHandle("pushCallDown", START, "newPeople");
		elevator->pushCallDown(singlePeople->getInFloor());  //该楼层的向下按钮被按。
		supt->timeHandle("pushCallDown", END);
	}
	else
	{
		supt->timeHandle("pushCallUp", START, "newPeople");
		elevator->pushCallUp(singlePeople->getInFloor());  //该楼层的向上按钮被按。
		supt->timeHandle("pushCallUp", END);
	}

	//注册达到容忍时刻后此人是否还要继续等待的判断事件。
	this->timer->addEventList(peopleevent2, singlePeople->getGiveupTime());

}

void Building::peopleOutIn()
{
	bool flag = this->elevator->thisFloorOut();
	if (supt->getParamValueWithNameAndKey("thisFloorPeopleOut", "elevator_thisFloorOut") == 1)
		flag = true;
	//本层电梯里面的人先出来，出来的速度是每过25个t出来一个人
	/*if (this->elevator->thisFloorOut() == true)*/
	if (flag == true)
	{
		supt->timeHandle("thisFloorPeopleOut", START, "open");
		this->elevator->thisFloorPeopleOut();
		supt->timeHandle("thisFloorPeopleOut_return", START, "thisFloorPeopleOut");
		supt->timeHandle("thisFloorPeopleOut_return", END);
		supt->timeHandle("thisFloorPeopleOut", END);
		return;
	}
	flag = this->buildingQueue[this->elevator->getCurrentFloor()]->empty();
	if (supt->getParamValueWithNameAndKey("thisFloorPeopleIn", "getCurrentFloor_empty") == 1)
		flag = true;
	//本层电梯外面的人开始进入，进来的速度是每过25个t进来一个人,注意进来的时候需要将电梯内部的CallCar属性按下（由人发出）
	if (flag == false) {
		while (this->buildingQueue[this->elevator->getCurrentFloor()]->empty() == false && this->buildingQueue[this->elevator->getCurrentFloor()]->front()->getCurrentState() == leaving)
		{
			this->buildingQueue[this->elevator->getCurrentFloor()]->pop();
		}
		flag = this->buildingQueue[this->elevator->getCurrentFloor()]->empty();
		if (supt->getParamValueWithNameAndKey("thisFloorPeopleIn", "getCurrentFloor_empty") == 1)
			flag = true;
		if (flag == false)
		{
			//从楼层外部往电梯里面进一个人
			supt->timeHandle("thisFloorPeopleIn", START, "open");
			this->elevator->thisFloorPeopleIn(this->buildingQueue[this->elevator->getCurrentFloor()]->front());
			supt->timeHandle("thisFloorPeopleIn_return", START, "thisFloorPeopleIn");
			supt->timeHandle("thisFloorPeopleIn_return", END);
			supt->timeHandle("thisFloorPeopleIn", END);
			this->buildingQueue[this->elevator->getCurrentFloor()]->pop();
			//25个t后再次查看队列里面是否有人需要进入电梯内部。
			this->timer->addEventList(peopleevent3,25);		
			return;
		}
	}
	//如果本楼层的队列为空，置D1=0，D3！=0。
	if (this->buildingQueue[this->elevator->getCurrentFloor()]->empty() == true)
	{
		this->elevator->setD1(0);
		this->elevator->setD3(1);
	}
}

void Building::deletePeople()
{
	pf.deletePeople(this->timer->getTime(), this->elevator->getCurrentFloor(), this->elevator->getD1());
}
