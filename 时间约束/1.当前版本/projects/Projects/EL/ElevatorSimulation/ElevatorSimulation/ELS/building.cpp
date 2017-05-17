#include"building.h"

tetrad dataSequence[MAXPEOPLENUMBER*FLOORSNUMBER];
static int dataIndex = 0;


Building::Building(ITimer * t)
{
	for (int i = 0; i < FLOORSNUMBER; i++)
	{
		this->buildingQueue[i] = new CircularQueue<People *, MAXPEOPLENUMBER>;
	}
	this->timer = t;
	elevator = new Elevator(t);
}

Building::~Building()
{
	//析构每个楼层队列里面的人员对象。
	for (int i = 0; i < FLOORSNUMBER; i++)
	{
		while (!this->buildingQueue[i]->empty())
		{
			delete this->buildingQueue[i]->frontElement();
			this->buildingQueue[i]->popElement();
		}
		//析构楼层队列数据结构。	
		delete this->buildingQueue[i];
	}

	//析构电梯对象。
	delete elevator;
}


void Building::newPeople()
{
	
	//在随机数序列里面剔除掉InFloor与OutFloor相等的随机数组元素。
	while (dataSequence[dataIndex].InFloor == dataSequence[dataIndex].OutFloor)
	{
		++dataIndex;
	}
	//产生人员对象。
	People * singlePeople = new People(dataSequence[dataIndex].InFloor, dataSequence[dataIndex].OutFloor, dataSequence[dataIndex].GiveupTime, this->timer->getTime());
	
	this->InterTime = dataSequence[dataIndex].InterTime;
	
	//为下一个人员对象的产生做准备。即注册产生人员对象事件。
	this->timer->addEventList(peopleevent1, this->InterTime);

	//产生下一个人员对象后，需要直接注册那个时刻的电梯响应事件。
	this->timer->addEventList(elevatorevent1, this->InterTime);


	//加入对应楼层队列。
	this->buildingQueue[singlePeople->getInFloor()]->pushElement(singlePeople);


	//按下电钮并且等候。
	if (singlePeople->getInFloor() > singlePeople->getOutFloor())
	{
		std::cout << "pushCallDown" << std::endl;
		elevator->pushCallDown(singlePeople->getInFloor());  //该楼层的向下按钮被按。
	}
	else
	{
		std::cout << "pushCallUp" << std::endl;
		elevator->pushCallUp(singlePeople->getInFloor());  //该楼层的向上按钮被按。
	}
	//注册达到容忍时刻后此人是否还要继续等待的判断事件。
	this->timer->addEventList(peopleevent2, singlePeople->getGiveupTime());

	++dataIndex;
}

void Building::peopleOutIn()
{
	//本层电梯里面的人先出来，出来的速度是每过25个t出来一个人
	if (this->elevator->thisFloorOut() == true)
	{
		this->elevator->thisFloorPeopleOut();
		return;
	}
	//本层电梯外面的人开始进入，进来的速度是每过25个t进来一个人,注意进来的时候需要将电梯内部的CallCar属性按下（由人发出）
	//注意要跳过对应楼层队列里面的空指针情况，空指针是因为该人由于超过等待的容忍时间而走掉了。
	if (this->buildingQueue[this->elevator->getCurrentFloor()]->empty() == false) {
		while (this->buildingQueue[this->elevator->getCurrentFloor()]->empty() == false && this->buildingQueue[this->elevator->getCurrentFloor()]->frontElement() == nullptr)
		{
			this->buildingQueue[this->elevator->getCurrentFloor()]->popElement();
		}
		if (this->buildingQueue[this->elevator->getCurrentFloor()]->empty() == false)
		{
			//从楼层外部往电梯里面进一个人
			this->elevator->thisFloorPeopleIn(this->buildingQueue[this->elevator->getCurrentFloor()]->frontElement());
			this->buildingQueue[this->elevator->getCurrentFloor()]->popElement();
			//25个t后再次查看队列里面是否有人需要进入电梯内部。
			this->timer->addEventList(peopleevent3, 25);
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
	//依次轮询每个楼层队列里的人们，找出哪些准备放弃的人员对象，然后析构掉这些对象，并且指针置空。
	/*for (int i = 0; i < FLOORSNUMBER; i++)
	{
		//如果当前时刻减去giveuptime刚好等于某个人员对象产生的时间，那么就要进行是否删除的判断了。
		People * * tempPtr;
		std::cout << "+++++++++++" << std::endl;
		if (this->buildingQueue[i]->empty())
			continue;
		std::cout << "+++++++++++0" << std::endl;
		while (tempPtr = this->buildingQueue[i]->nextElement())
		{
			std::cout << "+++++++++++1:" << (*tempPtr)->getGiveupTime()
				<< "," << (*tempPtr)->getStartTime()
				<< "," << this->timer->getTime()<<std::endl;
			if ((*tempPtr)->getGiveupTime() + (*tempPtr)->getStartTime() == this->timer->getTime())
			{
				std::cout << "+++++++++++2" << std::endl;
				//查看当前电梯是否在这个人期望的那一层
				if ((*tempPtr)->getInFloor() != this->elevator->getCurrentFloor() || this->elevator->getD1() == 0)
				{
					std::cout <<"+++++++++++3"<< std::endl;
					//该人放弃，进行删除。
					delete (*tempPtr);
					std::cout << "+++++++++++4" << std::endl;
					//队列指针置空
					*tempPtr = nullptr;
					std::cout << "+++++++++++5" << std::endl;
				}
			}
		}
		std::cout << "www" << std::endl;
	}*/
}


//随机数序列，辅助数据
tetrad * createRandomSequence()
{
	std::default_random_engine dre;
	//随机楼层
	std::uniform_int_distribution<int> di1(0, FLOORSNUMBER);
	//随机在300-600的时间单位内产生一个人，并且该人的容忍时间设置为300-600的时间单位。
	std::uniform_int_distribution<int> di2(50, 100);

	for (int i = 0; i < MAXPEOPLENUMBER; i++)
	{
		dataSequence[i].InFloor = di1(dre);
		dataSequence[i].OutFloor = di1(dre);
		dataSequence[i].GiveupTime = di2(dre) / 10 * 10;
		dataSequence[i].InterTime = di2(dre) / 10 * 10;
	}
	return dataSequence;
}