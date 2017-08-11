#include"peoplefactory.h"

//1000个人
Triple peopleSequence[MAXPEOPLENUMBER];

//随机数序列，辅助数据
Triple * createRandomSequence()
{
	std::default_random_engine dre;
	//随机楼层
	std::uniform_int_distribution<int> di1(0, FLOORSNUMBER-1);
	//随机产生人，并且该人的容忍时间设置为10s到20s。
	std::uniform_int_distribution<int> di2(100, 200);

	for (int i = 0; i < MAXPEOPLENUMBER; i++)
	{
		peopleSequence[i].InFloor = di1(dre);
		peopleSequence[i].OutFloor = di1(dre);
		peopleSequence[i].GiveupTime = di2(dre) / 10 * 10;
	}
	return peopleSequence;
}

PeopleFactory::PeopleFactory()
{
	createRandomSequence();
	for (int i = 0; i < MAXPEOPLENUMBER; i++)
	{
		peopleSet[i] = new People(peopleSequence[i].InFloor, peopleSequence[i].OutFloor, peopleSequence[i].GiveupTime);
	}
	this->index = 0;
}

PeopleFactory::~PeopleFactory()
{
	for (int i = 0; i < MAXPEOPLENUMBER; i++)
	{
		delete peopleSet[i];
	}
}

People * PeopleFactory::newPeople()
{
	while (peopleSet[this->index]->getInFloor() == peopleSet[this->index]->getOutFloor())
	{
		this->index = (this->index + 1) % MAXPEOPLENUMBER;
	}
	int temp = this->index;
	this->index = (this->index + 1) % MAXPEOPLENUMBER;
	return peopleSet[temp];
}


void PeopleFactory::deletePeople(int currentTime,int currentFloor,int eD1)
{
	for (int i = 0; i < this->index; i++)
	{
		if (this->peopleSet[i]->getCurrentState() == waiting)
		{
			if (this->peopleSet[i]->getGiveupTime() + this->peopleSet[i]->getStartTime() == currentTime)
			{
				if (this->peopleSet[i]->getInFloor() != currentFloor || eD1 == 0)
				{
					this->peopleSet[i]->setCurrentState(leaving);
				}
			}
		}
	}
}