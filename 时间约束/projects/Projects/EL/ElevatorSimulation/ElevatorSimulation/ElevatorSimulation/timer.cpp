#include "timer.h"

//用来记录触发E9活动的事件节点
EventNode * e9 = nullptr;

Timer::Timer(int timeLen)
{
	this->building = new Building(this);
	this->eventList = new EventNode[timeLen];
	this->elevator = this->building->getElevator();
	this->timeIndex = 0;
	this->timeLen = timeLen;
}

void Timer::init()
{
	createRandomSequence();
	this->addEventList(peopleevent1,0);
	this->addEventList(elevatorevent1,0);
}

void Timer::run()
{
	do
	{
		executeEvent();
		drawState();			//绘出状态数值  方法体为空。
		this->timeIndex++;
		Sleep(TIMEUNIT);
	} while (this->timeIndex <= this->timeLen);
}

void Timer::executeEvent()
{	
	if (this->eventList[timeIndex].eventID != -1)
	{
		EventNode * event = &eventList[timeIndex];
		do
		{
			switch (event->eventID) {
			case peopleevent1:
				this->building->newPeople();
				break;
			case peopleevent2:
				//暂不处理
				break;
			case peopleevent3:
				this->building->peopleOutIn();
				break;
			case elevatorevent1:
				this->elevator->response();
				break;
			case elevatorevent2:
				this->elevator->openDoor();
				break;
			case elevatorevent3:
				this->elevator->closeDoor();
				break;
			case elevatorevent4:
				this->elevator->prepareMove();
				break;
			case elevatorevent5:
				this->elevator->updateState();
				break;
			case elevatorevent6:
				this->elevator->makeE9();
				break;
			case elevatorevent7:
				this->elevator->goUpstairs();
				break;
			case elevatorevent8:
				this->elevator->goDownstairs();
				break;
			case trival:
				break;
			}
			//即将执行该时刻的下一个事件。
			event = event->next;
		} while (event != &eventList[timeIndex]);		
	}
}

void Timer::drawState()
{
	std::cout << "此处可以绘出电梯状态" << std::endl;
}

void Timer::cancelE9()
{
	e9->eventID = trival;
}

Timer::~Timer()
{
	//关闭程序时，析构所有事件节点。注意要先于时间轴数列堆对象的销毁
	while (timeIndex >= 0)
	{
		if (eventList[timeIndex].eventID != -1)
		{
			EventNode * event = eventList[timeIndex].next;
			EventNode * p;
			while (event!= &eventList[timeIndex])
			{
				p = event;
				event = event->next;
				delete p;
			}
		}
		timeIndex--;
	}

	delete building;
	delete [] eventList;
}


//注册事件，eventID表示事件名称,interval表示事件间隔。
void Timer::addEventList(int eventID, int interval)
{
	if (eventList[timeIndex + interval].eventID == -1)
	{
		eventList[timeIndex + interval].eventID = eventID;
		if (eventID == elevatorevent6)
		{
			e9 = &eventList[timeIndex + interval];
		}
	}
	else {
		//创建事件节点
		EventNode * event = new EventNode;
		event->eventID = eventID;
		//装载到双向链表里面。
		eventList[timeIndex + interval].prior->next = event;
		event->prior = eventList[timeIndex + interval].prior;
		event->next = &eventList[timeIndex + interval];
		eventList[timeIndex + interval].prior = event;
		if (eventID == elevatorevent6)
		{
			e9 = event;
		}
	}
}
