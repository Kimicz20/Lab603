#include "timer.h"

//用来记录触发E9活动的事件节点
EventNode * e9 = nullptr;

Timer::Timer(int timeLen)
{
	this->building = new Building(this);
	this->eventList = new EventNode[timeLen+1000];
	this->elevator = this->building->getElevator();
	this->timeIndex = 0;
	this->timeLen = timeLen;
}

void Timer::init()
{
	this->addEventList(peopleevent1,0);
	this->addEventList(elevatorevent1,0);

	building->supt = supt;
	elevator->supt = supt;
}

void Timer:: run()
{
	//起始时间初始化
	supt->timeHandle("run", INIT);
	do
	{
		executeEvent();
		drawState();			//绘出状态数值  方法体为空。
		this->timeIndex++;
		usleep(100000);			//linux 下sleep按秒来计时 usleep 用微妙计时
		if (this->timeIndex % 10 == 0)
		{
			std::cout << "now sec" << this->timeIndex / 10 << endl;
		}
		/*if (this->timeIndex==this->timeLen)
		{
			std::getchar();
		}*/
	} while (this->timeIndex <= this->timeLen);
	//起始时间初始化
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
					supt->timeHandle("peopleComing", START, "run");
					this->building->peopleComing();
					supt->timeHandle("peopleComing", END);
					break;
				case peopleevent2:
					this->building->deletePeople();
					break;
				case peopleevent3:
					supt->timeHandle("peopleOutIn", START, "open_return");
					this->building->peopleOutIn();
					supt->timeHandle("peopleOutIn", END);
					break;
				case elevatorevent1:
					supt->timeHandle("response", START, "pushCallUp");
					this->elevator->response();
					supt->timeHandle("response", END);
					break;
				case elevatorevent2:
					this->elevator->openDoor();
					break;
				case elevatorevent3:
					this->elevator->closeDoor();
					break;
				case elevatorevent4:
					supt->timeHandle("prepareMove", START, "close_return");
					this->elevator->prepareMove();
					supt->timeHandle("prepareMove", END);
					break;
				case elevatorevent5:
					this->elevator->updateState();
					break;
				case elevatorevent6:
					this->elevator->makeE9();
					break;
				case elevatorevent7:
					supt->timeHandle("goUpstairs", START, "prepareMove");
					this->elevator->goUpstairs();
					supt->timeHandle("goUpstairs", END);
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
	//std::cout << "此处可以绘出电梯状态" << std::endl;
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