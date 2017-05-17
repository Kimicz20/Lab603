#include "timer.h"

//������¼����E9����¼��ڵ�
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
		//drawState();			//���״̬��ֵ  ������Ϊ�ա�
		this->timeIndex++;
		usleep(100000);//linux ��sleep��������ʱ usleep ��΢���ʱ
		//cout << "timeIndex:" << this->timeIndex << endl;
	} while (this->timeIndex <= this->timeLen);
}

void Timer::executeEvent()
{	
	struct timeval start, end;
	int interval, pinterval;
	if (this->eventList[timeIndex].eventID != -1)
	{
		EventNode * event = &eventList[timeIndex];
		do
		{
			switch (event->eventID) {
				case peopleevent1:
					std::cout << "newPeople: " ;
					gettimeofday(&start, NULL);
					pinterval = 1000000 * (start.tv_sec - supt->preProcessTime.tv_sec) + (start.tv_usec - supt->preProcessTime.tv_usec);

					this->building->newPeople();
					gettimeofday(&end, NULL);
					supt->preProcessTime = end;
					interval = 1000000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
					printf("pinterval = %f ,interval = %f\n ", pinterval / 1000.0 ,interval / 1000.0);
					
					break;
				case peopleevent2:
					std::cout << "deletePeople" << std::endl;
					
					gettimeofday(&start, NULL);
					pinterval = 1000000 * (start.tv_sec - supt->preProcessTime.tv_sec) + (start.tv_usec - supt->preProcessTime.tv_usec);

					this->building->deletePeople();
					gettimeofday(&end, NULL);
					supt->preProcessTime = end;
					interval = 1000000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
					printf("pinterval = %f ,interval = %f\n ", pinterval / 1000.0, interval / 1000.0);

					break;
				case peopleevent3:
					std::cout << "peopleOutIn" << std::endl;
					this->building->peopleOutIn();
					break;
				case elevatorevent1:
					std::cout << "response" << std::endl;
				
					gettimeofday(&start, NULL);
					pinterval = 1000000 * (start.tv_sec - supt->preProcessTime.tv_sec) + (start.tv_usec - supt->preProcessTime.tv_usec);

					this->elevator->response();
					gettimeofday(&end, NULL);
					supt->preProcessTime = end;
					interval = 1000000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
					printf("pinterval = %f ,interval = %f\n ", pinterval / 1000.0, interval / 1000.0);
					break;
				case elevatorevent2:
					std::cout << "openDoor" << std::endl;
					this->elevator->openDoor();
					break;
				case elevatorevent3:
					std::cout << "closeDoor" << std::endl;
					gettimeofday(&start, NULL);
					this->elevator->closeDoor();
					gettimeofday(&end, NULL);
					break;
				case elevatorevent4:
					std::cout << "prepareMove" << std::endl;

					gettimeofday(&start, NULL);
					pinterval = 1000000 * (start.tv_sec - supt->preProcessTime.tv_sec) + (start.tv_usec - supt->preProcessTime.tv_usec);

					this->elevator->prepareMove();

					gettimeofday(&end, NULL);
					supt->preProcessTime = end;
					interval = 1000000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
					printf("pinterval = %f ,interval = %f\n", pinterval / 1000.0, interval / 1000.0);

					break;
				case elevatorevent5:
					std::cout << "updateState" << std::endl;
					this->elevator->updateState();
					break;
				case elevatorevent6:
					std::cout << "makeE9" << std::endl;
					this->elevator->makeE9();
					break;
				case elevatorevent7:
					std::cout << "goUpstairs" << std::endl;
					this->elevator->goUpstairs();
					break;
				case elevatorevent8:
					std::cout << "goDownstairs" << std::endl;
					this->elevator->goDownstairs();
					break;
				case trival:
					break;
			}
			//����ִ�и�ʱ�̵���һ���¼���
			event = event->next;
		} while (event != &eventList[timeIndex]);		
	}
}

void Timer::drawState()
{
	std::cout << "el state" << std::endl;
}

void Timer::cancelE9()
{
	e9->eventID = trival;
}

Timer::~Timer()
{
	//�رճ���ʱ�����������¼��ڵ㡣ע��Ҫ����ʱ�������жѶ��������
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


//ע���¼���eventID��ʾ�¼�����,interval��ʾ�¼������
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
		//�����¼��ڵ�
		EventNode * event = new EventNode;
		event->eventID = eventID;
		//װ�ص�˫���������档
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
