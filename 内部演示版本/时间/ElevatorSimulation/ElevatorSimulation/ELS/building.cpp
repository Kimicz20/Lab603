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
	//����ÿ��¥������������Ա����
	for (int i = 0; i < FLOORSNUMBER; i++)
	{
		//����¥��������ݽṹ��	
		delete this->buildingQueue[i];
	}

	//�������ݶ���
	delete elevator;
}


void Building::peopleComing()
{
	
	//������Ա����
	supt->timeHandle("newPeople", START, "peopleComing");
	People * singlePeople = pf.newPeople();
	supt->timeHandle("newPeople", END);

	singlePeople->setStartTime(timer->getTime());

	std::default_random_engine dre;

	std::uniform_int_distribution<int> di2(100, 200);

	this->InterTime = di2(dre) / 10 * 10;

	//Ϊ��һ����Ա����Ĳ�����׼������ע�������Ա�����¼���
	this->timer->addEventList(peopleevent1, this->InterTime);

	//������һ����Ա�������Ҫֱ��ע���Ǹ�ʱ�̵ĵ�����Ӧ�¼���
	this->timer->addEventList(elevatorevent1, this->InterTime);

	//�����Ӧ¥����С�
	this->buildingQueue[singlePeople->getInFloor()]->push(singlePeople);

	//���µ�ť���ҵȺ�
	if (singlePeople->getInFloor() > singlePeople->getOutFloor())
	{
		supt->timeHandle("pushCallDown", START, "newPeople");
		elevator->pushCallDown(singlePeople->getInFloor());  //��¥������°�ť������
		supt->timeHandle("pushCallDown", END);
	}
	else
	{
		supt->timeHandle("pushCallUp", START, "newPeople");
		elevator->pushCallUp(singlePeople->getInFloor());  //��¥������ϰ�ť������
		supt->timeHandle("pushCallUp", END);
	}

	//ע��ﵽ����ʱ�̺�����Ƿ�Ҫ�����ȴ����ж��¼���
	this->timer->addEventList(peopleevent2, singlePeople->getGiveupTime());

}

void Building::peopleOutIn()
{
	bool flag = this->elevator->thisFloorOut();
	if (supt->getParamValueWithNameAndKey("thisFloorPeopleOut", "elevator_thisFloorOut") == 1)
		flag = true;
	//���������������ȳ������������ٶ���ÿ��25��t����һ����
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
	//�������������˿�ʼ���룬�������ٶ���ÿ��25��t����һ����,ע�������ʱ����Ҫ�������ڲ���CallCar���԰��£����˷�����
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
			//��¥���ⲿ�����������һ����
			supt->timeHandle("thisFloorPeopleIn", START, "open");
			this->elevator->thisFloorPeopleIn(this->buildingQueue[this->elevator->getCurrentFloor()]->front());
			supt->timeHandle("thisFloorPeopleIn_return", START, "thisFloorPeopleIn");
			supt->timeHandle("thisFloorPeopleIn_return", END);
			supt->timeHandle("thisFloorPeopleIn", END);
			this->buildingQueue[this->elevator->getCurrentFloor()]->pop();
			//25��t���ٴβ鿴���������Ƿ�������Ҫ��������ڲ���
			this->timer->addEventList(peopleevent3,25);		
			return;
		}
	}
	//�����¥��Ķ���Ϊ�գ���D1=0��D3��=0��
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
