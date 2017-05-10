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
	//����ÿ��¥������������Ա����
	for (int i = 0; i < FLOORSNUMBER; i++)
	{
		while (!this->buildingQueue[i]->empty())
		{
			delete this->buildingQueue[i]->frontElement();
			this->buildingQueue[i]->popElement();
		}
		//����¥��������ݽṹ��	
		delete this->buildingQueue[i];
	}

	//�������ݶ���
	delete elevator;
}


void Building::newPeople()
{
	//����������������޳���InFloor��OutFloor��ȵ��������Ԫ�ء�
	while (dataSequence[dataIndex].InFloor == dataSequence[dataIndex].OutFloor)
	{
		++dataIndex;
	}

	//������Ա����
	People * singlePeople = new People(dataSequence[dataIndex].InFloor,dataSequence[dataIndex].OutFloor,dataSequence[dataIndex].GiveupTime);

	this->InterTime = dataSequence[dataIndex].InterTime;

	//Ϊ��һ����Ա����Ĳ�����׼������ע�������Ա�����¼���
	this->timer->addEventList(peopleevent1, this->InterTime);

	//����һ����Ա�������Ҫֱ��ע���Ǹ�ʱ�̵ĵ�����Ӧ�¼���
	this->timer->addEventList(elevatorevent1, this->InterTime);

	//�����Ӧ¥����С�
	this->buildingQueue[singlePeople->getInFloor()]->pushElement(singlePeople);

	//���µ�ť���ҵȺ�
	if (singlePeople->getInFloor() > singlePeople->getOutFloor())
	{
		elevator->pushCallDown(singlePeople->getInFloor());  //��¥������°�ť������
	}
	else
	{
		elevator->pushCallUp(singlePeople->getInFloor());  //��¥������ϰ�ť������
	}
		
	++dataIndex;
}

void Building::peopleOutIn()
{
	//���������������ȳ������������ٶ���ÿ��25��t����һ����
	if (this->elevator->thisFloorOut() == true)
	{
		this->elevator->thisFloorPeopleOut();
		return;
	}
	//�������������˿�ʼ���룬�������ٶ���ÿ��25��t����һ����,ע�������ʱ����Ҫ�������ڲ���CallCar���԰��£����˷�����
	//ע��Ҫ������Ӧ¥���������Ŀ�ָ���������ָ������Ϊ�������ڳ����ȴ�������ʱ����ߵ��ˡ�
	if (this->buildingQueue[this->elevator->getCurrentFloor()]->empty() == false) {
		while (this->buildingQueue[this->elevator->getCurrentFloor()]->empty() == false && this->buildingQueue[this->elevator->getCurrentFloor()]->frontElement() == nullptr)
		{
			this->buildingQueue[this->elevator->getCurrentFloor()]->popElement();
		}
		if (this->buildingQueue[this->elevator->getCurrentFloor()]->empty() == false)
		{
			//��¥���ⲿ�����������һ����
			this->elevator->thisFloorPeopleIn(this->buildingQueue[this->elevator->getCurrentFloor()]->frontElement());
			this->buildingQueue[this->elevator->getCurrentFloor()]->popElement();
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


//��������У���������
tetrad * createRandomSequence()
{
	std::default_random_engine dre;
	//���¥��
	std::uniform_int_distribution<int> di1(0, FLOORSNUMBER);
	//�����300-600��ʱ�䵥λ�ڲ���һ���ˣ����Ҹ��˵�����ʱ������Ϊ300-600��ʱ�䵥λ��
	std::uniform_int_distribution<int> di2(300, 600); 

	for (int i = 0; i < MAXPEOPLENUMBER; i++)
	{
		dataSequence[i].InFloor = di1(dre);
		dataSequence[i].OutFloor = di1(dre);
		dataSequence[i].GiveupTime = di2(dre)/10*10;
		dataSequence[i].InterTime = di2(dre)/10*10;
	}
	return dataSequence;
}
