#ifndef BUILDING_H_
#define BUILDING_H_

#include<random>
#include"elevator.h"
#include"circularqueue.h"

class Building {

private:
	
	//�¸��˳��ֵ�ʱ������
	int InterTime;

	//�ö�¥��Ӧ�ĵ��ݡ�
	Elevator * elevator;

	//������Ա�����¥����У�ÿ��¥����Ա�����������Ҫ�Ŷӵȴ���
	CircularQueue<People *, MAXPEOPLENUMBER> * buildingQueue[FLOORSNUMBER];

	//��ʱ��ָ�룬������ʱ��ע���¼���
	ITimer * timer;

public:

	Building(ITimer *);

	~Building();

	//�������һ����Ա������Ҫ�趨��ֵ�������ݵ�callUp,callDown���ԡ�
	void newPeople();

	//��¥�������������ȳ������ȴ������ٽ�ȥ��
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
