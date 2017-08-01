#ifndef BUILDING_H_
#define BUILDING_H_

#include"elevator.h"
#include<queue>
#include"peoplefactory.h"
#include"../SupportClass/SupportClass.h"

class Building {

private:
	
	//�¸��˳��ֵ�ʱ������
	int InterTime;

	//�ö�¥��Ӧ�ĵ��ݡ�
	Elevator * elevator;

	//������Ա�����¥����У�ÿ��¥����Ա�����������Ҫ�Ŷӵȴ���
	std::queue<People *> * buildingQueue[FLOORSNUMBER];

	//��ʱ��ָ�룬������ʱ��ע���¼���
	ITimer * timer;

	PeopleFactory pf;

public:
	SupportClass *supt;

	Building(ITimer *);

	~Building();

	//�������һ����Ա������Ҫ�趨��ֵ�������ݵ�callUp,callDown���ԡ�
	void peopleComing();

	//��¥�������������ȳ������ȴ������ٽ�ȥ��
	void peopleOutIn();
	
	Elevator * getElevator() {
		return elevator;
	}

	//ɾ����Щ��Ϊ��������ʱ�����������
	void deletePeople();

};

#endif // !BUILDING_H_
