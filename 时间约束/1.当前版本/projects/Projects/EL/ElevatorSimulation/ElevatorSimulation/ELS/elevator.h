#ifndef ELEVATOR_H_
#define ElEVATOR_H_

#include<stack>
#include"people.h"
#include"common.h"
#include"timeinterface.h"
#include"door.h"
#include"../SupportClass/SupportClass.h"

using std::stack;

class Elevator {

private:

	//��������������״̬��
	enum { goingDown = -1, idle, goingUp } State;

	//�ⲿ������źš�
	bool callUp[FLOORSNUMBER];
	bool callDown[FLOORSNUMBER];

	//�ڲ�������źš�
	bool callCar[FLOORSNUMBER];

	//��ǰ�������ڵĲ�����
	int Floor;

	//�������ڽ�����뿪����ʱ��ֵӦΪ1��
	int D1;

	//��������Ѿ���ĳ��Ⱥ�300t���ϣ�ֵӦΪ0��
	int D2;

	//D3�ص��ע��ֵӦ��Ϊ0����Ϊ0��ʱ���ʾ����������Ҫ�����ˡ�
	int D3;

	//����Ҫ�洢����Ա��û�����ơ�
	stack<People *> * elevatorContainer[FLOORSNUMBER];

	//����������Ҫ��ʱ�䡣
	int doorTime;

	//������ĳ�㾲ֹ�����ʱ�䡣
	int stillMaxTime;

	//��ʱ��ָ�룬������ʱ��ע���¼���
	ITimer * timer;

	//E6����Controller��־��
	bool E6Call;

	//�Ŷ���
	Door door;

public:
	SupportClass *supt;

	Elevator(ITimer *);

	~Elevator();

	//����׼���ƶ������߼�����Ӧ�ڵ��ݻE6
	void prepareMove();

	//���Ż
	void openDoor();

	//���Ż
	void closeDoor();

	//controller����
	void controller();

	//����һ��
	void goUpstairs();

	//�½�һ��
	void goDownstairs();
	
	//������Ӧ¥���callUp
	void pushCallUp(int floor);

	//������Ӧ¥���callDown
	void pushCallDown(int floor);

	//���µ����ڲ���callCar
	void pushCallCar(int floor);

	//������Ӧ����������ְ�������������жϵ�����������˰���ʱ����ô��������¼������ڲ�����Ա�������ã���������Ա�������µ����ⲿ���°�ť����
	void response();

	//�����ڵ�ǰFloorͣ��ʱ�����˵�OutFloor=Floor
	bool thisFloorOut();

	//�����ڵ�ǰFloorͣ��ʱ�����˿�ʼ������
	void thisFloorPeopleOut();

	//�ӵ�ǰ¥�����һ����Ա������Ҫ�趨���ݵ�callCar���ԡ�
	void thisFloorPeopleIn(People *);

	//���ص������ڵ�¥����Ϣ
	int getCurrentFloor() {
		return Floor;
	}

	//����D1����
	void setD1(int n) {
		D1 = n;
	}

	//����D1ֵ����
	int getD1() {
		return D1;
	}

	//����D3����
	void setD3(int n) {
		D3 = n;
	}

	//���ݸı�״̬�����Ӧ��E2
	void updateState();

	//�����ò��ָʾ��  E9
	void makeE9();

};

#endif // !ELEVATOR_H_
