#ifndef TIMER_H_
#define TIMER_H_

#include"building.h"
#include<stdlib.h>
#include <unistd.h>
#include"../SupportClass/SupportClass.h"

class Timer : public ITimer {

private:

	Building * building;

	Elevator * elevator;

	//ʱ�䳤�ȡ�
	int timeLen;

	//ʱ����������
	int timeIndex;

	//�¼��������У���������ʱ�������С��¼���������˳��洢��ÿ��ʱ����ϵĴ�ִ���¼�������˫������洢��
	EventNode * eventList;
	

public:
	SupportClass *supt;

	Timer(int timeLen);

	//��ʼ�����񣺴�0��ʼʱ�����˰����ݣ����ݿ�ʼ��Ӧ��
	void init();

	//���ΰ���ʱ�����ϵ�˳������ִ���¼���
	void run();

	//ע���¼�,eventID��ʾ�¼���־��interval��ʾ���¼�����ڵ�ǰʱ�����ϵ�ʱ��ƫ��������ζ�Ź�����ʱ�䵥λ��ʼִ�и��¼���
	void addEventList(int eventID,int interval);

	//ִ��ĳ���ض�ʱ����ϵ��¼�����
	void executeEvent();

	//���״̬ͼ��
	void drawState();

	//ȡ���E9
	void cancelE9();

	//���ص�ǰ��ʱ��
	int getTime() {
		return timeIndex;
	}

	~Timer();

};

#endif // !TIME_H_

