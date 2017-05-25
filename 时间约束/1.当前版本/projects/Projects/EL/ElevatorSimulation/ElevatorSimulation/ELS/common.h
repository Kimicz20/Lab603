#ifndef COMMON_H_
#define COMMON_H_

#include<iostream>


#define TIMEUNIT 100		//ʱ�䵥λ��Ŀǰ��100ms��1s=1000ms  
#define FLOORSNUMBER 5		//¥����
#define MAXPEOPLENUMBER 1000 //�����Ա����

struct EventNode
{
	int eventID;			//�¼���־
	EventNode * next;
	EventNode * prior;
	EventNode() {
		eventID = -1;
		next = this;
		prior = this;
	}
};

enum
{
	/*
		�¼�����
		�¼�ȫ���ɶ�ʱ������	
	*/
	peopleevent1,		//���޵��д�����Ա�����¼���
	peopleevent2,		//ɾ����ʱ��Ⱥ��·�������Ա�����¼���
	peopleevent3,		//�ڱ���¥���������ȳ��ˣ�Ȼ���ڽ����¼������ݻE4�Ĵ����¼���

	elevatorevent1,		//����Ӧ����Ӧ�ⲿ��Ա�����°�ť�¼���
	elevatorevent2,		//����Ӧ�ÿ����¼���
	elevatorevent3,		//���ݼ���Ƿ�Ӧ�ù����¼���
	elevatorevent4,		//����׼���ƶ��¼���
	elevatorevent5,		//���ݸı�����״̬��״̬���������½�����ֹ���֣��¼���
	elevatorevent6,		//���ݻE9�Ĵ����¼���
	elevatorevent7,		//���ݵ�����һ��¥�����¼���
	elevatorevent8,		//���ݵ��½�һ��¥�����¼���

	trival,				//��Ч�¼���ʲôҲ��������
};

enum PeopleState
{
	waiting,
	leaving,
};



#endif // COMMON_H_

