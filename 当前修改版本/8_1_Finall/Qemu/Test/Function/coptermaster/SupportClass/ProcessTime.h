//����ʱ����
#ifndef ProcessTime_class
#define ProcessTime_class

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <list>
#include <map>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

class ProcessTime{

	struct timeval startTime,endTime;
	double interval = 0;
	//double pinterval = 0;
public:
	//ʱ��� s
	double setInterval(){
		return 1000.0 * (endTime.tv_sec - startTime.tv_sec)
			+ (endTime.tv_usec - startTime.tv_usec) / 1000.0;
	}
	/*double setPinterval(myProcessTime pre){
		return 1000.0 * (startTime.tv_sec - pre.startTime.tv_sec)
			+ (startTime.tv_usec - pre.startTime.tv_usec) / 1000.0;
	}
	void show(){
		printf("pinterval = %lf ms ,interval = %lf ms\n", pinterval, interval);
	}*/
	void showInterval(string p){
		cout << p << ": interval = " << interval << " ms" << endl;
	}
	double getInterval(){
		return interval/100;
	}
	/*
	��ʼ������ʱ��
		0:�ʼ��ʼ��
		1:��ʼʱ��
		2:��ֹʱ��
	*/
	void init(int i){
		switch (i){
			case 1:
				gettimeofday(&startTime, NULL);
				break;
			case 2:
				gettimeofday(&endTime, NULL);
				interval = setInterval();
				break;
		}
	}
};
#endif