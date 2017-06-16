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

	//时间长度。
	int timeLen;

	//时间轴索引。
	int timeIndex;

	//事件发生序列，索引代表时间轴序列。事件发生序列顺序存储。每个时间点上的待执行事件排列用双向链表存储。
	EventNode * eventList;
	

public:
	SupportClass *supt;

	Timer(int timeLen);

	//初始化任务：从0开始时刻有人按电梯，电梯开始响应。
	void init();

	//依次按照时间轴上的顺序依次执行事件。
	void run();

	//注册事件,eventID表示事件标志，interval表示该事件相对于当前时间轴上的时间偏移量（意味着过多少时间单位开始执行该事件）
	void addEventList(int eventID,int interval);

	//执行某个特定时间点上的事件序列
	void executeEvent();

	//绘出状态图像
	void drawState();

	//取消活动E9
	void cancelE9();

	//返回当前的时间
	int getTime() {
		return timeIndex;
	}

	~Timer();

};

#endif // !TIME_H_

