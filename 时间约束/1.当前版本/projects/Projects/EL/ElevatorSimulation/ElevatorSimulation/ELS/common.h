#ifndef COMMON_H_
#define COMMON_H_

#include<iostream>


#define TIMEUNIT 100		//时间单位，目前是100ms，1s=1000ms  
#define FLOORSNUMBER 5		//楼层数
#define MAXPEOPLENUMBER 1000 //最大人员数量

struct EventNode
{
	int eventID;			//事件标志
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
		事件名称
		事件全部由定时器发出	
	*/
	peopleevent1,		//从无到有创建人员对象事件。
	peopleevent2,		//删除长时间等候导致放弃的人员对象事件。
	peopleevent3,		//在本层楼触发电梯先出人，然后在进人事件。电梯活动E4的触发事件。

	elevatorevent1,		//电梯应该响应外部人员对象按下按钮事件。
	elevatorevent2,		//电梯应该开门事件。
	elevatorevent3,		//电梯检查是否应该关门事件。
	elevatorevent4,		//电梯准备移动事件。
	elevatorevent5,		//电梯改变运行状态（状态有上升，下降，静止三种）事件。
	elevatorevent6,		//电梯活动E9的触发事件。
	elevatorevent7,		//电梯的上升一层楼触发事件。
	elevatorevent8,		//电梯的下降一层楼触发事件。

	trival,				//无效事件，什么也不用做。
};

enum PeopleState
{
	waiting,
	leaving,
};



#endif // COMMON_H_

