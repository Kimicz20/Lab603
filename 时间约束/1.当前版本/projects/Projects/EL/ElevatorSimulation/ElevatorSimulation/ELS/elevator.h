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

	//电梯所处的三个状态。
	enum { goingDown = -1, idle, goingUp } State;

	//外部输入的信号。
	bool callUp[FLOORSNUMBER];
	bool callDown[FLOORSNUMBER];

	//内部输入的信号。
	bool callCar[FLOORSNUMBER];

	//当前电梯所在的层数。
	int Floor;

	//人们正在进入和离开电梯时，值应为1。
	int D1;

	//如果电梯已经在某层等候300t以上，值应为0。
	int D2;

	//D3重点关注的值应该为0，当为0的时候表示接下来电梯要关门了。
	int D3;

	//电梯要存储的人员，没有限制。
	stack<People *> * elevatorContainer[FLOORSNUMBER];

	//开关门所需要的时间。
	int doorTime;

	//电梯在某层静止的最大时间。
	int stillMaxTime;

	//定时器指针，用来向定时器注册事件。
	ITimer * timer;

	//E6调用Controller标志。
	bool E6Call;

	//门对象
	Door door;

public:
	SupportClass *supt;

	Elevator(ITimer *);

	~Elevator();

	//电梯准备移动处理逻辑，对应于电梯活动E6
	void prepareMove();

	//开门活动
	void openDoor();

	//关门活动
	void closeDoor();

	//controller函数
	void controller();

	//上升一层
	void goUpstairs();

	//下降一层
	void goDownstairs();
	
	//按下相应楼层的callUp
	void pushCallUp(int floor);

	//按下相应楼层的callDown
	void pushCallDown(int floor);

	//按下电梯内部的callCar
	void pushCallCar(int floor);

	//电梯响应函数，函数职责是用来进行判断电梯在面对有人按下时该怎么做。这个事件紧跟在产生人员对象后调用（产生的人员对象负责按下电梯外部上下按钮任务）
	void response();

	//电梯在当前Floor停下时，有人的OutFloor=Floor
	bool thisFloorOut();

	//电梯在当前Floor停下时，有人开始出电梯
	void thisFloorPeopleOut();

	//从当前楼层进入一个人员对象，需要设定电梯的callCar属性。
	void thisFloorPeopleIn(People *);

	//返回电梯所在的楼层信息
	int getCurrentFloor() {
		return Floor;
	}

	//设置D1函数
	void setD1(int n) {
		D1 = n;
	}

	//返回D1值函数
	int getD1() {
		return D1;
	}

	//设置D3函数
	void setD3(int n) {
		D3 = n;
	}

	//电梯改变状态活动，对应于E2
	void updateState();

	//电梯置不活动指示器  E9
	void makeE9();

};

#endif // !ELEVATOR_H_
