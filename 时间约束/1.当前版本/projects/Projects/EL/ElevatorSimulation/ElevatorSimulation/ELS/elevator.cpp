#include"elevator.h"

Elevator::Elevator(ITimer * t) {
	//��ʼ������״ֵ̬
	this->Floor = 1;
	this->D1 = this->D2 = this->D3 = 0;
	this->State = idle;
	this->doorTime = 20;
	this->stillMaxTime = 300;
	this->timer = t;
	this->E6Call = false;
	for (int i = 0; i < FLOORSNUMBER; i++)
	{
		this->callCar[i] = this->callDown[i] = this->callUp[i] = false;
		//���ɴ洢�����ڲ���Ա���������
		this->elevatorContainer[i] = new stack<People *>;
	}
}

Elevator::~Elevator() {

	//��������ջ�ڲ���Ա����
	for (int i = 0; i < FLOORSNUMBER; i++)
	{
		//��������ջ���ݽṹ
		delete this->elevatorContainer[i];
	}
}

void Elevator::prepareMove()
{
	this->callCar[this->Floor] = 0;
	if (this->State != goingDown)
	{
		this->callUp[this->Floor] = 0;
	}
	if (this->State != goingUp)
	{
		this->callDown[this->Floor] = 0;
	}
	//�ɻE6������controller()����
	this->E6Call = true;
	this->controller();

	if (this->State=idle)
	{
		//���State=Idle,��ʹ�Ѿ�ִ����Controller()������ҲӦת��E1��
		//this->response();
		this->Floor = 1;
	}
	else
	{
		if (this->D2 != 0)
		{
			//ȡ�����ݻE9
			this->timer->cancelE9();
		}
	}
	if (this->State == goingUp)
	{
		this->timer->addEventList(elevatorevent7,15);
	}
	if (this->State == goingDown)
	{
		this->timer->addEventList(elevatorevent8,15);
	}
}

//�E3,����
void Elevator::openDoor()
{
	this->D1 = 1;
	this->D2 = 1;
	supt->timeHandle("open", START, "controller");
	door.open();			//���Ż
	supt->timeHandle("open", END);
	supt->timeHandle("open_return", START, "open");
	supt->timeHandle("open_return", END);
	this->timer->addEventList(elevatorevent6, 300);
	this->timer->addEventList(elevatorevent3, 76);
	this->timer->addEventList(peopleevent3, 20);	
}

void Elevator::closeDoor()
{
	if (this->D1 != 0)
	{
		this->timer->addEventList(elevatorevent3,40);
	}
	else
	{
		this->D3 = 0;
		supt->timeHandle("close", START, "thisFloorPeopleIn");
		door.close();
		supt->timeHandle("close_return", START, "close");
		supt->timeHandle("close_return", END);
		supt->timeHandle("close", END);
		this->timer->addEventList(elevatorevent4,20);
	}
}

void Elevator::controller()
{
	//C1
	if (this->State==idle)
	{
		this->callUp[1] = 1;
		//C2
		if ((this->callUp[1] != 0 || this->callDown[1] != 0 || this->callCar[1] != 0) && this->Floor == 1)
		{
			//Ԥ��20��t������E3
			this->timer->addEventList(elevatorevent2, 20);
			return;
		}
		//C3  ǰ�벿��
		//�ҵ���С��¥�㣬ʹ�ø�¥���Ӧ��������ť�����б������µİ�ť��
		int j;
		for (j = 0; j < FLOORSNUMBER; j++)
		{
			if (this->callUp[j] != 0 || this->callDown[j] != 0 || this->callCar[j] != 0) {
				break;
			}
		}
		//C4
		if (j < FLOORSNUMBER)
		{
			if (Floor > j)
			{
				State = goingDown;
			}
			else if (Floor < j)
			{
				State = goingUp;
			}
		}
		//C3 ��벿��
		else
		{
			//�ж��Ƿ�ʱE6����
			if (this->E6Call==true)
			{
				j = 1;
				this->E6Call = false;
			}
			else
			{
				return;
			}
		}
		//C5
		if (this->Floor == 1 && j != 1)
		{
			this->timer->addEventList(elevatorevent4,20);
			return;
		}
	}
}

void Elevator::goUpstairs()
{
	static bool upShouldWait = true;
	if (upShouldWait == true)
	{
		this->Floor = this->Floor + 1;
		this->timer->addEventList(elevatorevent7,51);
		upShouldWait = false;
	}
	else
	{
		//�������е�j>Floor,����callUp[j]=callDown[j]=callCar[j]=0��
		bool flag = true;
		for (int j = this->Floor+1; j < FLOORSNUMBER; j++)
		{
			if (this->callCar[j]!=0 || this->callDown[j]!=0 || this->callCar[j]!=0) {
				flag = false;
			}
		}

		if (this->callCar[this->Floor] == 1 || this->callUp[this->Floor] == 1 || ((this->Floor == 1 || this->callDown[this->Floor] == 1) && flag == true))
		{
			this->timer->addEventList(elevatorevent5,14);
		}
		else
		{
			//�ظ�E7��������ݼ�������
			this->timer->addEventList(elevatorevent7,0);
			upShouldWait = true;
		}
	}
}

void Elevator::goDownstairs()
{
	static bool downShouldWait = true;
	if (downShouldWait == true)
	{
		this->Floor = this->Floor - 1;
		this->timer->addEventList(elevatorevent8, 61);
		downShouldWait = false;
	}
	else
	{
		//�������е�j<Floor,����callUp[j]=callDown[j]=callCar[j]=0��
		bool flag = true;
		for (int j = this->Floor - 1; j >= 0; j--)
		{
			if (this->callCar[j] != 0 || this->callDown[j] != 0 || this->callCar[j] != 0) {
				flag = false;
			}
		}

		if (this->callCar[this->Floor] == 1 || this->callDown[this->Floor] == 1 || ((this->Floor == 1 || this->callUp[this->Floor] == 1) && flag == true))
		{
			this->timer->addEventList(elevatorevent5, 23);
		}
		else
		{
			//�ظ�E8��������ݼ����½�
			this->timer->addEventList(elevatorevent8, 0);
			downShouldWait = true;
		}
	}
}

void Elevator::pushCallUp(int floor)
{
	callUp[floor] = true;
}

void Elevator::pushCallDown(int floor)
{
	callDown[floor] = true;
}

void Elevator::pushCallCar(int floor)
{
	callCar[floor] = true;
}

void Elevator::response()
{
	//�ڵ�һ��Ⱥ�ʱ���õ��ݱ��������Ч��Ӧ
	//�E1��ʾ��һ��ͣ�򣬵ȼ���this->State==idle && this->Floor==1

	if (supt->getParamValueWithNameAndKey("controller", "State") == 1)
		State = idle;
	Floor = supt->getParamValueWithNameAndKey("controller", "Floor");
	if (this->State==idle && this->Floor==1)
	{
		supt->timeHandle("controller", START, "response");
		controller();
		supt->timeHandle("controller", END);
	}
}

bool Elevator::thisFloorOut()
{
	if (this->elevatorContainer[this->Floor]->empty() == false)
	{
		return true;
	}
	return false;
}

void Elevator::thisFloorPeopleOut()
{
	this->elevatorContainer[this->Floor]->top()->setCurrentState(leaving);
	this->elevatorContainer[this->Floor]->pop();
	this->timer->addEventList(peopleevent3,25);
}

//�ӵ�ǰ¥�����һ����Ա������Ҫ�趨���ݵ�callCar���ԡ�
void Elevator::thisFloorPeopleIn(People * commer)
{
	this->elevatorContainer[commer->getOutFloor()]->push(commer);
	this->callCar[commer->getOutFloor()] = 1;	
}

void Elevator::updateState()
{
	if (this->State == goingUp)
	{
		bool hasPeopleWaitUp = false;
		for (int j = this->Floor + 1; j < FLOORSNUMBER; j++)
		{
			if (this->callCar[j] != 0 || this->callDown[j] != 0 || this->callCar[j] != 0) {
				hasPeopleWaitUp = true;
			}
		}
		bool hasPeopleWaitDown = false;
		for (int j = this->Floor - 1; j >= 0; j--)
		{
			if (this->callCar[j] != 0 || this->callDown[j] != 0 || this->callCar[j] != 0) {
				hasPeopleWaitDown = true;
			}
		}
		if (hasPeopleWaitDown==false && hasPeopleWaitUp==false)
		{
			this->State = idle;
		}
		if (hasPeopleWaitUp == false && hasPeopleWaitDown == true)
		{
			this->State = goingDown;
		}
	}else if (this->State == goingDown) {
		bool hasPeopleWaitUp = false;
		for (int j = this->Floor + 1; j < FLOORSNUMBER; j++)
		{
			if (this->callCar[j] != 0 || this->callDown[j] != 0 || this->callCar[j] != 0) {
				hasPeopleWaitUp = true;
			}
		}
		bool hasPeopleWaitDown = false;
		for (int j = this->Floor - 1; j >= 0; j--)
		{
			if (this->callCar[j] != 0 || this->callDown[j] != 0 || this->callCar[j] != 0) {
				hasPeopleWaitDown = true;
			}
		}
		if (hasPeopleWaitDown == false && hasPeopleWaitUp == false)
		{
			this->State = idle;
		}
		if (hasPeopleWaitUp == true && hasPeopleWaitDown == false)
		{
			this->State = goingUp;
		}
	}
	this->timer->addEventList(elevatorevent2,0);
}

void Elevator::makeE9()
{
	this->D2 = 0;
	this->controller();
}

