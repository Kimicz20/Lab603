#include "people.h"

People::People(int InFloor, int OutFloor, int GiveupTime)
{
	this->InFloor = InFloor;
	this->OutFloor = OutFloor;
	this->GiveupTime = GiveupTime;
	this->currentState = waiting;
	this->StartTime = -10000;
}