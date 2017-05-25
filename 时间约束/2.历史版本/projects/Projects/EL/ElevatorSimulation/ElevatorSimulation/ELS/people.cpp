#include "people.h"

People::People(int InFloor, int OutFloor, int GiveupTime, int StartTime)
{
	this->InFloor = InFloor;
	this->OutFloor = OutFloor;
	this->GiveupTime = GiveupTime;
	this->StartTime = StartTime;
	this->consumeTime = 25;
}
