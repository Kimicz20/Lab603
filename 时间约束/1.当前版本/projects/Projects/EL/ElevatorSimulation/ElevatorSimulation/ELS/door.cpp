#include"door.h"

Door::Door()
{
	state = closed;
}

void Door::open()
{
	state = opened;
}

void Door::close()
{
	state = closed;
}
