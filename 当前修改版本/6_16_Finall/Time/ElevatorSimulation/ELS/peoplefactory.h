#ifndef PEOPLEFACTORY_H_
#define PEOPLEFACTORY_H_

#include"people.h"
#include<random>

class PeopleFactory
{
private:
	People * peopleSet[MAXPEOPLENUMBER];
	int index;

public:
	PeopleFactory();
	People * newPeople();
	void deletePeople(int currentTime, int currentFloor, int eD1);
	~PeopleFactory();
};

struct Triple
{
	int InFloor;
	int OutFloor;
	int GiveupTime;
};

Triple * createRandomSequence();

#endif