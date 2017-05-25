#ifndef DOOR_H_
#define DOOR_H_

class Door {

private:
	enum
	{
		opened,
		closed
	}state;
public:

	Door();

	void open();

	void close();
};

#endif // !DOOR_H_

