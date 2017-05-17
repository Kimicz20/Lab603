#ifndef INTERFACE_TIME_H_
#define INTERFACE_TIME_H_

class ITimer {
public:
	virtual void init() = 0;
	virtual void run() = 0;
	virtual void addEventList(int eventID, int interval) = 0;
	virtual void executeEvent() = 0;
	virtual void drawState() = 0;
	virtual void cancelE9() = 0;
	virtual int getTime() = 0;
	virtual ~ITimer() {
	};
};

#endif // !INTERFACE_TIME_H_

