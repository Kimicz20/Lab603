#ifndef CIRCULARQUEUE_H_
#define CIRCULARQUEUE_H_

template <typename Type,int len=100>
class CircularQueue {

private:
	
	Type circularQueue[len];
	int front;
	int rear;
	int iterator;

public:

	CircularQueue();

	//插入队列
	void pushElement(Type one);

	//访问队头元素
	Type & frontElement();

	//删除队头元素
	void popElement();

	//从尾部到队头遍历元素
	Type * nextElement();

	//判断队列是否为空
	bool empty();

	//判断队列是否已满，在电梯实现中可能用不到。
	bool full();

};

template<typename Type, int len>
CircularQueue<Type, len>::CircularQueue()
{
	front = rear = iterator = 0;
}

template<typename Type, int len>
void CircularQueue<Type, len>::pushElement(Type one)
{
	rear = (rear + 1) % len;
	circularQueue[rear] = one;
	iterator = rear;
}

template<typename Type, int len>
Type & CircularQueue<Type, len>::frontElement()
{
	return circularQueue[(front + 1) % len];
}

template<typename Type, int len>
void CircularQueue<Type, len>::popElement()
{
	front = (front + 1) % len;
}

template<typename Type, int len>
Type * CircularQueue<Type, len>::nextElement()
{
	int temp = iterator;
	iterator = (iterator + len - 1) % len;
	if (temp==front)
	{
		return nullptr;
	}
	return &circularQueue[temp];
}

template<typename Type, int len>
bool CircularQueue<Type, len>::empty()
{
	if (rear==front)
	{
		return true;
	}
	else
	{
		return false;
	}
}

template<typename Type, int len>
bool CircularQueue<Type, len>::full()
{
	if ((rear+1)%len==front)
	{
		return true;
	}
	else
	{
		return false;
	}
}
#endif // !CIRCULARQUEUE_H_