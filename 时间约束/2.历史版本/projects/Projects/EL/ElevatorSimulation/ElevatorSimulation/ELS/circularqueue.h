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

	//�������
	void pushElement(Type one);

	//���ʶ�ͷԪ��
	Type & frontElement();

	//ɾ����ͷԪ��
	void popElement();

	//��β������ͷ����Ԫ��
	Type * nextElement();

	//�ж϶����Ƿ�Ϊ��
	bool empty();

	//�ж϶����Ƿ��������ڵ���ʵ���п����ò�����
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