#include "queue.hpp"



template <class T> 
QUEUE<T>::QUEUE(int capacity):capacity(capacity), size(0), head(0), tail(0)
{
    data = new T[capacity];
}

template <class T> 
QUEUE<T>::QUEUE():capacity(MAXCAPACITY), size(0), head(0), tail(0)
{
    data = new T[capacity];
}

template <class T> 
QUEUE<T>::~QUEUE()
{
    delete data;
}

template <class T> 
bool QUEUE<T>::IsEmpty() const
{
    if(head == tail) return true;
    else return false;
}

template <class T> 
bool QUEUE<T>::IsFull() const
{
    if(tail - head == capacity) return true;
    else return false;
}

template <class T> 
bool QUEUE<T>::Push(const T &value)
{
    if(IsFull()) 
    {
        throw("Queue is Full!");
        return false;
    }

    data[tail] = value;
    tail = (tail + 1) % capacity;
    size++;
    return true;
}

template <class T> 
T QUEUE<T>::Pop()
{
    if(IsEmpty()) 
    {
        throw("Queue is Empty!");
    }

    T value = data[head];
    head = (head + 1) % capacity;
    size--;
    return value;
}

template <class T> 
int QUEUE<T>::GetSize() const
{
    return size;
}
