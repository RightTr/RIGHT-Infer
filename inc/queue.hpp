#ifndef QUEUE_HPP
#define QUEUE_HPP

#include <iostream>

#define MAXCAPACITY 10



template <class T> class QUEUE
{
    private:
        T *data;
        int head, tail;
        int capacity;
        int size;


    public:
        
        QUEUE(int capacity);

        QUEUE();

        ~QUEUE();

        bool Push(const T &value);

        T Pop();

        bool IsEmpty() const;

        bool IsFull() const;

        int GetSize() const;
};

template class QUEUE<int>;

#endif