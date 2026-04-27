#include "CircBuffer.h"
#include <stdint.h>

template <typename T>
CircBuffer<T>::CircBuffer(int size)
{
    // constructor initializations
    this->size = size;
    buffer = new T[size];
    clear();
}

// destructor
template <typename T>
CircBuffer<T>::~CircBuffer()
{
    delete[] buffer;
}

template <typename T>
void CircBuffer<T>::push(T item) // this is a stack implementation
{
    if (isFull())
    {
        tail = (tail + 1) % size;
        count--; // prevent addition to the stack
    }

    // add a new head to our linked list
    buffer[head] = item;
    head = (head + 1) % size;
    count++;
}

template <typename T>
T CircBuffer<T>::pop()
{
    if (isEmpty())
    {
        return T(); // nothing to remove
    }
    
    // reroute head in our linked list, remove from tail
    T item = buffer[tail];
    tail = (tail + 1) % size;
    count--;
    return item;
}

// look at end without removing it
template <typename T>
T CircBuffer<T>::peek()
{
    if (isEmpty())
    {
        return T();
    }
    return buffer[tail];
}

// getters and setters
template <typename T> int CircBuffer<T>::getCount() { return count; }
template <typename T> bool CircBuffer<T>::isFull() { return count == size; }
template <typename T> bool CircBuffer<T>::isEmpty() { return count == 0; }
template <typename T> void CircBuffer<T>::clear() { head = 0; tail = 0; count = 0; }
template <typename T> int CircBuffer<T>::getSize() { return size; }

template<typename T> T& CircBuffer<T>::operator[](int index) { return buffer[(tail + index) % size]; }
template<typename T> T CircBuffer<T>::operator[](int index) const { 
    return buffer[(tail + index) % size]; }

template class CircBuffer<uint8_t>;
