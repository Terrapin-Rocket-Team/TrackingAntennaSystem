#ifndef DATA_REPORTER_INL
#define DATA_REPORTER_INL

#include "DataPoint.h"
#include "DataReporter.h"

template <typename T>
void DataReporter::insertColumn(int place, const char *fmt, T *variable, const char *label) 
{
    DataPoint *packedInfo = make_dp<T>(fmt, variable, label); // content of our node

    if (first == nullptr)
    {
        // linked list does not exist
        first = packedInfo;
        last = packedInfo;
    }
    // linked list exists cases
    else if (place == 0)
    {
        // insert at head
        packedInfo->next = first;
        first = packedInfo;
    }
    else if (place < 0 || place >= numColumns)
    {
        // insert at tail
        packedInfo->next
        last->next = packedInfo;
        last = packedInfo;
    }
    else
    {
        // insert at index
        DataPoint *current = first;
        int index = 0;

        while (current->next != nullptr && index < place-1)
        {
            // loop to just before insertion spot
            current = current->next;
            ++index;
        }
        
        packedInfo->next = current->next;
        current->next = packedInfo;
        if (packedInfo->next == nullptr) {last = packedInfo}; // set tail as needed
    }
    
    ++numColumns;
}