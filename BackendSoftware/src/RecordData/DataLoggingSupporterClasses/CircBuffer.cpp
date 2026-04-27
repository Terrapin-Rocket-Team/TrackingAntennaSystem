#include "CircBufferLog.h"

// these circ buffer methods are dependent on utility classes
bool CircBufferLog::begin()
{
    if (rdy == true) {return true;} // if logging is setup, were good to go

    buf = new CircBuffer<uint8_t>(size); // if a successful utilities/buffer is created, we return so

    if (buf != nullptr) {return rdy = true;}

    return false; // otherwise, return false
}

bool CircBufferLog::end()
{
    // destroys the relevant buffer classes
    // returns to affirm it is gones
    rdy = false;
    delete buf;
    buf = nullptr;
    return true;
}

bool CircBufferLog::wantsPrefix() const { return prefix; }

bool CircBufferLog::ok() const { return rdy; }

size_t CircBufferLog::write(uint8_t b)
{
    if (rdy)
    {
        buf->push(b);
        return 1; // error code
    } 

    return 0;
}

bool CircBufferLog::transfer(ILogSink &other)
{
    // operating our buffer based on our boolean for logging verification
    if (!rdy || buf->isEmpty())
    {
        return false;
    }

    while (!buf->isEmpty())
    {
        other.write(buf->pop());
    }

    return true;
}
