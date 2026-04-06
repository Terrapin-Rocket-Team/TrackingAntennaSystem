#include "UARTLog.h"

bool UARTLog::begin() override 
{
    // recall - these fields were initialized w/ our initializer list in the constructor
    s.begin(baud);
    return rdy = true;
}

bool UARTLog::end() override
{
    s.end();
    return true;
}

// getters/setter adjacent methods - onse liners are ok
// these methods will be called to verify logging details, like start/prefix recieving
bool UARTLog::wantsPrefix() const override { return prefix; }
bool UARTLog::ok() const override { return rdy; }
size_t UARTLog::write(uint8_t b) override { return s.write(b); }