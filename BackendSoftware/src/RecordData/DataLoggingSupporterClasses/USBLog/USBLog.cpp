#include "USBLog.h"

bool USBLog::begin()
{
    if (!ok()) // calls prefix checker
    {
        s.begin(baud); // enables auto logging
    }

    return rdy = true;
}

bool USBLog::end()
{
    s.end(); // end autologging
    rdy = false;
    return true;
}

// getters and setters
bool USBLog::wantsPrefix() const { return prefix; }
bool USBLog::ok() const { return rdy; }
size_t USBLog::write(uint8_t b) { return s.write(b); }
