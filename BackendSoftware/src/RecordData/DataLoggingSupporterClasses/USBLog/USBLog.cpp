#include "USBLog.h"

bool USBLog::begin() override
{
    if (!ok()) // calls prefix checker
    {
        s.begin(baud); // enables auto logging
    }

    return rdy = true;
}

bool USBLog::end() override
{
    s.end(); // end autologging
    rdy = false;
    return true;
}

// getters and setters
bool USBLog::wantsPrefix() const override { return prefix; }
bool USBLog::ok() const override { return rdy; }
size_t USBLog::write(uint8_t b) override { return s.write(b); }