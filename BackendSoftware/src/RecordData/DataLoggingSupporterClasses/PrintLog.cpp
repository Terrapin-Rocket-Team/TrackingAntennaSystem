#include "PrintLog.h"

//these methods rely on the rdy boolean to set logging status
bool PrintLog::begin() override
{
    return rdy = true; 
}

bool PrintLog::begin() override
{
    rdy = false;
    return true;
}

bool PrintLog::wantsPrefix() const override { return prefix; }
bool PrintLog::ok() const override { return rdy; }
size_t PrintLog::write(uint8_t b) override { return p.write(b); }