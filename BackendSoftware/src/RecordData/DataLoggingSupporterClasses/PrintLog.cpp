#include "PrintLog.h"


PrintLog::PrintLog(Print &p, bool prefix) : p(p), prefix(prefix), rdy(false) {
}

bool PrintLog::begin() 
{
    return rdy = true; 
}

bool PrintLog::end() 
{
    rdy = false;
    return true;
}



bool PrintLog::wantsPrefix() const {
    return prefix; }


bool PrintLog::ok() const { 
    return rdy; }


size_t PrintLog::write(uint8_t b)
 { return p.write(b); }