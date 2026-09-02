#include <cstdio>
#include <thread>
#include <chrono>


#include "TerminalPrint.h"

// all of these implementations just make calls to printing systems in the standard cpp library
size_t TerminalPrint::write(uint8_t b)
{
    return std::putchar(b) == EOF ? 0 : 1;
}

size_t TerminalPrint::write(const uint8_t *buffer, size_t size)
{
    return std::fwrite(buffer, 1, size, stdout);
}

void TerminalPrint::flush()
{
    std::fflush(stdout);
}




