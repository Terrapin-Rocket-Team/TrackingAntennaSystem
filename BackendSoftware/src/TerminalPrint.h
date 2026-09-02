#ifndef TERMINALPRINT_H
#define TERMINALPRINT_H


#include "Data/DataLogging/DataLogger.h"
#include "Data/Backend/PrintLog.h"
#include "Data/Backend/ILogSink.h"

// we implement the print interface for use in writing
class TerminalPrint : public Print
{
    public: 
        size_t write(uint8_t b) override;
        size_t write(const uint8_t *buffer, size_t size) override;
        void flush() override;
};

#endif // TERMINALPRINT_H