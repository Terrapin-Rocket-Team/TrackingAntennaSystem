#include "RecordData/DataLoggingSupporterClasses/ILogSink.h"

// we implement the print interface for use in writing
class TerminalPrint : public Print
{
    public: 
        size_t write(uint8_t b) override;
        size_t write(const uint8_t *buffer, size_t size) override;
        void flush() override;
};