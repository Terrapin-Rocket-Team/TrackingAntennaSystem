#ifndef ILOGSINK_H
#define ILOGSINK_H

#ifdef NATIVE
#include <cstddef>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstring>

class Print
{
public:
    virtual ~Print() = default;
    virtual size_t write(uint8_t b) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size)
    {
        size_t written = 0;
        while (written < size)
        {
            written += write(buffer[written]);
        }
        return written;
    }
    size_t write(const char *s)
    {
        return write(reinterpret_cast<const uint8_t *>(s), std::strlen(s));
    }
    size_t print(const char *s) { return write(s); }
    size_t printf(const char *fmt, ...)
    {
        char buf[128];
        va_list args;
        va_start(args, fmt);
        int n = std::vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        if (n <= 0)
        {
            return 0;
        }
        size_t len = static_cast<size_t>(n);
        if (len >= sizeof(buf))
        {
            len = sizeof(buf) - 1;
        }
        return write(reinterpret_cast<const uint8_t *>(buf), len);
    }
    virtual void flush() {}
};
#else
#include <Print.h>
#endif
#include <stdint.h>



class ILogSink : public Print //base level interface for writing to a log sink 

{
    public:
        // these methods override the teensy read/write functionality
        // we set some kind of error/success coding system

        // this class is abstract
        virtual bool begin() = 0;
        virtual bool end() = 0;
        virtual bool ok() const = 0;
        virtual bool wantsPrefix() const = 0;
        virtual size_t write(uint8_t) override = 0;

        using Print::write; //use the base print class write methods for strings and buffers
};

#endif // ILOGSINK_H
