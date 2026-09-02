#ifndef EVENTLOGGER_H
#define EVENTLOGGER_H

#include <new>
#include "../Backend/ILogSink.h"
#include <stdarg.h>
#include <stdint.h>

#ifndef NATIVE
#include <Arduino.h>
#endif

class EventLogger //believe this is a Singleton class, we only want one instance of this, and it will be used globally.
{
    public:
        EventLogger(ILogSink **sinks, uint8_t count, int maxMsgLen = 500); //describe what this event logger is for,
        //im assuming its to log bad events. In what cases in ou rsystem would we have bad events though?
        //i know its astra its mainly for sensors that don't end up working, but we only have one sensor.

        bool init();
        bool info(const char *fmt, ...);
        bool warn(const char *fmt, ...);
        bool err(const char *fmt, ...);
        bool dbg(const char *fmt, ...);

        static void configure(ILogSink **sinks, uint8_t count);
        static EventLogger &instance();
        static bool available();

        EventLogger(const EventLogger&) = delete; //copy contructor 
        EventLogger& operator=(const EventLogger&) = delete; //assignment operator
        
        bool vrecord(const char *lvl, const char *fmt, va_list ap); // helper method to record a log message with a va_list
        int min(int a, int b); // helper method to get the minimum of two integers

    private:
        // these are our fields
        ILogSink **_sinks = nullptr; //sinks for data (SD card, File, Serial, etc.)
        uint8_t _count = 0;
        bool _ok = false;
        int _maxMsgLen = 0;
        static EventLogger _global; //shared global instance of the event logger, this is what will be used by the static methods
};

#define LOGI(...) EventLogger::instance().info(__VA_ARGS__)
#define LOGW(...) EventLogger::instance().warn(__VA_ARGS__)
#define LOGE(...) EventLogger::instance().err(__VA_ARGS__)
#define LOGD(...) EventLogger::instance().dbg(__VA_ARGS__)

#endif // EVENTLOGGER_H
