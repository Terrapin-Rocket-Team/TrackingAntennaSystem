#include "DataReporter.h"

// i havent added the stdout override for printing
// i didnt think we needed that since were not really even using std out, that's fine we really aren't
bool DataLogger::init()
{
    bool any = false;
    for (uint8_t i = 0; i < _countSinks; i++) // go through all our sinks and start them
        if (_sinks[i]->begin())
        {
            any = true;
            printHeaderTo(_sinks[i]); // start the printing process after starting
        }
    return _ok = any;
}

bool DataLogger::appendLine()
{
    if (!_ok) // if call is not recieved, then appending didnt work
    {
        return false;
    }

    #ifdef NATIVE //check this
    // if this build is native, we must handle it seperately
    #ifndef NATIVE_NO_STDOUT_DATA
        if (_countReporters > 0 && _reporterRegistry[0]->getNumColumns() > 0)
        {
            stdoutPrint.print("TELEM/");
        }
        for (int j = 0; j < _countReporters; ++j)
        {
            for (DataPoint *d = _reporterRegistry[j]->getDataPoints(); d != nullptr; d = d->next)
            {
                d->emit(&stdoutPrint, d);
                if (d != _reporterRegistry[j]->getLastPoint())
                    stdoutPrint.write(',');
            }
            if (j != _countReporters - 1)
                stdoutPrint.write(',');
            else
                stdoutPrint.write('\n');
        }
        fflush(stdout);
        #endif
    #endif

    // otherwise, our appending logic loops and continues
    for (int i = 0; i < _countSinks; i++)
    {
        if (!_sinks[i]->ok())
            continue;
        if (_sinks[i]->wantsPrefix() && _countReporters > 0 && _reporterRegistry[0]->getNumColumns() > 0)
            _sinks[i]->print("TELEM/");
        for (int j = 0; j < _countReporters; ++j)
        {
            for (DataPoint *d = _reporterRegistry[j]->getDataPoints(); d != nullptr; d = d->next)
            {
                d->emit(_sinks[i], d);
                if (d != _reporterRegistry[j]->getLastPoint())
                    _sinks[i]->write(',');
            }
            if (j != _countReporters - 1)
                _sinks[i]->write(',');
            else
                _sinks[i]->write('\n');
        }
        _sinks[i]->flush();
    }
    return true;
}

void DataLogger::printHeaderTo(ILogSink *sink)
{
    if (!sink || !sink->ok())
        return;

    if (sink->wantsPrefix() && _countReporters > 0 && _reporterRegistry[0]->getNumColumns() > 0)
        sink->print("TELEM/");

    for (int j = 0; j < _countReporters; j++) // go through every count and setup
    {
        DataPoint *d = _reporterRegistry[j]->getDataPoints();
        while (d != nullptr)
        {
            sink->printf("%s - %s", _reporterRegistry[j]->getName(), d->label);
            if (d != _reporterRegistry[j]->getLastPoint())
                sink->write(',');
            d = d->next;
        }
        if (j != _countReporters - 1)
            sink->write(',');
        else
            sink->write('\n');
    }
    sink->flush(); // method tied to teensy
}

// a lot of this revolves around the global nature of the class
// is this even necessary?
void DataLogger::configure(ILogSink **sinks, uint8_t numSinks)
{
    _global._sinks = sinks;
    _global._countSinks = numSinks;
    _global.init();
}

bool DataLogger::registerReporter(DataReporter *reporter)
{
    if (!reporter)
        return false;

    if (_global._countReporters >= MAX_REPORTERS)
        return false;

    for (uint8_t i = 0; i < _global._countReporters; i++)
    {
        if (_global._reporterRegistry[i] == reporter)
            return false;
    }

    _global._reporterRegistry[_global._countReporters++] = reporter;
    return true;
}

bool DataLogger::unregisterReporter(DataReporter *reporter) // must undo the last method calls
{
    if (!reporter)
        return false;

    for (uint8_t i = 0; i < _global._countReporters; i++)
    {
        if (_global._reporterRegistry[i] == reporter)
        {
            for (uint8_t j = i; j < _global._countReporters - 1; j++)
            {
                _global._reporterRegistry[j] = _global._reporterRegistry[j + 1];
            }
            _global._countReporters--;
            _global._reporterRegistry[_global._countReporters] = nullptr;
            return true;
        }
    }
    return false;
}

DataLogger &DataLogger::instance()
{
    return _global;
}

bool DataLogger::available()
{
    return _global._ok;
}

void DataLogger::reset()
{
    _global._sinks = nullptr;
    _global._countSinks = 0;
    _global._countReporters = 0;
    for (uint8_t i = 0; i < MAX_REPORTERS; i++)
    {
        _global._reporterRegistry[i] = nullptr;
    }
    _global._ok = false;
}

// one liner methods
DataReporter *const *DataLogger::getReporters() const { return _reporterRegistry; }
uint8_t DataLogger::getNumReporters() const { return _countReporters; }




