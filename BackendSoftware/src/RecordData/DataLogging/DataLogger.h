#include "../DataReporter/DataReporter.h"

class DataLogger
{
    public:
        // constructors - these are handled with an initializer list
        DataLogger() 
        : _sinks(nullptr), _countSinks(0), _countReporters(0), _reporterRegistry{}, _ok(false) {}

        DataLogger(ILogSink **sinks, uint8_t numSinks) // oveloaded constructor
        : _sinks(sinks), _countSinks(numSinks), _countReporters(0), _reporterRegistry{}, _ok(false) {}

        // logging logic
        bool init(); // initialization method
        bool appendLine();
        void printHeaderTo(ILogSink *sink);

        // getters/setters
        DataReporter *const *getReporters() const;
        uint8_t getNumReporters() const;

        // class-tied methods
        // these are all static as a result
        static void configure(ILogSink **sinks, uint8_t numSinks);
        static bool registerReporter(DataReporter *reporter);
        static bool unregisterReporter(DataReporter *reporter);
        static DataLogger &instance();
        static bool available();
        static void reset(); // Reset for testing - clears all reporters and sinks

    private:
        // these are our fields
        static constexpr uint8_t MAX_REPORTERS = 32;

        ILogSink **_sinks = nullptr;
        uint8_t _countSinks = 0;
        uint8_t _countReporters = 0;
        DataReporter *_reporterRegistry[MAX_REPORTERS];
        bool _ok = false;
        static DataLogger _global;
        bool vrecord(const char *lvl, const char *fmt, va_list ap);
};