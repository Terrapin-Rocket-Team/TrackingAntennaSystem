class PrintLog : public ILogSink
{
    public:
        PrintLog(Print &p, bool prefix = false) //why is there a print log? I thought we were just going to log to the sd card, is this for testing purposes?
        //is this for logging print statements?
        : p(p), prefix(prefix) {} // fewer fields need to be initialized for print logging

        // logging methods to override
        bool begin() override;
        bool end() override;
        bool wantsPrefix() const override;
        bool ok() const override;
        size_t write(uint8_t b) override;

    private: // needs the same fields as the other logging classes
        Print &p; // requires teensy library to create this object
        bool prefix;
        bool rdy;
};