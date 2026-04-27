class UARTLog : public ILogSink
{
    public:
        UARTLog(SerialUART_t &s, int baud, bool prefix = false) 
        : s(s), baud(baud), prefix(prefix) {} // method call is an initializer list for these fields

        // utilizing our ILogSink methods - all log classes must do this
        bool begin() override;
        bool end() override;
        bool wantsPrefix() const override;
        bool ok() const override;
        size_t write(uint8_t b) override;

    protected:
        SerialUART_t &s;

        // class fields
        int baud;
        bool prefix;
        bool rdy = false; // default of false
};