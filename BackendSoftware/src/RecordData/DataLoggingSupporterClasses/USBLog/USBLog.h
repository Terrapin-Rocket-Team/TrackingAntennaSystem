class USBLog : public ILogSink
{
    public:
        // initializer list for this class
        USBLog(SerialUSB_t &s, int baud, bool prefix = false)
        : s(s), baud(baud), prefix(prefix) {}

        // log classes all override these logsink methods
        bool begin() override;
        bool end() override;
        bool wantsPrefix() const override;
        bool ok() const override;
        size_t write(uint8_t b) override;

    private:
        // common class fields
        int baud;
        bool prefix;
        bool rdy = false; // default of false

        // this field is for usb only. gives error types
        SerialUSB_t &s;
};