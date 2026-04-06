class CircBufferLog : public ILogSink
{
    public:
        CircBufferLog(int size, bool prefix = false)
        : size(size), prefix(prefix) {} // this class also doesn't need a super long IL

        // basic logging overrides
        bool begin() override;
        bool end() override;
        bool wantsPrefix() const override;
        bool ok() const override;
        size_t write(uint8_t b) override;

        // unique to circ buffer logging
        bool transfer(ILogSink &other);

    private:

        // similar fields + error system
        CircBuffer<uint8_t> *buf = nullptr; // utility class that needs implementation
        int size;
        bool prefix;
        bool rdy = false;
};