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