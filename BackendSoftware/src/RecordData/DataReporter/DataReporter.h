//Created by Divyansh Srivastava on 3/23/2026
//this file will be used for the low level data reporter which holds 
//a linked list of data points and can be used to report data to the user/MCU

#ifndef DATAREPORTER_H
#define DATAREPORTER_H

#include "DataPoint.h"

// Templated helper to allocate & populate a node
    template <typename T>
    inline DataPoint *make_dp(const char *fmt, const T *p, const char *label)

     {
        auto *d = new DataPoint{};
        d->fmt = fmt;
        d->label = label;
        d->data = p;
        d->emit = [](Print *s, const DataPoint *self)
        {
            const T *val = static_cast<const T *>(self->data);
            // Portable: format into a buffer, then write
            char buf[64];
            int n = snprintf(buf, sizeof(buf), self->fmt, *val);
            if (n > 0)
                s->write(reinterpret_cast<const uint8_t *>(buf), (size_t)n);
        };
        return d;
    }

    class DataReporter
    {
    public:
        static int numReporters;

        DataReporter(const char *name = nullptr);
        virtual ~DataReporter();

        virtual const char *getName() const;
        virtual void setName(const char *n);

        int getNumColumns();
        DataPoint *getDataPoints();
        DataPoint *getLastPoint();

        // Initializes the reporter and sets up any necessary parameters (calls init() internally)
        // Returns 0 on success, library-specific error code on failure
        virtual int begin() = 0;

        // Updates the reporter's fields by querying for new data (calls read() internally)
        // @param currentTime - Current time in seconds (for SITL/HITL), -1 uses millis()
        // Returns 0 on success, library-specific error code on failure
        virtual int update(double currentTime = -1) = 0;

        virtual bool isInitialized() const; // Returns whether the reporter has been initialized or not

        virtual explicit operator bool() const { return initialized }; // Returns whether the reporter has been initialized or not


        //Data reporters are automatically updated by the managing system. If another reporter owns this one, 
        //you can set this to false to prevent multiple updates per cycle.
        virtual void setAutoUpdate(bool update);
        virtual bool getAutoUpdate() const;

    protected:
        uint8_t numColumns = 0;
        DataPoint *first = nullptr, *last = nullptr;

        template <typename T>
        void insertColumn(int place, const char *fmt, T *variable, const char *label);

        template <typename T>
        void addColumn(const char *fmt, T *variable, const char *label);

        void removeColumn(const char *label);

        void clearColumns();

        bool initialized = false;
        bool autoUpdate = true;

    
    private: 
        char *name = nullptr;

        
    }







#endif