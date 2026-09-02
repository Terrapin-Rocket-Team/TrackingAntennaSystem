
#ifndef FILE_LOG_SINK_H
#define FILE_LOG_SINK_H

#include "../../Storage/IStorage.h"
#include "../../Storage/IFile.h"

class FileLogSink : public ILogSink
{
    public:
       
        /**
         * @brief Construct FileLogSink with provided backend (advanced usage)
         * @param filename Path to log file
         * @param backend Pre-created storage backend (not owned by FileLogSink)
         * @param prefix Whether to add prefixes to log messages
         */
        FileLogSink(const char *filename, IStorage *backend, bool prefix = false);

        ~FileLogSink();

        // logging override methods
        bool begin() override;
        bool end() override;
        bool ok() const override;
        bool wantsPrefix() const override;

        // we have two overloaded write methods w/ different functionality
        size_t write(uint8_t b) override;
        size_t write(const uint8_t *buffer, size_t size) override;

        // flush is an override of a teensy methods
        void flush() override;

    private:
        // these fields are storage types. will be implemented in the storage/retrieval class flow
        IStorage *_backend;
        IFile *_file;
        const char *_filename;
        bool _ownsBackend;
        bool _prefix;
};

#endif // FILE_LOG_SINK_H