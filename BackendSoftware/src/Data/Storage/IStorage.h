#ifndef ISTORAGE_H
#define ISTORAGE_H

#include "IFile.h"
//this is the hardware storage specific interface. This class is responsible for 
// defining basic operations a file system should do. 

    /**
     * @brief Interface for storage backend operations
     *
     * Represents a filesystem/storage medium (e.g., eMMC chip, SD card, flash).
     * Platform-specific implementations handle the actual hardware/library calls.
     */
    class IStorage
    {
    public:
        virtual ~IStorage() {}

        // Lifecycle
        virtual bool begin() = 0;
        virtual bool end() = 0;
        virtual bool ok() const = 0;

        // File operations - return IFile handles
        virtual IFile *openRead(const char *filename) = 0;
        virtual IFile *openWrite(const char *filename, bool append = true) = 0;

        // Filesystem operations
        virtual bool exists(const char *filename) = 0;
        virtual bool remove(const char *filename) = 0;
        virtual bool mkdir(const char *path) = 0;
        virtual bool rmdir(const char *path) = 0;
    };



#endif