#ifndef FILE_READER_H
#define FILE_READER_H

#include "../Storage/IStorage.h"
#include "../Storage/IFile.h"
#include <Arduino.h>

class FileReader
{
    public:
        /**
        * @brief Construct FileReader
        * @param backend Storage backend to read from (not owned by FileReader)
        */
        FileReader(IStorage *backend) : _backend(backend) {} // using an initializer list to create an IStorage object

        /**
        * @brief Print entire file to Serial
        * @param filename Path to file
        * @return true if file was read successfully
        */
        bool printFile(const char *filename);

        /**
        * @brief Read file line by line with callback
        * @param filename Path to file
        * @param callback Function called for each line
        * @return true if file was read successfully
        */
        bool readLines(const char *filename, void (*callback)(const char *line));

        /**
        * @brief Check if file exists
        * @param filename Path to file
        * @return true if file exists
        */
        bool exists(const char *filename);

        /**
        * @brief Delete a file
        * @param filename Path to file
        * @return true if file was deleted successfully
        */
        bool deleteFile(const char *filename);


    private:
        IStorage *_backend;
        static constexpr size_t READ_BUFFER_SIZE = 1024;
        static constexpr size_t LINE_BUFFER_SIZE = 1024;
};

#endif // FILE_READER_H