//Created by Divyansh Srivastava on 5/23/2026
#ifndef TEENSYFILE_H
#define TEENSYFILE_H

//This file is copied from ASTRA, implementation of IFile for the Teensy 4.1 using the SdFat library.
// This class wraps an FsFile handle and implements the IFile interface for use in our storage backend.

#include "../IFile.h"
#include <SdFat.h>
#include <utility>


class TeensyFile : public IFile {
public:
    TeensyFile(FsFile&& handle);
    size_t write(uint8_t b) override;
    size_t write(const uint8_t* buffer, size_t size) override;
    bool flush() override;
    int read() override;
    int readBytes(uint8_t* buffer, size_t length) override;
    int available() override;
    bool seek(uint32_t pos) override;
    uint32_t position() override;
    uint32_t size() override;
    bool close() override;
    bool isOpen() const override;
private:
    FsFile _handle;
};

#endif // TEENSYFILE_H