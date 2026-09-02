//Created by Divyansh Srivastava on 5/23/2026

#ifndef TEENSY_SDCARD_BACKEND_H
#define TEENSY_SDCARD_BACKEND_H
 

//This is taken from ASTRA, implementation of IStorage for the Teensy 4.1 using the SdFat library. 
//This class implements the IStorage interface for use in our storage backend, and uses the 
//SdFs class to interact with the SD card. It also uses the TeensyFile class to wrap FsFile handles in IFile interfaces.


#include "../IStorage.h"
#include "TeensyFile.h"
#include <SdFat.h>
 
class SDCardStorage : public IStorage
{
private:
    SdFs _sd;
    bool _initialized;
 
public:
    SDCardStorage();
 
    bool begin() override;
    bool end() override;
    bool ok() const override;
 
    IFile *openRead(const char *filename) override;
    IFile *openWrite(const char *filename, bool append = true) override;
 
    bool exists(const char *filename) override;
    bool remove(const char *filename) override;
    bool mkdir(const char *path) override;
    bool rmdir(const char *path) override;
};
 
#endif
