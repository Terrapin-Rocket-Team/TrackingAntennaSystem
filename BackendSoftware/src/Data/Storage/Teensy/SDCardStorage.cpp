#include "SDCardStorage.h"
 
SDCardStorage::SDCardStorage() : _initialized(false) {}
 
bool SDCardStorage::begin()
{
    delay(250);
 
    bool initSuccess = false;
    for (int attempt = 1; attempt <= 3; attempt++)
    {
        if (_sd.begin(SdioConfig(FIFO_SDIO)))
        {
            initSuccess = true;
            break;
        }
        if (attempt < 3) //here to make sure we don't delay after the last attempt
            delay(500);
    }
 
    if (!initSuccess)
    {
        Serial.println("[SDCardBackend] SD card initialization FAILED");
        _initialized = false;
        return false;
    }
 
    _initialized = true;
    return true;
}
 
bool SDCardStorage::end()
{
    _initialized = false;
    return true;
}
 
bool SDCardStorage::ok() const
{
    return _initialized;
}
 
IFile *SDCardStorage::openRead(const char *filename)
{
    if (!_initialized)
        return nullptr;
 
    FsFile file = _sd.open(filename, O_READ);
    if (!file)
        return nullptr;
 
    return new TeensyFile(std::move(file));
}
 
IFile *SDCardStorage::openWrite(const char *filename, bool append)
{
    if (!_initialized)
        return nullptr;
 
    oflag_t mode = append ? (O_WRITE | O_CREAT | O_APPEND) : (O_WRITE | O_CREAT | O_TRUNC);
 
    FsFile file = _sd.open(filename, mode);
    if (!file)
    {
        Serial.print("[SDCardStorage] Failed to open file: ");
        Serial.println(filename);
        return nullptr;
    }
 
    return new TeensyFile(std::move(file));
}
 
bool SDCardStorage::exists(const char *filename)
{
    if (!_initialized)
        return false;
    return _sd.exists(filename);
}
 
bool SDCardStorage::remove(const char *filename)
{
    if (!_initialized)
        return false;
    return _sd.remove(filename);
}
 
bool SDCardStorage::mkdir(const char *path)
{
    if (!_initialized)
        return false;
    return _sd.mkdir(path);
}
 
bool SDCardStorage::rmdir(const char *path)
{
    if (!_initialized)
        return false;
    return _sd.rmdir(path);
}
 