// This File reader is a higher level layer of the IFile interface. IFIel reads and writes specific bytes while
//this can read and write whole lines along with delete whole files and stuff 

#include "FileReader.h"

bool FileReader::printFile(const char *filename)
{
    // error handling for null events or failed mounts
    if (!_backend || !_backend->ok())
    {
        Serial.println("ERROR: Storage backend not ready");
        return false;
    }

    if (!_backend->exists(filename))
    {
        Serial.print("ERROR: File not found: ");
        Serial.println(filename);
        return false;
    }

    IFile *file = _backend->openRead(filename);
    if (!file || !file->isOpen())
    {
        Serial.println("ERROR: Failed to open file");
        return false;
    }

    // handshake-y print
    Serial.println("|----------BOF----------|");

    uint8_t buffer[READ_BUFFER_SIZE];
    int bytesRead;
    while ((bytesRead = file->readBytes(buffer, sizeof(buffer))) > 0)
    {
        Serial.write(buffer, bytesRead); // write while content exists in the file
    }

    Serial.println("|----------EOF----------|");

    file->close();
    delete file;
    return true;
}

bool FileReader::readLines(const char *filename, void (*callback)(const char *line))
{
    // error handling and failed mounts
    if (!_backend || !_backend->ok())
        return false;
    if (!_backend->exists(filename))
        return false;

    IFile *file = _backend->openRead(filename);
    if (!file || !file->isOpen())
        return false;

    char lineBuffer[LINE_BUFFER_SIZE];
    int linePos = 0;

    while (file->available())
    {
        int c = file->read();
        if (c == -1)
            break;

        if (c == '\n' || linePos >= (LINE_BUFFER_SIZE - 1))
        {
            lineBuffer[linePos] = '\0';
            callback(lineBuffer);
            linePos = 0;
        }
        else if (c != '\r')
        {
            lineBuffer[linePos++] = (char)c;
        }
    }

    // Process final line if it doesn't end with newline
    if (linePos > 0)
    {
        lineBuffer[linePos] = '\0';
        callback(lineBuffer);
    }

    file->close();
    delete file;
    return true;
}

bool FileReader::exists(const char *filename)
{
    if (!_backend || !_backend->ok()) // if it doesnt mount then it doesnt exist
        return false;

    return _backend->exists(filename); // call object-specific method to handle boolean
}

bool FileReader::deleteFile(const char *filename)
{
    if (!_backend || !_backend->ok())
        return false; // couldnt remove, doesn't exist
    return _backend->remove(filename); // call object-specific method
}

