#include "TeensyFile.h"

TeensyFile::TeensyFile(FsFile &&handle) : _handle(std::move(handle)) {}

// ─── Writing ─────────────────────────────────────────────────────────────

size_t TeensyFile::write(uint8_t b)
{
    return _handle.write(b);
}

size_t TeensyFile::write(const uint8_t *buffer, size_t size)
{
    return _handle.write(buffer, size);
}

bool TeensyFile::flush()
{
    return _handle.sync(); // SdFat uses sync() instead of flush()
}

// ─── Reading ─────────────────────────────────────────────────────────────

int TeensyFile::read()
{
    return _handle.read();
}

int TeensyFile::readBytes(uint8_t *buffer, size_t length)
{
    return _handle.read(buffer, length);
}

int TeensyFile::available()
{
    return _handle.available();
}

// ─── File operations ─────────────────────────────────────────────────────

bool TeensyFile::seek(uint32_t pos)
{
    return _handle.seek(pos);
}

uint32_t TeensyFile::position()
{
    return _handle.position();
}

uint32_t TeensyFile::size()
{
    return _handle.size();
}

bool TeensyFile::close()
{
    _handle.close();
    return true;
}

// ─── Status ───────────────────────────────────────────────────────────────

bool TeensyFile::isOpen() const
{
    return _handle.isOpen();
}