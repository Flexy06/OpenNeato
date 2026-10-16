#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H
#include <Arduino.h>
#include <SPIFFS.h>
#include <vector>
#include "json_fields.h"
struct LogReader { virtual ~LogReader() = default; virtual size_t read(uint8_t *b, size_t n) = 0; };
struct PlainLogReader : public LogReader {
    File file;
    explicit PlainLogReader(File f) : file(std::move(f)) {}
    size_t read(uint8_t *b, size_t n) override { return file.read(b, n); }
};
class DataLogger {
public:
    void logGuided(const String& ev, const std::vector<Field>& extra = {});
};
#endif
