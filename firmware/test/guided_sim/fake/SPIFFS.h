#ifndef FAKE_SPIFFS_H
#define FAKE_SPIFFS_H
#include <Arduino.h>
#include <cstdio>
#define FILE_READ "r"
#define FILE_WRITE "w"
#define FILE_APPEND "a"
// Files live under a temp directory on the host
std::string fakeFsPath(const char *p);
class File {
public:
    File() = default;
    explicit File(FILE *f) : f_(f) {}
    File(File&& o) noexcept : f_(o.f_) { o.f_ = nullptr; }
    File& operator=(File&& o) noexcept { if (f_) fclose(f_); f_ = o.f_; o.f_ = nullptr; return *this; }
    File(const File&) = delete;
    ~File() { if (f_) fclose(f_); }
    explicit operator bool() const { return f_ != nullptr; }
    void close() { if (f_) fclose(f_); f_ = nullptr; }
    size_t write(const uint8_t *d, size_t n) { return fwrite(d, 1, n, f_); }
    size_t print(const String& t) { return fwrite(t.c_str(), 1, t.length(), f_); }
    int read(uint8_t *d, size_t n) { return static_cast<int>(fread(d, 1, n, f_)); }
    size_t size() { long p = ftell(f_); fseek(f_, 0, SEEK_END); long s = ftell(f_); fseek(f_, p, SEEK_SET); return s; }
    int available() { int c = fgetc(f_); if (c == EOF) return 0; ungetc(c, f_); return 1; }
    String readStringUntil(char t) { std::string s; int c; while ((c = fgetc(f_)) != EOF && c != t) s += static_cast<char>(c); return String(s); }
private:
    FILE *f_ = nullptr;
};
struct FakeFS {
    File open(const char *p, const char *mode) { return File(fopen(fakeFsPath(p).c_str(), mode)); }
    File open(const String& p, const char *mode) { return open(p.c_str(), mode); }
    bool exists(const char *p) { FILE *f = fopen(fakeFsPath(p).c_str(), "r"); if (f) fclose(f); return f != nullptr; }
    bool remove(const char *p) { return ::remove(fakeFsPath(p).c_str()) == 0; }
    bool rename(const char *a, const char *b) { return ::rename(fakeFsPath(a).c_str(), fakeFsPath(b).c_str()) == 0; }
};
extern FakeFS SPIFFS;
#endif
