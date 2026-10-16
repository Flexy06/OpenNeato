// Minimal Arduino shim for host-side simulation of guided clean logic.
#ifndef FAKE_ARDUINO_H
#define FAKE_ARDUINO_H
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cctype>
#include <string>
inline bool isDigit(char c) { return std::isdigit(static_cast<unsigned char>(c)) != 0; }
inline bool isHexadecimalDigit(char c) { return std::isxdigit(static_cast<unsigned char>(c)) != 0; }

extern unsigned long g_simMillis;
inline unsigned long millis() { return g_simMillis; }

class String {
public:
    String() = default;
    String(const char *s) : s_(s ? s : "") {}
    String(const std::string& s) : s_(s) {}
    String(char c) : s_(1, c) {}
    String(int v) : s_(std::to_string(v)) {}
    String(unsigned v) : s_(std::to_string(v)) {}
    String(long v) : s_(std::to_string(v)) {}
    String(unsigned long v) : s_(std::to_string(v)) {}
    String(float v, int dec = 2) { fmt(v, dec); }
    String(double v, int dec = 2) { fmt(v, dec); }
    unsigned length() const { return static_cast<unsigned>(s_.size()); }
    bool isEmpty() const { return s_.empty(); }
    const char *c_str() const { return s_.c_str(); }
    char charAt(unsigned i) const { return i < s_.size() ? s_[i] : 0; }
    char operator[](unsigned i) const { return charAt(i); }
    int indexOf(char c, unsigned from = 0) const { auto p = s_.find(c, from); return p == std::string::npos ? -1 : static_cast<int>(p); }
    int indexOf(const String& t, unsigned from = 0) const { auto p = s_.find(t.s_, from); return p == std::string::npos ? -1 : static_cast<int>(p); }
    int lastIndexOf(char c) const { auto p = s_.rfind(c); return p == std::string::npos ? -1 : static_cast<int>(p); }
    String substring(unsigned a) const { return a >= s_.size() ? String() : String(s_.substr(a)); }
    String substring(unsigned a, unsigned b) const { if (a >= s_.size() || b <= a) return String(); return String(s_.substr(a, b - a)); }
    long toInt() const { return std::strtol(s_.c_str(), nullptr, 10); }
    float toFloat() const { return std::strtof(s_.c_str(), nullptr); }
    void trim() { auto b = s_.find_first_not_of(" \t\r\n"); auto e = s_.find_last_not_of(" \t\r\n"); s_ = b == std::string::npos ? "" : s_.substr(b, e - b + 1); }
    bool startsWith(const String& p) const { return s_.rfind(p.s_, 0) == 0; }
    bool endsWith(const String& p) const { return s_.size() >= p.s_.size() && s_.compare(s_.size() - p.s_.size(), p.s_.size(), p.s_) == 0; }
    void reserve(unsigned n) { s_.reserve(n); }
    void remove(unsigned i, unsigned n = 1) { if (i < s_.size()) s_.erase(i, n); }
    void replace(const String& a, const String& b) {
        if (a.s_.empty()) return;
        size_t p = 0;
        while ((p = s_.find(a.s_, p)) != std::string::npos) { s_.replace(p, a.s_.size(), b.s_); p += b.s_.size(); }
    }
    bool equalsIgnoreCase(const String& o) const {
        if (o.s_.size() != s_.size()) return false;
        for (size_t i = 0; i < s_.size(); i++) if (std::tolower(s_[i]) != std::tolower(o.s_[i])) return false;
        return true;
    }
    void toLowerCase() { for (auto& c: s_) c = static_cast<char>(std::tolower(c)); }
    void toUpperCase() { for (auto& c: s_) c = static_cast<char>(std::toupper(c)); }
    String& operator+=(const String& o) { s_ += o.s_; return *this; }
    String& operator+=(const char *o) { s_ += o; return *this; }
    String& operator+=(char c) { s_ += c; return *this; }
    bool operator==(const String& o) const { return s_ == o.s_; }
    bool operator==(const char *o) const { return s_ == o; }
    bool operator!=(const String& o) const { return s_ != o.s_; }
    bool operator!=(const char *o) const { return s_ != o; }
    bool operator<(const String& o) const { return s_ < o.s_; }
    friend String operator+(const String& a, const String& b) { return String(a.s_ + b.s_); }
    friend String operator+(const String& a, const char *b) { return String(a.s_ + b); }
    friend String operator+(const char *a, const String& b) { return String(std::string(a) + b.s_); }
    const std::string& str() const { return s_; }

private:
    std::string s_;
    void fmt(double v, int dec) { char b[64]; std::snprintf(b, sizeof b, "%.*f", dec, v); s_ = b; }
};

struct FakeSerial {
    void printf(const char *fmt, ...) {
        if (!enabled) return;
        va_list ap; va_start(ap, fmt); std::vprintf(fmt, ap); va_end(ap);
    }
    bool enabled = false;
};
extern FakeSerial Serial;

template<typename T> T constrain(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
#endif
