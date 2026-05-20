#ifndef TEST_SUPPORT_ARDUINO_H
#define TEST_SUPPORT_ARDUINO_H

#include <cstddef>
#include <cstdint>
#include <string>

// GCC native builds define unix as a target macro; production Common.h has a
// TimeData field with that name.
#ifdef unix
#undef unix
#endif

inline void delay(unsigned long) {}

// stub for Arduino String so native tests compile with Serial.println(String(...))
struct String {
    explicit String(const char* s = "") : s_(s) {}
    explicit String(int, int = 10) {}
    String operator+(const String&) const { return String{}; }
    String operator+(const char*) const { return String{}; }
    friend String operator+(const char*, const String&) { return String{}; }
private:
    std::string s_;
};

inline constexpr int HEX = 16;

class Stream {
public:
    size_t print(const char*) { return 0; }
    size_t print(char) { return 0; }
};

class SerialStub {
public:
    template <typename... Args>
    void printf(const char*, Args...) {}

    template <typename T>
    void println(T) {}
};

inline SerialStub Serial;

#endif
