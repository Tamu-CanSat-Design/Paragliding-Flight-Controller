#ifndef TEST_SUPPORT_ARDUINO_H
#define TEST_SUPPORT_ARDUINO_H

#include <cstddef>
#include <cstdint>

inline void delay(unsigned long) {}

class SerialStub {
public:
    template <typename... Args>
    void printf(const char*, Args...) {}

    void println(const char*) {}
};

inline SerialStub Serial;

#endif
