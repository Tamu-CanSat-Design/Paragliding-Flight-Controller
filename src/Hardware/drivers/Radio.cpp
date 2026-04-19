#include "Hardware/drivers/Radio.h"



bool Radio::Initialize() {
    return true;
}

bool Radio::Send(const uint8_t* data, size_t length) {
    (void)data;
    (void)length;
    return true;
}

int Radio::Receive(uint8_t* buffer, size_t maxLength) {
    (void)buffer;
    (void)maxLength;
    return 0;
}
