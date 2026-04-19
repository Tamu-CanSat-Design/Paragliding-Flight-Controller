#ifndef TEST_SUPPORT_WIRE_H
#define TEST_SUPPORT_WIRE_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <utility>
#include <vector>

class TwoWire {
public:
    struct WriteEntry {
        uint8_t address;
        std::vector<uint8_t> bytes;
    };

    void begin() {}
    void setClock(int) {}

    void beginTransmission(uint8_t address) {
        currentAddress_ = address;
        txBuffer_.clear();
    }

    size_t write(uint8_t value) {
        txBuffer_.push_back(value);
        return 1;
    }

    uint8_t endTransmission(bool = true) {
        if (txBuffer_.empty()) {
            return 0;
        }

        writeLog_.push_back({currentAddress_, txBuffer_});

        if (txBuffer_.size() == 1) {
            registerPointer_[currentAddress_] = txBuffer_[0];
            return 0;
        }

        const uint8_t reg = txBuffer_[0];
        for (size_t i = 1; i < txBuffer_.size(); ++i) {
            registers_[MakeKey(currentAddress_, static_cast<uint8_t>(reg + i - 1))] = txBuffer_[i];
        }
        registerPointer_[currentAddress_] = reg;
        return 0;
    }

    size_t requestFrom(uint8_t address, uint8_t quantity) {
        rxBuffer_.clear();
        rxIndex_ = 0;

        const uint8_t reg = registerPointer_[address];
        if (reg == 0x78 && quantity == 7 && !fifoWords_[address].empty()) {
            const std::array<uint8_t, 7> word = fifoWords_[address].front();
            fifoWords_[address].pop_front();
            rxBuffer_.assign(word.begin(), word.end());
            return rxBuffer_.size();
        }

        if (reg == 0x3A && quantity == 2) {
            const uint16_t words = static_cast<uint16_t>(fifoWords_[address].size());
            rxBuffer_.push_back(static_cast<uint8_t>(words & 0xFF));
            rxBuffer_.push_back(static_cast<uint8_t>(((words >> 8) & 0x03) | fifoStatus2Bits_[address]));
            return rxBuffer_.size();
        }

        for (uint8_t offset = 0; offset < quantity; ++offset) {
            rxBuffer_.push_back(registers_[MakeKey(address, static_cast<uint8_t>(reg + offset))]);
        }
        return rxBuffer_.size();
    }

    int available() {
        return static_cast<int>(rxBuffer_.size() - rxIndex_);
    }

    int read() {
        if (rxIndex_ >= rxBuffer_.size()) {
            return -1;
        }
        return rxBuffer_[rxIndex_++];
    }

    void clear() {
        registers_.clear();
        registerPointer_.clear();
        fifoWords_.clear();
        fifoStatus2Bits_.clear();
        writeLog_.clear();
        txBuffer_.clear();
        rxBuffer_.clear();
        rxIndex_ = 0;
        currentAddress_ = 0;
    }

    void setRegister(uint8_t address, uint8_t reg, uint8_t value) {
        registers_[MakeKey(address, reg)] = value;
    }

    void pushFifoWord(uint8_t address, const std::array<uint8_t, 7>& word) {
        fifoWords_[address].push_back(word);
    }

    void setFifoStatus2Bits(uint8_t address, uint8_t bits) {
        fifoStatus2Bits_[address] = bits;
    }

    size_t fifoWordCount(uint8_t address) const {
        const auto it = fifoWords_.find(address);
        if (it == fifoWords_.end()) {
            return 0;
        }
        return it->second.size();
    }

    const std::vector<WriteEntry>& writeLog() const {
        return writeLog_;
    }

private:
    static uint16_t MakeKey(uint8_t address, uint8_t reg) {
        return static_cast<uint16_t>((static_cast<uint16_t>(address) << 8) | reg);
    }

    uint8_t currentAddress_ = 0;
    size_t rxIndex_ = 0;
    std::vector<uint8_t> txBuffer_{};
    std::vector<uint8_t> rxBuffer_{};
    std::unordered_map<uint16_t, uint8_t> registers_{};
    std::unordered_map<uint8_t, uint8_t> registerPointer_{};
    std::unordered_map<uint8_t, std::deque<std::array<uint8_t, 7>>> fifoWords_{};
    std::unordered_map<uint8_t, uint8_t> fifoStatus2Bits_{};
    std::vector<WriteEntry> writeLog_{};
};

inline TwoWire Wire;

#endif
