#ifndef COMMLINK_H
#define COMMLINK_H
#define SERIAL_BAUDRATE 115200
class Commlink{
public:
    void Initialize();
    void WriteGroundRadio(const char* data);
    void ReadGroundRadio(char* buffer, int length);
    void ParseCommand(const char* command);


};

#endif // COMMLINK_H