#ifndef COMMLINK_H
#define COMMLINK_H
#define TEAM_ID      "1030"
#define XBEE_SERIAL   Serial1
#define XBEE_BAUDRATE 115200

class HardwareManager;

class Commlink{
public:
    void Initialize();
    void WriteGroundRadio(const char* data);
    void ReadGroundRadio(char* buffer, int length);
    void ParseCommand(const char* command);
    void SetHardwareManager(HardwareManager* hwManager);
private:
    static int index;
    HardwareManager* hardwareManager_;
};

#endif // COMMLINK_H