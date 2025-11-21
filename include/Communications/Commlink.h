

class Commlink{
public:
    void Initialize();
    void WriteGroundRadio(const char* data);
    void ReadGroundRadio(char* buffer, int length);
    void ParseCommand(const char* command);
    
private:
    const int SERIAL_BAUDRATE = 115200;
    
};