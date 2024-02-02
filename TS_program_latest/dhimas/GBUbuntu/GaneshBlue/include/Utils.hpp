#ifndef _utils
#define _utils

#include <string>
#include <termios.h>

namespace Utils
{
    void foo();
    void serWrite(int _port, char _msg[]);
    int tcpWrite(int _socketDescriptor, char _data[]);
    void getTime(char *_timeDest);
    int openPort(std::string portname, int BAUDRATE, std::string connectionType="Serial");
    int write2LOG(char *logname, char *message, bool _trunc);

    std::string convertToString(char *a);
    // int sendDatalog(bool deb);

    struct Communication
    {
        // Serial Communication for Serial
        std::string loc;
        int baudrate;
        int port;
        bool isSet()
        {
            if (port > 0)
                return true;
            return false;
        }
        void printConnectionStatus();
        virtual void sendACK();

        // variables for multithread - flagging
        bool thread;

        // processing data
        std::string rawdata;
        std::string dataSender;
        std::string dataCommand;
        std::string dataArgs;
        int dataCode;
        int isACK;

        void activateThread()
        {
            tcflush(port, TCIOFLUSH);
            thread = true;
        }
        void deactivateThread()
        {
            tcflush(port, TCIOFLUSH);
            thread = false;
        }
    };
    // inherit the Communication structure
    struct TCPCommunication : public Communication{
        // TCP Communication using Serial Comm as Parent
        // Overiding sendAck methods
        void sendACK() override;
    };

    void Parse(Communication &c);
    int loadConfig(Communication &c, std::string name);
    struct GliderUtility
    {
        void parsingData(std::string str, int *BE_input, int *MM_input, int *rudder_input, int *yaw_input, int *main_input);
    };
}

extern std::string workPath;
extern int totalDefaultMessage;

class defaultMessage
{
public:
    std::string message;
    int code;
    defaultMessage(std::string m, int c)
    {
        message = m;
        code = c;
    }
};

enum defaultmessage
{
    AT = 1,
    RTB = 2,
    START = 3,
    PARAM = 4,
    WAYPOINT = 5,
    PING = 6,
    DATA = 7,
    STOP = 8,
    OPERATION = 9,
    RTO = 10,
    CTM = 11,
    SELFTEST = 12,
    MANUALGLIDE = 17,
    PITCHCONTROL = 14,
    DEPTHCONTROL = 15,
    NGOTAK = 16,
    ZIGZAG = 18,
    YAWCONTROL = 19,
    TESTING = 20,
};
#endif