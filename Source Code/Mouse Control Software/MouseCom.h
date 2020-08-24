#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include <thread>
#include <atomic>


struct Ccmnd {
    bool valid = false;
    int val1 = 0, val2 = 0, val3 = 0;
    int command = 7; //FIXIT for TypCmd
};


class CMousCtrlSet;


class CMouseCom {
public:
    CMouseCom();

    virtual ~CMouseCom();

    typedef enum Commands {
        SetMotorPos,
        PosReached,
        GetSensorValue,
        SensorValue,
        KneeSensorValue,
        FootSensorValue,
        ServoSensorValue,
        PIDValues,
        SetLed,
        SetMotorOff,
        MPwrOff,        // commands spine
        MoveLeg,
        StepLeg,
        StepDone,       // commands rpi internal
        InitMouse,
        Trott,
        StopAll
    } typCmd;                        // commands from shell
    std::atomic <Ccmnd> consoleCmnd;// for thread communication

    // setup values
    int Motors[13] = {00, 01, 02, 03, 10, 11, 12, 13, 20, 21, 22, 23, 24};
    int MotorP = 17;
    int MotorI = 0;
    int MotorD = 35;

    // sensor value buffer
    bool left_is_free = true, right_is_free = true;
    static const int storageBuffer = 10000;
    static const int storageSensorBoards = 4; //01 -> 0 / 03 -> 1 / 11 -> 2 / 13 -> 3
    static const int storageVariablesFoot = 1; //value
    static const int storageVariablesKnee = 2; //BX, BY
    int StoreArraySensor[storageSensorBoards][storageVariablesKnee + storageVariablesFoot][storageBuffer];
    int sensor_index[storageSensorBoards][2];

    // position and PID value buffer
    static const int storageServoBoards = 13;
    static const int storageVariablesPos = 2; //value
    static const int storageVariablesPID = 3; //BX, BY
    int StoreArrayPos[storageServoBoards][storageVariablesPos][storageBuffer];
    int StoreArrayPID[storageServoBoards][storageVariablesPID][storageBuffer];
    int position_index[storageServoBoards];
    int pid_index[storageServoBoards];

    //Functions
    void startThread();

    void clearSensorComplete();

    void startUART();

    void setConsoleCcmnd(typCmd cmd, int val1 = 0, int val2 = 0, int val3 = 0);

    void ProcessSpine(typCmd cmd, int val1, int val2 = 0, int val3 = 0);


    //OLD COMMUNICATION
    void setMotorOFF(int id);

    void setMotorPwrOFF();

    void sendNL();

    void MotorPwrCycle();

    void setMotorPID();

    void setMotorSilent(int id, int val1);

    void MotorSetup();

    void ReceiveMsg(typCmd cmd, int val1 = 0, int val2 = 0, int val3 = 0, int val4 = 0);

    // Semaphore for sensor value requests on SWU
    void setLeftBlock();

    void setRightBlock();

protected:
    //virtual void ProcessSpine(typCmd cmd, int val1, int val2, int val3);
    virtual CMousCtrlSet &InitRPI() {}


private:

    std::thread t1; //waiting loop thread
    // UART STREAMS
    int uart0_readstream = -1;
    int uart0_sendstream = -1;

    //FUNCTIONS
    void setup_uart_send(); //sets up uart for read and write
    void setup_uart_read(); //sets up uart for read and write
    void MouseInputLoop();

    //sending funtions return true on success
    bool sendMotor_Serial(int id, int pos, int speed);

    void setMotorLed(int id, int state); //0,1,2
    bool sendStreamRequest(int id, int frequency, int amount);

    bool sendSensorRequest(int id, int sensor);

    //recieve returns amount of arguments, 0 for nothing to read, -1 for error
    int receiveData(); // give array with MAX_ARG_LENGTH
    void checkComndConsole();

    bool sendUartMessage(char buffer[], int l);

    int IDtoStoreArrayIndex(int ID);

    void clearFootPart(int index);

    void clearKneePart(int index);

    int convertHexToInt(char digit);

    unsigned int convertToInt(char *val, int length);
};


#endif // SERIAL_COM_H
