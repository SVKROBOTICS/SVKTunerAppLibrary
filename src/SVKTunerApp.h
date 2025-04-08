#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

/// @brief Struct for hex Param Ids for each parameter
struct ParamIDs {
    // PID Parameters
    static const uint8_t PID_PARAM_HEADER = 0x50; // PID Param header byte 'P' in bin
    static const uint8_t KP = 0x01;
    static const uint8_t KI = 0x02;
    static const uint8_t KD = 0x03;

    // Speed Parameters
    static const uint8_t SPEED_PARAM_HEADER = 0x53; // Speed Param header byte 'S' in hex
    static const uint8_t BASE_SPEED = 0x10;
    static const uint8_t MAX_SPEED = 0x11;
    static const uint8_t ACCELERATION = 0x12;

    // Custom Variable Values
    static const uint8_t CUSTOM_VAR_HEADER = 0x43; // 'C' for Custom variables
    static const uint8_t CUSTOM_VAR_1 = 0x20;
    static const uint8_t CUSTOM_VAR_2 = 0x21;
    static const uint8_t CUSTOM_VAR_3 = 0x22;
    static const uint8_t CUSTOM_VAR_4 = 0x23;
    static const uint8_t CUSTOM_VAR_5 = 0x24;

    // Command markers
    static const uint8_t COMMAND_START = 0x55;
    static const uint8_t COMMAND_STOP = 0xAA;
};

/// @brief Enum for setting/getting robot state
enum START_STOP_STATE {
    STOPPED,
    RUNNING
};

class SVKTunerApp {
public:
    /// @brief Class Constructor
    /// @param serial Memory reference to SoftwareSerial object
    SVKTunerApp(SoftwareSerial& serial);
    /// @brief Begins connection
    void begin(long baudRate);
    /// @brief Reads bluetooth buffer and processes all bluetooth data correctly based on input
    bool processBluetoothData();
    /// @brief Reads bluetooth buffer and checks only for start and stop commands
    bool processStartStopCommands();

    // Functions to read PID values from EEPROM

    /// @brief Read Kp value
    float readKp();
    /// @brief Read Ki value
    float readKi();
    /// @brief Read Kd value
    float readKd();

    /// Functions to read other parameters from EEPROM

    /// @brief Read robot base Speed
    int readBaseSpeed();
    /// @brief Read robot max Speed
    int readMaxSpeed();
    /// @brief Read robot acceleration
    int readAcceleration();

    /// @brief Returns Robot State, either STOPPED or RUNNING
    /// @return Start Stop Enum Value ( STOPPED || RUNNING )
    inline START_STOP_STATE getRobotState() { return _currentState; }

    /// Functions to log PID values to Serial Monitor

    /// @brief Logs Kp to Serial monitor
    void logKp();
    /// @brief Logs Ki to Serial monitor
    void logKi();
    /// @brief Logs Kd to Serial monitor
    void logKd();
    /// @brief Logs BaseSpeed to Serial monitor
    void logBaseSpeed();
    /// @brief Logs MaxSpeed to Serial monitor
    void logMaxSpeed();
    /// @brief Logs Acceleration to Serial monitor
    void logAcceleration();
    /// @brief Logs Custom Variable values
    void logCustomVariables();
    /// @brief Logs all PID parameters to Serial monitor
    void logAllParameters();

    /// @brief Bluetooth debugging to Serial monitor
    void debugBluetoothStream();

private:
    /// @brief Stored Reference to a SoftwareSerial object
    SoftwareSerial& bluetoothSerial;

    /// @brief Parses Pid parameters from bluetooth data, and writes them to EEPROM
    void parsePidData();
    /// @brief Parses robot speed parameters from bluetooth data, and writes them to EEPROM
    void parseSpeedData();
    /// @brief Parses custom variable values from bluetooth data, and writes them to EEPROM
    void parseCustomVariableData();
    /// @brief Reads next float from bluetooth buffer (4 bytes) and converts it to float
    /// @return Next float value from bluetooth buffer
    float readFloatFromBluetooth();
    /// @brief Reads next integer form bluetooth buffer (2 bytes) and converts it to int
    /// @return Next int value from bluetooth buffer
    int readIntFromBluetooth();
    /// @brief Changes robot state to Start
    void startRobot();
    /// @brief Changes robot state to Stop
    void stopRobot();
    /// @brief Write float to EEPROM
    /// @param address EEPROM Memory address
    /// @param value Float value
    void writeFloatToEEPROM(int address, float value);
    /// @brief Read float from EEPROM
    /// @param address EEPROM Memory address
    /// @return Float value
    float readFloatFromEEPROM(int address);
    /// @brief Write int to EEPROM
    /// @param address EEPROM Memory address
    /// @param value Integer value
    void writeIntToEEPROM(int address, int value);
    /// @brief Read int to EEPROM
    /// @param address EEPROM Memory address
    /// @return Integer value
    int readIntFromEEPROM(int address);
    /// Functions to write new PID values to EEPROM

    /// @brief Write Kp value to EEPROM
    /// @param Kp Float Kp value
    void writeKp(float Kp);
    /// @brief Write Ki value to EEPROM
    /// @param Ki Float Ki value
    void writeKi(float Ki);
    /// @brief Write Kd value to EEPROM
    /// @param Kd Float Kd value
    void writeKd(float Kd);

    /// Functions to write other parameters to EEPROM

    /// @brief Writes base speed value to EEPROM
    /// @param baseSpeed Int baseSpeed value
    void writeBaseSpeed(int baseSpeed);
    /// @brief Writes max speed value to EEPROM
    /// @param maxSpeed Int maxSpeed value
    void writeMaxSpeed(int maxSpeed);
    /// @brief Writes acceleration value to EEPROM
    /// @param acceleration Int acceleration value
    void writeAcceleration(int acceleration);

    /// @brief Adds and writes custom variable values to EEPROM
    /// @param customVarType Custom Variable Unique Param ID
    /// @param value Float value of Variable
    void addCustomVariable(byte customVarType, float value);

    /// @brief Prints Bluetooth data buffer for debugging
    /// @param buffer Bluetooth 64 byte buffer
    void printDebugBuffer(String &buffer);

    /// @brief Tracks the last write time
    unsigned long _lastWriteTime = 0;
    /// 5-second timeout
    static const unsigned long WRITE_TIMEOUT = 1000;

    /// Addresses for PID parameters in EEPROM
    static const int KP_ADDRESS = 0;
    static const int KI_ADDRESS = 4;
    static const int KD_ADDRESS = 8;

    /// Addresses for other parameters
    static const int BASE_SPEED_ADDRESS = 12;
    static const int MAX_SPEED_ADDRESS = 14;
    static const int ACCELERATION_ADDRESS = 16;

    /// Addresses for custom variables (up to 5)
    static const int CUSTOM_VAR_START_ADDRESS = 20; /// After PID and other parameters
    static const int MAX_CUSTOM_VARS = 5;
    static const int CUSTOM_VAR_SIZE = 4; /// Each custom variable is flaot value

    // Instace of PARAM_IDs Struct
    static const ParamIDs PARAM_IDS;

    // Start-Stop state
    START_STOP_STATE _currentState;

};