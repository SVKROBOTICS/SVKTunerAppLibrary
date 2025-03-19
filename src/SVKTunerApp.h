#pragma once

#include <Arduino.h>
#include <EEPROM.h>

class SVKTunerApp {
public:
    // Constructor
    SVKTunerApp();
    /// Initialize Serial communication
    void begin(long baudRate);

    // Start-Stop functionality
    bool isStartSignalReceived();
    bool isStopSignalReceived();

    /// Read Bluetooth data and update EEPROM
    void updateParameters();

    /// Functions to read PID values from EEPROM
    float readKp();
    float readKi();
    float readKd();

    /// Functions to read other parameters from EEPROM
    int readBaseSpeed();
    int readMaxSpeed();
    int readAcceleration();

    /// Functions to log PID values to Serial Monitor
    void logKp();
    void logKi();
    void logKd();
    void logAllParameters();

    /// Functions to write new PID values to EEPROM
    void writeKp(float Kp);
    void writeKi(float Ki);
    void writeKd(float Kd);

    /// Functions to write other parameters to EEPROM
    void writeBaseSpeed(int baseSpeed);
    void writeMaxSpeed(int maxSpeed);
    void writeAcceleration(int acceleration);

    /// Function to validate float values
    bool isValidFloat(String value);

    /// Functions for custom variables
    void addCustomVariable(String name, float value);
    float readCustomVariable(String name);
    void logCustomVariables();

    /// Parse received data
    void parseData(String data);

private:
    /// Write float to EEPROM
    void writeFloatToEEPROM(int address, float value);
    /// Read float from EEPROM
    float readFloatFromEEPROM(int address);       
    /// Write int to EEPROM     
    void writeIntToEEPROM(int address, int value);
    /// Read int from EEPROM
    int readIntFromEEPROM(int address);
    /// Helper function to parse fixed variable data
    void parseFixedVariable(String data, const String& prefix, void (SVKTunerApp::*writeFunction)(float), SVKTunerApp* instance);

    /// @brief Tracks the last write time
    unsigned long lastWriteTime = 0;
    /// 5-second timeout
    static const unsigned long WRITE_TIMEOUT = 5000;

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
    static const int CUSTOM_VAR_SIZE = 20; /// Each custom variable uses 20 bytes (name + value)

    /// Structure to store custom variable names and values
    struct CustomVariable {
        char name[16]; /// Variable name (up to 15 characters + null terminator)
        float value;   /// Variable value
    };

    // Start-Stop signals
    String lastReceivedData; // Store the last received data for start/stop checks
};