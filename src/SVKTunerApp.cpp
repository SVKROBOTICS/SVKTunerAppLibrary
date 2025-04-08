#include "./SVKTunerApp.h"

// =============== DEBUG CONFIGURATION ===============
// Uncomment the following line to enable debug mode
#define SVKTUNER_DEBUG 

// Debug output macros that only compile when debug is enabled
#ifdef SVKTUNER_DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_PRINTF(...)
#endif
// ==================================================


// Instance of ParamIDs Struct
const ParamIDs SVKTunerApp::PARAM_IDS;

/*
 * Public methods
 */

SVKTunerApp::SVKTunerApp(SoftwareSerial& serial) 
    : bluetoothSerial(serial)  // Initialize reference member properly
{
    _currentState = STOPPED;
    bluetoothSerial.listen();
}

void SVKTunerApp::begin(long baudRate)
{
    bluetoothSerial.begin(baudRate); // Initialize SoftwareSerial communication
}

bool SVKTunerApp::processBluetoothData() {
    while (bluetoothSerial.available()) {
        byte receivedByte = bluetoothSerial.read();

        // Check for the command header bytes and process accordingly
        if (receivedByte == PARAM_IDS.PID_PARAM_HEADER) {
            parsePidData();
        } else if (receivedByte == PARAM_IDS.SPEED_PARAM_HEADER) {
            parseSpeedData();
        } else if (receivedByte == PARAM_IDS.CUSTOM_VAR_HEADER) {
            parseCustomVariableData();
        } else if (receivedByte == PARAM_IDS.COMMAND_START) {
            startRobot();
        } else if (receivedByte == PARAM_IDS.COMMAND_STOP) {
            stopRobot();
        }
    }
    return true; // Return true if data was processed
}

bool SVKTunerApp::processStartStopCommands() {
    while (bluetoothSerial.available()) {
        byte receivedByte = bluetoothSerial.read();
        
        // Check for the start signal
        if (receivedByte == PARAM_IDS.COMMAND_START) {
            DEBUG_PRINTLN("Start command received");
            startRobot();  // Call startRobot function to start the robot
        }
        // Check for the stop signal
        else if (receivedByte == PARAM_IDS.COMMAND_STOP) {
            DEBUG_PRINTLN("Stop command received");
            stopRobot();  // Call stopRobot function to stop the robot
        }
    }
    return true; // Return true if data was processed
}

float SVKTunerApp::readKp()
{
    return readFloatFromEEPROM(KP_ADDRESS);
}

float SVKTunerApp::readKi()
{
    return readFloatFromEEPROM(KI_ADDRESS);
}

float SVKTunerApp::readKd()
{
    return readFloatFromEEPROM(KD_ADDRESS);
}

int SVKTunerApp::readBaseSpeed()
{
    return readIntFromEEPROM(BASE_SPEED_ADDRESS);
}

int SVKTunerApp::readMaxSpeed()
{
    return readIntFromEEPROM(MAX_SPEED_ADDRESS);
}

int SVKTunerApp::readAcceleration()
{
    return readIntFromEEPROM(ACCELERATION_ADDRESS);
}

void SVKTunerApp::logKp()
{
    Serial.print("Kp: ");
    Serial.println(readKp());
}

void SVKTunerApp::logKi()
{
    Serial.print("Ki: ");
    Serial.println(readKi());
}

void SVKTunerApp::logKd()
{
    Serial.print("Kd: ");
    Serial.println(readKd());
}

void SVKTunerApp::logAllParameters()
{
    logKp();
    logKi();
    logKd();
    logCustomVariables();
}

/*
 * Private methods
 */


void SVKTunerApp::parsePidData() {
    byte pidType = bluetoothSerial.read();  // Read the next byte for the PID type (KP, KI, KD)
    
    switch (pidType) {
        case PARAM_IDS.KP:
            float newKp = readFloatFromBluetooth();
            writeKp(newKp);  // Write Kp to EEPROM
            DEBUG_PRINTLN("Updated Kp: " + String(newKp));
            break;
        case PARAM_IDS.KI:
            float newKi = readFloatFromBluetooth();
            writeKi(newKi);  // Write Ki to EEPROM
            DEBUG_PRINTLN("Updated Ki: " + String(newKi));
            break;
        case PARAM_IDS.KD:
            float newKd = readFloatFromBluetooth();
            writeKd(newKd);  // Write Kd to EEPROM
            DEBUG_PRINTLN("Updated Kd: " + String(newKd));
            break;
        default:
            DEBUG_PRINTLN("Unknown PID type.");
            break;
    }
}

void SVKTunerApp::parseSpeedData() {
    byte speedType = bluetoothSerial.read(); // Read the next byte for the speed parameter type
    
    switch (speedType) {
        case PARAM_IDS.BASE_SPEED:
            int newBaseSpeed = readIntFromBluetooth();
            writeBaseSpeed(newBaseSpeed); // Save to EEPROM
            DEBUG_PRINTLN("Updated BaseSpeed: " + String(newBaseSpeed));
            break;
        case PARAM_IDS.MAX_SPEED:
            int newMaxSpeed = readIntFromBluetooth();
            writeMaxSpeed(newMaxSpeed); // Save to EEPROM
            DEBUG_PRINTLN("Updated MaxSpeed: " + String(newMaxSpeed));
            break;
        case PARAM_IDS.ACCELERATION:
            int newAcceleration = readIntFromBluetooth();
            writeAcceleration(newAcceleration); // Save to EEPROM
            DEBUG_PRINTLN("Updated Acceleration: " + String(newAcceleration));
            break;
        default:
            DEBUG_PRINTLN("Unknown Speed parameter.");
            break;
    }
}

void SVKTunerApp::parseCustomVariableData() {
    byte customVarType = bluetoothSerial.read(); // Read custom variable identifier

    switch (customVarType) {
        case PARAM_IDS.CUSTOM_VAR_1:
            float customVar1 = readFloatFromBluetooth();
            addCustomVariable(customVar1);
            DEBUG_PRINTLN("Updated Custom Variable 1: " + String(customVar1));
            break;
        case PARAM_IDS.CUSTOM_VAR_2:
            float customVar2 = readFloatFromBluetooth();
            addCustomVariable(customVar2);
            DEBUG_PRINTLN("Updated Custom Variable 2: " + String(customVar2));
            break;
        case PARAM_IDS.CUSTOM_VAR_3:
            float customVar3 = readFloatFromBluetooth();
            addCustomVariable(customVar3);
            DEBUG_PRINTLN("Updated Custom Variable 3: " + String(customVar3));
            break;
        case PARAM_IDS.CUSTOM_VAR_4:
            float customVar4 = readFloatFromBluetooth();
            addCustomVariable(customVar4);
            DEBUG_PRINTLN("Updated Custom Variable 4: " + String(customVar4));
            break;
        case PARAM_IDS.CUSTOM_VAR_5:
            float customVar5 = readFloatFromBluetooth();
            addCustomVariable(customVar5);
            DEBUG_PRINTLN("Updated Custom Variable 5: " + String(customVar5));
            break;
        default:
            DEBUG_PRINTLN("Unknown Custom Variable type.");
            break;
    }
}

float SVKTunerApp::readFloatFromBluetooth() {
    byte floatBytes[4];
    bluetoothSerial.readBytes(floatBytes, 4);
    float value;
    memcpy(&value, floatBytes, sizeof(value)); // Convert bytes to float
    return value;
}

int SVKTunerApp::readIntFromBluetooth() {
    byte intBytes[2];
    bluetoothSerial.readBytes(intBytes, 2);
    int value;
    memcpy(&value, intBytes, sizeof(value)); // Convert bytes to int
    return value;
}

void SVKTunerApp::startRobot() {
    DEBUG_PRINTLN("Starting robot...");
    _currentState = RUNNING;
}

void SVKTunerApp::stopRobot() {
    DEBUG_PRINTLN("Stopping robot...");
    _currentState = STOPPED;
}

void SVKTunerApp::writeFloatToEEPROM(int address, float value)
{
    float currentValue;
    EEPROM.get(address, currentValue);

    if(currentValue != value)
    {
        EEPROM.put(address, value); // Write float to EEPROM
    }
}

float SVKTunerApp::readFloatFromEEPROM(int address)
{
    float value;
    EEPROM.get(address, value); // Read float from EEPROM
    return value;
}

void SVKTunerApp::writeIntToEEPROM(int address, int value)
{
    int currentValue;
    EEPROM.get(address, currentValue);

    if(currentValue != value)
    {
        EEPROM.put(address, value); // Write int to EEPROM
    }
}

int SVKTunerApp::readIntFromEEPROM(int address)
{
    int value;
    EEPROM.get(address, value); // Read int from EEPROM
    return value;
}

void SVKTunerApp::writeKp(float value)
{
    if (millis() - _lastWriteTime >= WRITE_TIMEOUT) {
        writeFloatToEEPROM(KP_ADDRESS, value);
        _lastWriteTime = millis(); // Update the last write time
        Serial.println("Kp updated in EEPROM.");
    } else {
        Serial.println("Write timeout: Kp not updated.");
    }
}

void SVKTunerApp::writeKi(float value)
{
    if (millis() - _lastWriteTime >= WRITE_TIMEOUT) {
        writeFloatToEEPROM(KI_ADDRESS, value);
        _lastWriteTime = millis(); // Update the last write time
        Serial.println("Ki updated in EEPROM.");
    } else {
        Serial.println("Write timeout: Ki not updated.");
    }
}

void SVKTunerApp::writeKd(float value)
{
    if (millis() - _lastWriteTime >= WRITE_TIMEOUT) {
        writeFloatToEEPROM(KD_ADDRESS, value);
        _lastWriteTime = millis(); // Update the last write time
        Serial.println("Kd updated in EEPROM.");
    } else {
        Serial.println("Write timeout: Kd not updated.");
    }
}

void SVKTunerApp::writeBaseSpeed(int value)
{
    if (millis() - _lastWriteTime >= WRITE_TIMEOUT) {
        writeIntToEEPROM(BASE_SPEED_ADDRESS, value);
        _lastWriteTime = millis(); // Update the last write time
        Serial.println("BaseSpeed updated in EEPROM.");
    } else {
        Serial.println("Write timeout: BaseSpeed not updated.");
    }
}

void SVKTunerApp::writeMaxSpeed(int value) {
    if (millis() - _lastWriteTime >= WRITE_TIMEOUT) {
        writeIntToEEPROM(MAX_SPEED_ADDRESS, value);
        _lastWriteTime = millis(); // Update the last write time
        Serial.println("MaxSpeed updated in EEPROM.");
    } else {
        Serial.println("Write timeout: MaxSpeed not updated.");
    }
}

void SVKTunerApp::writeAcceleration(int value)
{
    if (millis() - _lastWriteTime >= WRITE_TIMEOUT) {
        writeIntToEEPROM(ACCELERATION_ADDRESS, value);
        _lastWriteTime = millis(); // Update the last write time
        Serial.println("Acceleration updated in EEPROM.");
    } else {
        Serial.println("Write timeout: Acceleration not updated.");
    }
}

void SVKTunerApp::addCustomVariable() {
    // Read which variable (1-5) from Bluetooth stream
    byte customVarType = bluetoothSerial.read();
    
    // Validate the custom variable ID
    if(customVarType < PARAM_IDS.CUSTOM_VAR_1 || customVarType > PARAM_IDS.CUSTOM_VAR_5) {
        DEBUG_PRINTLN("Invalid custom variable ID");
        return;
    }
    
    // Calculate which variable (0-4 index)
    int varIndex = customVarType - PARAM_IDS.CUSTOM_VAR_1;
    
    // Calculate EEPROM address (sequential floats starting at CUSTOM_VAR_START_ADDRESS)
    int valueAddress = CUSTOM_VAR_START_ADDRESS + (varIndex * CUSTOM_VAR_SIZE);
    
    // Read the float value from Bluetooth
    float value = readFloatFromBluetooth();
    
    // Write to EEPROM with timeout protection
    if (millis() - _lastWriteTime >= WRITE_TIMEOUT) {
        writeFloatToEEPROM(valueAddress, value);
        _lastWriteTime = millis();
        
        DEBUG_PRINT("Updated CustomVar");
        DEBUG_PRINT(varIndex + 1);
        DEBUG_PRINT(": ");
        DEBUG_PRINTLN(value, 4); // Print with 4 decimal places
    } else {
        DEBUG_PRINTLN("Write timeout: Custom variable not updated");
    }
}

void SVKTunerApp::logCustomVariables() {
    DEBUG_PRINTLN("\nCustom Variables:");
    
    for(int i = 0; i < MAX_CUSTOM_VARS; i++) {
        int address = CUSTOM_VAR_START_ADDRESS + (i * CUSTOM_VAR_SIZE);
        float value = readFloatFromEEPROM(address);
        
        DEBUG_PRINT("  CustomVar");
        DEBUG_PRINT(i + 1);
        DEBUG_PRINT(": ");
        DEBUG_PRINTLN(value, 4); // Print with 4 decimal places
    }
}

void SVKTunerApp::debugBluetoothStream() {
    static String debugBuffer;
    static unsigned long lastCharTime = millis();
    const unsigned long timeout = 100;  // ms timeout for incomplete messages
    const int maxBufferSize = 48;      // Leave room for hex conversions (64-16)

    while (bluetoothSerial.available()) {
        // Check if we have buffer space remaining
        if (debugBuffer.length() >= maxBufferSize) {
            printDebugBuffer("[BUFFER FULL] " + debugBuffer);
            debugBuffer = "";
        }

        char c = bluetoothSerial.read();
        lastCharTime = millis();

        // Handle newline as message terminator
        if (c == '\n') {
            if (debugBuffer.length() > 0) {
                printDebugBuffer(debugBuffer);
                debugBuffer = "";
            }
            continue;
        }

        // Skip carriage returns
        if (c == '\r') continue;

        // Handle printable ASCII
        if (c >= 32 && c <= 126) {
            debugBuffer += c;
        } 
        // Format non-printables as hex
        else {
            char hex[5];
            snprintf(hex, sizeof(hex), "[%02X]", c);
            debugBuffer += hex;
        }
    }

    // Handle timeout for incomplete messages
    if (debugBuffer.length() > 0 && (millis() - lastCharTime > timeout)) {
        printDebugBuffer("[TIMEOUT] " + debugBuffer);
        debugBuffer = "";
    }
}

void SVKTunerApp::printDebugBuffer(String &message) {
    // Split long messages to respect Serial buffer
    const int chunkSize = 60;
    for (int i = 0; i < message.length(); i += chunkSize) {
        Serial.print("[BT] ");
        Serial.print(millis());
        Serial.print("ms: ");
        Serial.println(message.substring(i, i + chunkSize));
    }
}