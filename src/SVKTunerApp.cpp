#include "./SVKTunerApp.h"

#define START_MARKER '<'
#define END_MARKER '>'
#define SEPARATOR '|'
#define ACK_STRING "<ACK>"
#define MAX_PACKET_SIZE 64
#define PACKET_TIMEOUT 2000 // 2 seconds timeout for packet reception

SVKTunerApp::SVKTunerApp(SoftwareSerial& serial) 
    : bluetoothSerial(serial)  // Initialize reference member properly
{
    bluetoothSerial.listen();  // Optional: Ensure SoftwareSerial is listening
    resetPacketState();
}

void SVKTunerApp::resetPacketState()
{
    packetBuffer = "";
    currentPacketNumber = 0;
    totalPacketsExpected = 0;
    lastPacketTime = 0;
    packetReceptionInProgress = false;
}

void SVKTunerApp::begin(long baudRate)
{
    bluetoothSerial.begin(baudRate); // Initialize SoftwareSerial communication
    resetPacketState();
}

bool SVKTunerApp::receiveDataPackets() {
    static String incomingData;
    
    while (bluetoothSerial.available()) {
        char c = bluetoothSerial.read();
        
        // Skip any characters until we get a start marker
        if (!packetReceptionInProgress && c != START_MARKER) {
            continue; // Ignore garbage before packet starts
        }
        
        if (c == START_MARKER) {
            incomingData = "";
            packetReceptionInProgress = true;
            continue;
        }
        
        if (c == END_MARKER && packetReceptionInProgress) {
            packetReceptionInProgress = false;
            
            // Basic packet format validation
            if (incomingData.length() < 5) { // Minimum valid packet: "0|1|d"
                Serial.println("Packet too short - discarded");
                incomingData = "";
                continue;
            }
            
            if (processIncomingPacket(incomingData)) {
                incomingData = "";
                return true;
            }
            incomingData = "";
            continue;
        }
        
        if (packetReceptionInProgress) {
            // Prevent buffer overflow
            if (incomingData.length() < MAX_PACKET_SIZE) {
                incomingData += c;
            } else {
                Serial.println("Packet too long - resetting");
                resetPacketState();
                incomingData = "";
            }
        }
    }
    
    if (packetReceptionInProgress && millis() - lastPacketTime > PACKET_TIMEOUT) {
        Serial.println("Packet timeout - resetting");
        resetPacketState();
        return false;
    }
    
    return false;
}

bool SVKTunerApp::processIncomingPacket(String packet)
{
    // Parse packet number and total packets
    int firstSeparator = packet.indexOf(SEPARATOR);
    int secondSeparator = packet.indexOf(SEPARATOR, firstSeparator + 1);
    
    if (firstSeparator == -1 || secondSeparator == -1) {
        Serial.println("Invalid packet format");
        return false;
    }
    
    int packetNum = packet.substring(0, firstSeparator).toInt();
    int totalPackets = packet.substring(firstSeparator + 1, secondSeparator).toInt();
    String chunk = packet.substring(secondSeparator + 1);
    
    // Validate packet number
    if (packetNum < 0 || packetNum >= totalPackets) {
        Serial.println("Invalid packet number");
        return false;
    }
    
    // If this is the first packet, reset the buffer
    if (packetNum == 0) {
        packetBuffer = chunk;
        totalPacketsExpected = totalPackets;
        currentPacketNumber = 1;
    } 
    // If it's the expected next packet
    else if (packetNum == currentPacketNumber) {
        packetBuffer += chunk;
        currentPacketNumber++;
    } 
    // Out of sequence packet
    else {
        Serial.println("Out of sequence packet");
        return false;
    }
    
    // Send acknowledgment
    bluetoothSerial.print(ACK_STRING);
    bluetoothSerial.print(packetNum);
    bluetoothSerial.println(END_MARKER);
    
    lastPacketTime = millis();
    
    // Check if we've received all packets
    if (currentPacketNumber >= totalPacketsExpected) {
        // Process the complete message
        parseData(packetBuffer);
        resetPacketState();
        return true;
    }
    
    return false;
}

String SVKTunerApp::getLastCommand() {
    if (bluetoothSerial.available()) {
        lastReceivedData = readBluetoothLine();
    }
    return lastReceivedData;
}

bool SVKTunerApp::isStartSignalReceived() {
    String result = getLastCommand();
    return result == "!START!\n";
}

bool SVKTunerApp::isStopSignalReceived() {
    String result = getLastCommand();
    return result == "!STOP!\n";
}

void SVKTunerApp::updateParameters()
{
    // Check for and process incoming packets
    if (receiveDataPackets()) {
        Serial.println("Parameters updated from packet data");
    }
}

void SVKTunerApp::parseData(String data) {
    Serial.print("Processing data batch: ");
    Serial.println(data);

    // Variables to store parsed values (initialize with "no update" indicators)
    float newKp = NAN;
    float newKi = NAN;
    float newKd = NAN;
    int newBaseSpeed = -1;
    int newMaxSpeed = -1;
    int newAcceleration = -1;

    // Parse standard parameters
    if (data.indexOf("Kp=") != -1) {
        newKp = parseValue(data, "Kp=");
        if (!isValidFloat(String(newKp))) newKp = NAN;
    }
    if (data.indexOf("Ki=") != -1) {
        newKi = parseValue(data, "Ki=");
        if (!isValidFloat(String(newKi))) newKi = NAN;
    }
    if (data.indexOf("Kd=") != -1) {
        newKd = parseValue(data, "Kd=");
        if (!isValidFloat(String(newKd))) newKd = NAN;
    }
    if (data.indexOf("baseSpeed=") != -1) {
        newBaseSpeed = (int)parseValue(data, "baseSpeed=");
    }
    if (data.indexOf("maxSpeed=") != -1) {
        newMaxSpeed = (int)parseValue(data, "maxSpeed=");
    }
    if (data.indexOf("acceleration=") != -1) {
        newAcceleration = (int)parseValue(data, "acceleration=");
    }

    // Process the batch of standard parameter updates
    writeParameterBatch(newKp, newKi, newKd, newBaseSpeed, newMaxSpeed, newAcceleration);

    // Parse custom variables
    int customVarIndex = 0;
    int customVarCount = 0;
    
    while (customVarIndex != -1 && customVarCount < MAX_CUSTOM_VARS) {
        customVarIndex = data.indexOf(',', customVarIndex);
        if (customVarIndex != -1) {
            customVarIndex++; // Move past comma
            int equalsIndex = data.indexOf('=', customVarIndex);
            if (equalsIndex == -1) break;

            int commaIndex = data.indexOf(',', equalsIndex);
            if (commaIndex == -1) commaIndex = data.length();

            String varName = data.substring(customVarIndex, equalsIndex);
            String varValue = data.substring(equalsIndex + 1, commaIndex);

            if (isValidFloat(varValue)) {
                // Check if we're in the write timeout window
                if (millis() - lastWriteTime >= WRITE_TIMEOUT) {
                    addCustomVariable(varName, varValue.toFloat());
                    lastWriteTime = millis(); // Reset timeout
                    customVarCount++;
                    Serial.print("Added custom var: ");
                    Serial.print(varName);
                    Serial.print(" = ");
                    Serial.println(varValue);
                } else {
                    Serial.println("Custom var update skipped due to write timeout");
                    break; // Skip remaining custom vars in this batch
                }
            }
            customVarIndex = commaIndex;
        }
    }

    if (customVarCount >= MAX_CUSTOM_VARS) {
        Serial.println("Custom variable limit reached!");
    }
}

void SVKTunerApp::writeParameterBatch(float kp, float ki, float kd,
                                    int baseSpeed, int maxSpeed, int acceleration) {
    // Only check timeout once per batch
    if (millis() - lastWriteTime < WRITE_TIMEOUT) {
        Serial.println("Write timeout - skipping batch update");
        return;
    }

    bool changesDetected = false;

    // Check and update Kp
    if (!isnan(kp)) {
        float currentKp = readKp();
        if (abs(kp - currentKp) > 0.001f) { // Floating point comparison threshold
            writeKp(kp);
            changesDetected = true;
        }
    }

    // Check and update Ki
    if (!isnan(ki)) {
        float currentKi = readKi();
        if (abs(ki - currentKi) > 0.001f) {
            writeKi(ki);
            changesDetected = true;
        }
    }

    // Check and update Kd
    if (!isnan(kd)) {
        float currentKd = readKd();
        if (abs(kd - currentKd) > 0.001f) {
            writeKd(kd);
            changesDetected = true;
        }
    }

    // Check and update baseSpeed
    if (baseSpeed >= 0) { // Using -1 as "no update" indicator
        int currentBaseSpeed = readBaseSpeed();
        if (baseSpeed != currentBaseSpeed) {
            writeBaseSpeed(baseSpeed);
            changesDetected = true;
        }
    }

    // Check and update maxSpeed
    if (maxSpeed >= 0) {
        int currentMaxSpeed = readMaxSpeed();
        if (maxSpeed != currentMaxSpeed) {
            writeMaxSpeed(maxSpeed);
            changesDetected = true;
        }
    }

    // Check and update acceleration
    if (acceleration >= 0) {
        int currentAcceleration = readAcceleration();
        if (acceleration != currentAcceleration) {
            writeAcceleration(acceleration);
            changesDetected = true;
        }
    }

    // Only update lastWriteTime if actual changes were made
    if (changesDetected) {
        lastWriteTime = millis();
        Serial.println("Batch update completed successfully");
    } else {
        Serial.println("No parameter changes detected in batch");
    }
}

float SVKTunerApp::parseValue(String data, const String& prefix)
{
    int index = data.indexOf(prefix);
    if (index != -1) {
        int start = index + prefix.length();
        int end = data.indexOf(',', start);
        if (end == -1) end = data.length();

        String valueStr = data.substring(start, end);
        if (isValidFloat(valueStr)) {
            return valueStr.toFloat();
        }
    }
    return 0.0; // Return 0 if the prefix is not found or the value is invalid
}

bool SVKTunerApp::isValidFloat(String value)
{
    for (char c : value) {
        if (!isdigit(c) && c != '.' && c != '-') {
            return false;
        }
    }
    return true;
}

// Functions to read/write floats to EEPROM
void SVKTunerApp::writeFloatToEEPROM(int address, float value)
{
    EEPROM.put(address, value); // Write float to EEPROM
}

float SVKTunerApp::readFloatFromEEPROM(int address)
{
    float value;
    EEPROM.get(address, value); // Read float from EEPROM
    return value;
}

// Functions to read/write integers to EEPROM
void SVKTunerApp::writeIntToEEPROM(int address, int value)
{
    EEPROM.put(address, value); // Write int to EEPROM
}

int SVKTunerApp::readIntFromEEPROM(int address)
{
    int value;
    EEPROM.get(address, value); // Read int from EEPROM
    return value;
}

// Functions to read PID values from EEPROM
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

// Functions to log PID values to Serial Monitor
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

// Functions to write new PID values to EEPROM
void SVKTunerApp::writeKp(float value)
{
    if (millis() - lastWriteTime >= WRITE_TIMEOUT) {
        EEPROM.put(KP_ADDRESS, value);
        lastWriteTime = millis(); // Update the last write time
        Serial.println("Kp updated in EEPROM.");
    } else {
        Serial.println("Write timeout: Kp not updated.");
    }
}

void SVKTunerApp::writeKi(float value)
{
    if (millis() - lastWriteTime >= WRITE_TIMEOUT) {
        EEPROM.put(KI_ADDRESS, value);
        lastWriteTime = millis(); // Update the last write time
        Serial.println("Ki updated in EEPROM.");
    } else {
        Serial.println("Write timeout: Ki not updated.");
    }
}

void SVKTunerApp::writeKd(float value)
{
    if (millis() - lastWriteTime >= WRITE_TIMEOUT) {
        EEPROM.put(KD_ADDRESS, value);
        lastWriteTime = millis(); // Update the last write time
        Serial.println("Kd updated in EEPROM.");
    } else {
        Serial.println("Write timeout: Kd not updated.");
    }
}

void SVKTunerApp::writeBaseSpeed(int value)
{
    if (millis() - lastWriteTime >= WRITE_TIMEOUT) {
        EEPROM.put(BASE_SPEED_ADDRESS, value);
        lastWriteTime = millis(); // Update the last write time
        Serial.println("BaseSpeed updated in EEPROM.");
    } else {
        Serial.println("Write timeout: BaseSpeed not updated.");
    }
}

void SVKTunerApp::writeMaxSpeed(int value) {
    if (millis() - lastWriteTime >= WRITE_TIMEOUT) {
        EEPROM.put(MAX_SPEED_ADDRESS, value);
        lastWriteTime = millis(); // Update the last write time
        Serial.println("MaxSpeed updated in EEPROM.");
    } else {
        Serial.println("Write timeout: MaxSpeed not updated.");
    }
}

void SVKTunerApp::writeAcceleration(int value)
{
    if (millis() - lastWriteTime >= WRITE_TIMEOUT) {
        EEPROM.put(ACCELERATION_ADDRESS, value);
        lastWriteTime = millis(); // Update the last write time
        Serial.println("Acceleration updated in EEPROM.");
    } else {
        Serial.println("Write timeout: Acceleration not updated.");
    }
}

// Functions for custom variables
void SVKTunerApp::addCustomVariable(String name, float value)
{
    // Check if the timeout has passed
    if (millis() - lastWriteTime < WRITE_TIMEOUT) {
        Serial.println("Write timeout: Custom variable not updated.");
        return; // Skip the write if the timeout has not passed
    }

    CustomVariable var;
    name.toCharArray(var.name, 16); // Copy name to struct
    var.value = value;

    // Find the next available slot in EEPROM
    for (int i = 0; i < MAX_CUSTOM_VARS; i++) {
        int address = CUSTOM_VAR_START_ADDRESS + (i * CUSTOM_VAR_SIZE);
        CustomVariable storedVar;
        EEPROM.get(address, storedVar);

        if (storedVar.name[0] == '\0' || strcmp(storedVar.name, var.name) == 0) {
            EEPROM.put(address, var); // Write custom variable to EEPROM
            lastWriteTime = millis(); // Update the last write time
            Serial.print("Custom variable ");
            Serial.print(var.name);
            Serial.println(" updated in EEPROM.");
            return;
        }
    }
    Serial.println("Custom variable limit reached!");
}

float SVKTunerApp::readCustomVariable(String name)
{
    for (int i = 0; i < MAX_CUSTOM_VARS; i++) {
        int address = CUSTOM_VAR_START_ADDRESS + (i * CUSTOM_VAR_SIZE);
        CustomVariable var;
        EEPROM.get(address, var);

        if (strcmp(var.name, name.c_str()) == 0) {
            return var.value;
        }
    }
    return 0.0; // Return 0 if variable not found
}

void SVKTunerApp::logCustomVariables()
{
    for (int i = 0; i < MAX_CUSTOM_VARS; i++) {
        int address = CUSTOM_VAR_START_ADDRESS + (i * CUSTOM_VAR_SIZE);
        CustomVariable var;
        EEPROM.get(address, var);

        if (var.name[0] != '\0') {
            Serial.print(var.name);
            Serial.print(": ");
            Serial.println(var.value);
        }
    }
}

String SVKTunerApp::readBluetoothLine() {
    String data = "";
    while (bluetoothSerial.available()) {
        char c = bluetoothSerial.read();
        
        // Stop completely if we hit newline
        if (c == '\n') break;
        
        // Only add non-whitespace characters
        if (!isWhitespace(c)) {
            data += c;
        }
    }
    return data;
}

void SVKTunerApp::debugBluetoothStream() {
    static String debugBuffer;
    static unsigned long lastPrint = 0;
    const unsigned long printInterval = 100; // ms between prints
    
    // Read all available bytes
    while (bluetoothSerial.available()) {
        char c = bluetoothSerial.read();
        
        // Filter non-printable characters (show hex codes instead)
        if (c >= 32 && c <= 126) {  // Printable ASCII range
            debugBuffer += c;
        } else {
            char hex[5];
            sprintf(hex, "[%02X]", c);  // Show hex code
            debugBuffer += hex;
        }
        
        // Check for line endings or buffer size limit
        if (c == '\n' || debugBuffer.length() >= 64) {
            printDebugBuffer(debugBuffer);
            debugBuffer = "";
        }
    }
    
    // Periodic flush if no line ending
    if (millis() - lastPrint >= printInterval && debugBuffer.length() > 0) {
        printDebugBuffer(debugBuffer);
        debugBuffer = "";
    }
}

// Helper function for formatted output
void SVKTunerApp::printDebugBuffer(String &buffer) {
    if (buffer.length() == 0) return;
    
    Serial.print("[BT] ");
    Serial.print(millis());
    Serial.print("ms: ");
    
    // Split into multiple lines if contains newlines
    int startIdx = 0;
    int nlPos;
    while ((nlPos = buffer.indexOf('\n', startIdx)) >= 0) {
        Serial.println(buffer.substring(startIdx, nlPos));
        Serial.print("     ");  // Indent continuation lines
        startIdx = nlPos + 1;
    }
    
    // Print remaining content
    if (startIdx < buffer.length()) {
        Serial.println(buffer.substring(startIdx));
    }
    
    buffer = "";
}
