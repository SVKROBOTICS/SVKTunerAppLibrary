#include "./SVKTunerApp.h"

SVKTunerApp::SVKTunerApp() {
    // Constructor 
}

void SVKTunerApp::begin(long baudRate) {
    Serial.begin(baudRate); // Initialize Serial communication
}

bool SVKTunerApp::isStartSignalReceived() {
    if (Serial.available()) {
        lastReceivedData = Serial.readStringUntil('\n'); // Read the incoming data
        if (lastReceivedData == "START") {
            return true; // Start signal received
        }
    }
    return false; // No start signal
}

bool SVKTunerApp::isStopSignalReceived() {
    if (Serial.available()) {
        lastReceivedData = Serial.readStringUntil('\n'); // Read the incoming data
        if (lastReceivedData == "STOP") {
            return true; // Stop signal received
        }
    }
    return false; // No stop signal
}

void SVKTunerApp::updateParameters() {
    if (Serial.available()) {
        String data = Serial.readStringUntil('\n'); // Read data until newline
        parseData(data); // Parse the received data
    }
}

void SVKTunerApp::parseData(String data) {

    // Parse fixed variables
    parseFixedVariable(data, "Kp=", [this](float value) { writeKp(value); });
    parseFixedVariable(data, "Ki=", [this](float value) { writeKi(value); });
    parseFixedVariable(data, "Kd=", [this](float value) { writeKd(value); });
    parseFixedVariable(data, "baseSpeed=", [this](float value) { writeBaseSpeed((int)value); });
    parseFixedVariable(data, "maxSpeed=", [this](float value) { writeMaxSpeed((int)value); });
    parseFixedVariable(data, "acceleration=", [this](float value) { writeAcceleration((int)value); });

    // Parse custom variables
    int customVarIndex = 0;
    int customVarCount = 0; // Counter for custom variables
    while (customVarIndex != -1 && customVarCount < MAX_CUSTOM_VARS) {
        customVarIndex = data.indexOf(',', customVarIndex); // Find the next comma
        if (customVarIndex != -1) {
            customVarIndex++; // Move past the comma
            int equalsIndex = data.indexOf('=', customVarIndex);
            if (equalsIndex == -1) break; // No more variables

            int commaIndex = data.indexOf(',', equalsIndex);
            if (commaIndex == -1) commaIndex = data.length(); // Last variable

            // Extract the variable name and value
            String varName = data.substring(customVarIndex, equalsIndex);
            String varValue = data.substring(equalsIndex + 1, commaIndex);

            // Validate and store the custom variable
            if (isValidFloat(varValue)) {
                addCustomVariable(varName, varValue.toFloat());
                customVarCount++; // Increment the counter
            }

            // Move to the next variable
            customVarIndex = commaIndex;
        }
    }

    // Log if the custom variable limit is reached
    if (customVarCount >= MAX_CUSTOM_VARS) {
        Serial.println("Custom variable limit reached!");
    }
}

// Helper function to parse fixed variables
void SVKTunerApp::parseFixedVariable(String data, const String& prefix, std::function<void(float)> writeFunction) {
    int index = data.indexOf(prefix);
    if (index != -1) {
        int start = index + prefix.length();
        int end = data.indexOf(',', start);
        if (end == -1) end = data.length();

        String valueStr = data.substring(start, end);
        if (isValidFloat(valueStr)) {
            writeFunction(valueStr.toFloat());
        }
    }
}

bool SVKTunerApp::isValidFloat(String value) {
    for (char c : value) {
        if (!isdigit(c) && c != '.' && c != '-') {
            return false;
        }
    }
    return true;
}

// Functions to read/write floats to EEPROM
void SVKTunerApp::writeFloatToEEPROM(int address, float value) {
    EEPROM.put(address, value); // Write float to EEPROM
}

float SVKTunerApp::readFloatFromEEPROM(int address) {
    float value;
    EEPROM.get(address, value); // Read float from EEPROM
    return value;
}

// Functions to read/write integers to EEPROM
void SVKTunerApp::writeIntToEEPROM(int address, int value) {
    EEPROM.put(address, value); // Write int to EEPROM
}

int SVKTunerApp::readIntFromEEPROM(int address) {
    int value;
    EEPROM.get(address, value); // Read int from EEPROM
    return value;
}

// Functions to read PID values from EEPROM
float SVKTunerApp::readKp() {
    return readFloatFromEEPROM(KP_ADDRESS);
}

float SVKTunerApp::readKi() {
    return readFloatFromEEPROM(KI_ADDRESS);
}

float SVKTunerApp::readKd() {
    return readFloatFromEEPROM(KD_ADDRESS);
}

int SVKTunerApp::readBaseSpeed() {
    return readIntFromEEPROM(BASE_SPEED_ADDRESS);
}

int SVKTunerApp::readMaxSpeed() {
    return readIntFromEEPROM(MAX_SPEED_ADDRESS);
}

int SVKTunerApp::readAcceleration() {
    return readIntFromEEPROM(ACCELERATION_ADDRESS);
}



// Functions to log PID values to Serial Monitor
void SVKTunerApp::logKp() {
    Serial.print("Kp: ");
    Serial.println(readKp());
}

void SVKTunerApp::logKi() {
    Serial.print("Ki: ");
    Serial.println(readKi());
}

void SVKTunerApp::logKd() {
    Serial.print("Kd: ");
    Serial.println(readKd());
}

void SVKTunerApp::logAllParameters() {
    logKp();
    logKi();
    logKd();
    logCustomVariables();
}

// Functions to write new PID values to EEPROM
void SVKTunerApp::writeKp(float value) {
    if (millis() - lastWriteTime >= WRITE_TIMEOUT) {
        EEPROM.put(KP_ADDRESS, value);
        lastWriteTime = millis(); // Update the last write time
        Serial.println("Kp updated in EEPROM.");
    } else {
        Serial.println("Write timeout: Kp not updated.");
    }
}

void SVKTunerApp::writeKi(float value) {
    if (millis() - lastWriteTime >= WRITE_TIMEOUT) {
        EEPROM.put(KI_ADDRESS, value);
        lastWriteTime = millis(); // Update the last write time
        Serial.println("Ki updated in EEPROM.");
    } else {
        Serial.println("Write timeout: Ki not updated.");
    }
}

void SVKTunerApp::writeKd(float value) {
    if (millis() - lastWriteTime >= WRITE_TIMEOUT) {
        EEPROM.put(KD_ADDRESS, value);
        lastWriteTime = millis(); // Update the last write time
        Serial.println("Kd updated in EEPROM.");
    } else {
        Serial.println("Write timeout: Kd not updated.");
    }
}

void SVKTunerApp::writeBaseSpeed(int value) {
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

void SVKTunerApp::writeAcceleration(int value) {
    if (millis() - lastWriteTime >= WRITE_TIMEOUT) {
        EEPROM.put(ACCELERATION_ADDRESS, value);
        lastWriteTime = millis(); // Update the last write time
        Serial.println("Acceleration updated in EEPROM.");
    } else {
        Serial.println("Write timeout: Acceleration not updated.");
    }
}

// Functions for custom variables
void SVKTunerApp::addCustomVariable(String name, float value) {
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

float SVKTunerApp::readCustomVariable(String name) {
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

void SVKTunerApp::logCustomVariables() {
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