/* SVK Tuner Start/Stop Test with Debug Support */
#define SVKTUNER_DEBUG  // Comment out to disable debug prints

#include <SVKTunerApp.h>
#include <SoftwareSerial.h>

// Bluetooth module connections
#define BT_RX 3  // Connect to Bluetooth TX
#define BT_TX 2  // Connect to Bluetooth RX

SoftwareSerial bluetoothSerial(BT_RX, BT_TX);
SVKTunerApp tuner(bluetoothSerial);

void setup() {
    Serial.begin(115200);  // Faster baud for debug output
    bluetoothSerial.begin(9600);
    
    #ifdef SVKTUNER_DEBUG
    while (!Serial);  // Wait for Serial monitor on debug builds
    DEBUG_PRINTLN(F("\n\n=== Bluetooth Debug Monitor ==="));
    DEBUG_PRINTLN(F("System initialized"));
    DEBUG_PRINTLN(F("Waiting for !START! or !STOP!..."));
    #else
    Serial.println(F("Waiting for signals..."));
    #endif
}

void loop() {
    // First check if we're receiving any data at all
    #ifdef SVKTUNER_DEBUG
    if (bluetoothSerial.available()) {
        DEBUG_PRINTLN(F("[BT] Data detected in buffer..."));
    }
    #endif

    // Process commands
    String command = tuner.getLastCommand();
    
    if (tuner.isStartSignalReceived()) {
        Serial.println(F("ACTION: Start command received!"));
        // Add your start logic here
    } 
    else if (tuner.isStopSignalReceived()) {
        Serial.println(F("ACTION: Stop command received!"));
        // Add your stop logic here
    }
    
    #ifdef SVKTUNER_DEBUG
    // Additional debug for connection health
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 2000) {  // Every 2 seconds
        lastDebug = millis();
        DEBUG_PRINTLN(F("[STATUS] System active..."));
    }
    #endif

    delay(10);  // Short delay for stability
}