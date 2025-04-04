/* SVK Tuner App Start-Stop Test with Debug Support */
#define SVKTUNER_DEBUG  // Uncomment to enable debug output

#include <SVKTunerApp.h>
#include <SoftwareSerial.h>

// Bluetooth module connections
#define BT_RX 3  // Bluetooth TX → Arduino pin 3
#define BT_TX 2  // Bluetooth RX → Arduino pin 2

SoftwareSerial bluetoothSerial(BT_RX, BT_TX);
SVKTunerApp tuner(bluetoothSerial);

void setup() {
    Serial.begin(9600);
    bluetoothSerial.begin(9600);
    
    #ifdef SVKTUNER_DEBUG
    Serial.println(F("[DEBUG] System initialized"));
    Serial.println(F("[DEBUG] Bluetooth ready"));
    Serial.println(F("[DEBUG] Waiting for !START! or !STOP!..."));
    #else
    Serial.println(F("Waiting for start-stop signal..."));
    #endif
}

void loop() {
    // Read and process Bluetooth data
    String command = tuner.getLastCommand();
    
    #ifdef SVKTUNER_DEBUG
    if (command.length() > 0) {
        Serial.print(F("[DEBUG] Processing command: "));
        Serial.println(command);
    }
    #endif

    // Check commands
    if (tuner.isStartSignalReceived()) {
        #ifdef SVKTUNER_DEBUG
        Serial.println(F("[DEBUG] >>> START CONFIRMED <<<"));
        #else
        Serial.println(F("Start signal received!"));
        #endif
    } 
    else if (tuner.isStopSignalReceived()) {
        #ifdef SVKTUNER_DEBUG
        Serial.println(F("[DEBUG] >>> STOP CONFIRMED <<<"));
        #else
        Serial.println(F("Stop signal received!"));
        #endif
    }

    delay(10);  // Reduced delay for better responsiveness
}