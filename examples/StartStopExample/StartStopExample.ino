/* Example code for checking the Start-Stop functionality of the SVK Tuner App for mobiles.
 * NOTE: This doesn't start the robot you are connected to, just checks if the Start-Stop functionality is
 * working correctly. To actually start the robot, go to your robot's example page and use the specific
 * start-stop function.
 */

// Define macro to enable or disable start-stop functionality
#define ENABLE_START_STOP 1 // Set to 1 to enable, 0 to disable

#include <SVKTunerApp.h>
#include <SoftwareSerial.h>

// Define RX and TX pins for Bluetooth module
#define BT_RX 3  // Bluetooth module TX -> Arduino pin 3
#define BT_TX 2  // Bluetooth module RX -> Arduino pin 2

SoftwareSerial bluetoothSerial(BT_RX, BT_TX);

SVKTunerApp tuner(bluetoothSerial);

void setup() {
    Serial.begin(9600); // Initialize Serial communication
    bluetoothSerial.begin(9600); // Initialize Bluetooth communication

    Serial.println("Waiting for start-stop signal...");
}

void loop() {

    tuner.debugBluetoothStream();
    
#if ENABLE_START_STOP
    // Check for start signal
    if (tuner.isStartSignalReceived()) {
        Serial.println("Start signal received!");
    }

    // Check for stop signal
    if (tuner.isStopSignalReceived()) {
        Serial.println("Stop signal received!");
    }
#endif

    delay(100);
}
