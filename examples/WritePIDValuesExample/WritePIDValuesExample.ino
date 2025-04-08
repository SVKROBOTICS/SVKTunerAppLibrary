#include <SVKTunerApp.h>
#include <SoftwareSerial.h>

#define BT_RX 3 // Arduino RX Pin
#define BT_TX 2 // Arduino TX Pin

SoftwareSerial bluetoothSerial(BT_RX, BT_TX);
SVKTunerApp tuner(bluetoothSerial);

bool dataReceived = false; // Flag to indicate if Bluetooth data was processed

void setup() {
    Serial.begin(9600);
    tuner.begin(9600); // Initialize your tuner object
    Serial.println("SVKTuner Ready");
}

void loop() {
    // Check if data is available in the Bluetooth buffer
    if (bluetoothSerial.available() && !dataReceived) {
        // Process the Bluetooth data
        if (tuner.processBluetoothData()) {
            dataReceived = true; // Mark that data has been processed
            Serial.println("Data processed and written to EEPROM.");
            tuner.logAllParameters(); // Log all parameters after writing to EEPROM
        }
    }

    // Reset the flag when no data is available, or when you want to start a new cycle
    if (!bluetoothSerial.available()) {
        dataReceived = false;
    }

    delay(100); // Prevent CPU overload and allow time for other tasks
}
