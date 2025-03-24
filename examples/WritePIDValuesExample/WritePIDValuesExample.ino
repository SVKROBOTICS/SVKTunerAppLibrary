#include <SVKTunerApp.h>
#include <SoftwareSerial.h>

#define BT_RX 3 // Arduino RX Pin
#define BT_TX 2 // Arduino TX Pin

SoftwareSerial bluetoothSerial(BT_RX, BT_TX);
SVKTunerApp tuner(bluetoothSerial);

void setup() {
    Serial.begin(9600);
    tuner.begin(9600); 
    Serial.println("SVKTuner Ready");
}

void loop() {

    if (tuner.isStartSignalReceived()) {
        Serial.println("START command received");
    }

    if (tuner.isStopSignalReceived()) {
        Serial.println("STOP command received");
    }

    if (bluetoothSerial.available()) {
        tuner.updateParameters();
        tuner.logAllParameters();
    }

    delay(100); // Prevent CPU overload
}
