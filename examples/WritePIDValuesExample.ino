/* Example code for Reading/Writing PID and Robot Parameters using the SVK Tuner App for mobiles 
 * Permanently writes values to parameters/custom variables to EEPROM
*/

#include <SVKTunerApp.h>

SVKTunerApp tuner;

void setup() {
    Serial.begin(9600); // Initialize Serial communication for debugging
    tuner.begin(9600);  // Initialize Bluetooth communication

    Serial.println("SVKTunerApp Example: WritePIDValues");
    Serial.println("Waiting for data from HC-05...");
}

void loop() {
    // Check if new data is available from HC-05
    if (Serial.available()) 
    {
        // Process the new data and update EEPROM
        tuner.updateParameters();

        // Log the current PID values and custom variables
        tuner.logAllParameters();
    }

    delay(100); // Small delay to reduce CPU usage
}