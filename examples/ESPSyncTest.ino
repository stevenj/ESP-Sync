#include <Arduino.h>
#include <ESPSync.h>

ESPSync FSSync;

void setup()
{
    // Start the Serial Port we use for testing
    Serial.begin(115200);

    // Tell the ESPSync class what serial port it needs to transmit on.
    FSSync.setSerial(&Serial);
}

void loop()
{
    if (Serial.available() > 0) {
        FSSync.ProcessByte(Serial.read());
    }
}