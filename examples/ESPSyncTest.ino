#include <Arduino.h>
#include <ESPSync.h>
#include <FS.h>

ESPSync FSSync;


void setup()
{
    SPIFFS.begin();

    // Start the Serial Port we use for testing
    Serial.begin(115200);

    // Tell the ESPSync class what serial port it needs to transmit on.
    FSSync.setSerial(&Serial);

#ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
#endif    
}

static bool wait_for_ms(uint32_t delay) {
    static uint32_t dt = millis();
    uint32_t now = millis();

    if (dt == now) return false; // Already triggered so don't retrigger

    if (now - dt >= delay) {
        dt = now;
    }

    return (dt==now);

}

void PrintMenu(void) 
{
    Serial.println("");
    Serial.println(" --- ESP-Sync Tester ---");
    Serial.println(" ? = This menu ");
    Serial.println(" S = SPIFFS Status ");
}

void PrintSPIFFSStatus(void) 
{
    FSInfo fs_info;
    bool ok = SPIFFS.info(fs_info);

    Serial.println("");
    Serial.println("SPIFFS Status : ");
    if (!ok) {
        Serial.println("    ERROR: Could not get status!");
    } else {
        Serial.printf("  Total Bytes     = %d\n", fs_info.totalBytes);
        Serial.printf("  Used  Bytes     = %d\n", fs_info.usedBytes);
        Serial.printf("  Block Size      = %d\n", fs_info.blockSize);
        Serial.printf("  Page Size       = %d\n", fs_info.pageSize);
        Serial.printf("  Max Open Files  = %d\n", fs_info.maxOpenFiles);
        Serial.printf("  Max Path Length = %d\n", fs_info.maxPathLength);
    }
}

void loop()
{
    uint8_t data;

    if (FSSync.getData(&data)) {
        Serial.write(data); /* Echo received data */

        // Simple command menu
        switch (data) {
            case '?':
                PrintMenu();
            break;

            case 'S': case 's':
                PrintSPIFFSStatus();
        }
    }

    if (wait_for_ms(1000)) {
#ifdef LED_BUILTIN
        // Toggle LED
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif        
    }
}