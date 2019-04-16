#include <Arduino.h>
#include <ESPSync.h>
#include <FS.h>

ESPSync FSSync;


void setup()
{
    // Init random number generator
    randomSeed(analogRead(0));

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
    Serial.println(" ? = This menu");
    Serial.println(" S = SPIFFS Status");
    Serial.println(" M = Make random file");
    Serial.println(" F = Format SPIFFS");
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

void MakeRandomFile(void)
{
    char fname[33];
    uint8_t fname_length;
    uint32_t x;
    File f;
    uint32_t fsize;
    uint8_t data;


    fname_length = random(5, 28);
    for (x = 0; x < fname_length; x++) {
        fname[x] = random(' ','~');
    }
    fname[x] = '.';
    fname[x+1] = 'r';
    fname[x+2] = 'n';
    fname[x+3] = 'd';
    fname[x+4] = 0x00;

    Serial.printf("\nCreating Random file: '%s'\n", fname);
    f = SPIFFS.open(fname,"w");
    if (!f) {
        Serial.println("file open failed");
    } else {
        fsize = random(100,8192);
        Serial.printf("Random Size = %d Bytes\n", fsize);
        for (x = 0; x < fsize; x++) {
                    data = random(0,255);
            if (1 != f.write(data)) {
                break;
            }
            if ((x % (fsize/25)) == 0) {
                Serial.print('.');
            }
        }
            Serial.printf("\nDone, wrote %d Bytes.\n", x);
        f.close();
    }
}

void FormatSPIFFS(void) 
{
    uint32_t now = millis();

    Serial.printf("\nFormatting SPIFFS\n");
    if (SPIFFS.format()) {
        Serial.printf("  Done OK.\n");
    } else {
        Serial.printf("  ERROR: Format failed.\n");
    }

    now = millis()-now;

    Serial.printf("Format took %d.%03d seconds\n", now / 1000, now % 1000);

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
            break;

            case 'M': case 'm':
                MakeRandomFile();
            break;

            case 'F': case 'f':
                FormatSPIFFS();
            break;

        }
    }

    if (wait_for_ms(1000)) {
#ifdef LED_BUILTIN
        // Toggle LED
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif        
    }
}