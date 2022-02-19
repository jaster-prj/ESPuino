// !!! MAKE SURE TO EDIT settings.h !!!
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "settings.h" // Contains all user-relevant settings (general)

#include "AudioPlayer.h"
#include "Battery.h"
#include "Bluetooth.h"
#include "Button.h"
#include "Cmd.h"
#include "Common.h"
#include "Ftp.h"
#include "IrReceiver.h"
#include "Led.h"
#include "Log.h"
#include "Mqtt.h"
#include "MemX.h"
#include "Port.h"
#include "Queues.h"
#include "Rfid.h"
#include "RotaryEncoder.h"
#include "SdCard.h"
#include "System.h"
#include "Web.h"
#include "Wlan.h"
#include "revision.h"
#include "Power.h"

#ifdef PLAY_LAST_RFID_AFTER_REBOOT
    bool recoverLastRfid = true;
    bool recoverBootCount = true;
    bool resetBootCount = false;
    uint32_t bootCount = 0;
#endif

////////////

#ifdef SPI1
    SPIClass spi1(VSPI);
#endif
#ifdef SPI2
    SPIClass spi2(HSPI);
#endif
#if defined(I2C1)
    TwoWire i2c1(0);
#endif
#if defined(I2C2)
    TwoWire i2c2(1);
#endif


String ssid =     "LT-Home WLAN 2,4";
String password = "NJ290920!8L&T!40720!8";


#if (HAL == 2)
    #include "AC101.h"
    static TwoWire i2cBusOne = TwoWire(0);
    static AC101 ac(&i2cBusOne);
#endif

#ifdef PLAY_LAST_RFID_AFTER_REBOOT
    // If a problem occurs, remembering last rfid can lead into a boot loop that's hard to escape of.
    // That reason for a mechanism is necessary to prevent this.
    // At start of a boot, bootCount is incremented by one and after 30s decremented because
    // uptime of 30s is considered as "successful boot".
    void recoverBootCountFromNvs(void) {
        if (recoverBootCount) {
            recoverBootCount = false;
            resetBootCount = true;
            bootCount = gPrefsSettings.getUInt("bootCount", 999);

            if (bootCount == 999) {         // first init
                bootCount = 1;
                gPrefsSettings.putUInt("bootCount", bootCount);
            } else if (bootCount >= 3) {    // considered being a bootloop => don't recover last rfid!
                bootCount = 1;
                gPrefsSettings.putUInt("bootCount", bootCount);
                gPrefsSettings.putString("lastRfid", "-1");     // reset last rfid
                Log_Println((char *) FPSTR(bootLoopDetected), LOGLEVEL_ERROR);
                recoverLastRfid = false;
            } else {                        // normal operation
                gPrefsSettings.putUInt("bootCount", ++bootCount);
            }
        }

        if (resetBootCount && millis() >= 30000) {      // reset bootcount
            resetBootCount = false;
            bootCount = 0;
            gPrefsSettings.putUInt("bootCount", bootCount);
            Log_Println((char *) FPSTR(noBootLoopDetected), LOGLEVEL_INFO);
        }
    }

    // Get last RFID-tag applied from NVS
    void recoverLastRfidPlayedFromNvs(void) {
        if (recoverLastRfid) {
            if (System_GetOperationMode() == OPMODE_BLUETOOTH) { // Don't recover if BT-mode is desired
                recoverLastRfid = false;
                return;
            }
            recoverLastRfid = false;
            String lastRfidPlayed = gPrefsSettings.getString("lastRfid", "-1");
            if (!lastRfidPlayed.compareTo("-1")) {
                Log_Println((char *) FPSTR(unableToRestoreLastRfidFromNVS), LOGLEVEL_INFO);
            } else {
                char *lastRfid = x_strdup(lastRfidPlayed.c_str());
                xQueueSend(gRfidCardQueue, lastRfid, 0);
                snprintf(Log_Buffer, Log_BufferLength, "%s: %s", (char *) FPSTR(restoredLastRfidFromNVS), lastRfidPlayed.c_str());
                Log_Println(Log_Buffer, LOGLEVEL_INFO);
            }
        }
    }
#endif


void setup() {

    delay(5000);

    #ifdef SPI1
        spi1.begin(SPI1_SCK, SPI1_MISO, SPI1_MOSI);
    #endif
    #ifdef SPI2
        spi2.begin(SPI2_SCK, SPI2_MISO, SPI2_MOSI);
    #endif
    #if defined(I2C1)
        i2c1.begin(I2C1_SDA, I2C1_SCL);
    #endif
    #if defined(I2C2)
        i2c2.begin(I2C2_SDA, I2C2_SCL);
    #endif

    Log_Init();
    Queues_Init();

    // Make sure all wakeups can be enabled *before* initializing RFID, which can enter sleep immediately
    Button_Init();  // To preseed internal button-storage with values
    #ifdef PN5180_ENABLE_LPCD
        Rfid_Init();
        // Check if wakeup-reason was card-detection (PN5180 only)
        esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
        if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
            Rfid_WakeupCheck();
        }
    #endif

    System_Init();

    // Needs i2c first if port-expander is used
    Port_Init();

    // If port-expander is used, port_init has to be called first, as power can be (possibly) done by port-expander
    Power_Init();

    Battery_Init();

    // Init audio before power on to avoid speaker noise
    AudioPlayer_Init();

    // All checks that could send us to sleep are done, power up fully
    Power_PeripheralOn();

    memset(&gPlayProperties, 0, sizeof(gPlayProperties));
    gPlayProperties.playlistFinished = true;

    Led_Init();

    // Only used for ESP32-A1S-Audiokit
    #if (HAL == 2)
        i2cBusOne.setPins(IIC_DATA, IIC_CLK);
        i2cBusOne.setClock(40000);

        while (not ac.begin()) {
            Serial.println(F("AC101 Failed!"));
            delay(1000);
        }
        Serial.println(F("AC101 via I2C - OK!"));

        pinMode(22, OUTPUT);
        digitalWrite(22, HIGH);
        ac.SetVolumeHeadphone(80);
    #endif

    // Needs power first
    SdCard_Init();

    // welcome message
    Serial.println(F(""));
    Serial.println(F("  _____   ____    ____            _                 "));
    Serial.println(F(" | ____| / ___|  |  _ \\   _   _  (_)  _ __     ___  "));
    Serial.println(F(" |  _|   \\__  \\  | |_) | | | | | | | | '_ \\   / _ \\"));
    Serial.println(F(" | |___   ___) | |  __/  | |_| | | | | | | | | (_) |"));
    Serial.println(F(" |_____| |____/  |_|      \\__,_| |_| |_| |_|  \\___/ "));
    Serial.print(F(" Rfid-controlled musicplayer\n\n"));
    Serial.printf("%s\n\n", softwareRevision);
    Serial.println("ESP-IDF version: " + String(ESP.getSdkVersion()));

    // print wake-up reason
    System_ShowWakeUpReason();
	// print SD card info
    SdCard_PrintInfo();

    Ftp_Init();
    Mqtt_Init();
    #ifndef PN5180_ENABLE_LPCD
        #if defined (RFID_READER_TYPE_MFRC522_SPI) || defined (RFID_READER_TYPE_MFRC522_I2C) || defined(RFID_READER_TYPE_PN5180)
//            Rfid_Init();
        #endif
    #endif
    RotaryEncoder_Init();
    Wlan_Init();
    Bluetooth_Init();

    if (OPMODE_NORMAL == System_GetOperationMode()) {
        Wlan_Cyclic();
    }
    
    IrReceiver_Init();
    System_UpdateActivityTimer(); // initial set after boot
    Led_Indicate(LedIndicatorType::BootComplete);

    snprintf(Log_Buffer, Log_BufferLength, "%s: %u", (char *) FPSTR(freeHeapAfterSetup), ESP.getFreeHeap());
    Log_Println(Log_Buffer, LOGLEVEL_DEBUG);
    snprintf(Log_Buffer, Log_BufferLength, "PSRAM: %u bytes", ESP.getPsramSize());
    Log_Println(Log_Buffer, LOGLEVEL_DEBUG);
    snprintf(Log_Buffer, Log_BufferLength, "Flash-size: %u bytes", ESP.getFlashChipSize());
    Log_Println(Log_Buffer, LOGLEVEL_DEBUG);
    if (Wlan_IsConnected()) {
        snprintf(Log_Buffer, Log_BufferLength, "RSSI: %d dBm", Wlan_GetRssi());
        Log_Println(Log_Buffer, LOGLEVEL_DEBUG);
    }
    System_ShowUpgradeWarning();
}

void loop() {
    if (OPMODE_BLUETOOTH == System_GetOperationMode()) {
        Bluetooth_Cyclic();
    } else {
        Wlan_Cyclic();
        Web_Cyclic();
        Ftp_Cyclic();
        RotaryEncoder_Cyclic();
        Mqtt_Cyclic();
    }

    AudioPlayer_Cyclic();
    Battery_Cyclic();
    //Port_Cyclic(); // called by button (controlled via hw-timer)
    Button_Cyclic();
    System_Cyclic();
    Rfid_PreferenceLookupHandler();

    #ifdef PLAY_LAST_RFID_AFTER_REBOOT
        recoverBootCountFromNvs();
        recoverLastRfidPlayedFromNvs();
    #endif

    IrReceiver_Cyclic();
    vTaskDelay(portTICK_RATE_MS * 5u);
}
