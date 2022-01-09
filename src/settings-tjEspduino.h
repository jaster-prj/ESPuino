#ifndef __ESPUINO_SETTINGS_TJESPDUINO_H__
#define __ESPUINO_SETTINGS_TJESPDUINO_H__
    #include "Arduino.h"

    //######################### INFOS ####################################
    /* This is not a develboard-specific config-file. It's intended for your own use.
    It's been originally derived from lolin32, but just change it according your needs!
    */

    //################## GPIO-configuration ##############################
    // Please note: GPIOs 34, 35, 36, 39 are input-only and don't have pullup-resistors.
    // So if connecting a button to these, make sure to add a 10k-pullup-resistor for each button.
    // Further infos: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
    
    #define SPI1
    #define SPI2
    #define I2C1
    #define I2C2

    #if defined(SPI1)
        #define SPI1_SCK                    18
        #define SPI1_MISO                   19
        #define SPI1_MOSI                   23
    #endif

    #if defined(SPI2)
        #define SPI2_SCK                    14
        #define SPI2_MISO                   12
        #define SPI2_MOSI                   13
    #endif

    #if defined(I2C1)
        #define I2C1_SCL                    22
        #define I2C1_SDA                    21
    #endif

    #if defined(I2C2)
        #define I2C1_SCL                    35
        #define I2C1_SDA                    34
    #endif
    
    #ifdef SD_MMC_1BIT_MODE
        // uSD-card-reader (via SD-MMC 1Bit)
        //
        // SD_MMC uses fixed pins
        //  MOSI    15
        //  SCK     14
        //  MISO    2
    #else
        // uSD-card-reader (via SPI)
        #define SD_CS                       25          // GPIO for chip select (SD)
        #define SD_SPI                      SPI1
    #endif

    // RFID (via SPI)
    #define RST_PIN                         99          // Not necessary but has to be set anyway; so let's use a dummy-number
    #define RFID_CS                         04          // GPIO for chip select (RFID)
    #define RFID_SPI                        SPI2         // GPIO for clock-signal (RFID)

    #ifdef RFID_READER_TYPE_PN5180
        #define RFID_BUSY                   33          // PN5180 BUSY PIN
        #define RFID_RST                    27          // PN5180 RESET PIN
        #define RFID_IRQ                    99          // PN5180 IRQ PIN (only needed for low power card detection)
    #endif

    // VS1053 (DAC)
    #if defined(VS1053_ENABLE)
        #define VS1053_CS                   05
        #define VS1053_DCS                  32
        #define VS1053_RST                  26
        #define VS1053_DREG                 02
        #define VS1053_SPI                  SPI1
    #endif

    // I2S (DAC)
    #if !defined(VS1053_ENABLE)
        #define I2S_DOUT                    25          // Digital out (I2S)
        #define I2S_BCLK                    27          // BCLK (I2S)
        #define I2S_LRC                     26          // LRC (I2S)
    #endif

    // Rotary encoder
    #if defined(USEROTARY_ENABLE)
        #define ROTARYENCODER_CLK           34          // If you want to reverse encoder's direction, just switch GPIOs of CLK with DT (in software or hardware)
        #define ROTARYENCODER_DT            35          // Info: Lolin D32 / Lolin D32 pro 35 are using 35 for battery-voltage-monitoring!
        #define ROTARYENCODER_BUTTON        32          // (set to 99 to disable; 0->39 for GPIO; 100->115 for port-expander)
    #endif

    // Amp enable (optional)
    //#define GPIO_PA_EN                      112         // To enable amp for loudspeaker (GPIO or port-channel)
    //#define GPIO_HP_EN                      113         // To enable amp for headphones (GPIO or port-channel)

    // Control-buttons (set to 99 to DISABLE; 0->39 for GPIO; 100->115 for port-expander)
    #define NEXT_BUTTON                    100          // Button 0: GPIO to detect next
    #define PREVIOUS_BUTTON                102          // Button 1: GPIO to detect previous (Important: as of 19.11.2020 changed from 33 to 2; make sure to change in SD-MMC-mode)
    #define PAUSEPLAY_BUTTON               101          // Button 2: GPIO to detect pause/play
    #define BUTTON_4                        99          // Button 4: unnamed optional button
    #define BUTTON_5                        99          // Button 5: unnamed optional button

    // Channels of port-expander can be read cyclic or interrupt-driven. It's strongly recommended to use the interrupt-way!
    // Infos: https://forum.espuino.de/t/einsatz-des-port-expanders-pca9555/306
    #if defined(PORT_EXPANDER_ENABLE)
        #define PE_INTERRUPT                16          // GPIO that is used to receive interrupts from port-expander
        #define PE_RST                      17          // GPIO that is used to receive interrupts from port-expander
        #if defined(PE_PCA9555) || defined(PE_MCP23017)
            #define PE_I2C                  I2C1
        #elif defined(PE_MCP23S17)
            #define PE_SPI                  SPI2
        #endif
    #endif

    // I2C-configuration (necessary for RC522 [only via i2c - not spi!] or port-expander)
    #if defined(RFID_READER_TYPE_MFRC522_I2C)
        #define RFID_I2C                    I2C1        // i2c1
    #endif

    // Wake-up button => this also is the interrupt-pin if port-expander is enabled!
    // Please note: only RTC-GPIOs (0, 4, 12, 13, 14, 15, 25, 26, 27, 32, 33, 34, 35, 36, 39, 99) can be used! Set to 99 to DISABLE.
    // Please note #2: this button can be used as interrupt-pin for port-expander. If so, all pins connected to port-expander can wake up ESPuino.
    #define WAKEUP_BUTTON                   99          // Defines the button that is used to wake up ESPuino from deepsleep.

    // (optional) Power-control
    #define POWER                           99          // GPIO used to drive transistor-circuit, that switches off peripheral devices while ESP32-deepsleep

    // (optional) Neopixel
    #define LED_PIN                         99          // GPIO for Neopixel-signaling

    // (optinal) Headphone-detection
    #if defined(HEADPHONE_ADJUST_ENABLE)
        //#define DETECT_HP_ON_HIGH                       // Per default headphones are supposed to be connected if HT_DETECT is LOW. DETECT_HP_ON_HIGH will change this behaviour to HIGH.
        #define HP_DETECT                   22          // GPIO that detects, if there's a plug in the headphone jack or not
    #endif

    // (optional) Monitoring of battery-voltage via ADC
    #if defined(MEASURE_BATTERY_VOLTAGE)
        #define VOLTAGE_READ_PIN            33          // GPIO used to monitor battery-voltage. Change to 35 if you're using Lolin D32 or Lolin D32 pro as it's hard-wired there!
        constexpr float referenceVoltage = 3.35;                  // Voltage between 3.3V and GND-pin at the develboard in battery-mode (disconnect USB!)
        constexpr float offsetVoltage = 0.1;                      // If voltage measured by ESP isn't 100% accurate, you can add an correction-value here
    #endif

    // (optional) For measuring battery-voltage a voltage-divider is necessary. Their values need to be configured here.
    #if defined(MEASURE_BATTERY_VOLTAGE)
        constexpr uint8_t rdiv1 = 129;                               // Rdiv1 of voltage-divider (kOhms) (measure exact value with multimeter!)
        constexpr uint16_t rdiv2 = 129;                              // Rdiv2 of voltage-divider (kOhms) (measure exact value with multimeter!) => used to measure voltage via ADC!
    #endif

    // (Optional) remote control via infrared
    #if defined(IR_CONTROL_ENABLE)
        #define IRLED_PIN                   22              // GPIO where IR-receiver is connected (only tested with VS1838B)
        #define IR_DEBOUNCE                 200             // Interval in ms to wait at least for next signal (not used for actions volume up/down)

        // Actions available. Use your own remote control and have a look at the console for "Command=0x??". E.g. "Protocol=NEC Address=0x17F Command=0x68 Repeat gap=39750us"
        // Make sure to define a hex-code not more than once as this will lead to a compile-error
        // https://forum.espuino.de/t/neues-feature-fernsteuerung-per-infrarot-fernbedienung/265
        #define RC_PLAY                     0x68            // command for play
        #define RC_PAUSE                    0x67            // command for pause
        #define RC_NEXT                     0x6b            // command for next track of playlist
        #define RC_PREVIOUS                 0x6a            // command for previous track of playlist
        #define RC_FIRST                    0x6c            // command for first track of playlist
        #define RC_LAST                     0x6d            // command for last track of playlist
        #define RC_VOL_UP                   0x1a            // Command for volume up (one step)
        #define RC_VOL_DOWN                 0x1b            // Command for volume down (one step)
        #define RC_MUTE                     0x1c            // Command to mute ESPuino
        #define RC_SHUTDOWN                 0x2a            // Command for deepsleep
        #define RC_BLUETOOTH                0x72            // Command to enable/disable bluetooth
        #define RC_FTP                      0x65            // Command to enable FTP-server
    #endif
#endif