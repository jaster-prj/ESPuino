#include <Arduino.h>
#include "settings.h"
#include "Port.h"
#include "Log.h"

// Infos:
// PCA9555 has 16 channels that are subdivided into 2 ports with 8 channels each.
// Every channels is represented by a bit.
// Examples for ESPuino-configuration:
// 100 => port 0 channel/bit 0
// 107 => port 0 channel/bit 7
// 108 => port 1 channel/bit 0
// 115 => port 1 channel/bit 7

#ifdef PE_PCA9555
#include <PCA9555.h>
PCA9555 expander(expanderI2cAddress);

uint8_t Port_ExpanderPortsInputChannelStatus[2];
static uint8_t Port_ExpanderPortsOutputChannelStatus[2] = {255, 255}; 
void Port_ExpanderHandler(void);
uint8_t Port_ChannelToBit(const uint8_t _channel);
void Port_WriteInitMaskForOutputChannels(void);
void Port_Test(void);

#if (PE_INTERRUPT_PIN >= 0 && PE_INTERRUPT_PIN <= 39)
    #define PE_INTERRUPT_PIN_ENABLE
    void IRAM_ATTR PORT_ExpanderISR(void);
    bool Port_AllowReadFromPortExpander = false;
    bool Port_AllowInitReadFromPortExpander = true;
#endif

void Port_Init(void) {
    #ifdef PORT_EXPANDER_ENABLE
        Port_Test();
        Port_WriteInitMaskForOutputChannels();
        Port_ExpanderHandler();
    #endif

    #ifdef PE_INTERRUPT_PIN_ENABLE
        pinMode(PE_INTERRUPT_PIN, INPUT_PULLUP);
        attachInterrupt(PE_INTERRUPT_PIN, PORT_ExpanderISR, FALLING);
        Log_Println(portExpanderInterruptEnabled, LOGLEVEL_NOTICE);
    #endif
}

void Port_Cyclic(void) {
    #ifdef PORT_EXPANDER_ENABLE
        Port_ExpanderHandler();
    #endif
}

// Wrapper: reads from GPIOs (via digitalRead()) or from port-expander (if enabled)
// Behaviour like digitalRead(): returns true if not pressed and false if pressed
bool Port_Read(const uint8_t _channel) {
    switch (_channel) {
        case 0 ... 39: // GPIO
            return digitalRead(_channel);

        #ifdef PORT_EXPANDER_ENABLE
            case 100 ... 115: // Port-expander (port 0)
                return (Port_ExpanderPortsInputChannelStatus & (1 << (_channel - 100)));       // Remove offset 100 (return false if pressed)
        #endif

        default: // Everything else (doesn't make sense at all) isn't supposed to be pressed
            return true;
    }
}

// Configures OUTPUT-mode for GPIOs (non port-expander)
// Output-mode for port-channels is done via Port_WriteInitMaskForOutputChannels()
void Port_Write(const uint8_t _channel, const bool _newState, const bool _initGpio) {
    #ifdef GPIO_PA_EN
        if (_channel == GPIO_PA_EN) {
            if (_newState) {
                Log_Println((char *) FPSTR(paOn), LOGLEVEL_NOTICE);
            } else {
                Log_Println((char *) FPSTR(paOff), LOGLEVEL_NOTICE);
            }
        }
    #endif

    #ifdef GPIO_HP_EN
        if (_channel == GPIO_HP_EN) {
            if (_newState) {
                Log_Println((char *) FPSTR(hpOn), LOGLEVEL_NOTICE);
            } else {
                Log_Println((char *) FPSTR(hpOff), LOGLEVEL_NOTICE);
            }
        }
    #endif

    if (_initGpio) {
        if (_channel >= 0 && _channel <= 39) {
            pinMode(_channel, OUTPUT);
        }
    }
    Port_Write(_channel, _newState);
}

// Wrapper: writes to GPIOs (via digitalWrite()) or to port-expander (if enabled)
void Port_Write(const uint8_t _channel, const bool _newState) {
    switch (_channel) {
        case 0 ... 39: { // GPIO
            digitalWrite(_channel, _newState);
            break;
        }

        #ifdef PORT_EXPANDER_ENABLE
            case 100 ... 115: {
                if (_newState) {
                    expander.digitalWrite(_channel-100, 1);
                } else {
                    expander.digitalWrite(_channel-100, 0);
                }
                Port_ExpanderPortsOutputChannelStatus = expander.read();
                break;
            }
        #endif

        default: {
            break;
        }
    }
}

// Translates digitalWrite-style "GPIO" to bit
uint8_t Port_ChannelToBit(const uint8_t _channel) {
    switch (_channel) {
        case 100:
        case 108:
            return 0;
            break;
        case 101:
        case 109:
            return 1;
            break;
        case 102:
        case 110:
            return 2;
            break;
        case 103:
        case 111:
            return 3;
            break;
        case 104:
        case 112:
            return 4;
            break;
        case 105:
        case 113:
            return 5;
            break;
        case 106:
        case 114:
            return 6;
            break;
        case 107:
        case 115:
            return 7;
            break;

        default:
            return 255;       // not valid!
    }
}

// Writes initial port-configuration (I/O) for port-expander PCA9555
// If no output-channel is necessary, nothing has to be configured as all channels are in input-mode as per default (255)
// So every bit representing an output-channel needs to be set to 0.
void Port_WriteInitMaskForOutputChannels(void) {
    const uint16_t portBaseValueBitMask = 0xFFFF;
    uint16_t OutputBitMaskAsPerPort = 0xFFFF;

    #ifdef GPIO_PA_EN
        if (GPIO_PA_EN >= 100 && GPIO_PA_EN <= 115) {
            // Bits of channels to be configured as input are 1 by default.
            // So in order to change I/O-direction to output we need to set those bits to 0.
            OutputBitMaskAsPerPort &= ~(1 << (GPIO_PA_EN-100));
            //Serial.printf("PA LO: %u\n", OutputBitMaskAsPerPort);
        }
    #endif

    #ifdef GPIO_HP_EN
        if (GPIO_HP_EN >= 100 && GPIO_HP_EN <= 107) {
            OutputBitMaskAsPerPort &= ~(1 << (GPIO_HP_EN-100));
            //Serial.printf("HP LO: %u\n", OutputBitMaskAsPerPort);
        }
    #endif

    // Only change port-config if necessary (at least bitmask changed from base-default for one port)
    if ((OutputBitMaskAsPerPort != portBaseValueBitMask)) {
        expander.writeRegister(PCA9555Register::CONFIG_A, lowByte(OutputBitMaskAsPerPort), highByte(OutputBitMaskAsPerPort));
        expander.write(0x0000);
    }
}

// Reads input from port-expander and writes output into global array
// Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf
void Port_ExpanderHandler(void) {
    // If interrupt-handling is active, only ready port-expander's register if interrupt was fired
    #ifdef PE_INTERRUPT_PIN_ENABLE
        if (!Port_AllowReadFromPortExpander && !Port_AllowInitReadFromPortExpander) {
            //Serial.println("Interrupt false!");
            return;
        } else if (Port_AllowInitReadFromPortExpander) {
            Port_AllowInitReadFromPortExpander = false;
        } else if (Port_AllowReadFromPortExpander || Port_AllowInitReadFromPortExpander) {
            //Serial.println("Interrupt true!");
            Port_AllowReadFromPortExpander = false;
        }
    #endif

    i2cBusTwo.beginTransmission(expanderI2cAddress);
    for (uint8_t i = 0; i < 2; i++) {
        i2cBusTwo.write(0x00 + i);                      // Pointer to input-register...
        i2cBusTwo.endTransmission();
        i2cBusTwo.requestFrom(expanderI2cAddress, 1u);   // ...and read its byte

        if (i2cBusTwo.available()) {
            Port_ExpanderPortsInputChannelStatus[i] = i2cBusTwo.read();
        }
    }
}

// Make sure ports are read finally at shutdown in order to clear any active IRQs that could cause re-wakeup immediately
void Port_Exit(void) {
    i2cBusTwo.beginTransmission(expanderI2cAddress);
    for (uint8_t i = 0; i < 2; i++) {
        i2cBusTwo.write(0x00 + i);                      // Pointer to input-register...
        i2cBusTwo.endTransmission();
        i2cBusTwo.requestFrom(expanderI2cAddress, 1u);   // ...and read its byte

        if (i2cBusTwo.available()) {
            Port_ExpanderPortsInputChannelStatus[i] = i2cBusTwo.read();
        }
    }
}

// Tests if port-expander can be detected at address configured
void Port_Test(void) {
    int error = expander.checkAck();
    if (!error) {
        Log_Println(portExpanderFound, LOGLEVEL_NOTICE);
    } else {
        Log_Println(portExpanderNotFound, LOGLEVEL_ERROR);
    }
}

#ifdef PE_INTERRUPT_PIN_ENABLE
    void IRAM_ATTR PORT_ExpanderISR(void) {
        Port_AllowReadFromPortExpander = true;
    }
#endif

#endif