#include <Arduino.h>
#include "settings.h"
#include "Port.h"
#include "Log.h"
#include <Wire.h>

// Infos:
// PCA9555 has 16 channels that are subdivided into 2 ports with 8 channels each.
// Every channels is represented by a bit.
// Examples for ESPuino-configuration:
// 100 => port 0 channel/bit 0
// 107 => port 0 channel/bit 7
// 108 => port 1 channel/bit 0
// 115 => port 1 channel/bit 7

#ifdef PE_MCP23017
#include <MCP23017.h>
#define PE_SDA  ext_IIC_DATA
#define PE_SCL  ext_IIC_CLK
TwoWire *pe_i2c = &i2c;

MCP23017 expander(expanderAddress, pe_i2c);

static uint16_t Port_ExpanderPortsInputChannelStatus;
static uint16_t Port_ExpanderPortsOutputChannelStatus = 0xFFFF;  // Stores current configuration of output-channels locally
static uint16_t Port_ExpanderPortsDirectionChannelStatus = 0x00FF;
void Port_ExpanderHandler(void);
uint8_t Port_ChannelToBit(const uint8_t _channel);
void Port_CalculateMaskForOutputChannels(void);
void Port_WriteInitMaskForOutputChannels(void);
void Port_Test(void);

#if defined(PE_INTERRUPT_PIN) && (PE_INTERRUPT_PIN >= 0 && PE_INTERRUPT_PIN <= MAX_GPIO)
    #define PE_INTERRUPT_ENABLE
    void IRAM_ATTR PORT_ExpanderISR(void);
    bool Port_AllowReadFromPortExpander = false;
    bool Port_AllowInitReadFromPortExpander = true;
#endif

void Port_Init(void) {

    expander.begin(PE_SDA, PE_SCL);
#if defined(PE_RST)
    pinMode(PE_RST, OUTPUT);
    digitalWrite(PE_RST, HIGH);
#endif
    
    Port_Test();
    expander.init();
    Port_CalculateMaskForOutputChannels();

#if defined(PE_INTERRUPT_ENABLE)
    pinMode(PE_INTERRUPT_PIN, INPUT_PULLUP);
    expander.interruptMode(MCP23X17InterruptMode::Or);
    expander.interrupt(MCP23X17Port::A, FALLING);
    expander.read();
    expander.clearInterrupts();
    attachInterrupt(digitalPinToInterrupt(PE_INTERRUPT_PIN), PORT_ExpanderISR, FALLING);
    Log_Println(portExpanderInterruptEnabled, LOGLEVEL_NOTICE);
#elif defined(WAKEUP_BUTTON)
    expander.interruptMode(MCP23X17InterruptMode::Or);
    expander.interrupt(MCP23X17Port::A, FALLING);
#endif


    Port_WriteInitMaskForOutputChannels();

// If automatic HP-detection is not used...
#ifndef HEADPHONE_ADJUST_ENABLE
    #ifdef GPIO_PA_EN
    Port_Write(GPIO_PA_EN, true, true);       // ...but it's necessary to enable loudspeaker amp...
    #endif
    #ifdef GPIO_HP_EN
    Port_Write(GPIO_HP_EN, true, true);      // ...or headphones-amp
    #endif
#endif
}

void Port_Cyclic(void) {
    Port_ExpanderHandler();
}

// Wrapper: reads from GPIOs (via digitalRead()) or from port-expander (if enabled)
// Behaviour like digitalRead(): returns true if not pressed and false if pressed
bool Port_Read(const uint8_t _channel) {
    switch (_channel) {
        case 0 ... MAX_GPIO: // GPIO
            return digitalRead(_channel);
#ifdef PORT_EXPANDER_ENABLE
        case 100 ... 115: // Port-expander (port 0)
            return (Port_ExpanderPortsInputChannelStatus & (1 << (_channel - 100)));       // Remove offset 100 (return false if pressed)
#endif
        default: // Everything else (doesn't make sense at all) isn't supposed to be pressed
            return true;
    }
}

// Wrapper-function to reverse detection of connected headphones.
// Normally headphones are supposed to be plugged in if a given GPIO/channel is LOW/false.
bool Port_Detect_Mode_HP(bool _state) {
    #ifndef DETECT_HP_ON_HIGH
        return _state;
    #else
        return !_state;
    #endif
}

// Configures OUTPUT-mode for GPIOs (non port-expander)
// Output-mode for port-channels is done via Port_WriteInitMaskForOutputChannels()
void Port_Write(const uint8_t _channel, const bool _newState, const bool _initGpio) {
    if (_initGpio) {
        switch (_channel) {
            case 0 ... MAX_GPIO: { // GPIO
                pinMode(_channel, OUTPUT);
                break;
            }
#ifdef PORT_EXPANDER_ENABLE
            case 100 ... 115: {
                expander.pinMode(_channel, OUTPUT);
                break;
            }
#endif
            default: {
                break;
            }
        }
    }
    Port_Write(_channel, _newState);
}

// Wrapper: writes to GPIOs (via digitalWrite()) or to port-expander (if enabled)
void Port_Write(const uint8_t _channel, const bool _newState) {
    switch (_channel) {
        case 0 ... MAX_GPIO: { // GPIO
            digitalWrite(_channel, _newState);
            break;
        }
#ifdef PORT_EXPANDER_ENABLE
        case 100 ... 115: {
            expander.digitalWrite(_channel-100, _newState);
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

void Port_CalculateMaskForOutputChannels(void) {
    int8_t shift;
    #ifdef GPIO_PA_EN
        shift = GPIO_PA_EN - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus &= ~(1 << (shift));
        }
    #endif

    #ifdef GPIO_HP_EN
        shift = GPIO_HP_EN - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus &= ~(1 << (shift));
        }
    #endif

    #ifdef POWER
        shift = POWER - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus &= ~(1 << (shift));
        }
    #endif

    #ifdef BUTTONS_LED
        shift = BUTTONS_LED - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus &= ~(1 << (shift));
        }
    #endif

    #ifdef GPIO_LED0
        shift = GPIO_LED0 - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus &= ~(1 << (shift));
        }
    #endif

    #ifdef GPIO_LED1
        shift = GPIO_LED1 - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus &= ~(1 << (shift));
        }
    #endif

    #ifdef GPIO_LED2
        shift = GPIO_LED2 - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus &= ~(1 << (shift));
        }
    #endif

    #ifdef GPIO_LED3
        shift = GPIO_LED3 - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus &= ~(1 << (shift));
        }
    #endif

    #ifdef GPIO_LED4
        shift = GPIO_LED4 - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus &= ~(1 << (shift));
        }
    #endif

    #ifdef GPIO_LED5
        shift = GPIO_LED5 - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus &= ~(1 << (shift));
        }
    #endif

    #ifdef BUTTON_0
        shift = BUTTON_0 - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus |= (1 << (shift));
        }
    #endif 

    #ifdef BUTTON_1
        shift = BUTTON_1 - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus |= (1 << (shift));
        }
    #endif

    #ifdef BUTTON_2
        shift = BUTTON_2 - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus |= (1 << (shift));
        }
    #endif

    #ifdef BUTTON_3
        shift = BUTTON_3 - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus |= (1 << (shift));
        }
    #endif

    #ifdef BUTTON_4
        shift = BUTTON_4 - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus |= (1 << (shift));
        }
    #endif

    #ifdef BUTTON_5
        shift = BUTTON_5 - 100;
        if (shift >= 0 && shift <= 15) {
            Port_ExpanderPortsDirectionChannelStatus |= (1 << (shift));
        }
    #endif
}

// Writes initial port-configuration (I/O) for port-expander PCA9555
// If no output-channel is necessary, nothing has to be configured as all channels are in input-mode as per default (255)
// So every bit representing an output-channel needs to be set to 0.
void Port_WriteInitMaskForOutputChannels(void) {
    const uint16_t portBaseValueBitMask = 0xFFFF;

    // Only change port-config if necessary (at least bitmask changed from base-default for one port)
    if ((Port_ExpanderPortsDirectionChannelStatus != portBaseValueBitMask)) {
        expander.portMode(MCP23X17Port::A, lowByte(Port_ExpanderPortsDirectionChannelStatus));
        expander.portMode(MCP23X17Port::B, highByte(Port_ExpanderPortsDirectionChannelStatus));
        expander.write(0x0000);
    }
}

// Reads input from port-expander and writes output into global array
// Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf
void Port_ExpanderHandler(void) {
// If interrupt-handling is active, only ready port-expander's register if interrupt was fired
#ifdef PE_INTERRUPT_ENABLE
    if (!Port_AllowReadFromPortExpander && !Port_AllowInitReadFromPortExpander) {
        //Serial.println("Interrupt false!");
        return;
    } else if (Port_AllowInitReadFromPortExpander) {
        Log_Println("Clear Interrupt", LOGLEVEL_NOTICE);
        Port_ExpanderPortsInputChannelStatus = expander.read();
        expander.clearInterrupts();
        Port_AllowInitReadFromPortExpander = false;
    } else if (Port_AllowReadFromPortExpander || Port_AllowInitReadFromPortExpander) {
        //Serial.println("Interrupt true!");
        Log_Println("Clear Interrupt", LOGLEVEL_NOTICE);
        Port_ExpanderPortsInputChannelStatus = expander.read();
        expander.clearInterrupts();
        Port_AllowReadFromPortExpander = false;
    }
#else
    Port_ExpanderPortsInputChannelStatus = expander.read();
#endif
}

// Make sure ports are read finally at shutdown in order to clear any active IRQs that could cause re-wakeup immediately
void Port_Exit(void) {
    Port_ExpanderPortsInputChannelStatus = expander.read();
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

#ifdef PE_INTERRUPT_ENABLE
void PORT_ExpanderISR(void) {
    Log_Println("Expander Interrupt", LOGLEVEL_NOTICE);
    Port_AllowReadFromPortExpander = true;
}
#endif

#endif