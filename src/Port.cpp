#include "settings.h"
#include "Port.h"

// Infos:
// PCA9555 has 16 channels that are subdivided into 2 ports with 8 channels each.
// Every channels is represented by a bit.
// Examples for ESPuino-configuration:
// 100 => port 0 channel/bit 0
// 107 => port 0 channel/bit 7
// 108 => port 1 channel/bit 0
// 115 => port 1 channel/bit 7

#if !defined(PE_MCP23017) && !defined(PE_PCA9555)
    void Port_Init(void) {}
    void Port_Cyclic(void) {}
    bool Port_Read(const uint8_t _channel) {}

    // Wrapper-function to reverse detection of connected headphones.
    // Normally headphones are supposed to be plugged in if a given GPIO/channel is LOW/false.
    bool Port_Detect_Mode_HP(bool _state) {}

    // Configures OUTPUT-mode for GPIOs (non port-expander)
    // Output-mode for port-channels is done via Port_WriteInitMaskForOutputChannels()
    void Port_Write(const uint8_t _channel, const bool _newState, const bool _initGpio) {}

    // Wrapper: writes to GPIOs (via digitalWrite()) or to port-expander (if enabled)
    void Port_Write(const uint8_t _channel, const bool _newState) {}

    // Translates digitalWrite-style "GPIO" to bit
    uint8_t Port_ChannelToBit(const uint8_t _channel) {}

    // Writes initial port-configuration (I/O) for port-expander PCA9555
    // If no output-channel is necessary, nothing has to be configured as all channels are in input-mode as per default (255)
    // So every bit representing an output-channel needs to be set to 0.
    void Port_WriteInitMaskForOutputChannels(void) {}

    // Reads input from port-expander and writes output into global array
    // Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf
    void Port_ExpanderHandler(void) {}

    // Make sure ports are read finally at shutdown in order to clear any active IRQs that could cause re-wakeup immediately
    void Port_Exit(void) {}

    // Tests if port-expander can be detected at address configured
    void Port_Test(void) {}

#endif