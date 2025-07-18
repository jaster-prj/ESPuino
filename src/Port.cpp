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

// Wrapper-function to reverse detection of connected headphones.
// Normally headphones are supposed to be plugged in if a given GPIO/channel is LOW/false.
bool Port_Detect_Mode_HP(bool _state) {
#ifndef DETECT_HP_ON_HIGH
	return _state;
#else
	return !_state;
#endif
}

#if !defined(PORT_EXPANDER_ENABLE)

void Port_Init(void) {
}

void Port_Cyclic(void) {
}

bool Port_Read(const uint8_t _channel) {
}

static uint16_t Port_ExpanderPortsInputChannelStatus;
static uint16_t Port_ExpanderPortsOutputChannelStatus = 0xFFFF; // Stores current configuration of output-channels locally
static uint16_t Port_ExpanderPortsDirectionChannelStatus = 0xFFFF;
void Port_ExpanderHandler(void);
void Port_WriteInitMaskForOutputChannels(void);
void Port_Test(void);

	#if (PE_INTERRUPT_PIN >= 0 && PE_INTERRUPT_PIN <= MAX_GPIO)
		#define PE_INTERRUPT_PIN_ENABLE
void IRAM_ATTR PORT_ExpanderISR(void);
bool Port_AllowReadFromPortExpander = false;
	#endif
#endif
