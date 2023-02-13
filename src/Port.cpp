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


#if !defined(PORT_EXPANDER_ENABLE)
    void Port_Init(void) {}
    void Port_Cyclic(void) {}
    bool Port_Read(const uint8_t _channel) {}

	uint8_t Port_ExpanderPortsInputChannelStatus[2];
	static uint8_t Port_ExpanderPortsOutputChannelStatus[2] = {255, 255};          // Stores current configuration of output-channels locally
	void Port_ExpanderHandler(void);
	uint8_t Port_ChannelToBit(const uint8_t _channel);
	void Port_WriteInitMaskForOutputChannels(void);
	void Port_Test(void);

	#if (PE_INTERRUPT_PIN >= 0 && PE_INTERRUPT_PIN <= MAX_GPIO)
		#define PE_INTERRUPT_PIN_ENABLE
		void IRAM_ATTR PORT_ExpanderISR(void);
		bool Port_AllowReadFromPortExpander = false;
		bool Port_AllowInitReadFromPortExpander = true;
	#endif
#endif
