#include <Arduino.h>
#include "settings.h"

#include "Log.h"
#include "Port.h"

#include <Wire.h>

// Infos:
// PCA9555 has 16 channels that are subdivided into 2 ports with 8 channels each.
// Every channels is represented by a bit.
// Examples for ESPuino-configuration:
// 100 => port 0 channel/bit 0
// 107 => port 0 channel/bit 7
// 108 => port 1 channel/bit 0
// 115 => port 1 channel/bit 7

#if defined(PE_PCA9555)
	#include <PCA9555.h>
	#define PE_SDA ext_IIC_DATA
	#define PE_SCL ext_IIC_CLK
TwoWire *pe_i2c = &i2cBusTwo;

PCA9555 expander(expanderAddress, pe_i2c);

static uint16_t Port_ExpanderPortsInputChannelStatus;
static uint16_t Port_ExpanderPortsOutputChannelStatus = 0xFFFF; // Stores current configuration of output-channels locally
static uint16_t Port_ExpanderPortsDirectionChannelStatus = 0xFFFF;

void Port_ExpanderHandler(void);
void Port_WriteInitMaskForOutputChannels(void);
void Port_MakeSomeChannelsOutputForShutdown(void);
void Port_Test(void);

	#if defined(PE_INTERRUPT_PIN) && (PE_INTERRUPT_PIN >= 0 && PE_INTERRUPT_PIN <= MAX_GPIO)
		#define PE_INTERRUPT_ENABLE
void IRAM_ATTR PORT_ExpanderISR(void);
bool Port_AllowReadFromPortExpander = false;
bool Port_AllowInitReadFromPortExpander = true;
	#endif

void Port_Init(void) {

	pe_i2c->setPins(PE_SDA, PE_SCL);
	pe_i2c->begin();
	#if defined(PE_RST)
	pinMode(PE_RST, OUTPUT);
	digitalWrite(PE_RST, HIGH);
	#endif

	Port_Test();
	expander.init();
	Port_CalculateMaskForOutputChannels();
	Port_WriteInitMaskForOutputChannels();
	#if defined(PE_INTERRUPT_ENABLE)
	Port_AllowReadFromPortExpander = true;
	#endif
	Port_ExpanderHandler();

	#ifdef PE_INTERRUPT_ENABLE
	pinMode(PE_INTERRUPT_PIN, INPUT_PULLUP);
	attachInterrupt(PE_INTERRUPT_PIN, PORT_ExpanderISR, FALLING);
	Log_Println(portExpanderInterruptEnabled, LOGLEVEL_NOTICE);
	#endif
}

void Port_Cyclic(void) {
	Port_ExpanderHandler();
}

// Wrapper: reads from GPIOs (via digitalRead()) or from port-expander (if enabled)
// Behaviour like digitalRead(): returns true if not pressed and false if pressed
bool Port_Read(const uint8_t _channel) {
	switch (_channel) {
		case 0 ... 39: // GPIO
			return digitalRead(_channel);

	#ifdef PORT_EXPANDER_ENABLE
		case 100 ... 115: // Port-expander (port 0)
			return (Port_ExpanderPortsInputChannelStatus & (1 << (_channel - 100))); // Remove offset 100 (return false if pressed)
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
			Log_Println(paOn, LOGLEVEL_NOTICE);
		} else {
			Log_Println(paOff, LOGLEVEL_NOTICE);
		}
	}
	#endif

	#ifdef GPIO_HP_EN
	if (_channel == GPIO_HP_EN) {
		if (_newState) {
			Log_Println(hpOn, LOGLEVEL_NOTICE);
		} else {
			Log_Println(hpOff, LOGLEVEL_NOTICE);
		}
	}
	#endif

	if (_initGpio) {
		switch (_channel) {
			case 0 ... 39: { // GPIO
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
		case 0 ... 39: { // GPIO
			digitalWrite(_channel, _newState);
			break;
		}

	#ifdef PORT_EXPANDER_ENABLE
		case 100 ... 115: {
			if (_newState) {
				expander.digitalWrite(_channel - 100, 1);
			} else {
				expander.digitalWrite(_channel - 100, 0);
			}
			uint8_t err = expander.read(Port_ExpanderPortsOutputChannelStatus);
			if (err != 0) {
				Log_Println("Port_Write read failed", LOGLEVEL_ERROR);
			}
			break;
		}
	#endif

		default: {
			break;
		}
	}
}

void Port_CalculateMaskForOutputChannels(void) {
	int8_t shift;

	#ifdef HP_DETECT
	shift = HP_DETECT - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannelStatus &= ~(1 << (shift));
	}
	#endif

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
		expander.writeRegister(PCA9555Register::CONFIG_A, lowByte(Port_ExpanderPortsDirectionChannelStatus), highByte(Port_ExpanderPortsDirectionChannelStatus));
		expander.write(0x0000);
	}
}

// Some channels are configured as output before shutdown in order to avoid unwanted interrupts while ESP32 sleeps
void Port_MakeSomeChannelsOutputForShutdown(void) {
	int8_t shift;
	const uint16_t portBaseValueBitMask = 0xFFFF;
	uint16_t Port_ExpanderPortsDirectionChannel = 0x0000;

	#ifdef HP_DETECT
	shift = HP_DETECT - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel &= ~(1 << (shift));
	}
	#endif

	#ifdef GPIO_PA_EN
	shift = GPIO_PA_EN - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel &= ~(1 << (shift));
	}
	#endif

	#ifdef GPIO_HP_EN
	shift = GPIO_HP_EN - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel &= ~(1 << (shift));
	}
	#endif

	#ifdef POWER
	shift = POWER - 100;
	if (shift >= 0 && shift <= 15) {
		#ifdef INVERT_POWER
		Port_ExpanderPortsDirectionChannel |= (1 << (shift));
		#else
		Port_ExpanderPortsDirectionChannel &= ~(1 << (shift));
		#endif
	}
	#endif

	#ifdef GPIO_LED0
	shift = GPIO_LED0 - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel &= ~(1 << (shift));
	}
	#endif

	#ifdef GPIO_LED1
	shift = GPIO_LED1 - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel &= ~(1 << (shift));
	}
	#endif

	#ifdef GPIO_LED2
	shift = GPIO_LED2 - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel &= ~(1 << (shift));
	}
	#endif

	#ifdef GPIO_LED3
	shift = GPIO_LED3 - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel &= ~(1 << (shift));
	}
	#endif

	#ifdef GPIO_LED4
	shift = GPIO_LED4 - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel &= ~(1 << (shift));
	}
	#endif

	#ifdef GPIO_LED5
	shift = GPIO_LED5 - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel &= ~(1 << (shift));
	}
	#endif

	#ifdef BUTTON_0
	shift = BUTTON_0 - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel |= (1 << (shift));
	}
	#endif

	#ifdef BUTTON_1
	shift = BUTTON_1 - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel |= (1 << (shift));
	}
	#endif

	#ifdef BUTTON_2
	shift = BUTTON_2 - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel |= (1 << (shift));
	}
	#endif

	#ifdef BUTTON_3
	shift = BUTTON_3 - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel |= (1 << (shift));
	}
	#endif

	#ifdef BUTTON_4
	shift = BUTTON_4 - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel |= (1 << (shift));
	}
	#endif

	#ifdef BUTTON_5
	shift = BUTTON_5 - 100;
	if (shift >= 0 && shift <= 15) {
		Port_ExpanderPortsDirectionChannel |= (1 << (shift));
	}
	#endif

	// Only change port-config if necessary (at least bitmask changed from base-default for one port)
	if ((Port_ExpanderPortsDirectionChannel != portBaseValueBitMask)) {
		expander.writeRegister(PCA9555Register::CONFIG_A, lowByte(Port_ExpanderPortsDirectionChannel), highByte(Port_ExpanderPortsDirectionChannel));
		expander.write(0x0000);
	}
}

// Reads input from port-expander and writes output into global array
// Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf
void Port_ExpanderHandler(void) {
	static uint16_t inputChanged = 0x0000; // Used to debounce once in case of register-change
	static uint16_t inputPrev = 0x0000;
	// If interrupt-handling is active, only ready port-expander's register if interrupt was fired
	#ifdef PE_INTERRUPT_ENABLE
	if (Port_AllowReadFromPortExpander) {
		Port_AllowReadFromPortExpander = false;
	} else if (Port_AllowInitReadFromPortExpander) {
		Port_AllowInitReadFromPortExpander = false;
		Port_AllowReadFromPortExpander = false;
	} else if (!inputChanged) {
		return;
	}
	#endif

	uint16_t read;
	uint8_t err = expander.read(read);
	if (err != 0) {
		Log_Println("Port_ExpanderHandler read failed", LOGLEVEL_ERROR);
	#ifdef PE_INTERRUPT_ENABLE
		Port_AllowReadFromPortExpander = true;
	#endif
		return;
	}
	uint16_t inputCurr = read & Port_ExpanderPortsDirectionChannelStatus;
	inputChanged = inputPrev ^ inputCurr;
	uint16_t inputStable = Port_ExpanderPortsInputChannelStatus;
	inputStable &= inputChanged;
	inputStable |= (~inputChanged & inputCurr);
	Port_ExpanderPortsInputChannelStatus = inputStable;
	inputPrev = inputCurr;
	#ifdef PE_INTERRUPT_ENABLE
	// input is stable; go back to interrupt mode
	if (!inputChanged) {
		attachInterrupt(digitalPinToInterrupt(PE_INTERRUPT_PIN), PORT_ExpanderISR, ONLOW);
	}
	#endif
}

// Make sure ports are read finally at shutdown in order to clear any active IRQs that could cause re-wakeup immediately
void Port_Exit(void) {
	Port_MakeSomeChannelsOutputForShutdown();
	uint8_t err = expander.read(Port_ExpanderPortsInputChannelStatus);
	if (err != 0) {
		Log_Println("Port_ExpanderHandler read failed", LOGLEVEL_ERROR);
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

	#ifdef PE_INTERRUPT_ENABLE
		#if (defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR < 3))
void IRAM_ATTR PORT_ExpanderISR(void) {
		#else
void PORT_ExpanderISR(void) {
		#endif

	// check if the interrupt pin is actually low and only if it is
	// trigger the handler (there are a lot of false calls to this ISR
	// where the interrupt pin isn't low...)
	Port_AllowReadFromPortExpander = !digitalRead(PE_INTERRUPT_PIN);

	// until the interrupt is handled we don't need any more ISR calls
	if (Port_AllowReadFromPortExpander) {
		detachInterrupt(digitalPinToInterrupt(PE_INTERRUPT_PIN));
	}
}
	#endif
#endif
