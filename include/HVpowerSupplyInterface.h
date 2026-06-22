//
// HVpowerSupplyInterface.h — application header
// Target: Adafruit ItsyBitsy M0 (SAMD21G18A)
//
#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------------

// Digital outputs
#define POWER   3   // HIGH = 24 VDC rail enabled to power supply
#define POL     7   // Polarity relay: LOW = positive output, HIGH = negative
#define POSPWM  13  // PWM positive channel — TCC2/WO[1]
#define NEGPWM  12  // PWM negative channel — TCC0/WO[3]

// Digital inputs — relay feedback: HIGH when that polarity mode is active
#define POS     A1  // Positive-mode sense input
#define NEG     A0  // Negative-mode sense input

// Analog inputs
#define IMON    A3  // Current monitor ADC input
#define VMON    A2  // Voltage monitor ADC input

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

#define DeviceID   0x81         // I²C device identity byte
#define SIGNATURE  0xAA55A5A5u  // Flash data validity marker (must match on restore)
#define ESC        27           // ASCII Escape — exits the TWI_SERIAL tunnel mode

// ---------------------------------------------------------------------------
// I²C command byte definitions
//
// Commands 0x00–0x7F are set (write) operations.
// Commands 0x80–0xFF are get (read) operations.
// ---------------------------------------------------------------------------
#if UseWireClient

// Special protocol commands
#define TWI_SERIAL          0x27  // Enter transparent ASCII-serial-tunnel mode
                                  // (exit by sending ESC)
#define TWI_CMD             0x7F  // Execute one newline-terminated ASCII command;
                                  // result available on next master read
#define TWI_READ_AVALIBLE   0x82  // Next request event returns available-byte count
                                  // as a 16-bit little-endian unsigned integer
#define TWI_READ_ID         0xF0  // Returns the DeviceID byte

// Set commands (master → device)
#define TWI_SET_Power       0x01  // bool   — true = power on
#define TWI_SET_Voltage     0x03  // float  — output voltage setpoint (V)
#define TWI_SET_Pol         0x04  // bool   — true = positive polarity
#define TWI_SET_Range       0x05  // float  — full-scale voltage range (V, ± for bipolar)
#define TWI_SET_MaxV        0x06  // float  — maximum voltage limit (V)
#define TWI_SET_MaxI        0x07  // float  — maximum current limit (mA)
#define TWI_SET_Save        0x08  // (none) — save current settings to flash
#define TWI_SET_Restore     0x09  // (none) — restore settings from flash
#define TWI_SET_Reversable  0x0C  // bool   — true = bipolar output enabled
#define TWI_SET_CurEnable   0x0D  // bool   — true = current-limit trip active

// Get commands (device → master)
#define TWI_GET_Power       0x81  // bool
#define TWI_GET_Voltage     0x83  // float (V)
#define TWI_GET_Pol         0x84  // bool
#define TWI_GET_Range       0x85  // float (V)
#define TWI_GET_MaxV        0x86  // float (V)
#define TWI_GET_MaxI        0x87  // float (mA)
#define TWI_GET_Reversable  0x8C  // bool
#define TWI_GET_CurEnable   0x8D  // bool

// Readback get commands — return measured hardware values
#define TWI_GET_Vrb         0x88  // float — measured output voltage (V)
#define TWI_GET_Irb         0x89  // float — measured output current (mA)
#define TWI_GET_Prb         0x8A  // bool  — measured output polarity

#endif // UseWireClient

// ---------------------------------------------------------------------------
// Persistent data record
//
// Stored in internal flash via FlashStorage.  The Signature field guards
// against loading stale or uninitialised flash after a firmware update.
// Size records the struct length at save time for future layout-change
// detection (currently informational — the Signature check is the primary
// validity guard).
// ---------------------------------------------------------------------------
typedef struct
{
    int16_t      Size;          // sizeof(Data) at the time of the last save
    char         Name[20];      // Human-readable board name
    int8_t       Rev;           // Board revision number

    #if UseWireClient
    int          TWIadd;        // I²C peripheral bus address
    #endif

    // Output control parameters
    bool         power;         // true = 24 V rail enabled
    bool         polarity;      // true = positive, false = negative
    float        setpoint;      // commanded output voltage (V)
    float        maxVoltage;    // software voltage ceiling (V)
    float        maxCurrent;    // software current trip threshold (mA)

    // Range / hardware configuration
    float        range;         // full-scale voltage range (V);
                                // sign indicates default polarity direction
    bool         reversable;    // true = bipolar output fitted (POL relay present)
    bool         curMonEnabled; // true = current-limit protection active

    // ADC/DAC calibration — field meanings defined by Devices library
    ADCchan      Vmonitor;      // voltage monitor ADC calibration
    ADCchan      Imonitor;      // current monitor ADC calibration
    DACchan      Vcontrol;      // voltage control DAC calibration

    unsigned int Signature;     // must equal SIGNATURE for record to be valid
} Data;

// ---------------------------------------------------------------------------
// Function declarations
// ---------------------------------------------------------------------------

// Flash persistence (internal FlashStorage)
void SaveSettings(void);    // command handler: save settings + send ACK
void RestoreSettings(void); // command handler: restore settings + send ACK or NAK

// UF2 bootloader entry
void bootloader(void);

// SPI flash filesystem (FAT volume on external flash)
void saveDefaults(void);
void loadDefaults(void);
void saveCalibrations(void);
void loadCalibrations(void);

// I²C address configuration
#if UseWireClient
void setTWIaddress(void);
void getTWIaddress(void);
#endif

// Power and polarity command handlers
void powerON(void);
void powerOFF(void);
void setPolarity(void);
void getPolarity(void);
void getPolarityRB(void);
