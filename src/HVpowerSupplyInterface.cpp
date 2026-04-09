//
// HVpowerSupplyInterface.cpp
//
// Target:  Adafruit ItsyBitsy M0 (SAMD21G18A, 48 MHz, 256 KB flash, 32 KB RAM)
//
// Analog output architecture
// --------------------------
// Two complementary PWM channels (0 … PWM_PERIOD-1 counts) produce a single
// bipolar ±10 V control signal for the HV power supply:
//   Pin 13 (POSPWM) — TCC2/WO[1] — active for positive counts
//   Pin 12 (NEGPWM) — TCC0/WO[3] — active for negative counts (inverted)
// Only one channel carries a non-zero duty at any time.
//
// Host interface
// --------------
// ASCII commands over USB-CDC serial and, when UseWireClient is true, via
// I²C as a peripheral device at address data.TWIadd.
//
// MIPS host TWI command string format: AABB…BBTT
//   AA  — wire channel (LS 4 bits) and board address (MS 4 bits)
//   BB… — TWI command byte array
//   TT  — value type: 0=bool 1=int 2=char 3=byte 4=word 5=float
//

// ---------------------------------------------------------------------------
// Feature flags — defined before any #include that tests them
// ---------------------------------------------------------------------------
#define AppName       "HVpsIntr"
#define UseWireClient true   // enable I²C peripheral interface
#define UseThreads    true   // enable cooperative-thread update loop
#define UseSPIflash   true   // enable SPI-flash FAT filesystem for settings

// ---------------------------------------------------------------------------
// Includes — system and framework
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>

// GAACE library
#include <RingBuffer.h>
#include <commandProcessor.h>
#include <charAllocate.h>
#include <debug.h>
#include <Errors.h>
#include <Devices.h>

// Internal flash storage (erased on every firmware upload)
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>

// Status LED
#include <Adafruit_DotStar.h>

// Cooperative thread scheduler
#if UseThreads
#include <Thread.h>
#include <ThreadController.h>
#endif

// External SPI flash filesystem
#if UseSPIflash
#include <Adafruit_SPIFlash.h>
#include <FlashFS/flash_config.h>
#endif

// I²C peripheral helper
#if UseWireClient
#include <WireHelper.h>
#endif

// Project headers
#include "HVpowerSupplyInterface.h"
#include "Timer.h"

// ---------------------------------------------------------------------------
// Forward declarations — functions defined later in this file
// ---------------------------------------------------------------------------
void  statusLED(uint32_t color);
float analogInput(int pin, int num);
void  analogOutput(int count);
void  saveSettings(void);
bool  restoreSettings(void);
void  initAnalogOutputs(void);
void  printPolarity(bool pol);

// UF2 bootloader double-tap magic value — same on SAMD21 and SAMD51
static const uint32_t DBL_TAP_MAGIC = 0xf01669efUL;

// ---------------------------------------------------------------------------
// Thread objects
// ---------------------------------------------------------------------------
#if UseThreads
ThreadController control      = ThreadController();
Thread           SystemThread = Thread();
#endif

// ---------------------------------------------------------------------------
// SPI flash and FAT filesystem objects
// ---------------------------------------------------------------------------
#if UseSPIflash
Adafruit_SPIFlash flash(&flashTransport);
FatVolume         fatfs;
File32            file;
#endif

// ---------------------------------------------------------------------------
// Flags written by the I²C receive handler (ISR context) and read in loop().
// 'volatile' prevents the compiler from caching these in a register, ensuring
// loop() always sees the value set by the interrupt handler.
// ---------------------------------------------------------------------------
volatile bool saveFlag    = false;
volatile bool restoreFlag = false;

// ---------------------------------------------------------------------------
// Readback values — updated by the Update() thread, read by command handlers
// ---------------------------------------------------------------------------
float Vrb = 0.0f;   // calibrated voltage readback (V)
float Irb = 0.0f;   // calibrated current readback (mA)
bool  Prb = true;   // polarity readback: true = positive

// ---------------------------------------------------------------------------
// Status LED
//
// The ItsyBitsy M0 DotStar pixel uses BRG byte order, not RGB.
// strip.Color(r, g, b) packs bytes as [b, r, g], so the R and G arguments
// below are intentionally swapped to produce the correct visible colour.
// ---------------------------------------------------------------------------
Adafruit_DotStar strip =
    Adafruit_DotStar(DOTSTAR_NUM, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLK, DOTSTAR_BRG);

#define LED_RED   strip.Color(  0, 150,   0)   // visible: red
#define LED_GREEN strip.Color(150,   0,   0)   // visible: green
#define LED_BLUE  strip.Color(  0,   0, 150)   // visible: blue
#define LED_BLACK strip.Color(  0,   0,   0)   // LED off

// ---------------------------------------------------------------------------
// Working data and factory defaults
// ---------------------------------------------------------------------------
Data data;

// Default record — written to flash the first time, or after a firmware
// upload erases the reserved FlashStorage region.
Data Rev_1_data =
{
    sizeof(Data), AppName, 1,
    #if UseWireClient
    0x20,           // default I²C address
    #endif
    false,          // power: off
    false,          // polarity: negative
    0.0f,           // setpoint: 0 V
    5000.0f,        // maxVoltage: 5000 V ceiling
    1.0f,           // maxCurrent: 1.0 mA trip
    5000.0f,        // range: ±5000 V full scale
    true,           // reversable: bipolar output fitted
    false,          // curMonEnabled: current trip disabled
    {0, 98.24f,  0.0f},    // Vmonitor ADC calibration {offset, scale, zero}
    {0,  0.333f, 8.5f},    // Imonitor ADC calibration
    {0, 409.5f,  0.0f},    // Vcontrol DAC calibration
    SIGNATURE
};

// Reserve a region of internal flash for configuration storage.
// This region is erased every time the firmware is uploaded.
FlashStorage(flash_data, Data);

// ---------------------------------------------------------------------------
// Firmware version string
// ---------------------------------------------------------------------------
const char *Version = AppName ", version 1.0 Dec 24, 2025";

// ---------------------------------------------------------------------------
// Command processor and debug subsystem
// ---------------------------------------------------------------------------
commandProcessor cp;
debug            dbg(&cp);

// ---------------------------------------------------------------------------
// Parameter range constraints used by the command table
// ---------------------------------------------------------------------------
const float rngV[2]  = {    0.0f, 20000.0f };  // unipolar voltage (V)
const float rngVR[2] = {-20000.0f, 20000.0f };  // bipolar voltage (V)
const float rngI[2]  = {    0.0f,   100.0f };  // current limit (mA)

// ---------------------------------------------------------------------------
// Command table
//
// Naming convention: uppercase handlers (SaveSettings, RestoreSettings …)
// are the serial-command callbacks registered in this table; they call the
// underlying lowercase implementation (saveSettings, restoreSettings …) and
// then send the protocol ACK or NAK.
// ---------------------------------------------------------------------------
Command cmds[] =
{
    // System
    {"GVER",    CMDstr,       -1, (void *)Version,            NULL,          "Firmware version"},
    {"?NAME",   CMDstr,       -1, (void *)&data.Name,         NULL,          "Device name"},
    {"SAVE",    CMDfunction,   0, (void *)SaveSettings,        NULL,          "Save system settings to flash"},
    {"RESTORE", CMDfunction,   0, (void *)RestoreSettings,     NULL,          "Restore system settings from flash"},
    {"BLOAD",   CMDfunction,   0, (void *)bootloader,          NULL,          "Jump to UF2 bootloader"},
    #if UseWireClient
    {"STWIADD", CMDfunction,   1, (void *)setTWIaddress,       NULL,          "Set I2C address (hex)"},
    {"GTWIADD", CMDfunction,   0, (void *)getTWIaddress,       NULL,          "Get I2C address (hex)"},
    #endif
    #if UseSPIflash
    {"SAVEF",   CMDfunction,   0, (void *)saveDefaults,        NULL,          "Save settings to SPI flash FS"},
    {"LOADF",   CMDfunction,   0, (void *)loadDefaults,        NULL,          "Load settings from SPI flash FS"},
    {"SAVECAL", CMDfunction,   0, (void *)saveCalibrations,    NULL,          "Save calibration to SPI flash FS"},
    {"LOADCAL", CMDfunction,   0, (void *)loadCalibrations,    NULL,          "Load calibration from SPI flash FS"},
    #endif
    // Output control
    {"ON",      CMDfunction,   0, (void *)powerON,             NULL,          "Enable 24 VDC to power supply"},
    {"OFF",     CMDfunction,   0, (void *)powerOFF,            NULL,          "Disable 24 VDC from power supply"},
    {"SV",      CMDfloat,      1, (void *)&data.setpoint,      (void *)rngV,  "Set output voltage (V)"},
    {"GV",      CMDfloat,      0, (void *)&data.setpoint,      NULL,          "Get output voltage setpoint (V)"},
    {"SPOL",    CMDfunction,   1, (void *)setPolarity,         NULL,          "Set output polarity: POS or NEG"},
    {"GPOL",    CMDfunction,   0, (void *)getPolarity,         NULL,          "Get output polarity: POS or NEG"},
    {"SRNG",    CMDfloat,      1, (void *)&data.range,         (void *)rngVR, "Set voltage range (V)"},
    {"GRNG",    CMDfloat,      0, (void *)&data.range,         NULL,          "Get voltage range (V)"},
    {"SVMAX",   CMDfloat,      1, (void *)&data.maxVoltage,    (void *)rngV,  "Set maximum voltage limit (V)"},
    {"GVMAX",   CMDfloat,      0, (void *)&data.maxVoltage,    NULL,          "Get maximum voltage limit (V)"},
    {"SIMAX",   CMDfloat,      1, (void *)&data.maxCurrent,    (void *)rngI,  "Set maximum current limit (mA)"},
    {"GIMAX",   CMDfloat,      0, (void *)&data.maxCurrent,    NULL,          "Get maximum current limit (mA)"},
    {"SRVBLE",  CMDbool,       1, (void *)&data.reversable,    NULL,          "Enable reversable output (TRUE/FALSE)"},
    {"GRVBLE",  CMDbool,       0, (void *)&data.reversable,    NULL,          "Get reversable output flag"},
    {"SCURENA", CMDbool,       1, (void *)&data.curMonEnabled, NULL,          "Enable current monitor protection (TRUE/FALSE)"},
    {"GCURENA", CMDbool,       0, (void *)&data.curMonEnabled, NULL,          "Get current monitor enable flag"},
    // Readbacks
    {"VRB",     CMDfloat,      0, (void *)&Vrb,                NULL,          "Get voltage readback (V)"},
    {"IRB",     CMDfloat,      0, (void *)&Irb,                NULL,          "Get current readback (mA)"},
    {"PRB",     CMDfunction,   0, (void *)getPolarityRB,        NULL,          "Get polarity readback: POS or NEG"},
    {NULL}
};
static CommandList cmdList = {cmds, NULL};

// ---------------------------------------------------------------------------
// Debug command handler
//
// Called by the debug subsystem.  With a numeric argument in [-4095, 4095]:
// drives the analog output directly (useful for calibration and testing).
// With no argument: reinitialises both PWM channels.
// ---------------------------------------------------------------------------
void Debug(void)
{
    int value;

    if (cp.getValue(&value, -(int)(PWM_PERIOD - 1), (int)(PWM_PERIOD - 1)))
    {
        analogOutput(value);
        return;
    }
    // No argument — reinitialise PWM channels
    initAnalogOutputs();
}

// ---------------------------------------------------------------------------
// Update — 25 ms cooperative-thread body
//
// Applies setpoint limits, drives the power supply control outputs, and
// samples the ADC readbacks.  Runs as a cooperative thread via ArduinoThread;
// must return promptly to allow the scheduler to service other threads and
// the main loop to process serial commands.
// ---------------------------------------------------------------------------
#if UseThreads
void Update(void)
{
    statusLED(LED_GREEN);

    // Cache the absolute value of range — used three times below.
    // Using fabsf() explicitly targets the float overload and avoids silently
    // calling the integer abs() if <cmath> is not in scope.
    const float absRange = fabsf(data.range);

    // Clamp setpoint to the tighter of the full-scale range and the voltage
    // limit.  This handles the case where the setpoint was set under a wider
    // range or limit than the current configuration allows.
    if (data.setpoint > absRange)        data.setpoint = absRange;
    if (data.setpoint > data.maxVoltage) data.setpoint = data.maxVoltage;

    // Current-limit trip: cut power if the readback exceeds the threshold
    if (data.curMonEnabled && Irb > data.maxCurrent) data.power = false;

    if (data.power)
    {
        digitalWrite(POWER, HIGH);

        if (data.reversable)
        {
            // Non-blocking polarity-change settle
            //
            // When the commanded polarity changes from negative to positive,
            // the output must ramp to 0 V and hold for 100 ms to let the HV
            // output capacitors fully discharge before the POL relay switches.
            // Skipping this dwell can damage the power supply or load.
            //
            // A state machine is used instead of delay() so that the
            // cooperative scheduler continues running during the dwell period.
            static bool     settling    = false;
            static uint32_t settleStart = 0;

            if (!settling && data.polarity && digitalRead(POL) != LOW)
            {
                // Polarity is about to change NEG→POS: ramp to zero and arm timer
                analogOutput(Value2Counts(0.0f, &data.Vcontrol));
                settleStart = millis();
                settling    = true;
            }
            if (settling)
            {
                if (millis() - settleStart < 100)
                {
                    // Capacitors still discharging — skip output update this tick
                    statusLED(LED_BLACK);
                    return;
                }
                settling = false;   // dwell complete; proceed to switch relay
            }

            // Drive the POL relay only when the state actually changes —
            // avoids redundant GPIO writes and unnecessary relay transitions
            static bool lastPol = false;
            if (data.polarity != lastPol)
            {
                digitalWrite(POL, data.polarity ? LOW : HIGH);
                lastPol = data.polarity;
            }
        }

        // Scale setpoint (V) to 0–10 V control range, convert to DAC counts
        float v = (data.setpoint / absRange) * 10.0f;
        analogOutput(Value2Counts(v, &data.Vcontrol));
    }
    else
    {
        // Power off: de-energise 24 V rail and ramp control voltage to zero
        digitalWrite(POWER, LOW);
        analogOutput(Value2Counts(0.0f, &data.Vcontrol));
    }

    // ADC readbacks — 10-sample average through the library IIR filter
    float v = Counts2Value(analogInput(VMON, 10) * absRange / 10.0f, &data.Vmonitor);
    Vrb = Filter(Vrb, v);

    v = Counts2Value(analogInput(IMON, 10), &data.Imonitor);
    Irb = Filter(Irb, v);

    // Polarity readback from hardware sense inputs
    if (data.reversable)
    {
        if (digitalRead(POS) == HIGH) Prb = true;
        if (digitalRead(NEG) == HIGH) Prb = false;
    }
    else
    {
        // Non-reversable supply: derive polarity from the sign of the range
        Prb = (data.range >= 0.0f);
    }

    statusLED(LED_BLACK);
}
#endif // UseThreads

// ---------------------------------------------------------------------------
// I²C peripheral interface
// ---------------------------------------------------------------------------
#if UseWireClient

WireHelper wh;

// Called by Wire when the master requests data — flush the staged response buffer
void requestEvent(void)
{
    wh.requestEventProcessor();
}

// Called by Wire when the master sends a command byte (and optional payload).
// Runs in interrupt context — keep processing short; no flash writes here.
void receiveEvent(int /*howMany*/)
{
    uint8_t    cmd;
    int        i;
    static bool TWITALKmode = false;

    while (Wire.available() != 0)
    {
        cmd = Wire.read();

        if (TWITALKmode)
        {
            // Transparent serial tunnel: forward bytes to the command ring buffer
            if (cmd == ESC)
            {
                TWITALKmode = false;
            }
            else
            {
                cp.selectStream(&wh.sb);
                cp.rb->put(cmd);
            }
        }
        else switch (cmd)
        {
            // Enter transparent ASCII tunnel — master streams raw command bytes
            case TWI_SERIAL:
                cp.selectStream(&wh.sb);
                TWITALKmode = true;
                break;

            // Single-command dispatch: master sends a newline-terminated string;
            // result is staged in wh.sb for the next master read
            case TWI_CMD:
                wh.sb.clear();
                for (i = 0; i < 100; i++)
                {
                    if (Wire.available() != 0)
                    {
                        cmd = Wire.read();
                        cp.rb->put(cmd);
                        if (cmd == '\n') break;
                    }
                    delayMicroseconds(100);
                }
                cp.selectStream(&wh.sb);
                cp.processCommands();
                break;

            // Power
            case TWI_SET_Power:      wh.ReadBool(&data.power);         break;
            case TWI_GET_Power:      wh.SendBool(data.power);          break;

            // Voltage setpoint
            case TWI_SET_Voltage:    wh.ReadFloat(&data.setpoint);     break;
            case TWI_GET_Voltage:    wh.SendFloat(data.setpoint);      break;

            // Polarity
            case TWI_SET_Pol:        wh.ReadBool(&data.polarity);      break;
            case TWI_GET_Pol:        wh.SendBool(data.polarity);       break;

            // Voltage range
            case TWI_SET_Range:      wh.ReadFloat(&data.range);        break;
            case TWI_GET_Range:      wh.SendFloat(data.range);         break;

            // Maximum voltage limit
            case TWI_SET_MaxV:       wh.ReadFloat(&data.maxVoltage);   break;
            case TWI_GET_MaxV:       wh.SendFloat(data.maxVoltage);    break;

            // Maximum current limit
            case TWI_SET_MaxI:       wh.ReadFloat(&data.maxCurrent);   break;
            case TWI_GET_MaxI:       wh.SendFloat(data.maxCurrent);    break;

            // Reversable flag
            case TWI_SET_Reversable: wh.ReadBool(&data.reversable);    break;
            case TWI_GET_Reversable: wh.SendBool(data.reversable);     break;

            // Current monitor enable
            case TWI_SET_CurEnable:  wh.ReadBool(&data.curMonEnabled); break;
            case TWI_GET_CurEnable:  wh.SendBool(data.curMonEnabled);  break;

            // Deferred persistence — handled in loop() to keep flash writes
            // out of interrupt context
            case TWI_SET_Save:       saveFlag    = true;                break;
            case TWI_SET_Restore:    restoreFlag = true;                break;

            // Metadata
            case TWI_READ_AVALIBLE:  wh.setReturnAvailable();           break;
            case TWI_READ_ID:        wh.SendByte(DeviceID);             break;

            // Readbacks
            case TWI_GET_Vrb:        wh.SendFloat(Vrb);                 break;
            case TWI_GET_Irb:        wh.SendFloat(Irb);                 break;
            case TWI_GET_Prb:        wh.SendBool(Prb);                  break;

            default: break;
        }
    }
}

#endif // UseWireClient

// ---------------------------------------------------------------------------
// System helpers
// ---------------------------------------------------------------------------

// Drive the DotStar LED to 'color'.  Suppresses SPI writes when the color
// has not changed — the DotStar protocol requires a full frame even for
// no-change updates, so the guard meaningfully reduces SPI traffic.
void statusLED(uint32_t color)
{
    static uint32_t current = 0;

    if (current == color) return;
    strip.setPixelColor(0, color);
    strip.show();
    current = color;
}

// Return the mean of 'num' ADC samples from 'pin'.
// Returns float so that the sub-count precision is preserved for the
// calibration arithmetic that follows in the caller.
float analogInput(int pin, int num)
{
    float sum = 0.0f;
    for (int i = 0; i < num; i++) sum += analogRead(pin);
    return sum / num;
}

// Drive the bipolar analog output using two complementary PWM channels.
// Positive counts → POSPWM carries the duty; NEGPWM is silenced.
// Negative counts → NEGPWM carries the inverted duty; POSPWM is silenced.
// Input is clamped to ±(PWM_PERIOD-1).
void analogOutput(int count)
{
    const int maxCount = (int)PWM_PERIOD - 1;
    if (count >  maxCount) count =  maxCount;
    if (count < -maxCount) count = -maxCount;

    if (count < 0)
    {
        pwmWrite(NEGPWM, -count);
        pwmWrite(POSPWM, 0);
    }
    else
    {
        pwmWrite(NEGPWM, 0);
        pwmWrite(POSPWM, count);
    }
}

// Configure and start both TCC PWM channels at zero duty.
// Called from setup() and from the Debug handler when reinitialization is needed.
// configureTCC_M0() handles clock setup internally; no separate setupTccClock()
// calls are required.
void initAnalogOutputs(void)
{
    configureTCC_M0(TCC2, 1, POSPWM);  // pin 13 — TCC2/WO[1]
    configureTCC_M0(TCC0, 3, NEGPWM);  // pin 12 — TCC0/WO[3]
    pwmWrite(POSPWM, 0);
    pwmWrite(NEGPWM, 0);
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup()
{
    // LED red during initialization so a failed boot is visually apparent
    strip.begin();
    strip.show();
    strip.setBrightness(20);
    statusLED(LED_RED);

    // Digital I/O configuration
    pinMode(NEG,   INPUT);              // polarity sense — negative mode
    pinMode(POS,   INPUT);              // polarity sense — positive mode
    pinMode(POWER, OUTPUT);
    digitalWrite(POWER, LOW);          // keep 24 V supply off during init
    pinMode(POL,   OUTPUT);
    digitalWrite(POL,   LOW);          // default relay state: positive

    // Load configuration from internal flash; use defaults if invalid
    data = flash_data.read();
    if (data.Signature != SIGNATURE) data = Rev_1_data;

    // Serial command interface (baud 0 = USB-CDC native speed)
    Serial.begin(0);
    cp.registerStream(&Serial);
    cp.registerCommands(&cmdList);
    cp.registerCommands(dbg.debugCommands());
    dbg.registerDebugFunction(Debug);

    #if UseWireClient
    // Join I²C bus as peripheral at the configured address
    Wire.begin(data.TWIadd);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    cp.registerStream(&wh.sb);
    // I²C stream is driven by receiveEvent, not polled by loop()
    cp.setStreamActive(&wh.sb, false);
    #endif

    #if UseThreads
    SystemThread.setName((char *)"Update");
    SystemThread.onRun(Update);
    SystemThread.setInterval(25);       // 25 ms period
    control.add(&SystemThread);
    #endif

    // Initialize PWM outputs — configureTCC_M0 sets up clocks internally
    initAnalogOutputs();

    statusLED(LED_BLACK);               // initialization complete
}

// ---------------------------------------------------------------------------
// loop — main cooperative dispatcher
// ---------------------------------------------------------------------------
void loop()
{
    cp.processStreams();
    cp.processCommands();

    #if UseThreads
    control.run();
    #endif

    // Handle flash operations requested by the I²C receive handler.
    // Performed here (in loop context, not in the ISR) to avoid writing
    // to flash from within interrupt context, which is unsupported by
    // FlashStorage on SAMD21.
    if (saveFlag)
    {
        saveFlag = false;
        saveSettings();
    }
    if (restoreFlag)
    {
        restoreFlag = false;
        restoreSettings();
    }
}

// ---------------------------------------------------------------------------
// Settings persistence — internal flash (FlashStorage)
// ---------------------------------------------------------------------------

void saveSettings(void)
{
    data.Signature = SIGNATURE;
    flash_data.write(data);
}

// Command-table callback: call implementation then send protocol response
void SaveSettings(void)
{
    saveSettings();
    cp.sendACK();
}

bool restoreSettings(void)
{
    Data td = flash_data.read();
    if (td.Signature != SIGNATURE) return false;
    data = td;
    return true;
}

// Command-table callback
void RestoreSettings(void)
{
    if (restoreSettings()) cp.sendACK();
    else                   cp.sendNAK();
}

// ---------------------------------------------------------------------------
// SPI flash filesystem (Circuit Python FAT volume on external SPI flash)
//
// Provides non-volatile storage for defaults and calibration data that
// survives firmware uploads (unlike the internal FlashStorage region).
// ---------------------------------------------------------------------------
#if UseSPIflash

// Mount the SPI flash filesystem.  Returns true on success.
// Prints a diagnostic message to the command stream on any failure.
bool FSsetup(void)
{
    if (!flash.begin())
    {
        cp.println("Error: failed to initialize SPI flash chip");
        return false;
    }
    if (!fatfs.begin(&flash))
    {
        cp.println("Error: failed to mount SPI flash filesystem");
        return false;
    }
    return true;
}

void saveDefaults(void)
{
    if (!FSsetup()) return;
    file = fatfs.open("default.dat", O_WRITE | O_CREAT);
    if (!file) { cp.println("Error: cannot create default.dat"); return; }
    size_t n = file.write((const void *)&data, sizeof(Data));
    file.close();
    cp.print("default.dat written, bytes = ");
    cp.println((int)n);
}

void loadDefaults(void)
{
    if (!FSsetup()) return;
    file = fatfs.open("default.dat", O_READ);
    if (!file) { cp.println("Error: cannot open default.dat"); return; }
    Data h;
    size_t n = file.read((void *)&h, sizeof(Data));
    file.close();
    if (n != sizeof(Data) || h.Signature != SIGNATURE)
    {
        cp.println("Error: default.dat is invalid or corrupt");
        return;
    }
    data = h;
    cp.print("default.dat read, bytes = ");
    cp.println((int)n);
}

void saveCalibrations(void)
{
    if (!FSsetup()) return;
    file = fatfs.open("cal.dat", O_WRITE | O_CREAT);
    if (!file) { cp.println("Error: cannot create cal.dat"); return; }
    size_t n = 0;
    n += file.write((const void *)&data.Vmonitor, sizeof(ADCchan));
    n += file.write((const void *)&data.Imonitor, sizeof(ADCchan));
    n += file.write((const void *)&data.Vcontrol, sizeof(DACchan));
    file.close();
    cp.print("cal.dat written, bytes = ");
    cp.println((int)n);
}

void loadCalibrations(void)
{
    if (!FSsetup()) return;
    file = fatfs.open("cal.dat", O_READ);
    if (!file) { cp.println("Error: cannot open cal.dat"); return; }
    size_t n = 0;
    n += file.read((void *)&data.Vmonitor, sizeof(ADCchan));
    n += file.read((void *)&data.Imonitor, sizeof(ADCchan));
    n += file.read((void *)&data.Vcontrol, sizeof(DACchan));
    file.close();
    cp.print("cal.dat read, bytes = ");
    cp.println((int)n);
}

#endif // UseSPIflash

// ---------------------------------------------------------------------------
// Bootloader entry — UF2 double-tap reset protocol
//
// Writes the magic value to the last word of RAM then resets; the UF2
// bootloader reads it on startup and enters USB DFU mode instead of the app.
// ---------------------------------------------------------------------------
void bootloader(void)
{
#if defined(SAMD21_SERIES)
    __disable_irq();
    volatile uint32_t *addr =
        reinterpret_cast<volatile uint32_t *>(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4);
    *addr = DBL_TAP_MAGIC;
    NVIC_SystemReset();
    while (true);   // unreachable; suppresses "no return" warnings
#elif defined(SAMD51_SERIES)
    __disable_irq();
    volatile uint32_t *addr =
        reinterpret_cast<volatile uint32_t *>(HSRAM_ADDR + HSRAM_SIZE - 4);
    *addr = DBL_TAP_MAGIC;
    NVIC_SystemReset();
    while (true);
#endif
}

// ---------------------------------------------------------------------------
// I²C address commands
// ---------------------------------------------------------------------------
#if UseWireClient

void setTWIaddress(void)
{
    int i;
    if (cp.getValue(&i, 0, 255, HEX))
    {
        data.TWIadd = i;
        cp.sendACK();
    }
    else
    {
        cp.sendNAK();
    }
}

void getTWIaddress(void)
{
    cp.sendACK(false);
    cp.println(data.TWIadd, HEX);
}

#endif // UseWireClient

// ---------------------------------------------------------------------------
// Power commands
// ---------------------------------------------------------------------------

void powerON(void)
{
    data.power = true;
    cp.sendACK();
}

void powerOFF(void)
{
    data.power = false;
    cp.sendACK();
}

// ---------------------------------------------------------------------------
// Polarity commands
// ---------------------------------------------------------------------------

// Shared output formatter: print "POS" or "NEG" to the active stream
void printPolarity(bool pol)
{
    cp.println(pol ? "POS" : "NEG");
}

void setPolarity(void)
{
    char *pol;
    if (!cp.getValue(&pol, "POS,NEG")) { cp.sendNAK(); return; }
    data.polarity = (strcmp(pol, "POS") == 0);
    cp.sendACK();
}

void getPolarity(void)
{
    cp.sendACK(false);
    printPolarity(data.polarity);
}

void getPolarityRB(void)
{
    cp.sendACK(false);
    printPolarity(Prb);
}
