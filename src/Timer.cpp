//
// Timer.cpp — SAMD21 TCC PWM helpers
//
// Configures TCC0 and TCC2 for normal (single-slope) PWM with a 12-bit
// resolution counter (period = PWM_PERIOD = 4096 counts).
//
// At GCLK0 = 48 MHz and DIV1 prescaler the switching frequency is:
//   48 000 000 / 4096 ≈ 11.7 kHz
//
// Two output pins are supported:
//   Pin 12 (NEGPWM) — TCC0/WO[3], peripheral mux F (PIO_TIMER_ALT)
//   Pin 13 (POSPWM) — TCC2/WO[1], peripheral mux E (PIO_TIMER)
//

#include <Arduino.h>
#include <wiring_private.h>

#include "Timer.h"

// ---------------------------------------------------------------------------
// Internal: TCC clock-enable configuration table
//
// Each row maps a TCC instance to the GCLK CLKCTRL ID that feeds it and the
// Power Manager APBC mask bit that enables its bus clock.  TCC0 and TCC1
// share the same GCLK channel (GCLK_CLKCTRL_ID_TCC0_TCC1) but have separate
// PM bits, so they need separate table entries.
// ---------------------------------------------------------------------------
struct TccClockConfig
{
    Tcc     *tcc;
    uint8_t  gclk_id;
    uint32_t pm_mask;
};

static const TccClockConfig tccClockTable[] =
{
    { TCC0, GCLK_CLKCTRL_ID_TCC0_TCC1, PM_APBCMASK_TCC0 },
    { TCC1, GCLK_CLKCTRL_ID_TCC0_TCC1, PM_APBCMASK_TCC1 },
    { TCC2, GCLK_CLKCTRL_ID_TCC2_TC3,  PM_APBCMASK_TCC2 },
};

// ---------------------------------------------------------------------------
// setupTccClock
//
// Enable the APBC peripheral clock and route GCLK0 (48 MHz) to 'tcc'.
// Must be called before accessing any TCC peripheral registers.
// The GCLK write is idempotent — safe to call again if the clock is already
// running (e.g. when configureTCC_M0 calls it internally).
// ---------------------------------------------------------------------------
void setupTccClock(Tcc *tcc)
{
    const TccClockConfig *cfg = nullptr;
    for (const auto &c : tccClockTable)
    {
        if (c.tcc == tcc) { cfg = &c; break; }
    }
    if (!cfg) return;   // unrecognised TCC instance — do nothing

    // Step 1: gate on the peripheral bus clock in the Power Manager
    PM->APBCMASK.reg |= cfg->pm_mask;

    // Step 2: connect GCLK generator 0 to this TCC and enable the channel
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN |
                                    GCLK_CLKCTRL_GEN_GCLK0 |
                                    cfg->gclk_id);
    while (GCLK->STATUS.bit.SYNCBUSY);  // wait for clock-domain sync
}

// ---------------------------------------------------------------------------
// configureTCC_M0
//
// Configure 'tcc' channel 'ch' for N-PWM output on Arduino pin 'pin'.
//
// Calls setupTccClock() internally so the caller does not need to call it
// separately.
//
// Pin mux selection:
//   Pins > 12  use PIO_TIMER     (peripheral mux E, standard timer function)
//   Pins ≤ 12  use PIO_TIMER_ALT (peripheral mux F, alternate timer function)
// On the ItsyBitsy M0:
//   Pin 13 → TCC2/WO[1] → PIO_TIMER
//   Pin 12 → TCC0/WO[3] → PIO_TIMER_ALT
// ---------------------------------------------------------------------------
void configureTCC_M0(Tcc *tcc, int ch, int pin)
{
    // Clock must be enabled before touching any peripheral register
    setupTccClock(tcc);

    // Select the correct peripheral mux function for this pin
    pinPeripheral(pin, (pin <= 12) ? PIO_TIMER_ALT : PIO_TIMER);

    // Disable TCC before reconfiguring (required by SAMD21 datasheet)
    tcc->CTRLA.bit.ENABLE = 0;
    while (tcc->SYNCBUSY.bit.ENABLE);

    // Software reset — clears all TCC registers to reset values
    tcc->CTRLA.bit.SWRST = 1;
    while (tcc->SYNCBUSY.bit.SWRST);

    // No prescaler; resynchronise on the prescaler clock edge
    tcc->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_PRESCSYNC_PRESC;

    // Normal (single-slope) PWM — output high from 0 to CC, low from CC to PER
    tcc->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
    // Note: no SYNCBUSY check for WAVE on SAMD21 (register is not synchronised)

    // Set the period (top count); duty range is 0 … PWM_PERIOD-1
    tcc->PER.reg = PWM_PERIOD;
    while (tcc->SYNCBUSY.bit.PER);

    // Initialise compare to 50 % duty so the output is defined before pwmWrite
    tcc->CC[ch].reg = PWM_PERIOD / 2;
    while (tcc->SYNCBUSY.vec.CC & (1u << ch));  // wait for this channel's CC sync

    // Enable TCC — output becomes active
    tcc->CTRLA.bit.ENABLE = 1;
    while (tcc->SYNCBUSY.bit.ENABLE);
}

// ---------------------------------------------------------------------------
// pwmWrite
//
// Update the PWM duty cycle of the channel mapped to 'pin'.
// Writes to the buffered CCB register; the update takes effect at the next
// period boundary, preventing output glitches on mid-cycle changes.
//
// 'value' is clamped to [PWM_MIN, PWM_PERIOD-1].
// PWM_MIN > 0 avoids a 0 % duty cycle condition that can cause artefacts on
// some power-supply topologies.
//
// Supported pins:
//   12 → TCC0 channel 3   (NEGPWM)
//   13 → TCC2 channel 1   (POSPWM)
// ---------------------------------------------------------------------------
void pwmWrite(int pin, int value)
{
    Tcc *tcc;
    int  ch;

    switch (pin)
    {
        case 12: tcc = TCC0; ch = 3; break;    // NEGPWM — TCC0/WO[3]
        case 13: tcc = TCC2; ch = 1; break;    // POSPWM — TCC2/WO[1]
        default: return;                        // unsupported pin — ignore
    }

    // Clamp duty to the valid operating range
    if (value < (int)PWM_MIN)          value = (int)PWM_MIN;
    if (value >= (int)PWM_PERIOD)      value = (int)PWM_PERIOD - 1;

    // Spin until the previous CCB write has been transferred to CC
    while (tcc->SYNCBUSY.vec.CC & (1u << ch));
    tcc->CCB[ch].reg = (uint32_t)value;
}
