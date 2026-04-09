//
// Timer.h — SAMD21 TCC PWM configuration and output helpers
//
// Provides N-PWM output on two fixed pins using the TCC0 and TCC2
// peripherals at a 12-bit (4096-count) resolution.
//
// Supported output pins (ItsyBitsy M0):
//   Pin 12 (NEGPWM) — TCC0/WO[3], PIO_TIMER_ALT (mux F)
//   Pin 13 (POSPWM) — TCC2/WO[1], PIO_TIMER     (mux E)
//
#pragma once

// PWM counter period in counts.
// At GCLK0 = 48 MHz, DIV1 prescaler: switching frequency ≈ 11.7 kHz.
// Duty cycle range for pwmWrite() is [PWM_MIN, PWM_PERIOD-1].
static const uint16_t PWM_PERIOD = 4096;

// Minimum duty cycle — keeps PWM switching to avoid topology artefacts at 0 %
static const uint16_t PWM_MIN    = 5;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Enable the APBC clock and route GCLK0 (48 MHz) to 'tcc'.
// Called automatically by configureTCC_M0(); safe to call directly when only
// the clock setup is needed without a full channel reconfiguration.
void setupTccClock(Tcc *tcc);

// Fully configure 'tcc' channel 'ch' for N-PWM output on Arduino pin 'pin'.
// Handles clock setup, pin mux, TCC reset, period, and initial compare value.
// After this call the channel is running at 50 % duty; use pwmWrite() to set
// the operating duty cycle.
void configureTCC_M0(Tcc *tcc, int ch, int pin);

// Set the duty cycle of the PWM channel mapped to 'pin' (12 or 13).
// 'value' is in counts [PWM_MIN, PWM_PERIOD-1]; values outside that range
// are clamped.  The update takes effect at the next period boundary.
void pwmWrite(int pin, int value);
