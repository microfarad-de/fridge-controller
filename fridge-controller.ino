/*
 * Secop BD35F compressor speed controller.
 *
 * It is recommended to activate the watchdog support on the Arduino bootloader
 * by defining the WATCHDOG_MODS macro. This will reduce the bootloader's power-up
 * delay, thus invalidating the need to hold the power button for around 2 seconds for
 * the system to turn on.
 *
 * This source file is part of the follwoing repository:
 * http://www.github.com/microfarad-de/fridge-controller
 *
 * Please visit:
 *   http://www.microfarad.de
 *   http://www.github.com/microfarad-de
 *
 * Copyright (C) 2025 Karim Hraibi (khraibi@gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Version: 1.3.0
 * Date:    May 5, 2025
 */


#define VERSION_MAJOR 1  // Major version
#define VERSION_MINOR 3  // Minor version
#define VERSION_MAINT 0  // Maintenance version


#include <Arduino.h>
#include "src/Cli/Cli.h"
#include "src/Led/Led.h"
#include "src/Nvm/Nvm.h"
#include "src/Trace/Trace.h"
#include "src/MathMf/MathMf.h"
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>


/*
 * Pin assignment
 */
#define INPUT_PIN   12
#define OUTPUT_PIN  11
#define LED_PIN     LED_BUILTIN  // 13


/*
 * Configuration parameters
 */
//#define SERIAL_DEBUG                         // Serial debug printing
#define SERIAL_BAUD               9600       // Serial communication baud rate
#define NVM_MAGIC_WORD            0xDEADBEEF // Magic word stored in correctly initialized NVM
#define TRACE_BUF_SIZE            200        // Trace buffer size in words
#define TRACE_STAMP_RESOLUTION_MS 60000      // Trace time stamp resolution in milliseconds
#define INPUT_DEBOUNCE_DELAY_MS   1000       // Input debounce time delay in milliseconds
#define SPEED_ADJUST_PERIOD_MS    60000      // Time delay in milliseconds between consecutive speed adjustments
#define SPEED_RAMPUP_PERIOD_MS    10000      // Time delay in milliseconds between consecutive speed rampup steps
#define SPEED_LOCK_ON_DELAY_S     600        // Time delay in seconds for activating the speed lock
#define SPEED_LOCK_OFF_DELAY_S    600        // Time delay in seconds for deactivating the speed lock
#define DUTY_MEAS_BUF_EXP         7          // Duty cycle measurement buffer size exponent
#define DUTY_MEAS_BUF_SIZE        (1 << DUTY_MEAS_BUF_EXP) // Duty cycle measurement buffer size
#define DUTY_MEAS_WINDOW_SIZE_M   120        // Duty cycle measurement window size in minutes
#define DUTY_MEAS_TIEMOUT_S       1800       // Dyty cycle measurement is reset after this duration in seconds

#ifdef SERIAL_DEBUG
  #define DEBUG(x) x
#else
  #define DEBUG(x)
#endif

#define STRINGIFY(x) #x
#define QUOTE(x) STRINGIFY(x)

/*
 * Main state machine state definitions
 */
typedef enum  {
  STATE_OFF_ENTRY = 0,
  STATE_OFF_WAIT  = 1,
  STATE_OFF       = 2,
  STATE_ON_ENTRY  = 3,
  STATE_ON_WAIT   = 4,
  STATE_ON        = 5
} State_e;


/*
 * State variables
 */
struct State_t {
  State_e state = STATE_OFF_ENTRY;     // Main state machine state
  bool    crcOk              = false;  // CRC check status
  bool    inputEnabled       = false;  // Input pin state after debounce
  bool    speedLock          = true;   // Enforce compressor operation at minimum speed
  uint8_t pwmDutyCycle       = 0;      // PWM duty cycle at the output pin
  uint8_t savedPwmDutyCycle  = 0;      // Saved PWM duty cycle
  uint8_t targetPwmDutyCycle = 0;      // Target PWM duty cycle
  uint8_t dutyMeas[DUTY_MEAS_BUF_SIZE] = { 0 };  // Duty cycle measurement array
  uint8_t dutyMeasIdx        = 0;      // Duty cycle array index
  uint8_t dutyValidSamples   = 0;     // Number of valid duty cycle measurement samples
} S;


/*
 * Configuration parameters
 *
 * Measured PWM values for Secop BD35F:
 *   Min. RPM: 190 / 255
 *   Max. RPM: 40 / 255
 *
 * PWM setting range: 0..255
 */
struct Nvm_t {
  uint32_t magicWord = NVM_MAGIC_WORD; // Magic word proves correctly initialized NVM
  uint32_t minOnDurationS    = 300;    // Minimum allowed compressor on duration in seconds
  uint32_t minOffDurationS   = 60;     // Minimum allowed compressor off duration in seconds
  uint8_t  minRpmDutyCycle   = 190;    // PWM duty cycle for minimum compressor RPM (1..255), larger value decreases RPM
  uint8_t  maxRpmDutyCycle   = 120;    // PWM duty cycle for maximum compressor RPM (1..255), smaller value increases RPM
  uint8_t  traceEnable       = 1;      // Enable the trace logging
  uint8_t  padding0[1];                // Memory alignment padding
  uint32_t speedAdjustDelayS = 120;    // Wait this amount of time in seconds after minOnDurationS
                                       // or minOffDurationS elapses before adjusting compressor speed
  uint8_t  speedAdjustRate   = 5;      // Increase or decrease PWM by this amount of steps per minute
  uint8_t  padding1[11];               // Memory alignment padding
  uint32_t crc               = 0;      // CRC checksum
} Nvm;


/*
 * Class instances
 */
LedClass   Led;
TraceClass Trace;


/*
 * Trace message lookup table
 */
const char *traceMsgList[] = {
  "Compressor on",
  "Compressor off",
  "Power on",
  "Increase speed %d",
  "Decrease speed %d",
  "Set speed %d",
  "Speed lock %d",
  "CRC FAIL",
};
enum {
  TRC_COMPRESSOR_ON,
  TRC_COMPRESSOR_OFF,
  TRC_POWER_ON,
  TRC_INCREASE_SPEED,
  TRC_DECREASE_SPEED,
  TRC_SET_SPEED,
  TRC_SPEED_LOCK,
  TRC_CRC_FAIL,
  TRC_COUNT
};
static_assert(TRC_COUNT == sizeof(traceMsgList)/sizeof(traceMsgList[0]));


/*
 * Function declarations
 */
void powerSave (void);
void readInputPin (void);
void setPwm (void);
void ledManager   (void);
void speedManager (void);
void dutyCycleLogger (void);
void nvmValidate  (void);
void nvmRead      (void);
void nvmWrite     (void);
int cmdOn     (int argc, char **argv);
int cmdOff    (int argc, char **argv);
int cmdSet    (int argc, char **argv);
int cmdStatus (int argc, char **argv);
int cmdConfig (int argc, char **argv);
int cmdTrace  (int argc, char **argv);
int cmdSetMinOnDuration  (int argc, char **argv);
int cmdSetMinOffDuration (int argc, char **argv);
int cmdSetMinDutyCycle   (int argc, char **argv);
int cmdSetMaxDutyCycle   (int argc, char **argv);
int cmdSetSpeedAdjustParam (int argc, char **argv);
int cmdReset (int argc, char **argv);
void printVersion (uint8_t indent);
void helpText (void);


/*
 * Arduino initialization routine
 */
void setup (void)
{
  // Set PWM frequency on pins 3 and 11 which are controlled by Timer/Counter 2
  // See https://www.etechnophiles.com/how-to-change-the-pwm-frequency-of-arduino-nano/
  // See ATmega328P datasheet Section 22.11.2, Table 22-10
  // 31372.55 / 8 = 1960.53 Hz (for 8 mHz Arduino)
  // Where 31372.55 = 8000000 / 255
  TCCR2B = (TCCR2B & B11111000) | B00000010;

  // Disable the ADC
  ADCSRA &= ~_BV(ADEN);
  power_adc_disable();

  pinMode(INPUT_PIN, INPUT_PULLUP);
  pinMode(OUTPUT_PIN, OUTPUT);

  digitalWrite(OUTPUT_PIN, LOW);

  Led.initialize(LED_PIN);
  Trace.initialize(sizeof(Nvm), TRACE_BUF_SIZE, TRACE_STAMP_RESOLUTION_MS, traceMsgList, TRC_COUNT);
  Cli.init(SERIAL_BAUD, false, helpText);

  Serial.println(F("\r\n+ + +  F R I D G E  C O N T R O L L E R  + + +\r\n"));
  printVersion(0);
  Cli.newCmd    ("on",      "Turn on the compressor",  cmdOn);
  Cli.newCmd    ("off",     "Turn off the compressor", cmdOff);
  Cli.newCmd    ("set",     "Set the compressor speed (arg: <0..255>)", cmdSet);
  Cli.newCmd    ("s",       "Show the system status",        cmdStatus);
  Cli.newCmd    ("r",       "Show the system configuration", cmdConfig);
  Cli.newCmd    ("t",       "Print the trace log or enable/disable trace (arg: [0,1])", cmdTrace);
  Cli.newCmd    ("ond",     "Set min on duration (arg: <0..600>s)",  cmdSetMinOnDuration);
  Cli.newCmd    ("offd",    "Set min off duration (arg: <0..600>s)", cmdSetMinOffDuration);
  Cli.newCmd    ("pwml",    "Set the PWM duty for min RPM (arg: <1..255>)", cmdSetMinDutyCycle);
  Cli.newCmd    ("pwmh",    "Set the PWM duty for max RPM (arg: <1..255>)", cmdSetMaxDutyCycle);
  Cli.newCmd    ("speed",   "Set speed adjust delay & rate (args: <0..600>s, <0..255>)", cmdSetSpeedAdjustParam);
  Cli.newCmd    ("reset",   "Reset settings to defaults (arg: <1024>)", cmdReset);
  Cli.showHelp();

  nvmRead();
  Trace.log(TRC_POWER_ON);

  if (Nvm.traceEnable) Trace.start();
  else                 Trace.stop();

  S.savedPwmDutyCycle = Nvm.minRpmDutyCycle;

  // Enable the watchdog timer
  wdt_enable (WDTO_8S);
}


/*
 * Arduino main loop
 */
void loop (void)
{
  static bool     initialStartup  = true;
  static uint32_t compressorOnTs  = 0;
  static uint32_t compressorOffTs = 0;
  static uint32_t speedLockTs     = 0;
  uint32_t ts = millis();

  Cli.getCmd();
  Led.loopHandler();
  Trace.loopHandler();
  wdt_reset();
  readInputPin();
  setPwm();
  ledManager();
  speedManager();
  dutyCycleLogger();
  powerSave();

  // Main state machine
  switch (S.state) {

    // Compressor OFF state entry point
    case STATE_OFF_ENTRY:
      compressorOffTs      = ts;
      speedLockTs          = ts;
      S.targetPwmDutyCycle = 0;
      Trace.log(TRC_COMPRESSOR_OFF);

      if (initialStartup) {
        S.state = STATE_OFF;
        initialStartup = false;
      }
      else {
        S.state = STATE_OFF_WAIT;
      }
      break;

    // Wait for minimum OFF duration
    case STATE_OFF_WAIT:
      if (ts - compressorOffTs > Nvm.minOffDurationS * 1000) {
        S.state = STATE_OFF;
      }
      break;

    // Compressor OFF main state
    case STATE_OFF:
      if (true == S.inputEnabled) {
        S.state = STATE_ON_ENTRY;
      }
      if (ts - speedLockTs > (uint32_t)SPEED_LOCK_ON_DELAY_S * 1000 && !S.speedLock) {
        S.speedLock = true;
        S.savedPwmDutyCycle = Nvm.minRpmDutyCycle;
        Trace.log(TRC_SPEED_LOCK, S.speedLock);
      }
      break;

    // Compressor ON state entry point
    case STATE_ON_ENTRY:
      compressorOnTs = ts;
      speedLockTs    = ts;
      S.targetPwmDutyCycle = S.savedPwmDutyCycle;

      nvmRead ();  // Perform a CRC check
      if (S.crcOk) {
        Trace.log(TRC_COMPRESSOR_ON);
        S.state = STATE_ON_WAIT;
      }
      else  {
        S.state = STATE_OFF_ENTRY;
      }
      break;

    // Wait for minimum ON duration
    case STATE_ON_WAIT:
      if (ts - compressorOnTs > Nvm.minOnDurationS * 1000) {
        S.state = STATE_ON;
      }
      break;

    // Compressor ON main state
    case STATE_ON:
      if (false == S.inputEnabled) {
        S.state = STATE_OFF_ENTRY;
      }
      if (ts - speedLockTs > (uint32_t)SPEED_LOCK_OFF_DELAY_S * 1000 && S.speedLock && S.crcOk) {
        S.speedLock = false;
        Trace.log(TRC_SPEED_LOCK, S.speedLock);
      }
      break;

    default:
      break;
  }
}


/*
 * Power saving routine
 * Enables CPU sleep mode
 */
void powerSave (void)
{
  set_sleep_mode (SLEEP_MODE_IDLE);
  cli ();
  sleep_enable ();  // Enter sleep, wakeup will be triggered by the next Timer 0 interrupt
  sei ();
  sleep_cpu ();
  sleep_disable ();
}


/*
 * Read and debounce the input pin
 * *Input pin is active low*
 */
void readInputPin (void)
{
  static uint32_t inputTs = 0;
  uint32_t ts = millis();

  if (S.inputEnabled) {
    if (digitalRead(INPUT_PIN) == LOW) inputTs = ts;
    if (ts - inputTs > INPUT_DEBOUNCE_DELAY_MS) {
      S.inputEnabled = false;
    }
  }
  else {
    if (digitalRead(INPUT_PIN) == HIGH) inputTs = ts;
    if (ts - inputTs > INPUT_DEBOUNCE_DELAY_MS) {
      S.inputEnabled = true;
    }
  }
}


/*
 * Manage the LED blinking state
 */
void ledManager (void)
{
  bool running = (S.pwmDutyCycle > 0);

  if (!S.inputEnabled && !running) {
    Led.blink(-1, 200, 2800);
  }
  else if (S.inputEnabled && !running) {
    Led.blink(-1, 200, 800);
  }
  else if (S.inputEnabled && running) {
    Led.turnOn();
  }
  else if (!S.inputEnabled && running) {
    Led.blink(-1, 800, 200);
  }
}


/*
 * Decrement PWM duty cycle by one adjustment step
 */
bool decrementPwm (uint8_t * pwm)
{
  if (*pwm - Nvm.speedAdjustRate > Nvm.maxRpmDutyCycle) {
    *pwm -= Nvm.speedAdjustRate;
    DEBUG(Serial.println("Decrement PWM"));
    return true;
  }
  else if (*pwm != Nvm.maxRpmDutyCycle) {
    *pwm = Nvm.maxRpmDutyCycle;
    DEBUG(Serial.println("Decrement PWM"));
    return true;
  }
  else {
    return false;
  }
}


/*
 * increment PWM duty cycle by one adjustment step
 */
bool incrementPwm (uint8_t *pwm)
{
  if (*pwm + Nvm.speedAdjustRate < Nvm.minRpmDutyCycle) {
    *pwm += Nvm.speedAdjustRate;
    DEBUG(Serial.println("Increment PWM"));
    return true;
  }
  else if (*pwm != Nvm.minRpmDutyCycle) {
    *pwm = Nvm.minRpmDutyCycle;
    DEBUG(Serial.println("Increment PWM"));
    return true;
  }
  else {
    return false;
  }
}


/*
 * Apply the PWM duty cycle to the output pin
 * Slowly rampup compressor speed
 */
void setPwm (void)
{
  static uint8_t  lastPwmDutyCycle = 255 - Nvm.minRpmDutyCycle > Nvm.speedAdjustRate ? Nvm.minRpmDutyCycle + Nvm.speedAdjustRate : 255;
  static uint32_t adjustTs         = 0;
  uint32_t ts = millis();

  if (S.targetPwmDutyCycle != S.pwmDutyCycle) {
    // Speed adjust disabled
    if (Nvm.speedAdjustRate == 0) {
      S.pwmDutyCycle = S.targetPwmDutyCycle;
      analogWrite(OUTPUT_PIN, S.pwmDutyCycle);
      lastPwmDutyCycle = S.pwmDutyCycle;
    }
    // Stop
    else if (S.targetPwmDutyCycle == 0) {
      DEBUG(Serial.println("Stop"));
      S.pwmDutyCycle = 0;
      analogWrite(OUTPUT_PIN, 0);
      lastPwmDutyCycle = 255 - Nvm.minRpmDutyCycle > Nvm.speedAdjustRate ? Nvm.minRpmDutyCycle + Nvm.speedAdjustRate : 255;
    }
    // Decrease speed
    else if (S.targetPwmDutyCycle >= lastPwmDutyCycle) {
      DEBUG(Serial.println("Ramp down"));
      S.pwmDutyCycle = S.targetPwmDutyCycle;
      analogWrite(OUTPUT_PIN, S.pwmDutyCycle);
      lastPwmDutyCycle = S.pwmDutyCycle;
    }
    // Increase speed slowly
    else if (S.targetPwmDutyCycle < lastPwmDutyCycle)
    {
      if (ts - adjustTs >= SPEED_RAMPUP_PERIOD_MS) {
        if (decrementPwm(&lastPwmDutyCycle)) {
          DEBUG(Serial.println("Ramp up"));
          S.pwmDutyCycle = lastPwmDutyCycle;
          analogWrite(OUTPUT_PIN, S.pwmDutyCycle);
        }
        adjustTs = ts;
      }
    }
  }
  else {
    adjustTs = ts - SPEED_RAMPUP_PERIOD_MS;
  }
}


/*
 * Speed adjustment algorithm
 */
void speedManager (void)
{
  static enum {WAIT, ON, INCREASE, OFF, DECREASE} localState = WAIT;
  static uint32_t startTs  = 0;
  static uint32_t adjustTs = 0;

  if (Nvm.speedAdjustRate == 0) return;

  uint32_t ts  = millis();

  switch (localState) {
    case WAIT:
        if (STATE_ON == S.state) {
          startTs    = ts;
          localState = ON;
        }
        else if (STATE_OFF == S.state) {
          startTs    = ts;
          localState = OFF;
        }
      break;

    case ON:
      if ((ts - startTs > Nvm.speedAdjustDelayS * 1000) && !S.speedLock) {
        adjustTs   = ts - SPEED_ADJUST_PERIOD_MS;
        localState = INCREASE;
      }
      if (S.state != STATE_ON) {
        localState = WAIT;
      }
      break;

    case INCREASE:
      if (ts - adjustTs >= SPEED_ADJUST_PERIOD_MS) {
        if (decrementPwm(&S.savedPwmDutyCycle)) {
          DEBUG(Serial.println("Increase speed"));
          S.targetPwmDutyCycle = S.savedPwmDutyCycle;
          Trace.log(TRC_INCREASE_SPEED, S.savedPwmDutyCycle);
        }

        adjustTs = ts;
      }
      if (S.state != STATE_ON) {
        localState = WAIT;
      }
      break;

    case OFF:
      if (ts - startTs > Nvm.speedAdjustDelayS * 1000) {
        adjustTs   = ts - SPEED_ADJUST_PERIOD_MS;
        localState = DECREASE;
      }
      if (S.state != STATE_OFF) {
        localState = WAIT;
      }
      break;

    case DECREASE:
      if (ts - adjustTs >= SPEED_ADJUST_PERIOD_MS) {
        if (incrementPwm(&S.savedPwmDutyCycle)) {
          DEBUG(Serial.println("Decrease speed"));
          // S.targetPwmDutyCycle = 0
          Trace.log(TRC_DECREASE_SPEED, S.savedPwmDutyCycle);
        }
        adjustTs = ts;
      }
      if (S.state != STATE_OFF) {
        localState = WAIT;
      }
      break;

    default:
      break;
  }
}


/*
 * Compressor duty cycle logging routine
 */
void dutyCycleLogger (void)
{
  static uint32_t captureTs = 0;
  static uint32_t sampleTs  = 0;
  static uint32_t resetTs   = 0;

  uint32_t ts = millis();
  bool on = (S.pwmDutyCycle > 0);

  if (on) {
    resetTs = ts;
  }

  if (ts - resetTs > (uint32_t)DUTY_MEAS_TIEMOUT_S * 1000) {
    S.dutyValidSamples = 0;
  }

  if (ts - captureTs > 1000) {
    captureTs = ts;
    S.dutyMeas[S.dutyMeasIdx] += on;
  }

  if (ts - sampleTs > 60000) {
    sampleTs = ts;
    uint8_t mask = (1 << DUTY_MEAS_BUF_EXP) - 1;
    S.dutyMeasIdx++;
    S.dutyMeasIdx &= mask;
    S.dutyMeas[S.dutyMeasIdx] = 0;
    if (S.dutyValidSamples < DUTY_MEAS_WINDOW_SIZE_M) {
      S.dutyValidSamples++;
    }
  }
}


/*
 * Calculate the compressor duty cycle in percent
 */
uint8_t dutyCycleCalculate(void) {
  uint32_t sum = 0;
  uint32_t total = S.dutyValidSamples * 60;
  int16_t idx = S.dutyMeasIdx;

  for (uint8_t i = 0; i < S.dutyValidSamples; i++) {
    idx--;
    if (idx < 0) idx += DUTY_MEAS_BUF_SIZE;
    sum += S.dutyMeas[idx];
  }

  if (total > 0) {
    return (sum * 100) / total;
  }
  else {
    return 0;
  }
}


/*
 * Validate EEPROM data
 */
void nvmValidate (void)
{
  Nvm_t NvmInit;
  bool result;

  if (Nvm.magicWord != NVM_MAGIC_WORD) {
    Nvm = NvmInit;
    eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
    return;
  }

  result = Nvm.minRpmDutyCycle > 0;
  if (!result) {
    Nvm.minRpmDutyCycle = NvmInit.minRpmDutyCycle;
  }

  result  = Nvm.maxRpmDutyCycle <= Nvm.minRpmDutyCycle;
  result &= Nvm.maxRpmDutyCycle > 0;
  if (!result) {
    if (NvmInit.maxRpmDutyCycle < Nvm.minRpmDutyCycle) {
      Nvm.maxRpmDutyCycle = NvmInit.maxRpmDutyCycle;
    }
    else {
      Nvm.maxRpmDutyCycle = Nvm.minRpmDutyCycle;
    }
  }

  result = Nvm.minOnDurationS <= 600;
  if (!result) {
    Nvm.minOnDurationS = NvmInit.minOnDurationS;
  }

  result = Nvm.minOffDurationS <= 600;
  if (!result) {
    Nvm.minOffDurationS = NvmInit.minOffDurationS;
  }

  result = Nvm.speedAdjustDelayS <= 600;
  if (!result) {
    Nvm.speedAdjustDelayS = NvmInit.speedAdjustDelayS;
  }

  result  = Nvm.speedAdjustRate <= 255;
  result &= Nvm.speedAdjustRate >= 0;
  if (!result) {
    Nvm.speedAdjustRate = NvmInit.speedAdjustRate;
  }
}


/*
 * Read EEPROM data
 */
void nvmRead (void)
{
  eepromRead(0x0, (uint8_t*)&Nvm, sizeof (Nvm));
  nvmValidate();

  uint32_t crc = crcCalc((uint8_t*)&Nvm, sizeof (Nvm) - sizeof (Nvm.crc));
  if (crc != Nvm.crc) {
    S.crcOk = false;
    Serial.println(F("EEPROM CRC check failed - resetting EEPROM contents...\r\n"));
    Trace.log(TRC_CRC_FAIL);
    // Reset NVM on CRC failure
    Nvm.magicWord = 0;
    nvmWrite();
  }
  else {
    S.crcOk = true;
  }
}


/*
 * Write EEPROM data
 */
void nvmWrite (void)
{
  nvmValidate();
  Nvm.crc = crcCalc((uint8_t*)&Nvm, sizeof(Nvm) - sizeof(Nvm.crc));
  eepromWrite(0x0, (uint8_t*)&Nvm, sizeof (Nvm));
}


/*
 * Turn on the compressor
 */
int cmdOn (int argc, char **argv)
{
  S.state = STATE_ON_ENTRY;
  Serial.println(F("Compressor on\r\n"));
  return 0;
}


/*
 * Turn off the compressor
 */
int cmdOff (int argc, char **argv)
{
  S.state = STATE_OFF_ENTRY;
  Serial.println(F("Compressor off\r\n"));
  return 0;
}


/*
 * Set the compressor speed
 */
int cmdSet (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t pwm = atoi(argv[1]);
  if (pwm == 0) {
    S.state = STATE_OFF_ENTRY;
    Trace.log(TRC_SET_SPEED, pwm);
    Serial.println(F("Compressor off\r\n"));
    return 0;
  }
  else if (pwm < Nvm.maxRpmDutyCycle) {
    pwm = Nvm.maxRpmDutyCycle;
  }
  else if (pwm > Nvm.minRpmDutyCycle) {
    pwm = Nvm.minRpmDutyCycle;
  }
  S.savedPwmDutyCycle  = pwm;
  S.state = STATE_ON_ENTRY;
  Trace.log(TRC_SET_SPEED, S.savedPwmDutyCycle);
  Cli.xprintf("Output PWM duty cycle = %d\r\n\r\n", S.savedPwmDutyCycle);
  return 0;
}


/*
 * Set the minimum allowed compressor on duration
 */
int cmdSetMinOnDuration (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint32_t duration = atol(argv[1]);
  Nvm.minOnDurationS = duration;
  nvmWrite();
  Cli.xprintf("Min on duration = %lds\r\n\r\n", Nvm.minOnDurationS);
  return 0;
}


/*
 * Set the minimum allowed compressor off duration
 */
int cmdSetMinOffDuration (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint32_t duration = atol(argv[1]);
  Nvm.minOffDurationS = duration;
  nvmWrite();
  Cli.xprintf("Min off duration = %lds\r\n\r\n", Nvm.minOffDurationS);
  return 0;
}


/*
 * Set the PWM duty cycle corresponding to minimum RPM
 */
int cmdSetMinDutyCycle (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t dutyCycle   = atoi(argv[1]);
  Nvm.minRpmDutyCycle = dutyCycle;
  nvmWrite();
  Cli.xprintf("PWM duty cycle at min RPM = %d\r\n\r\n", Nvm.minRpmDutyCycle);
  return 0;
}


/*
 * Set the PWM duty cycle corresponding to minimum RPM
 */
int cmdSetMaxDutyCycle (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t dutyCycle   = atoi(argv[1]);
  Nvm.maxRpmDutyCycle = dutyCycle;
  nvmWrite();
  Cli.xprintf("PWM duty cycle at max RPM = %d\r\n\r\n", Nvm.maxRpmDutyCycle);
  return 0;
}


/*
 * Set the speed adjust parameters
 */
int cmdSetSpeedAdjustParam (int argc, char **argv)
{
  if (argc != 3) {
    return 1;
  }
  uint32_t delay = atol(argv[1]);
  uint8_t  rate  = atoi(argv[2]);
  Nvm.speedAdjustDelayS = delay;
  Nvm.speedAdjustRate = rate;
  nvmWrite();
  if (Nvm.speedAdjustRate == 0) {
    S.savedPwmDutyCycle = Nvm.minRpmDutyCycle;
  }
  Cli.xprintf("Speed adjust delay = %lds\r\n", Nvm.speedAdjustDelayS);
  Cli.xprintf("Speed adjust rate  = %d/min\r\n\r\n", Nvm.speedAdjustRate);
  return 0;
}


/*
 * Display the system status
 */
int cmdStatus (int argc, char **argv)
{
  Serial.println (F("System status:"));
  Cli.xprintf    (  "  State        = %d\r\n", S.state);
  Cli.xprintf    (  "  Input status = %d\r\n", S.inputEnabled);
  Cli.xprintf    (  "  Speed lock   = %d\r\n", S.speedLock);
  Cli.xprintf    (  "  Saved PWM    = %d\r\n", S.savedPwmDutyCycle);
  Cli.xprintf    (  "  Target PWM   = %d\r\n", S.targetPwmDutyCycle);
  Cli.xprintf    (  "  Output PWM   = %d\r\n", S.pwmDutyCycle);
  Cli.xprintf    (  "  Avg duty cyc = %d%% (%dm)\r\n", dutyCycleCalculate(), S.dutyValidSamples);
  Serial.println (  "");
  return 0;
}


/*
 * Display the system configuration
 */
int cmdConfig (int argc, char **argv)
{
  Serial.println (F("System configuration:"));
  Cli.xprintf    (  "  Min on duration    = %lds\r\n", Nvm.minOnDurationS);
  Cli.xprintf    (  "  Min off duration   = %lds\r\n", Nvm.minOffDurationS);
  Cli.xprintf    (  "  Min RPM duty cycle = %d\r\n"   , Nvm.minRpmDutyCycle);
  Cli.xprintf    (  "  Max RPM duty cycle = %d\r\n"   , Nvm.maxRpmDutyCycle);
  Cli.xprintf    (  "  Speed adjust delay = %lds\r\n" , Nvm.speedAdjustDelayS);
  Cli.xprintf    (  "  Speed adjust rate  = %d/min\r\n", Nvm.speedAdjustRate);
  Cli.xprintf    (  "  Trace enabled      = %d\r\n", Nvm.traceEnable);
  if (S.crcOk) Serial.print(F("  CRC pass "));
  else         Serial.print(F("  CRC fail "));
  Cli.xprintf ("(%lx)\r\n", Nvm.crc);
  printVersion(2);
  Serial.println (  "");
  return 0;
}


/*
 * Dump the trace log
 */
int cmdTrace (int argc, char **argv)
{
  if (argc == 2) {
    uint8_t enable = atoi(argv[1]);
    if (1 == enable) {
      Nvm.traceEnable = true;
      Trace.start();
      nvmWrite();
      Serial.println (F("Trace enabled\r\n"));
    }
    else if (0 == enable) {
      Nvm.traceEnable = false;
      Trace.stop();
      nvmWrite();
      Serial.println (F("Trace disabled\r\n"));
    }
    else {
      return 1;
    }
  }
  else {
    Serial.println (F("Trace messages:"));
    Trace.dump();
  }
  return 0;
}


/*
 * Reset EEPROM to default values
 */
int cmdReset (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint32_t confirm = atol(argv[1]);

  if (1024 == confirm) {
    Serial.println (F("Reset\r\n"));
    Nvm.magicWord = 0;
    nvmWrite();
  }
  else {
    return 1;
  }
  return 0;
}


/*
 * Print version info
 */
void printVersion (uint8_t indent)
{
  for (uint8_t i = 0; i < indent; i++) {
    Serial.print(" ");
  }
  Serial.println(F("V " QUOTE(VERSION_MAJOR) "." QUOTE(VERSION_MINOR) "." QUOTE(VERSION_MAINT)));
}


/*
 * Addtional help text
 */
void helpText (void)
{
  Serial.println(F("PWM range for Secop BD35F with 101N0212:"));
  Serial.println(F("  2000 RPM: 190 / 255 (75%)"));
  Serial.println(F("  3500 RPM:  40 / 255 (16%)"));
  Serial.println(F("     0 RPM:   0 / 255  (0%)"));
}



