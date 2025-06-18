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
 * Version: 2.1.0
 * Date:    June 16, 2025
 */


#define VERSION_MAJOR 2  // Major version
#define VERSION_MINOR 1  // Minor version
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
#define SPEED_LOCK_ON_DELAY_M     30         // Time delay in minutes for activating the speed lock
#define SPEED_LOCK_OFF_DELAY_M    10         // Time delay in minutes for deactivating the speed lock
#define DUTY_MEAS_NUM_SAMPLES     60         // Number of duty cycle measurement samples
#define DUTY_MEAS_SAMPLE_DUR_M    1          // Duty cycle measurement sample duration in minutes
#define DUTY_MEAS_TIEMOUT_M       60         // Duty cycle measurement is reset after this duration in minutes
#define DUTY_MEAS_BUF_SIZE        (DUTY_MEAS_NUM_SAMPLES + 1)  // Duty cycle measurement buffer size

#define ONE_SECOND  (uint32_t)1000     // One second duration in milliseconds
#define ONE_MINUTE  (uint32_t)60000    // One minute duration in milliseconds
#define ONE_HOUR    (uint32_t)3600000  // One hour duration in milliseconds

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
  bool    defrost            = false;  // Defrost cycle is active
  uint8_t pwmDutyCycle       = 0;      // PWM duty cycle at the output pin
  uint8_t savedPwmDutyCycle  = 0;      // Saved PWM duty cycle
  uint8_t targetPwmDutyCycle = 0;      // Target PWM duty cycle
  uint8_t dutyMeasIdx        = 0;      // Duty cycle array index
  uint8_t dutyValidSamples   = 0;      // Number of valid duty cycle measurement samples
  uint8_t dutyMeas[DUTY_MEAS_BUF_SIZE] = { 0 };  // Duty cycle measurement array
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
  uint8_t  minOnDurationM    = 10;     // Minimum allowed compressor on duration in minutes
  uint8_t  minOffDurationM   = 5;      // Minimum allowed compressor off duration in minutes
  uint8_t  minRpmDutyCycle   = 190;    // PWM duty cycle for minimum compressor RPM (1..255), larger value decreases RPM
  uint8_t  maxRpmDutyCycle   = 100;    // PWM duty cycle for maximum compressor RPM (1..255), smaller value increases RPM
  uint8_t  traceEnable       = 1;      // Enable the trace loggings
  uint8_t  speedIncrDelayM   = 10;     // Wait this amount of time in minutes before increasing compressor speed
  uint8_t  speedDecrDelayM   = 1;      // Wait this amount of time in minutes before decreasing compressor speed
  uint8_t  speedAdjustRate   = 5;      // Increase or decrease PWM by this amount of steps per minute
  uint8_t  defrostStartRt    = 3;      // Minimum compressor runtime in hours before starting defrost
  uint8_t  defrostStartDc    = 50;     // Maximum allowed compressor duty cycle before starting deforst
  uint8_t  defrostDurationM  = 45;     // Defrost cycle duration in minutes
  uint8_t  reserved[5];                // Reserved for future use
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
  "Defrost %d",
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
  TRC_DEFROST,
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
void defrostManager (void);
void dutyCycleLogger (void);
uint8_t dutyCycleCalculate(void);
void nvmValidate  (void);
void nvmRead      (void);
void nvmWrite     (void);
int cmdOn     (int argc, char **argv);
int cmdOff    (int argc, char **argv);
int cmdStatus (int argc, char **argv);
int cmdConfig (int argc, char **argv);
int cmdTrace  (int argc, char **argv);
int cmdSetMinOnDuration  (int argc, char **argv);
int cmdSetMinOffDuration (int argc, char **argv);
int cmdSetMinDutyCycle   (int argc, char **argv);
int cmdSetMaxDutyCycle   (int argc, char **argv);
int cmdSetSpeedIncrDelay (int argc, char **argv);
int cmdSetSpeedDecrDelay (int argc, char **argv);
int cmdSetSpeedRate      (int argc, char **argv);
int cmdSetDefrostRt      (int argc, char **argv);
int cmdSetDefrostDc       (int argc, char **argv);
int cmdSetDefrostDuration (int argc, char **argv);
void printVersion (uint8_t indent);
void helpText (void);
void callback (void);


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
  Trace.initialize(sizeof(Nvm), TRACE_BUF_SIZE, TRACE_STAMP_RESOLUTION_MS, traceMsgList, TRC_COUNT, callback);
  Cli.init(SERIAL_BAUD, false, helpText);

  Serial.println(F("\r\n+ + +  F R I D G E  C O N T R O L L E R  + + +\r\n"));
  printVersion(0);
  Cli.newCmd    ("on",      "Turn on the compressor",  cmdOn);
  Cli.newCmd    ("off",     "Turn off the compressor", cmdOff);
  Cli.newCmd    ("s",       "Show the system status",        cmdStatus);
  Cli.newCmd    ("r",       "Show the system configuration", cmdConfig);
  Cli.newCmd    ("t",       "Print the trace log or enable/disable trace (arg: [0,1])", cmdTrace);
  Cli.newCmd    ("ond",     "Set min on duration (arg: <0..60>m)",  cmdSetMinOnDuration);
  Cli.newCmd    ("offd",    "Set min off duration (arg: <0..60>m)", cmdSetMinOffDuration);
  Cli.newCmd    ("pwml",    "Set the PWM duty for min RPM (arg: <1..255>)", cmdSetMinDutyCycle);
  Cli.newCmd    ("pwmh",    "Set the PWM duty for max RPM (arg: <1..255>)", cmdSetMaxDutyCycle);
  Cli.newCmd    ("spdi",    "Set speed increase delay (arg: <0..60>m)", cmdSetSpeedIncrDelay);
  Cli.newCmd    ("spdd",    "Set speed decrease delay (arg: <0..60>m)", cmdSetSpeedDecrDelay);
  Cli.newCmd    ("spdr",    "Set speed adjust rate (arg <0..255>)", cmdSetSpeedRate);
  Cli.newCmd    ("defr",    "Set defrost start runtime (arg <0..24>h)", cmdSetDefrostRt);
  Cli.newCmd    ("defc",    "Set defrost start duty cycle (arg <0..100>%)", cmdSetDefrostDc);
  Cli.newCmd    ("defd",    "Set defrost duration (arg <0..60>m)", cmdSetDefrostDuration);

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
  defrostManager();
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
      if (ts - compressorOffTs > Nvm.minOffDurationM * ONE_MINUTE && !S.defrost) {
        S.state = STATE_OFF;
      }
      break;

    // Compressor OFF main state
    case STATE_OFF:
      if (true == S.inputEnabled) {
        S.state = STATE_ON_ENTRY;
      }
      if (ts - speedLockTs > SPEED_LOCK_ON_DELAY_M * ONE_MINUTE && !S.speedLock) {
        S.speedLock = true;
        S.savedPwmDutyCycle = Nvm.minRpmDutyCycle;
        Trace.log(TRC_SPEED_LOCK, S.speedLock);
      }
      break;

    // Compressor ON state entry point
    case STATE_ON_ENTRY:
      compressorOnTs       = ts;
      speedLockTs          = ts;
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
      if (ts - compressorOnTs > Nvm.minOnDurationM * ONE_MINUTE) {
        S.state = STATE_ON;
      }
      break;

    // Compressor ON main state
    case STATE_ON:
      if (false == S.inputEnabled) {
        S.state = STATE_OFF_ENTRY;
      }
      if (ts - speedLockTs > SPEED_LOCK_OFF_DELAY_M * ONE_MINUTE && S.speedLock && S.crcOk) {
        S.speedLock = false;
        Trace.log(TRC_SPEED_LOCK, S.speedLock);
      }
      break;

    default:
      break;
  }
}


/*
 * Callback function for executing time critical tasks
 * during trace dump operation
 */
void callback (void) {

  Led.loopHandler();
  dutyCycleLogger();
  defrostManager();
  wdt_reset();

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
    if (LOW == digitalRead(INPUT_PIN)) inputTs = ts;
    if (ts - inputTs > INPUT_DEBOUNCE_DELAY_MS) {
      S.inputEnabled = false;
    }
  }
  else {
    if (HIGH == digitalRead(INPUT_PIN)) inputTs = ts;
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
    if (0 == Nvm.speedAdjustRate) {
      S.pwmDutyCycle = S.targetPwmDutyCycle;
      analogWrite(OUTPUT_PIN, S.pwmDutyCycle);
      lastPwmDutyCycle = S.pwmDutyCycle;
    }
    // Stop
    else if (0 == S.targetPwmDutyCycle) {
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
  static uint32_t startTs   = 0;
  static uint32_t adjustTs  = 0;

  if (0 == Nvm.speedAdjustRate) {
    return;
  }

  uint32_t ts = millis();

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
      if ((ts - startTs > Nvm.speedIncrDelayM * ONE_MINUTE) && !S.speedLock) {
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
      if (ts - startTs > Nvm.speedDecrDelayM * ONE_MINUTE) {
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
 * Defrost control routine
 */
void defrostManager (void)
{
  static uint32_t durationTs = 0;
  static uint32_t secondTs   = 0;
  static uint32_t runtimeS   = 0;
  static uint8_t  dutyCycle  = 0;

  uint32_t ts = millis();
  bool     on = (S.pwmDutyCycle > 0);

  if (ts - secondTs >= ONE_SECOND) {
    secondTs += ONE_SECOND;
    if (on) {
      runtimeS++;
    }
    dutyCycle = dutyCycleCalculate();
  }

  if (0 == Nvm.defrostDurationM) {
    return;
  }

  if (false == S.defrost) {
    if (runtimeS * ONE_SECOND >= Nvm.defrostStartRt * ONE_HOUR && dutyCycle <= Nvm.defrostStartDc) {
      durationTs = ts;
      runtimeS   = 0;
      S.defrost  = true;
      S.state    = STATE_OFF_ENTRY;
      Trace.log(TRC_DEFROST, 1);
    }
  }
  else {
    if (ts - durationTs >= Nvm.defrostDurationM * ONE_MINUTE) {
      S.defrost  = false;
      Trace.log(TRC_DEFROST, 0);
    }
  }
}


/*
 * Compressor duty cycle logging routine
 */
void dutyCycleLogger (void)
{
  static uint32_t captureTs = 0;
  static uint32_t sampleTs  = 0;
  static uint32_t resetTs   = -DUTY_MEAS_TIEMOUT_M * ONE_MINUTE;
  const  uint32_t sampleDuration = DUTY_MEAS_SAMPLE_DUR_M * ONE_MINUTE;

  uint32_t ts = millis();
  uint32_t delta;
  bool     on = (S.pwmDutyCycle > 0);

  if (on) {
    resetTs = ts;
  }

  if (ts - resetTs >= DUTY_MEAS_TIEMOUT_M * ONE_MINUTE) {
    S.dutyValidSamples = 0;
    captureTs = ts;
    sampleTs  = ts;
  }

  delta = ts - captureTs;
  if (delta >= ONE_SECOND) {
    if (delta >= 2 * ONE_SECOND) {
      captureTs = ts;
    }
    else {
      captureTs += ONE_SECOND;
    }
    S.dutyMeas[S.dutyMeasIdx] += on;
  }

  delta = ts - sampleTs;
  if (delta >= sampleDuration) {
    if (delta >= 2 * sampleDuration) {
      sampleTs = ts;
    }
    else {
      sampleTs += sampleDuration;
    }

    S.dutyMeasIdx++;
    if (S.dutyMeasIdx >= DUTY_MEAS_BUF_SIZE) {
      S.dutyMeasIdx = 0;
    }
    S.dutyMeas[S.dutyMeasIdx] = 0;
    if (S.dutyValidSamples < DUTY_MEAS_NUM_SAMPLES) {
      S.dutyValidSamples++;
    }
  }
}


/*
 * Calculate the compressor duty cycle in percent
 */
uint8_t dutyCycleCalculate(void)
{
  uint32_t sum   = 0;
  uint32_t total = S.dutyValidSamples * DUTY_MEAS_SAMPLE_DUR_M * 60;
  int16_t  idx   = S.dutyMeasIdx;

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

  result = Nvm.minOnDurationM <= 60;
  if (!result) {
    Nvm.minOnDurationM = NvmInit.minOnDurationM;
  }

  result = Nvm.minOffDurationM <= 60;
  if (!result) {
    Nvm.minOffDurationM = NvmInit.minOffDurationM;
  }

  result = Nvm.speedIncrDelayM <= 60;
  if (!result) {
    Nvm.speedIncrDelayM = NvmInit.speedIncrDelayM;
  }

  result = Nvm.speedDecrDelayM <= 60;
  if (!result) {
    Nvm.speedDecrDelayM = NvmInit.speedDecrDelayM;
  }

  result  = Nvm.speedAdjustRate <= 255;
  result &= Nvm.speedAdjustRate >= 0;
  if (!result) {
    Nvm.speedAdjustRate = NvmInit.speedAdjustRate;
  }

  result  = Nvm.defrostStartRt <= 24;
  if (!result) {
    Nvm.defrostStartRt = NvmInit.defrostStartRt;
  }

  result  = Nvm.defrostStartDc <= 100;
  if (!result) {
    Nvm.defrostStartDc = NvmInit.defrostStartDc;
  }

  result  = Nvm.defrostDurationM <= 60;
  if (!result) {
    Nvm.defrostDurationM = NvmInit.defrostDurationM;
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
 * Set the minimum allowed compressor on duration
 */
int cmdSetMinOnDuration (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t duration   = atoi(argv[1]);
  Nvm.minOnDurationM = duration;
  nvmWrite();
  Serial.print(F("Min on duration = "));
  Serial.print(Nvm.minOnDurationM, DEC);
  Serial.println(F("m\r\n"));
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
  uint8_t duration    = atoi(argv[1]);
  Nvm.minOffDurationM = duration;
  nvmWrite();
  Serial.print(F("Min off duration = "));
  Serial.print(Nvm.minOffDurationM, DEC);
  Serial.println(F("m\r\n"));
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
  Serial.print(F("PWM duty cycle at min RPM = "));
  Serial.println(Nvm.minRpmDutyCycle, DEC);
  Serial.println("");
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
  Serial.print(F("PWM duty cycle at max RPM = "));
  Serial.println(Nvm.maxRpmDutyCycle, DEC);
  Serial.println("");
  return 0;
}


/*
 * Set the speed increase delay
 */
int cmdSetSpeedIncrDelay (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t delay       = atoi(argv[1]);
  Nvm.speedIncrDelayM = delay;
  nvmWrite();
  Serial.print(F("Speed increase delay = "));
  Serial.print(Nvm.speedIncrDelayM, DEC);
  Serial.println(F("m\r\n"));
  return 0;
}


/*
 * Set the speed decrease delay
 */
int cmdSetSpeedDecrDelay (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t delay       = atoi(argv[1]);
  Nvm.speedDecrDelayM = delay;
  nvmWrite();
  Serial.print(F("Speed decrease delay = "));
  Serial.print(Nvm.speedDecrDelayM, DEC);
  Serial.println(F("m\r\n"));
  return 0;
}


/*
 * Set the speed adjust rate
 */
int cmdSetSpeedRate (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t rate        = atoi(argv[1]);
  Nvm.speedAdjustRate = rate;
  nvmWrite();
  if (0 == Nvm.speedAdjustRate) {
    S.savedPwmDutyCycle = Nvm.minRpmDutyCycle;
  }
  Serial.print(F("Speed adjust rate = "));
  Serial.print(Nvm.speedAdjustRate, DEC);
  Serial.println(F("/m\r\n"));
  return 0;
}


/*
 * Set the minimum compressor runtime for defrost to start
 */
int cmdSetDefrostRt (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t interval     = atoi(argv[1]);
  Nvm.defrostStartRt = interval;
  nvmWrite();
  Serial.print(F("Defrost start runtime = "));
  Serial.print(Nvm.defrostStartRt, DEC);
  Serial.println(F("h\r\n"));
  return 0;
}


/*
 * Set maximum allowed compressor duty cycle for deforst to start
 */
int cmdSetDefrostDc (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t dc         = atoi(argv[1]);
  Nvm.defrostStartDc = dc;
  nvmWrite();
  Serial.print(F("Defrost start duty cycle = "));
  Serial.print(Nvm.defrostStartDc, DEC);
  Serial.println(F("%\r\n"));
  return 0;
}


/*
 * Set the defrost cycle duratrion
 */
int cmdSetDefrostDuration (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t duration     = atoi(argv[1]);
  Nvm.defrostDurationM = duration;
  nvmWrite();
  if (0 == Nvm.defrostDurationM) {
    S.defrost = false;
  }
  Serial.print(F("Defrost duration = "));
  Serial.print(Nvm.defrostDurationM, DEC);
  Serial.println(F("m\r\n"));
  return 0;
}


/*
 * Display the system status
 */
int cmdStatus (int argc, char **argv)
{
  Serial.println (F("System status:"));
  Serial.print(F("  State        = ")); Serial.println(S.state, DEC);
  Serial.print(F("  Input status = ")); Serial.println(S.inputEnabled, DEC);
  Serial.print(F("  Speed lock   = ")); Serial.println(S.speedLock, DEC);
  Serial.print(F("  Defrost      = ")); Serial.println(S.defrost, DEC);
  Serial.print(F("  Saved PWM    = ")); Serial.println(S.savedPwmDutyCycle, DEC);
  Serial.print(F("  Target PWM   = ")); Serial.println(S.targetPwmDutyCycle, DEC);
  Serial.print(F("  Output PWM   = ")); Serial.println(S.pwmDutyCycle, DEC);
  Serial.print(F("  Duty cycle   = ")); Serial.print(dutyCycleCalculate(), DEC);
  Serial.print(F("% (")); Serial.print(S.dutyValidSamples * DUTY_MEAS_SAMPLE_DUR_M, DEC); Serial.println(F("m)"));
  Serial.println("");
  return 0;
}


/*
 * Display the system configuration
 */
int cmdConfig (int argc, char **argv)
{
  Serial.println (F("System configuration:"));
  Serial.print(F("  Min on duration    = ")); Serial.print(Nvm.minOnDurationM,  DEC); Serial.println(F("m"));
  Serial.print(F("  Min off duration   = ")); Serial.print(Nvm.minOffDurationM, DEC); Serial.println(F("m"));
  Serial.print(F("  Min RPM duty cycle = ")); Serial.println(Nvm.minRpmDutyCycle, DEC);
  Serial.print(F("  Max RPM duty cycle = ")); Serial.println(Nvm.maxRpmDutyCycle, DEC);
  Serial.print(F("  Speed incr delay   = ")); Serial.print(Nvm.speedIncrDelayM,  DEC); Serial.println(F("m"));
  Serial.print(F("  Speed decr delay   = ")); Serial.print(Nvm.speedDecrDelayM,  DEC); Serial.println(F("m"));
  Serial.print(F("  Speed adjust rate  = ")); Serial.print(Nvm.speedAdjustRate,  DEC); Serial.println(F("/m"));
  Serial.print(F("  Defrost start RT   = ")); Serial.print(Nvm.defrostStartRt,   DEC); Serial.println(F("h"));
  Serial.print(F("  Defrost start DC   = ")); Serial.print(Nvm.defrostStartDc,   DEC); Serial.println(F("%"));
  Serial.print(F("  Defrost duration   = ")); Serial.print(Nvm.defrostDurationM, DEC); Serial.println(F("m"));
  Serial.print(F("  Trace enabled      = ")); Serial.println(Nvm.traceEnable, DEC);
  if (S.crcOk) Serial.print(F("  CRC PASS ("));
  else         Serial.print(F("  CRC FAIL ("));
  Serial.print(Nvm.crc, HEX); Serial.println(F(")"));
  printVersion(2);
  Serial.println ("");
  return 0;
}


/*
 * Dump the trace log
 */
int cmdTrace (int argc, char **argv)
{
  if (2 == argc) {
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



