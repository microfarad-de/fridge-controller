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
 * Version: 3.3.7
 * Date:    July 25, 2025
 */


#define VERSION_MAJOR 3  // Major version
#define VERSION_MINOR 3  // Minor version
#define VERSION_MAINT 7  // Maintenance version


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
#define SPEED_RAMPUP_RATE         5          // Speed adjust rate in AnalogWrite() steps per 10 sec for soft speed rampup
#define SPEED_LOCK_ON_DELAY_M     15         // Time delay in minutes for activating the speed lock
#define SPEED_LOCK_OFF_DELAY_M    10         // Time delay in minutes for deactivating the speed lock
#define REMOTE_TIMEOUT_M          10         // Remote control timeout in minutes - fall back to local control if no remote commands received during this time
#define DUTY_MEAS_NUM_SAMPLES     60         // Number of duty cycle measurement samples
#define DUTY_MEAS_SAMPLE_DUR_M    1          // Duty cycle measurement sample duration in minutes
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
  int8_t  defrost            = 0;      // Defrost cycle is active
  uint8_t pwm                = 0;      // PWM duty cycle at the output pin
  uint8_t savedPwm           = 0;      // Saved PWM duty cycle
  uint8_t targetPwm          = 0;      // Target PWM duty cycle
  uint8_t dutyMeasIdx        = 0;      // Duty cycle array index
  uint8_t dutyValidSamples   = 0;      // Number of valid duty cycle measurement samples
  uint8_t dutyCycleValue     = 0;      // Average compressor duty cycle value in percent
  uint8_t dutyMeas[DUTY_MEAS_BUF_SIZE] = { 0 };  // Duty cycle measurement array
  bool    remoteControl      = false;  // Activates remote control - ignore thermostat input value
  bool    remoteDefrost      = false;  // Activates defrost cycle over remote command
  bool    remoteOn           = false;  // Activates compressor over remote command
  uint8_t remotePwm          = 0;      // PWM value configured via remote control command
  uint32_t remoteTs          = 0;      // Millisecond timestamp for measuring remote control timeout
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
  uint8_t  minOnDurationM    = 11;     // Minimum allowed compressor on duration in minutes
  uint8_t  minOffDurationM   = 3;      // Minimum allowed compressor off duration in minutes
  uint8_t  minRpmPwm         = 190;    // PWM duty cycle for minimum compressor RPM (1..255), larger value decreases RPM
  uint8_t  maxRpmPwm         = 70;     // PWM duty cycle for maximum compressor RPM (1..255), smaller value increases RPM
  uint8_t  traceLevel        = 1;      // Trace log level
  uint8_t  speedTargetDuty   = 85;     // Target duty compressor duty cycle of speed adjustment algorithm in percent
  uint8_t  speedHysteresis   = 5;      // Hysteresis of speed adjustment algorithm in duty cycle percent
  uint8_t  speedAdjustRate   = 5;      // Increase or decrease PWM by this amount of steps per minute
  uint8_t  defrostStartRt    = 2;      // Minimum compressor runtime in hours before starting defrost
  uint8_t  defrostStartDc    = 70;     // Maximum allowed compressor duty cycle before starting deforst
  int8_t   defrostDurationM  = 45;     // Defrost cycle duration in minutes
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
  "Compressor on  %d%%",
  "Compressor off %d%%",
  "Power on",
  "Increase speed %d",
  "Decrease speed %d",
  "Speed lock %d",
  "Defrost %d",
  "Remote %d",
  "CRC FAIL",
};
enum {
  TRC_COMPRESSOR_ON,
  TRC_COMPRESSOR_OFF,
  TRC_POWER_ON,
  TRC_INCREASE_SPEED,
  TRC_DECREASE_SPEED,
  TRC_SPEED_LOCK,
  TRC_DEFROST,
  TRC_REMOTE,
  TRC_CRC_FAIL,
  TRC_COUNT
};
static_assert(TRC_COUNT == sizeof(traceMsgList)/sizeof(traceMsgList[0]));


/*
 * Trace macros
 */
#define TRACE2(level, msg)       if (Nvm.traceLevel >= level) Trace.log(msg);
#define TRACE3(level, msg, val)  if (Nvm.traceLevel >= level) Trace.log(msg, val);
#define GET_MACRO(_1, _2, _3, NAME, ...) NAME
#define TRACE(...) GET_MACRO(__VA_ARGS__, TRACE3, TRACE2)(__VA_ARGS__)

/*
 * Function declarations
 */
void powerSave (void);
void readInputPin (void);
void setPwm (void);
void ledManager   (void);
void speedManager (void);
void defrostManager (void);
void remoteManager (void);
void dutyCycleLogger (void);
uint8_t dutyCycleCalculate(void);
void nvmValidate  (void);
void nvmRead      (void);
void nvmWrite     (void);
int cmdOn     (int argc, char **argv);
int cmdOff    (int argc, char **argv);
int cmdControl (int argc, char **argv);
int cmdStatus  (int argc, char **argv);
int cmdConfig  (int argc, char **argv);
int cmdTrace   (int argc, char **argv);
int cmdSetMinOnDuration  (int argc, char **argv);
int cmdSetMinOffDuration (int argc, char **argv);
int cmdSetMinRpmPwm      (int argc, char **argv);
int cmdSetMaxRpmPwm      (int argc, char **argv);
int cmdSetSpeedDutyCycle  (int argc, char **argv);
int cmdSetSpeedHysteresis (int argc, char **argv);
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
  Cli.newCmd    ("on",      "Turn on the compressor", cmdOn);
  Cli.newCmd    ("off",     "Turn off the compressor", cmdOff);
  Cli.newCmd    ("s",       "Show the system status", cmdStatus);
  Cli.newCmd    ("r",       "Show the system configuration", cmdConfig);
  Cli.newCmd    ("t",       "Print or set the trace level ([0..2])", cmdTrace);
  Cli.newCmd    ("ond",     "Set min on duration (<0..60>m)", cmdSetMinOnDuration);
  Cli.newCmd    ("offd",    "Set min off duration (<0..60>m)", cmdSetMinOffDuration);
  Cli.newCmd    ("pwml",    "Set the PWM duty for min RPM (<1..255>)", cmdSetMinRpmPwm);
  Cli.newCmd    ("pwmh",    "Set the PWM duty for max RPM (<1..255>)", cmdSetMaxRpmPwm);
  Cli.newCmd    ("spdc",    "Set speed adjust target duty cycle (<41..99>%)", cmdSetSpeedDutyCycle);
  Cli.newCmd    ("spdh",    "Set speed adjust hysteresis (<1..40>%)", cmdSetSpeedHysteresis);
  Cli.newCmd    ("spdr",    "Set speed adjust rate (<0..255>)", cmdSetSpeedRate);
  Cli.newCmd    ("defr",    "Set defrost start runtime (<0..24>h)", cmdSetDefrostRt);
  Cli.newCmd    ("defc",    "Set defrost start duty cycle (<0..100>%)", cmdSetDefrostDc);
  Cli.newCmd    ("defd",    "Set defrost duration (<0..60>m)", cmdSetDefrostDuration);
  Cli.newCmd    ("c",       "Remote control <command>", cmdControl);

  Cli.showHelp();

  nvmRead();
  TRACE(1, TRC_POWER_ON);

  if (Nvm.traceLevel) Trace.start();
  else                Trace.stop();

  S.savedPwm = Nvm.minRpmPwm;

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
  remoteManager();
  dutyCycleLogger();
  powerSave();

  // Main state machine
  switch (S.state) {

    // Compressor OFF state entry point
    case STATE_OFF_ENTRY:
      compressorOffTs      = ts;
      S.targetPwm = 0;
      TRACE(1, TRC_COMPRESSOR_OFF, S.dutyCycleValue);

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
      if (ts - compressorOffTs > Nvm.minOffDurationM * ONE_MINUTE && S.defrost != 1) {
        S.state = STATE_OFF;
      }
      break;

    // Compressor OFF main state
    case STATE_OFF:
      if (true == S.inputEnabled) {
        S.state = STATE_ON_ENTRY;
      }
      break;

    // Compressor ON state entry point
    case STATE_ON_ENTRY:
      compressorOnTs = ts;
      S.targetPwm    = S.savedPwm;

      nvmRead ();  // Perform a CRC check
      if (S.crcOk) {
        TRACE(1, TRC_COMPRESSOR_ON, S.dutyCycleValue);
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

  if (S.remoteControl) {
    if (S.remoteOn) {
      S.inputEnabled = true;
    }
    else {
      S.inputEnabled = false;
    }
    return;
  }

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
  bool running = (S.pwm > 0);

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
bool decrementPwm (uint8_t * pwm, uint8_t steps)
{
  if (*pwm - steps > Nvm.maxRpmPwm) {
    *pwm -= steps;
    DEBUG(Serial.println("Decrement PWM"));
    return true;
  }
  else if (*pwm != Nvm.maxRpmPwm) {
    *pwm = Nvm.maxRpmPwm;
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
bool incrementPwm (uint8_t *pwm, uint8_t steps)
{
  if (*pwm + steps < Nvm.minRpmPwm) {
    *pwm += steps;
    DEBUG(Serial.println("Increment PWM"));
    return true;
  }
  else if (*pwm != Nvm.minRpmPwm) {
    *pwm = Nvm.minRpmPwm;
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
  static uint8_t  lastPwm = Nvm.minRpmPwm + SPEED_RAMPUP_RATE < 255 ? Nvm.minRpmPwm + SPEED_RAMPUP_RATE : 255;
  static uint32_t adjustTs         = 0;
  uint32_t ts = millis();

  if (S.targetPwm != S.pwm) {
    // Speed adjust disabled
    if (0 == Nvm.speedAdjustRate) {
      S.pwm = S.targetPwm;
      analogWrite(OUTPUT_PIN, S.pwm);
      lastPwm = S.pwm;
    }
    // Stop
    else if (0 == S.targetPwm) {
      DEBUG(Serial.println("Stop"));
      S.pwm = 0;
      analogWrite(OUTPUT_PIN, 0);
      lastPwm = Nvm.minRpmPwm + SPEED_RAMPUP_RATE < 255 ? Nvm.minRpmPwm + SPEED_RAMPUP_RATE : 255;
    }
    // Decrease speed
    else if (S.targetPwm >= lastPwm) {
      DEBUG(Serial.println("Ramp down"));
      S.pwm = S.targetPwm;
      analogWrite(OUTPUT_PIN, S.pwm);
      lastPwm = S.pwm;
    }
    // Increase speed slowly
    else if (S.targetPwm < lastPwm)
    {
      if (ts - adjustTs >= SPEED_RAMPUP_PERIOD_MS) {
        if (decrementPwm(&lastPwm, SPEED_RAMPUP_RATE)) {
          DEBUG(Serial.println("Ramp up"));
          S.pwm = lastPwm;
          analogWrite(OUTPUT_PIN, S.pwm);
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
  static enum {HOLD, INCREASE, DECREASE, MAX_SPEED} state = HOLD;
  static uint32_t speedLockTs = 0;
  static uint32_t adjustTs    = 0;
  static bool     maxSpeed    = false;

  uint32_t ts = millis();
  bool     on = (S.pwm > 0);

  // Speed lock handling routine
  if (STATE_OFF_ENTRY == S.state || STATE_ON_ENTRY == S.state) {
    speedLockTs = ts;
  }
  else if (STATE_OFF_WAIT == S.state || STATE_OFF == S.state) {
    if (ts - speedLockTs >= SPEED_LOCK_ON_DELAY_M * ONE_MINUTE && !S.speedLock) {
      S.speedLock = true;
      S.savedPwm  = Nvm.minRpmPwm;
      S.remotePwm = 0;
      TRACE(2, TRC_SPEED_LOCK, S.speedLock);
    }
  }
  else if (STATE_ON_WAIT == S.state || STATE_ON == S.state) {
    if (ts - speedLockTs >= SPEED_LOCK_OFF_DELAY_M * ONE_MINUTE && S.speedLock && S.crcOk) {
      S.speedLock = false;
      TRACE(2, TRC_SPEED_LOCK, S.speedLock);
    }
  }

  if (1 == S.defrost) {
    state    = HOLD;
    //maxSpeed = true;
    return;
  }

  if (0 == Nvm.speedAdjustRate || S.dutyValidSamples < DUTY_MEAS_NUM_SAMPLES || S.speedLock) {
    state = HOLD;
    return;
  }

  switch (state) {
    case HOLD:
        if (on && maxSpeed){
          state = MAX_SPEED;
        }
        else if (on && S.dutyCycleValue > Nvm.speedTargetDuty) {
          adjustTs = ts - SPEED_ADJUST_PERIOD_MS;
          state    = INCREASE;
        }
        else if (!on && S.dutyCycleValue < Nvm.speedTargetDuty - Nvm.speedHysteresis) {
          adjustTs = ts - SPEED_ADJUST_PERIOD_MS;
          state    = DECREASE;
        }
      break;

    case INCREASE:
      if (!on) {
        state = HOLD;
      }
      else if (ts - adjustTs >= SPEED_ADJUST_PERIOD_MS) {
        if (decrementPwm(&S.savedPwm, Nvm.speedAdjustRate)) {
          S.targetPwm = S.savedPwm;
          TRACE(2, TRC_INCREASE_SPEED, S.savedPwm);
        }
        maxSpeed = false;
        adjustTs = ts;
      }
      break;

    case DECREASE:
      if (on) {
        state = HOLD;
      }
      else if (ts - adjustTs >= SPEED_ADJUST_PERIOD_MS) {
        if (incrementPwm(&S.savedPwm, Nvm.speedAdjustRate)) {
          TRACE(2, TRC_DECREASE_SPEED, S.savedPwm);
        }
        adjustTs = ts;
      }
      break;

    case MAX_SPEED:
      if (!on) {
        state = HOLD;
      }
      else if (maxSpeed) {
        S.savedPwm  = Nvm.maxRpmPwm;
        S.targetPwm = S.savedPwm;
        TRACE(1, TRC_INCREASE_SPEED, S.savedPwm);
        maxSpeed = false;
      }
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

  uint32_t ts = millis();
  bool     on = (S.pwm > 0);

  if (ts - secondTs >= ONE_SECOND) {
    secondTs += ONE_SECOND;
    if (on) {
      runtimeS++;
    }
  }

  if (Nvm.defrostDurationM <= 0) {
    S.defrost = -1;
    return;
  }
  else if (S.defrost < 0) {
    S.defrost = 0;
  }


  // Defrost off
  if (0 == S.defrost) {
    if ((runtimeS * ONE_SECOND >= Nvm.defrostStartRt * ONE_HOUR && S.dutyCycleValue <= Nvm.defrostStartDc) || S.remoteDefrost) {
      runtimeS   = (Nvm.defrostStartRt * (ONE_HOUR / ONE_SECOND)) / 2;  // Restart after half of the runtime duration if interrupted
      S.defrost = 2;
    }
  }
  // Prepare defrost
  else if (2 == S.defrost) {
    if (STATE_OFF_ENTRY == S.state) {
      durationTs = ts;
      S.defrost  = 1;
      S.remoteDefrost = false;
      TRACE(1, TRC_DEFROST, 1);
    }
  }
  // Defrost on
  else {
    if (ts - durationTs >= Nvm.defrostDurationM * ONE_MINUTE) {
      runtimeS   = 0;
      S.defrost  = 0;
      TRACE(1, TRC_DEFROST, 0);
    }
  }
}


/*
 * Remote control handling routine
*/
void remoteManager (void)
{
  if (!S.remoteControl) {
    S.remotePwm     = 0;
    S.remoteOn      = false;
    S.remoteDefrost = false;
    return;
  }

  uint32_t ts = millis();

  if (ts - S.remoteTs >= REMOTE_TIMEOUT_M * ONE_MINUTE) {
    if (S.remoteControl) {
      TRACE(1, TRC_REMOTE, 0);
      S.remoteControl = false;
    }
    return;
  }

  if (S.remotePwm > 0) {
    if (!S.speedLock) {
      if (S.savedPwm > S.remotePwm) {
        S.savedPwm = S.remotePwm;
        TRACE(1, TRC_INCREASE_SPEED, S.savedPwm);
      }
      else if (S.savedPwm < S.remotePwm){
        S.savedPwm = S.remotePwm;
        TRACE(1, TRC_DECREASE_SPEED, S.savedPwm);
      }
      bool on = (S.pwm > 0);
      if (on) {
        S.targetPwm = S.savedPwm;
      }
      S.remotePwm = 0;
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
  const  uint32_t sampleDuration = DUTY_MEAS_SAMPLE_DUR_M * ONE_MINUTE;

  uint32_t ts = millis();
  uint32_t delta;
  bool     on = (S.pwm > 0);

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
    S.dutyCycleValue = dutyCycleCalculate();
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

  result = Nvm.minRpmPwm > 0;
  if (!result) {
    Nvm.minRpmPwm = NvmInit.minRpmPwm;
  }

  result  = Nvm.maxRpmPwm <= Nvm.minRpmPwm;
  result &= Nvm.maxRpmPwm > 0;
  if (!result) {
    if (NvmInit.maxRpmPwm <= Nvm.minRpmPwm) {
      Nvm.maxRpmPwm = NvmInit.maxRpmPwm;
    }
    else {
      Nvm.maxRpmPwm = Nvm.minRpmPwm;
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

  result  = Nvm.speedTargetDuty <= 99;
  result &= Nvm.speedTargetDuty >= 41;
  if (!result) {
    Nvm.speedTargetDuty = NvmInit.speedTargetDuty;
  }

  result  = Nvm.speedHysteresis <= 40;
  result &= Nvm.speedHysteresis >= 1;
  if (!result) {
    Nvm.speedHysteresis = NvmInit.speedHysteresis;
  }

  result  = Nvm.speedAdjustRate <= 255;
  result &= Nvm.speedAdjustRate >= 0;
  if (!result) {
    Nvm.speedAdjustRate = NvmInit.speedAdjustRate;
  }

  result = Nvm.defrostStartRt <= 24;
  if (!result) {
    Nvm.defrostStartRt = NvmInit.defrostStartRt;
  }

  result = Nvm.defrostStartDc <= 100;
  if (!result) {
    Nvm.defrostStartDc = NvmInit.defrostStartDc;
  }

  result  = Nvm.defrostDurationM <= 60;
  result &= Nvm.defrostDurationM >= -60;
  if (!result) {
    Nvm.defrostDurationM = NvmInit.defrostDurationM;
  }

  result = Nvm.traceLevel <= 2;
  if (!result) {
    Nvm.traceLevel = NvmInit.traceLevel;
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
    TRACE(1, TRC_CRC_FAIL);
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
 * Process remote control command
 */
int cmdControl (int argc, char **argv)
{
  if (argc != 2) {
    Serial.println('{');
    Serial.println(F("  \"error\": 1"));
    Serial.println('}');
    return 1;
  }

  uint8_t value = atoi(argv[1]);

  // 0: Compressor off
  if (0 == value) {
    S.remoteOn = false;
  }
  // 1..10: Set speed
  else if (value <= 10) {
    S.remotePwm = map(value, 1, 10, Nvm.minRpmPwm, Nvm.maxRpmPwm);
  }
  // 11: Compressor on
  else if (11 == value) {
    S.remoteOn = true;
  }
  // 12: Disable remote
  else if (12 == value) {
    if (S.remoteControl) {
      S.remoteControl = false;
      TRACE(1, TRC_REMOTE, 0);
    }
  }
  // 20: Defrost off
  else if (20 == value) {
    if (1 == S.defrost) {
      S.defrost = 0;
      TRACE(1, TRC_DEFROST, 0);
    }
    if (S.remoteDefrost) {
      S.remoteDefrost = false;
    }
  }
  // 21: Defrost on
  else if (21 == value) {
    if (0 == S.defrost) {
      S.remoteDefrost = true;
    }
  }
  // 22: Disable defrost
  else if (22 == value) {
    if (Nvm.defrostDurationM > 0) {
      Nvm.defrostDurationM = -Nvm.defrostDurationM;
      nvmWrite();
      TRACE(1, TRC_DEFROST, -1);
    }
    if (S.remoteDefrost) {
      S.remoteDefrost = false;
    }
  }
  // 23: Enable defrost
  else if (23 == value) {
    if (Nvm.defrostDurationM < 0) {
      Nvm.defrostDurationM = -Nvm.defrostDurationM;
      nvmWrite();
      TRACE(1, TRC_DEFROST, 0);
    }
  }
  // 30: System status
  else if (30 == value) {
    Serial.println('{');
    Serial.println(F("  \"ok\": 30,"));
    Serial.print(F("  \"state\": ")); Serial.print(S.state, DEC); Serial.println(',');
    Serial.print(F("  \"inputEnabled\": ")); Serial.print(S.inputEnabled, DEC); Serial.println(',');
    Serial.print(F("  \"speedLock\": ")); Serial.print(S.speedLock, DEC); Serial.println(',');
    Serial.print(F("  \"defrost\": ")); Serial.print(S.defrost, DEC); Serial.println(',');
    Serial.print(F("  \"remote\": ")); Serial.print(S.remoteControl, DEC); Serial.println(',');
    Serial.print(F("  \"savedPwm\": ")); Serial.print(S.savedPwm, DEC); Serial.println(',');
    Serial.print(F("  \"targetPwm\": ")); Serial.print(S.targetPwm, DEC); Serial.println(',');
    Serial.print(F("  \"outputPwm\": ")); Serial.print(S.pwm, DEC); Serial.println(',');
    Serial.print(F("  \"dutyCycle\": ")); Serial.println(S.dutyCycleValue, DEC);
    Serial.println('}');
  }
  else {
    Serial.println('{');
    Serial.println(F("  \"error\": 2"));
    Serial.println('}');
    return 1;
  }

  if (value != 30) {
    Serial.println('{');
    Serial.print(F("  \"ok\": ")); Serial.println(value, DEC);
    Serial.println('}');
  }

  if (value < 12) {
    if (!S.remoteControl) {
      TRACE(1, TRC_REMOTE, 1);
      S.remoteControl = true;
    }
    S.remoteTs = millis();
  }

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
int cmdSetMinRpmPwm (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t pwm   = atoi(argv[1]);
  Nvm.minRpmPwm = pwm;
  nvmWrite();
  Serial.print(F("PWM duty cycle at min RPM = "));
  Serial.println(Nvm.minRpmPwm, DEC);
  Serial.println("");
  return 0;
}


/*
 * Set the PWM duty cycle corresponding to minimum RPM
 */
int cmdSetMaxRpmPwm (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t pwm   = atoi(argv[1]);
  Nvm.maxRpmPwm = pwm;
  nvmWrite();
  Serial.print(F("PWM duty cycle at max RPM = "));
  Serial.println(Nvm.maxRpmPwm, DEC);
  Serial.println("");
  return 0;
}


/*
 * Set the speed increase delay
 */
int cmdSetSpeedDutyCycle (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t dutyCycle   = atoi(argv[1]);
  Nvm.speedTargetDuty = dutyCycle;
  nvmWrite();
  Serial.print(F("Speed target duty cycle = "));
  Serial.print(Nvm.speedTargetDuty, DEC);
  Serial.println(F("%\r\n"));
  return 0;
}


/*
 * Set the speed decrease delay
 */
int cmdSetSpeedHysteresis (int argc, char **argv)
{
  if (argc != 2) {
    return 1;
  }
  uint8_t hyst        = atoi(argv[1]);
  Nvm.speedHysteresis = hyst;
  nvmWrite();
  Serial.print(F("Speed hysteresis = "));
  Serial.print(Nvm.speedHysteresis, DEC);
  Serial.println(F("%\r\n"));
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
    S.savedPwm = Nvm.minRpmPwm;
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
  int8_t duration      = atoi(argv[1]);
  Nvm.defrostDurationM = duration;
  nvmWrite();
  if (0 == Nvm.defrostDurationM) {
    if (S.defrost) {
      TRACE(1, TRC_DEFROST, 0);
      S.defrost = 0;
    }
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
  Serial.print(F("  Remote       = ")); Serial.println(S.remoteControl, DEC);
  Serial.print(F("  Saved PWM    = ")); Serial.println(S.savedPwm, DEC);
  Serial.print(F("  Target PWM   = ")); Serial.println(S.targetPwm, DEC);
  Serial.print(F("  Output PWM   = ")); Serial.println(S.pwm, DEC);
  Serial.print(F("  Duty cycle   = ")); Serial.print(S.dutyCycleValue, DEC);
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
  Serial.print(F("  Min on duration   = ")); Serial.print(Nvm.minOnDurationM,   DEC); Serial.println(F("m"));
  Serial.print(F("  Min off duration  = ")); Serial.print(Nvm.minOffDurationM,  DEC); Serial.println(F("m"));
  Serial.print(F("  PWM at min RPM    = ")); Serial.println(Nvm.minRpmPwm,      DEC);
  Serial.print(F("  PWM at max RPM    = ")); Serial.println(Nvm.maxRpmPwm,      DEC);
  Serial.print(F("  Speed target DC   = ")); Serial.print(Nvm.speedTargetDuty,  DEC); Serial.println(F("%"));
  Serial.print(F("  Speed hysteresis  = ")); Serial.print(Nvm.speedHysteresis,  DEC); Serial.println(F("%"));
  Serial.print(F("  Speed adjust rate = ")); Serial.print(Nvm.speedAdjustRate,  DEC); Serial.println(F("/m"));
  Serial.print(F("  Defrost start RT  = ")); Serial.print(Nvm.defrostStartRt,   DEC); Serial.println(F("h"));
  Serial.print(F("  Defrost start DC  = ")); Serial.print(Nvm.defrostStartDc,   DEC); Serial.println(F("%"));
  Serial.print(F("  Defrost duration  = ")); Serial.print(Nvm.defrostDurationM, DEC); Serial.println(F("m"));
  Serial.print(F("  Trace level       = ")); Serial.println(Nvm.traceLevel,     DEC);
  if (S.crcOk) Serial.print(F("  CRC PASS ("));
  else         Serial.print(F("  CRC FAIL ("));
  Serial.print(Nvm.crc, HEX); Serial.println(F(")"));
  printVersion(2);
  Serial.println ("");
  return 0;
}


/*
 * Dump the trace log or set the trace level
 */
int cmdTrace (int argc, char **argv)
{
  if (2 == argc) {
    uint8_t level = atoi(argv[1]);
    Nvm.traceLevel = level;
    nvmWrite();
    if (0 == Nvm.traceLevel) {
      Trace.stop();
    }
    else {
      Trace.start();
    }
    Serial.print(F("Trace level = "));
    Serial.println(Nvm.traceLevel, DEC);
    Serial.println("");
    return 0;
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



