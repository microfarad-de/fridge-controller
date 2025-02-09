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
 * Copyright (C) 2022 Karim Hraibi (khraibi@gmail.com)
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
 * Version: 1.0.0
 * Date:    February 08, 2025
 */


#define VERSION_MAJOR 1  // Major version
#define VERSION_MINOR 0  // Minor version
#define VERSION_MAINT 0  // Maintenance version


#include <Arduino.h>
#include "src/Cli/Cli.h"
#include "src/Led/Led.h"
#include "src/Nvm/Nvm.h"
#include "src/Trace/Trace.h"
#include <avr/wdt.h>

/*
 * Pin assignment
 */
#define INPUT_PIN   12
#define OUTPUT_PIN  11
#define LED_PIN     LED_BUILTIN  // 13


/*
 * Configuration parameters
 */
#define SERIAL_BAUD               57600      // Serial communication baud rate
#define NVM_MAGIC_WORD            0xDEADBEEF // Magic word stored in correctly initialized NVM
#define TRACE_BUF_SIZE            100        // Trace buffer size in words
#define TRACE_STAMP_RESOLUTION_MS 60000      // Trace time stamp resolution in milliseconds
#define INPUT_DEBOUNCE_DELAY_MS   1000       // Input debounce time delay in ms


/*
 * State machine state definitions
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
  State_e state = STATE_OFF_ENTRY;   // Main state machine state
  bool    inputEnabled     = false;  // Input pin state after debounce
  uint8_t pwmDutyCycle     = 0;      // Current PWM duty cycle at the output pin
  uint8_t lastPwmDutyCycle = 0;      // Previous PWM duty cycle
} S;



/*
 * Configuration parameters
 */
struct Nvm_t {
  uint32_t magicWord = NVM_MAGIC_WORD; // Magic word proves correctly initialized NVM
  uint32_t minOnDurationS  = 240;      // Minimum allowed compressor on duration in seconds
  uint32_t minOffDurationS = 60;       // Minimum allowed compressor off duration in seconds
  uint8_t  minRpmDutyCycle = 255;      // PWM duty cycle for minimum compressor RPM
  uint8_t  maxRpmDutyCycle = 100;      // PWM duty cycle for maximum compressor RPM
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
  "Compressor off"
};
enum {
  TRC_COMPRESSOR_ON,
  TRC_COMPRESSOR_OFF,
  TRC_COUNT
};
static_assert(TRC_COUNT == sizeof(traceMsgList)/sizeof(traceMsgList[0]));



/*
 * Function declarations
 */
void readInputPin (void);
void setOutputPin (void);
void ledManager   (void);
void speedManager (void);
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






/*
 * Arduino initialization routine
 */
void setup (void)
{
  // Set PWM frequency on pin 3 and 11 are controlled by Timer/Counter 2
  // See ATmega328P datasheet Section 21.11.2, Table 22-10
  TCCR2B = (TCCR2B & B11111000) | B00000010;

  pinMode(INPUT_PIN, INPUT_PULLUP);
  pinMode(OUTPUT_PIN, OUTPUT);

  digitalWrite(OUTPUT_PIN, LOW);

  Led.initialize(LED_PIN);
  Trace.initialize(sizeof(Nvm), TRACE_BUF_SIZE, TRACE_STAMP_RESOLUTION_MS, traceMsgList, TRC_COUNT);
  Cli.init(SERIAL_BAUD);

  Serial.println (F("\r\n+ + +  F R I D G E  C O N T R O L L E R  + + +\r\n"));
  Cli.xprintf    ("V %d.%d.%d\r\n\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Cli.newCmd     ("on",      "Turn on the compressor",             cmdOn);
  Cli.newCmd     ("off",     "Turn off the compressor",            cmdOff);
  Cli.newCmd     ("status",  "Show the system status",             cmdStatus);
  Cli.newCmd     (".",       "Show the system status",             cmdStatus);
  Cli.newCmd     ("config",  "Show the system configuration",      cmdConfig);
  Cli.newCmd     ("r",       "Show the system configuration",      cmdConfig);
  Cli.newCmd     ("trace",   "Print the trace log",                cmdTrace);
  Cli.newCmd     ("t",       "Print the trace log",                cmdTrace);
  Cli.newCmd     ("ond",     "Set min. on duration (arg: <0..10> seconds)",  cmdSetMinOnDuration);
  Cli.newCmd     ("offd",    "Set min. off duration (arg: <0..10> seconds)", cmdSetMinOffDuration);
  Cli.newCmd     ("pwml",    "Set the PWM duty for min. RPM (arg: <127..255>)", cmdSetMinDutyCycle);
  Cli.newCmd     ("pwmh",    "Set the PWM duty for max. RPM (arg: <0..254>)",   cmdSetMaxDutyCycle);
  Cli.showHelp();

  nvmRead();

  S.lastPwmDutyCycle = Nvm.minRpmDutyCycle;

  // Enable the watchdog
  wdt_enable (WDTO_1S);
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
  setOutputPin();
  ledManager();
  speedManager();

  // Main state machine
  switch (S.state) {

    // Compressor OFF state entry point
    case STATE_OFF_ENTRY:
      compressorOffTs = ts;
      S.pwmDutyCycle = 0;

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
        Trace.log(TRC_COMPRESSOR_ON);
      }
      break;

    // Compressor ON state entry point
    case STATE_ON_ENTRY:
      compressorOnTs = ts;
      S.pwmDutyCycle = S.lastPwmDutyCycle;
      S.state = STATE_ON_WAIT;
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
        Trace.log(TRC_COMPRESSOR_OFF);
      }
      break;

    default:
      break;
  }

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
 * Apply the PWM duty cycle to the output pin
 */
void setOutputPin (void)
{
  static uint8_t lastPwmDutyCycle = 0;

  if (S.pwmDutyCycle != lastPwmDutyCycle) {
    analogWrite(OUTPUT_PIN, S.pwmDutyCycle);
    lastPwmDutyCycle = S.pwmDutyCycle;
  }
}


/*
 * Manage the LED blinking state
 */
void ledManager (void)
{
  if (!S.inputEnabled && 0 == S.pwmDutyCycle) {
    Led.blink(-1, 100, 2900);
  }
  else if (S.inputEnabled && 0 == S.pwmDutyCycle) {
    Led.blink(-1, 100, 900);
  }
  else if (S.inputEnabled && S.pwmDutyCycle > 0) {
    Led.turnOn();
  }
  else if (!S.inputEnabled && S.pwmDutyCycle > 0) {
    Led.blink(-1, 900, 100);
  }
}


/*
 * Speed adjustment algorithm
 */
void speedManager (void)
{

}


/*
 * Validate EEPROM data
 */
void nvmValidate (void)
{
  Nvm_t NvmInit;
  bool result  = true;

  if (Nvm.magicWord != NVM_MAGIC_WORD) {
    Nvm = NvmInit;
    eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
    return;
  }

  result &= Nvm.minRpmDutyCycle >= 10;
  if (!result) {
    Nvm.minRpmDutyCycle = NvmInit.minRpmDutyCycle;
    result = true;
  }

  result &= Nvm.maxRpmDutyCycle <= Nvm.minRpmDutyCycle - 10;
  result &= Nvm.maxRpmDutyCycle > 0;
  if (!result) {
    if (NvmInit.maxRpmDutyCycle < Nvm.minRpmDutyCycle - 10) {
      Nvm.maxRpmDutyCycle = NvmInit.maxRpmDutyCycle;
    }
    else {
      Nvm.maxRpmDutyCycle = Nvm.minRpmDutyCycle - 10;
    }
    result = true;
  }

  result &= Nvm.minOnDurationS <= 600;
  if (!result) {
    Nvm.minOnDurationS = NvmInit.minOnDurationS;
    result = true;
  }

  result &= Nvm.minOffDurationS <= 600;
  if (!result) {
    Nvm.minOffDurationS = NvmInit.minOffDurationS;
    result = true;
  }
}


/*
 * Read EEPROM data
 */
void nvmRead (void)
{
  eepromRead (0x0, (uint8_t*)&Nvm,    sizeof (Nvm_t));
  nvmValidate ();
}


/*
 * Write EEPROM data
 */
void nvmWrite (void)
{
  nvmValidate ();
  eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
}


/*
 * Activate the output pin
 */
int cmdOn (int argc, char **argv)
{
  S.state = STATE_ON_ENTRY;
  Serial.println(F("Compressor on\r\n"));
  return 0;
}


/*
 * Deactivate the output pin
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
  uint32_t duration = atol(argv[1]);
  Nvm.minOnDurationS = duration;
  nvmWrite();
  Cli.xprintf("Min. on duration = %ld s\r\n\r\n", Nvm.minOnDurationS);
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
  Cli.xprintf("Min. off duration = %ld s\r\n\r\n", Nvm.minOffDurationS);
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
  uint8_t dutyCycle = atoi(argv[1]);
  Nvm.minRpmDutyCycle = dutyCycle;
  nvmWrite();
  S.lastPwmDutyCycle = Nvm.minRpmDutyCycle;
  S.pwmDutyCycle     = Nvm.minRpmDutyCycle;
  S.state            = STATE_ON_ENTRY;
  Cli.xprintf("PWM duty cycle at min. RPM= %d\r\n\r\n", Nvm.minRpmDutyCycle);
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
  uint8_t dutyCycle = atoi(argv[1]);
  Nvm.maxRpmDutyCycle = dutyCycle;
  nvmWrite();
  S.lastPwmDutyCycle = Nvm.maxRpmDutyCycle;
  S.pwmDutyCycle     = Nvm.maxRpmDutyCycle;
  S.state            = STATE_ON_ENTRY;
  Cli.xprintf("PWM duty cycle at max. RPM = %d\r\n\r\n", Nvm.maxRpmDutyCycle);
  return 0;
}


/*
 * Display the system configuration
 */
int cmdStatus (int argc, char **argv)
{
  Serial.println (F("System status:"));
  Cli.xprintf    (  "  State           = %d\r\n", S.state);
  Cli.xprintf    (  "  Input status    = %d\r\n", S.inputEnabled);
  Cli.xprintf    (  "  Last PWM duty   = %d\r\n", S.lastPwmDutyCycle);
  Cli.xprintf    (  "  Output PWM duty = %d\r\n", S.pwmDutyCycle);
  Serial.println (  "");
  return 0;
}

/*
 * Display the system configuration
 */
int cmdConfig (int argc, char **argv)
{
  Serial.println (F("System configuration:"));
  Cli.xprintf    (  "  Min. on duration    = %ld s\r\n", Nvm.minOnDurationS);
  Cli.xprintf    (  "  Min. off duration   = %ld s\r\n", Nvm.minOffDurationS);
  Cli.xprintf    (  "  Min. RPM duty cycle = %d\r\n"   , Nvm.minRpmDutyCycle);
  Cli.xprintf    (  "  Max. RPM duty cycle = %d\r\n"   , Nvm.maxRpmDutyCycle);
  Cli.xprintf    (  "\r\n  V %d.%d.%d\r\n\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  return 0;
}

/*
 * Dump the trace log
 */
int cmdTrace (int argc, char **argv)
{
  Serial.println (F("Trace messages:"));
  Trace.dump();
  return 0;
}