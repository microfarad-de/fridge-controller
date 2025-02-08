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
#include <avr/wdt.h>

/*
 * Pin assignment
 */
#define INPUT_PIN   12
#define OUTPUT_PIN  6
#define LED_PIN     LED_BUILTIN  // 13


/*
 * Configuration parameters
 */
#define SERIAL_BAUD  57600   // Serial communication baud rate


// Configuration parameters stored in EEPROM (Nvm)
struct Nvm_t {

} Nvm;


// LED instance
LedClass Led;





/*
 * Function declarations
 */
int cmdOn  (int argc, char **argv);
int cmdOff (int argc, char **argv);




/*
 * Arduino initialization routine
 */
void setup(void)
{
  pinMode(INPUT_PIN, INPUT_PULLUP);
  pinMode(OUTPUT_PIN, OUTPUT);

  digitalWrite(OUTPUT_PIN, LOW);

  Led.initialize(LED_PIN);
  Cli.init(SERIAL_BAUD);

  Serial.println (F("\r\n+ + +  F R I D G E  C O N T R O L L E R  + + +\r\n"));
  Cli.xprintf    ("V %d.%d.%d\r\n\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Cli.newCmd     ("on",  "Turn on the compressor",  cmdOn);
  Cli.newCmd     ("off", "Turn off the compressor", cmdOff);
  Cli.showHelp();


  Led.blink(-1, 100, 900);

  // Enable the watchdog
  wdt_enable (WDTO_1S);
}



/*
 * Arduino main loop
 */
void loop(void)
{

  Cli.getCmd();

  Led.loopHandler();


  wdt_reset ();



}




/*
 * Activate the output pin
 */
int cmdOn (int argc, char **argv) {
  digitalWrite(OUTPUT_PIN, HIGH);
  Led.turnOn();
  Serial.println("Compressor on");
  return 0;
}


/*
 * Deactivate the output pin
 */
int cmdOff (int argc, char **argv) {
  digitalWrite(OUTPUT_PIN, LOW);
  Led.blink(-1, 100, 900);
  Serial.println("Compressor off");
  return 0;
}
