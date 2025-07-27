# Fridge Controller

Secop BD35F fridge compressor speed controller integration with Dometic RML10 series absorption refirgerator.

After retrofitting my Dometic RML10 absorption RV refrigerator with a Secop BD35F 12 V compressor unit, the need arose to implement a control unit to address the following issues:

* The compressor utilizes the RML10's built-in thermostat and is activated by setting the fridge to 12 V mode. It operates using the output originally intended for the 12 V absorber fridge heating unit it replaced. However, since the thermostat was designed for an absorption unit, it has a very narrow hysteresis, causing the compressor to cycle on and off too frequently.

* The BD35F compressor’s electronic control unit supports variable speed control, either by installing a coding resistor in series with the thermostat or by using a PWM signal to regulate the speed.

The controller in this project is positioned between the RML10 output that controls the 12 V heater relay and the BD35F thermostat input. It minimizes frequent compressor cycling by enforcing configurable minimum on and off durations.

Moreover, the controller dynamically adjusts the PWM signal that regulates the compressor's RPM. It continuously monitors the compressor's on and off durations. If the on duration surpasses a specific threshold, the controller gradually increases the compressor speed. Conversely, if the off duration exceeds a certain threshold, the controller gradually reduces the compressor speed.


## Circuit Diagram

<p align="center">
<img src="https://raw.githubusercontent.com/microfarad-de/fridge-controller/master/doc/fridge-controller-schematic.png" alt="Fridge Controller Schematic"/>
</p>

[📄 Download the full schematic (PDF)](https://raw.githubusercontent.com/microfarad-de/fridge-controller/master/doc/fridge-controller-schematic.pdf)

The schematic integrates several modules: the custom fridge controller, a Dometic RML10 built‑in fridge unit, a Secop BD35F 12 V compressor, a Raspberry Pi running Node‑RED, and an Arduino that receives remote commands via an HC‑05 Bluetooth UART module. An optocoupler allows the Pi to control the fridge’s D+ (ignition) signal, while diodes merge the Pi‑generated D+ signal with the signal from the towing vehicle.

The Dometic RML10 controller provides a relay output that controls the 12 V heating element used in absorber‑fridge operation. The relay coil is switched on the negative side via an open‑collector output. Between the relay’s positive terminal and ground sits an AMS1117 linear voltage regulator, protected by a 300 mA polyfuse and a diode for overcurrent and reverse‑polarity protection. Arduino digital pin 12 connects to the relay’s negative terminal through a protection diode; when the RML10 pulls this pin low, 12 V cooling is enabled. Otherwise, the pin floats and is pulled high internally.

Filtering capacitors C1 and C2 stabilize the AMS1117 regulator, and a bypass capacitor (C3) suppresses noise from the RML10 gas ignition system.

A TLP127 (LTV‑252T) optocoupler isolates the RML10 controller from the BD35F compressor. It is driven by a PWM signal generated on Arduino pin 11 via `analogWrite()`. The optocoupler emitter connects to the BD35F common (negative) terminal, while the collector connects to the temperature‑sensor (T) input.

Another TLP127 optocoupler isolates the Raspberry Pi GPIO from the 12 V supply, enabling the Pi to toggle the RML10’s D+ input. When D+ is enabled, the RML10 uses ignition‑controlled 12 V power for absorption cooling (disabling absorption cooling if the towing vehicle is disconnected). When D+ is disabled, the fridge falls back to propane powered absorbtion cooling.

Diodes D1 and D2 implement a simple OR circuit that merges the Pi‑generated D+ signal with the towing vehicle’s D+ signal so that the fridge can operate with either source present.

Finally, a voltage divider (R4/R3) steps down the Arduino’s 5 V logic to the 3.3 V level required by the HC‑05 Bluetooth module.


## Secop BD35F Speed Control

The BD35F speed control protocol is described in the Secop BD controllers manual ([Secop_BD_Controllers_Operating_Instructions.pdf](https://raw.githubusercontent.com/microfarad-de/fridge-controller/master/doc/Secop_BD_Controllers_Operating_Instructions.pdf)).

The compressor speed ranges between 2000 and 3500 RPM and can be either controlled by connecting a coding resistor between the compressor controller's C and T terminals, or by generating a PWM signal between these two pins. Regarding the PWM signal charateristics, the manual only mentions "open collerctor, 5kHz +-5%".

As shown in the circuit diagram, the open collerctor design has been implemented using a TLP127 (LTV-252T) optocoupler connected between the common (C) and temperature sensor (T) compressor controller terminals.

Since the exact 5 kHz frequency is not possible to configure out of the box, the PWM frequency of the analog output pin 11 has been configured to 1960.53 Hz. This has been achieved by configuring the clock select bits of Timer/Counter Control Register B (TCCR2B) to clkI/O / 8. Whereas this frequency is derived for an 8 MHz Arduino using the following formulas:

`clk_io = 8000000 / 255 = 31372.55 Hz`

`pwm_frequency = clk_io / 8 = 1960.53 Hz`

The TCCR2B clock select bits have been documented in the ATmega328P datasheet Section 22.11.2, Table 22-10 ([ATmega328P_Datasheet_201611.pdf](https://raw.githubusercontent.com/microfarad-de/fridge-controller/master/doc/ATmega328P_Datasheet_201611.pdf)):

| CS22 | CS21 | CS20 | Description |
|:----:|:----:|:----:|:------------|
| 0    | 0    | 0    | No clock source (Timer/Counter stopped) |
| 0    | 0    | 1    | clkI/O / 1 (No prescaling)              |
| 0    | 1    | 0    | clkI/O / 8                              |
| 0    | 1    | 1    | clkI/O / 32                             |
| 1    | 0    | 0    | clkI/O / 64                             |
| 1    | 0    | 1    | clkI/O / 128                            |
| 1    | 1    | 0    | clkI/O / 256                            |
| 1    | 1    | 1    | clkI/O / 1024                           |

The chosen PWM frequency has proven to work well with the Secop controller model 101N0212 in use. Whereas the following PWM duty cycle values have been derived via trial and error:

| RPM    | Duty Cycle | AnalogWrite (0..255) |
|-------:|-----------:|---------------------:|
| 2000   | 75 %       | 190                  |
| 3500   | 16 %       | 40                   |
| 0      | 0 %        | 0                    |

## Speed Control Algorithm

The compressor speed is continuously adjusted to maintain a configurable **target duty cycle**. This is achieved through a **closed-loop control algorithm**, which operates as follows:

- If the **measured average compressor duty cycle** exceeds the target, the speed is gradually **reduced** by a specified number of `analogWrite` steps per minute.
- If the **measured duty cycle** falls **below the target minus a defined hysteresis**, the compressor speed is gradually **increased** by the same step rate.

Speed adjustments are suspended if:
- There are **insufficient duty cycle samples** accumulated for reliable measurement.
- The feature has been **manually disabled** by the user.


## Evaporator Defrost Routine

The speed controller can be configured to periodically pause the cooling cycle for a predefined amount of time in order to allow for the evaporator ice buildup to melt. The defrost routine is initiated when the following condition are met:
* The compressor runtime has reached a preset number of hours.
* The compressor duty cycle does not exceed a preset percentage value.

Once the above conditions have been met, the compressor is turned of for a preset number of minutes in order to allow for any ice buildup to melt.

## Configuration

The the speed controller is configurable over the Arduino's serial interface (9600 Baud). It provides a command line interface and a self explanatory help screen. All configuration parameter's are stored within Arduino's EEPROM.

Following are some of the available commands:

* `h`: Show the help screen
* `on`: Turn on the compressor
* `off`: Turn off the compressor
* `c <command>`: Remote control command
* `s`: Show the system status
* `r`: Show the system configuration stored in EEPROM
* `t [0..2]`: If argument is set, configure the tracing level. Otherwise print the trace.
* `ond <0..60>`: Set the minimum compressor on duration in minutes
* `offd <0..60>`: Set the minimum compressor off duration in minutes
* `pwml <1..255>`: Set the AnalogWrite() input value for minimum allowed RPM
* `pwmh <1..255>`: Set the AnalogWrite() input value for maximum allowed RPM
* `spdc <41..99>`: Set the speed adjustment target duty cycle in percent
* `spdh <1..40>`: Set the speed adjustment hysteresis value in percent
* `spdr <1..255>`: Set the speed adjustment rate in AnalogWrite() steps per minute (0 = disabled)
* `defr <0..24>`: Set the defrost the minimum compressor runtime hours for starting a defrost cycle
* `defc <0..100>`: Set the maximum allowed compressor duty cycle in percent for starting a defrost cycle
* `defd <0..60>`: Set the defrost cycle duration in minutes (0 = disabled)


## Remote Operation

The RV uses a Raspberry Pi (RPi) running **Victron Venus OS** to collect data from several onboard devices, including the battery shunt (for charge and current), the solar charge controller, the Truma Combi heater (for indoor temperature), and BLE temperature sensors in the fridge and outside. Using this information, the RPi runs an optimized fridge control algorithm to improve energy efficiency.

A **Node-RED flow** on the RPi processes the sensor data and sends commands to an Arduino that controls the fridge compressor. Communication between the RPi and Arduino is handled over UART, either wirelessly via an HC‑05 Bluetooth module or through a wired connection using an FTDI-compatible USB-to-serial converter.

Remote control of the fridge is achieved through a set of commands:

| Command       | Description                                  |
|---------------|----------------------------------------------|
| `c 0`         | Turn **off** the compressor                  |
| `c 1`..`c 10` | Set compressor speed (`1 = min`, `10 = max`) |
| `c 11`        | Turn **on** the compressor                   |
| `c 12`        | Disable remote operation                     |
| `c 20`        | Stop the defrost cycle                       |
| `c 21`        | Start a new defrost cycle                    |
| `c 22`        | Disable periodic defrost (stored in EEPROM)  |
| `c 23`        | Enable periodic defrost (stored in EEPROM)   |
| `c 30`        | Request system status information            |

Sending any command between `c 0` and `c 11` automatically enables remote operation. While in this mode, the fridge controller ignores the RML10 relay signal and relies solely on remote commands to operate the compressor. If no command in this range is received within a predetermined timeout, remote operation is disabled and the controller reverts to local operation based on the RML10 relay signal.

The Node-RED flow provides several key functions, including:

- Controlling the compressor according to fridge temperature reported by BLE sensors
- Detecting sudden temperature increases (e.g., when the door is open) and temporarily setting the compressor to maximum speed
- Disabling the defrost cycle in high ambient temperatures
- Switching to propane refrigeration when the battery charge is low
- Activating propane refrigeration to assist cooling during high ambient temperatures


The Node-RED flow that implements this control algorithm is available on GitHub:
🔗 [microfarad-de/inetbox2mqtt](https://github.com/microfarad-de/inetbox2mqtt)

## Gallery

 <p align="center">
 <img src="https://raw.githubusercontent.com/microfarad-de/fridge-controller/master/doc/layout.png" alt="drawing" width="600"/>
 </p>

 <p align="center">
 <img src="https://raw.githubusercontent.com/microfarad-de/fridge-controller/master/doc/perspective-1.png" alt="drawing" width="600"/>
 </p>

 <p align="center">
 <img src="https://raw.githubusercontent.com/microfarad-de/fridge-controller/master/doc/perspective-2.png" alt="drawing" width="600"/>
 </p>

 <p align="center">
 <img src="https://raw.githubusercontent.com/microfarad-de/fridge-controller/master/doc/perspective-3.png" alt="drawing" width="600"/>
 </p>

## Notes

The code has been implemented and tested on an Arduino Pro Mini clone board based on the ATmega328P microcontroller.

This project uses Git submodules. In order to get its full source code, please clone this Git repository to your local workspace, then execute the follwoing command from within the repository's root directory: `git submodule update --init`.

The following dependencies should to be installed:

* `pip install pyserial`

This project can be compiled from command line using a makefile. Following are the available make commands:

* Compile the project: `make`
* Remove compiled artefacts: `make clean`
* Upload to Arduino board: `make upload`
* Connect to the serial console: `make serial`
* To generate the release archive: `make release`

Prior to uploading the firmware, please ensure that the following parameaters are configured correctly in `Makefile`:

* `BOARD_TAG`: Device type as listed in `boards.txt` or `make show_boards` (e.g.: pro, uno).
* `BOARD_SUB`: Submenu as listed in `boards.txt` or `make show_submenu` (e.g.: 16MHzatmega328, atmega168)
* `MONITOR_BAUDRATE`: Serial port Baud rate (possible values: 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200)
* `MONITOR_PORT`: Serial port device (e.g.: /dev/tty.usbserial-00000000)

For more information, please refer to [the Arduino-Makefile documentation](https://github.com/sudar/Arduino-Makefile).

