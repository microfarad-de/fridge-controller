# Fridge Controller
Secop BD35F fridge compressor speed controller

compressor unit, I needed to implement a control unit to address the following issues:

* The compressor unit utilizes the RML10's built-in thermostat and is activated by setting the fridge to 12V mode. It operates using the output originally intended for the 12V heating unit it replaced. However, since the thermostat was designed for an absorption unit, it has a very narrow hysteresis, causing the compressor to cycle on and off too frequently.

* The BD35F compressor’s electronic control unit supports variable speed control, either by installing a coding resistor in series with the thermostat or by using a PWM signal to regulate the speed.

The controller in this project is positioned between the RML10 output, which controls the 12V heater relay, and the BD35F thermostat input. It minimizes frequent compressor cycling by enforcing configurable minimum on and off durations.

Moreover, the controller dynamically adjusts the PWM signal that regulates the compressor's RPM. It continuously monitors the compressor's on and off durations. If the on duration surpasses a specific threshold, the controller gradually increases the compressor speed. Conversely, if the off duration exceeds a certain threshold, the controller gradually reduces the compressor speed.


<p align="center">
<img src="https://raw.githubusercontent.com/microfarad-de/tcs-intercom/master/doc/tcs-intercom-schematic.png" alt="drawing"/>
</p>

[tcs-intercom-schematic.pdf](https://raw.githubusercontent.com/microfarad-de/tcs-intercom/master/doc/tcs-intercom-schematic.pdf)



## Notes

The code has been implemented and tested on an Arduino Pro Mini clone board based on the ATmega328P microcontroller with an 8 MHz crystal.

This project uses Git submodules. In order to get its full source code, please clone this Git repository to your local workspace, then execute the follwoing command from within the repository's root directory: `git submodule update --init`. Alternatively, you may simply download and unzip `fridge-controller-x.y.z-full.zip` from the latest release under https://github.com/microfarad-de/tcs-intercom/releases, then open `fridge-controller.ino` using the Arduino IDE.

The following dependencies should to be installed:
* `pip install pyserial compiledb`

This project can be compiled from command line using the via makefile using the following commands:

* Compile: `make`
* Remove compiled artefacts: `make clean`
* Upload to Arduino board: `make upload`
* To generate the release archive: `make release`

This project is configured for using the clangd VSCode plugin for code indexing. In order to use the clangd plugin, `compile_commands.json` file needs to be generated. This file is already part of the rpository, however it needs to be regenerated if new source files have been added to the project. The generation of `compile_commands.json` has been performed using the `compileddb` Python tool (`pip install compiledb`, https://pypi.org/project/compiledb/). The generation si performd using the following command: `compiledb make`.