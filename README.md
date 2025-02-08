# Fridge Controller
Secop BD35F fridege compressor speed controller


The code has been implemented and tested on an Arduino Pro Mini clone board based on the ATmega328P microcontroller.


This project uses Git submodules. In order to get its full source code, please clone this Git repository to your local workspace, then execute the follwoing command from within the repository's root directory: `git submodule update --init`. Alternatively, you may simply download and unzip `fridge-controller-x.y.z-full.zip` from the latest release under https://github.com/microfarad-de/tcs-intercom/releases, then open `fridge-controller.ino` using the Arduino IDE.



## Notes

The following dependencies should to be installed:
* `pip install pyserial compiledb`

This project can be compiled from command line using the via makefile using the following commands:

* Compile: `make`
* Remove compiled artefacts: `make clean`
* Upload to Arduino board: `make upload`
* To generate the release archive: `make release`

This project is configured for using the clangd VSCode plugin for code indexing. In order to use the clangd plugin, `compile_commands.json` file needs to be generated. This file is already part of the rpository, however it needs to be regenerated if new source files have been added to the project. The generation of `compile_commands.json` has been performed using the `compileddb` Python tool (`pip install compiledb`, https://pypi.org/project/compiledb/). The generation si performd using the following command: `compiledb make`.