Target Compilation Test
======================================
This repo is a demonstration of how to compile a simple C file for multiple targets: Windows x86, x64, MacOS, Linux, and ARM.

Additionally, the serial_test.c is a demonstration of how to use the libserialport library for cross-platform serial communication.

To compile serial_test.c under Windows:
* Install [MSYS2](https://www.msys2.org/)
    * From the MSYS2 UCRT64 shell: `pacman -Syu`. If it asks to close the terminal to finish the update, click X or follow the prompt, then reopen "MSYS2 UCRT64" from your Start Menu.
    * Run `pacman -Su` once more to ensure everything is fully up to date.
    * Install the Toolchain (GCC, GDB, Make): `pacman -S mingw-w64-ucrt-x86_64-toolchain`
    * Install libserialport `pacman -S mingw-w64-ucrt-x86_64-libserialport`
* Compile
    * `cd /<local repo location>/Target_Test`
    * `gcc serial_test.c -o serial_test.exe -lserialport -lsetupapi -lcfgmgr32 -static`
    * Party!
