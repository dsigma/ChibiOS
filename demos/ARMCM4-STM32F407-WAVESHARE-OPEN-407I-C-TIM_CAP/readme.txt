*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M4 STM32F407.                            **
*****************************************************************************

** TARGET **

The demo runs on an WaveShare STM32F4 Open 407I-C board.

** The Demo **

This demonstrates a custom TIMCAP driver which uses all 4 channels of the 
STM32 timers to generate interupt callbacks when the rising edge of an input
line goes high.

** Build Procedure **

The demo has been tested by using the free Codesourcery GCC-based toolchain. 
Just modify the TRGT line in the makefile in order to use different GCC 
toolchains.

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.st.com
