#Warning
* to use the debugger you need to un-solder the r21 resistor on the WIFI board 

#Configuration
I test a configuration including
* NUCLEO 446RE
* X-NUCLEO-IDW01M1 the WIFI serial board from ST

#Goal

This project is a mere port of ST code STM32CubeExpansion_WIFI1_V1.1.0 (the socketclient)

#Prerequisite

Download the XCUBE WIFI software from ST (commercial name: X-CUBE-WIFI1)
http://www.st.com/web/catalog/tools/FM147/CL1794/SC961/SS1743/LN1920/PF262298

#Context

The original software from ST doesn't target the 446RE, I had to change the "configuration" for this board.
Here are the basic steps
* Create a blank HAL project with SW4STM32 (AC6) Targeting the 446RE
* Copied the Middlewares directory from ST WIFI sofwtare
* added some include path to the eclipse project
* copied few more files from the Drivers directory (stm32_spwf_wifi.c and .h)
* changed the stm32f4xx_it.c with Projects\Multi\Applications\Client_Socket\Src stm32_xx_it.c content, same for headers
* added some symbols (check the list from the eclipse project file)
* did some othe tweaks...

#Usage

* Load the eclipse project (use http://www.openstm32.org/HomePage system workbench from AC6)
* Change ssid and secret key to your WIFI access point context
* Compile it
* Connect the boards and link it to PC with the USB (jumper set on both board see ST documentation)
* Download the program to the target (the r21 resistor must be unsoldered to have the debugger working)
* Use putty (115200 8bit, 1stop, no party, no xon-xoff) to connect with the target (see the com port with the device manager)
* use the SocketTest software (Server, port 32000) to open a server on your PC listening to port 32000 (in the st WIFI stack)
* reset the target, some guideline should show-up in putty ... follow them
* don't forget to use your access point association mode to have the target WIF accepted...

