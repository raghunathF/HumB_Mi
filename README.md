
HummingbirdBit default firmware

This package contains Hummingbird Bit default firmwre, used to communicate over bluetooth with Birdblox, Bluebird connector, Microblocks.The Hummingbird Bit default firmware uses NRF SDk 12_3 and S130 softdevice.

The Source files for application are present in Hummingbird_Bit_Microbit_Application\examples\ble_peripheral\ble_app_uart\pca10028\s130\arm5_no_packs

Software tools required:
	1.Keil uVision 5
  2.Seggermode hex file -- Microbit

Hardware:
  1.Micro:bit / Micro:bit & HummingbirdBit

Running the project on micro:bit 

1. Download KeiluVsion 5 free/paid and open the project using the project file in 
Hummingbird_Bit_Microbit_Application\examples\ble_peripheral\ble_app_uart\pca10028\s130\arm5_no_packs and appropriately download the dependencies.Make sure you can build without any errors.

2. Download the free version of segger hexfile  for micro:bit , put the micro:bit in maintanance mode and place the hex file into the microbit. After the micro:bit resets the microbit files will be Segger.
Hex File : https://www.segger.com/downloads/jlink#BBC_microbit

3. Connect the micro:bit to the your laptop/PC , first flash the softdevice and then flash the application to the micro:bit


License:

1.The github repo license is applied to our work  which is in Hummingbird_Bit_Microbit_Application\examples\ble_peripheral\ble_app_uart\pca10028\s130\arm5_no_packs

2.Other files are according to the license in Hummingbird_Bit_Application which is provided by Nordic Semiconductors.
