# Proximity Sensor

This project is to store the firmware data for the Proximity Sensor

## Files Description

-  main.h -> The main file containing all the code that is updated to the MCU

-  vl53l0x.h -> Codes for configuring and getting data from the VL53L0X sensor

-  uplinkcomms.h -> Codes for setting up UART communication via RS485 between the
   proximity sensor and the main sensor board

-  stm32l4xx_envision_proximity.h -> This file contains the initialisation for GPIO
   pins and I2C bus, as well as the Read and Write functions for I2C

-  stm32l4xx_hal.h -> This file contains all the function prototypes for the HAL
   module driver

## General Flow

After doing all the setup needed, the VL53L0X sensor will start getting the range reading on a continuous timed mode with a delay of 50 milliseconds between each measurement, and send the data to the MCU through the I2C1 Bus. The data received is then optimised using a Low-Pass filter so that we can get a more accurate and less-fluctuating reading, and assigned to a temporary variable. This process is always repeated as long as the MCU is powered up.

The proximity sensor is also connected to the main sensor box via RS485, and they have a Master-Slave relation. Everytime the proximity gets a "request" data from the main box, it will trigger an interrupt function and then it will send a "reply" data consisting the variable mentioned earlier that is holding the current distance data.
