# Proximity Sensor

This project is to store the firmware data for the Proximity Sensor

### Environment Setup

We need to download STM32 SDKs which provide all nessesary resoures (CMSIS, BSP, peripheral drivers, middlewares, example, documents, etc.) to develope firmware for STM32L4 MCU. Please download [STM32L4 SDK](https://www.st.com/content/st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-mcu-mpu-packages/stm32cubel4.html)

In order to have correct reference to SDsK, the SDKs must be extracted and put in following directory structure:

```
|-- your_working_folder
        |-- proximity-sensor-stm32  -> git repo
            |-- firmware
                |-- proximity_mcu   -> **project for proximity MCU**
        |-- STM32_SDKs -> consists of firmware SDKs
            |-- STM32Cube_FW_L4_V1.15.0 -> SDK for proximity MCU
                |-- Drivers
                |-- ...
```

Please **don't change** name of of SDK folder, currently the projects are relied on specific version of SDKs.

We will use [Ac6 System Workbench for STM32](https://www.openstm32.org/Downloading%2Bthe%2BSystem%2BWorkbench%2Bfor%2BSTM32%2Binstaller) as IDE with free GNU GCC for Arm compiler, you will need to create a free account to able to download, please select correct installation package for coressponding OS.

After installation, can open and import project, can refer to this [example](https://www.youtube.com/watch?v=oa95SuiNPcY) or this [tutorial](https://www.ecse.rpi.edu/courses/F18/ECSE-4790/Documents/STM32%20Workbench%20Install%20and%20Usage%20Guide.pdf).

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
