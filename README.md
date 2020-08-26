# Edge HDB Lift Monitoring Project

This project is to store the hardware and firmware data with regard to the HDB Lift Monitoring Project POC.

## Firmware Development
### Environment Setup
We need to download STM32 SDKs which provide all nessesary resoures (CMSIS, BSP, peripheral drivers, middlewares, example, documents, etc.) to develope firmware for STM32F4 MCU. For main MCU please download [STM32F4 SDK](https://www.st.com/content/st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-mcu-mpu-packages/stm32cubef4.html) and [STM32L0 SDK](https://www.st.com/content/st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-mcu-mpu-packages/stm32cubel0.html) for slave MCU to which hall effect sensor is attached.

**Update** : The MCU for proximity sensor is STM32L412 MCU, please download [STM32L4 SDK](https://www.st.com/content/st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-mcu-mpu-packages/stm32cubel4.html)

In order to have correct reference to SDsK, the SDKs must be extracted and put in following directory structure:

```
|-- your_working_folder
        |-- edge-hdb-lift-monitoring  -> git repo
            |-- ...
            |-- firmware
                |-- main_mcu  -> projects for main MCU (ie. bootloader, application)
                |-- slave_mcu  -> projects for slave MCU (ie. bootloader, application)
                |-- proximity_mcu   -> **project for proximity MCU**
                |-- pc_utilities -> utilities ie. sensor visualisation, smart device simulator to EnOS  
        |-- STM32_SDKs -> consists of firmware SDKs
            |-- STM32Cube_FW_F4_V1.25.0 -> SDK for main MCU
                |-- Drivers
                |-- ...
            |-- STM32Cube_FW_L0_V1.11.0 -> SDK for slave MCU
                |-- Drivers
                |-- ...
            |-- STM32Cube_FW_L4_V1.15.0 -> SDK for proximity MCU
                |-- Drivers
                |-- ...
```
Please **don't change** name of of SDK folder, currently the projects are relied on specific version of SDKs (STM32Cube_FW_F4_V1.25.0 and STM32Cube_FW_L0_V1.11.0).

We will use [Ac6 System Workbench for STM32](https://www.openstm32.org/Downloading%2Bthe%2BSystem%2BWorkbench%2Bfor%2BSTM32%2Binstaller) as IDE with free GNU GCC for Arm compiler, you will need to create a free account to able to download, please select correct installation package for coressponding OS.

After installation, can open and import project located in `repo_folder/firmware/main_mcu/application/SW4STM32, can refer to this [example](https://www.youtube.com/watch?v=oa95SuiNPcY) or this [tutorial](https://www.ecse.rpi.edu/courses/F18/ECSE-4790/Documents/STM32%20Workbench%20Install%20and%20Usage%20Guide.pdf).

## Firmware Design
We will use FreeRTOS for main MCU application while bare OS for slave MCU.
We may have different tasks to handle sensors acquisition, VCP/RS232/RS485/DI communications, ettc.

**Different configuration for main MCU**

Currently there are 2 configuration for main MCU, one is used with Dev Kits and a new one is used with Envision sensor PCB. To switch between configurations, on the Ac6 IDE, right click on project `edge_sensor_main_app` then select *Build Configurations -> Set Active -> Debug_HW1 (Envison PCB) or Debug (Dev Kits)*, then rebuild project

**COMMAND LINE INTERFACE and PRINTF**
Application uses a UART interface (USART 2 on Dev kit which is connected to STLINK I/F) for command line and printf function.
The application provide simple command line interface (CLI) so that it allows firmware modules to register command and action (pls refer to the implementation :)).  
Please use `PRINTF` instead of `printf` (case sensitive) to print message to CLI.
_(to be continue)_

## PC Application to display sensor data as graphys
Please install maven and java JDK to use the sofware.
On terminal go to directory `your_repo/firmware/pc_utilities/sensorvisualisation` and type command

`mvn compile exec:java -Dexec.mainClass="com.envisioniot.sensorvisualisation.SensorVisualisation" -Dexec.args="-comport COM4"`

Change COM4 to your serial port name mapped on your system

## PC Application to display 3D Orientation and instantaneous data

<img src="images/3d-visualiser-program.jpg" width="500">

1. Download and install [Processing](https://processing.org/download/)
2. Open the `firmware\pc_utilities\lift_box_3d_visualiser\lift_box_3d_visualiser.pde`
3. Modify the serial port "COMX" with the actual path to the serial device reading the board debug output.


