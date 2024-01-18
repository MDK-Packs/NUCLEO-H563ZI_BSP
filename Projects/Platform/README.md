Platform project
================

The **Platform** project configures the hardware of the evaluation board
and is a CMSIS-RTOS2 based software template that can be further expanded.

RTOS: Keil RTX5 Real-Time Operating System
------------------------------------------

The real-time operating system [Keil RTX5](https://arm-software.github.io/CMSIS-RTX/latest/index.html) implements the resource management. 

It is configured with the following settings:

- [Global Dynamic Memory size](https://arm-software.github.io/CMSIS-RTX/latest/config_rtx5.html#systemConfig): 24000 bytes
- [Default Thread Stack size](https://arm-software.github.io/CMSIS-RTX/latest/config_rtx5.html#threadConfig): 3072 bytes
- [Event Recorder Configuration](https://arm-software.github.io/CMSIS-RTX/latest/config_rtx5.html#evtrecConfig)
  - [Global Initialization](https://arm-software.github.io/CMSIS-RTX/latest/config_rtx5.html#evtrecConfigGlobIni): 1
    - Start Recording: 1

Refer to [Configure RTX v5](https://arm-software.github.io/CMSIS-RTX/latest/config_rtx5.html) for a detailed description of all configuration options.

Board: STMicroelectronics [NUCLEO-H563Z](https://www.st.com/en/evaluation-tools/nucleo-h563zi.html)
------------------------------------------

Device: **STM32H563ZITx**  
System Core Clock: **250 MHz**

This setup is configured using **STM32CubeMX**, an interactive tool provided by STMicroelectronics for device configuration.
Refer to ["Create Projects with STM32Cube HAL and STM32CubeMX"](https://www.keil.com/pack/doc/STM32Cube/html/index.html) for additional information.

For **STM32CubeMX** configuration settings please refer to [STM32CubeMX Configuration](Board/STM32CubeMX/STM32CubeMX.pdf).

The Heap/stack settings and CMSIS-Driver assignments are configured in the configuration files of respective software components.

The example project can be re-configured to work on custom hardware.
Refer to ["Migrate STM32 Based Example Projects to Custom Hardware"](https://github.com/MDK-Packs/Documentation/tree/master/Porting_to_Custom_Hardware) for additional information.

### System Configuration

| System resource         | Setting
|:------------------------|:----------------------------------------
| Heap                    | 64 kB (configured in the startup file)
| Stack (MSP)             | 1 kB (configured in the startup file)

### STDIO mapping

**STDIO** is routed to Virtual COM port on the ST-Link (using USART3 peripheral)

### CMSIS-Driver mapping

| CMSIS-Driver   | Peripheral  | Physical connection
|:---------------|:------------|:------------------------------------
| Driver_SPI1    | SPI1        | Arduino UNO R3 connector (CN7)
| Driver_USART13 | LPUART1     | Arduino UNO R3 connector (CN10)
| Driver_USBD0   | USB_FS      | USB Device on USB Type-C connector (CN13)

### CMSIS-Driver Virtual I/O mapping

| CMSIS-Driver VIO  | Physical resource
|:------------------|:-------------------------------
| vioBUTTON0        | Button USER (PC13)
| vioLED0           | LED RED (PG4)
| vioLED1           | LED GREEN (PB0)
| vioLED2           | LED YELLOW (PF4)
