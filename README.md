# EcoApi Bee hive monitoring

description

## Features

- External i2c sensor (ex: as6200, si7021, shtxx, ...)
- Microphone analog with max9814 (FFT with dma)
- Microphone i2s (FFT with dma)
- Weight with load scale using hx711 (opimized for 3.3V with noise filter), calibration by led information
- CPU STM32F411CEU6
- Motion detection using mercure tilt vibration sensor
- Internal i2c sensor (ex: bmp180, bme180, bme280, ...)
- Lora using rfm95w 
- Powered battery like : Li-ion, Li-po, 3xNimh, 4xNimh, LifePO4, Alkaline 3x1.5 / 2x1.5, Li-SOCL2, Li-Mn02, 18650
- Reverse battery protection
- USB charger
- Ultra low power in sleep mode (wakeup by external rtc) ~2µA
- Power in running mode TBD
- USB cdc from stm32
- Power switch for internal / external VCC (3.3V)
- Fram
- EEprom
- Flash SPI
- External nano watchdog
- Debug by led, Uart, 

## Hardware



### Characteristics

- Cpu
- Consumption
- Modules
- Backup capacitor explanation
- ....

### Pcb

Front
[![](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/FrontV2.png?raw=true)](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/FrontV2.png?raw=true)

Back
[![](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/BackV2.png?raw=true)](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/BackV2.png?raw=true)

### Pinout

[![](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/STM32F411_pinout.png?raw=true)](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/STM32F411_pinout.png?raw=true)

## Software

To be define

### Description

- fsdfsf
- fdsdfs
- fqfdss

### Ide & compilation

- pio
- plugins
- ....

### lmic debug ###

in platformio.ini

(1) uncomment :
	-D LMIC_DEBUG_LEVEL=2
	-D LMIC_ENABLE_event_logging=1
	-D LMIC_X_DEBUG_LEVEL=1

(2) replace in lmic/lmic.h :

#  ifdef LMIC_DEBUG_PRINTF_FN
#    define LMIC_X_DEBUG_PRINTF(f, ...) LMIC_DEBUG_PRINTF_FN(f, ## __VA_ARGS__)
#  else
#    error "LMIC_DEBUG_PRINTF_FN must be defined for LMIC_X_DEBUG_LEVEL > 0."
#  endif

by :

#define LMIC_X_DEBUG_PRINTF(f, ...) LMIC_DEBUG_PRINTF(f, ## __VA_ARGS__)

### other 
[Editor.md](https://pandao.github.io/editor.md/en.html "Editor.md")

### JlcPcb smt assembly ###
https://support.jlcpcb.com/article/153-how-to-generate-bom-and-centroid-files-from-kicad-in-linux

### JlcPcb gerber files ###
https://support.jlcpcb.com/article/149-how-to-generate-gerber-and-drill-files-in-kicad
