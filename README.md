# EcoApi Bee hive monitoring

description

## Features

- External 1 wire sensor (ex: DS18B20)
- External i2c sensor (ex: as6200, si7021, shtxx, ...)
- Microphone analog with max9814
- Microphone i2s 
- Weight with load scale using hx711
- BlackPill uC board
- Motion detection using tilt vibration sensor
- Internal i2c sensor (ex: bmp180, bme180, bme280, ...)
- Lora using rfm95w 
- Powered by 18650 battery

## Hardware

### Characteristics

- Cpu
- Consumption
- Modules
- Backup capacitor explanation
- ....

### Pcb

Front
[![](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/EcoApi_pcb1.jpg?raw=true)](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/EcoApi_pcb1.jpg?raw=true)

[![](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/EcoApi_pcb2.jpg?raw=true)](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/EcoApi_pcb2.jpg?raw=true)

Back
[![](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/EcoApi_pcb3.jpg?raw=true)](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/EcoApi_pcb3.jpg?raw=true)

### other

Cpu board ([Board comparison](https://hackaday.com/2021/01/20/blue-pill-vs-black-pill-transitioning-from-stm32f103-to-stm32f411/ "Board comparison"))

[![](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/stm32_blue_black_pill_top.jpg?raw=true)](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/stm32_blue_black_pill_top.jpg?raw=true)


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


