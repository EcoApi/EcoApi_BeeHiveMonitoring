# EcoApi Bee hive monitoring

description TBD

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
- Lora
- Consumption
- Fram
- EEprom
- Flash
- Rtc
- WDG
- HX711
- Protection battery
- Backup capacitor explanation
- Battery charger
- USB cdc
- Ldo
- Buck-boost
- Power Load switch
- Wakeup parts (button, wdg, motion, rtc)
- Dip switch
- Connector (load cell, external sensors, Battery, internal sensor, Debug & SWD)
- Push button
- Debug (Uart, USB CDC, SWD)
- ....

### Pcb

Front
[![](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/FrontV2.png?raw=true)](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/FrontV2.png?raw=true)

Back
[![](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/BackV2.png?raw=true)](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/BackV2.png?raw=true)

### Pinout

[![](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/STM32F411_pinout.png?raw=true)](https://github.com/EcoApi/EcoApi_BeeHiveMonitoring/blob/main/Hardware/images/STM32F411_pinout.png?raw=true)

array of pins description

### pcb

describ options with 0R resistors. etc ...

#### JlcPcb production

- Bom and pick&place : https://support.jlcpcb.com/article/153-how-to-generate-bom-and-centroid-files-from-kicad-in-linux
- JlcPcb gerber files : https://support.jlcpcb.com/article/149-how-to-generate-gerber-and-drill-files-in-kicad

## Software

To be define (flow chart) ...

### Description

- Lora : ttn, webhook for beep, js decoder, frame format
- sensor template
- Sensors auto-detection
- Outside informations (temperature, pressure, humidity, wind, ...)
- Inside informations (Sound, temperature, humidity, gas, bee counter, custom sensor ...)
- System information (RSSI, VBATT, 
- describe directive in platforio.ini
- FFT on analog mic or I2S mic (parameters, range etc ...)
- sleep mode

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

## how to

- Scale full or half
- Tare button
- Led information
- Motion tilt
- Debug
- usb charger leds
- Sleep on VBATT or 3.3V with slide switch
- Dip switch
- USB with android application (flutter.io or other)

## other 
[Editor.md](https://pandao.github.io/editor.md/en.html "Editor.md")



