EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 2600 1350 2    50   Input ~ 0
LOADCELL_E+
Text GLabel 2600 1550 2    50   Input ~ 0
LOADCELL_A-
Text GLabel 2600 1650 2    50   Input ~ 0
LOADCELL_A+
Text GLabel 2600 3150 2    50   Input ~ 0
I2C1_SDA
Text GLabel 2600 3250 2    50   Input ~ 0
I2C1_SCL
$Comp
L Connector_Generic:Conn_01x06 J?
U 1 1 61BB1CF1
P 1450 4650
AR Path="/61BB1CF1" Ref="J?"  Part="1" 
AR Path="/62D38F6A/61BB1CF1" Ref="J?"  Part="1" 
AR Path="/62EA3721/61BB1CF1" Ref="J?"  Part="1" 
AR Path="/61B961C0/61BB1CF1" Ref="J7"  Part="1" 
F 0 "J7" H 1368 4125 50  0000 C CNN
F 1 "I2C INTERNAL" H 1368 4216 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical" H 1450 4650 50  0001 C CNN
F 3 "~" H 1450 4650 50  0001 C CNN
	1    1450 4650
	-1   0    0    1   
$EndComp
Text GLabel 1650 4550 2    50   Input ~ 0
I2C1_SCL
Text GLabel 1650 4650 2    50   Input ~ 0
I2C1_SDA
$Comp
L power:GND #PWR?
U 1 1 61BB1CFF
P 2200 4450
AR Path="/61BB1CFF" Ref="#PWR?"  Part="1" 
AR Path="/62D38F6A/61BB1CFF" Ref="#PWR?"  Part="1" 
AR Path="/62EA3721/61BB1CFF" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/61BB1CFF" Ref="#PWR0154"  Part="1" 
F 0 "#PWR0154" H 2200 4200 50  0001 C CNN
F 1 "GND" H 2205 4277 50  0000 C CNN
F 2 "" H 2200 4450 50  0001 C CNN
F 3 "" H 2200 4450 50  0001 C CNN
	1    2200 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 4350 1650 4350
Wire Wire Line
	1650 4450 2200 4450
NoConn ~ 1650 4750
NoConn ~ 1650 4850
Text Notes 1100 1750 0    50   ~ 0
E+ Red\nE- Black\nA- White\nA+ Green / Blue\nSh Yellow 
$Comp
L Device:R R?
U 1 1 61C32D6F
P 6550 1150
AR Path="/61C32D6F" Ref="R?"  Part="1" 
AR Path="/62EA3721/61C32D6F" Ref="R?"  Part="1" 
AR Path="/61B961C0/61C32D6F" Ref="R63"  Part="1" 
F 0 "R63" H 6620 1196 50  0000 L CNN
F 1 "0R" H 6620 1105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6480 1150 50  0001 C CNN
F 3 "~" H 6550 1150 50  0001 C CNN
	1    6550 1150
	1    0    0    -1  
$EndComp
$Comp
L EcoApi_pcb-rescue:+3V3-BluePillF4 #SUPPLY?
U 1 1 61C32D75
P 6550 850
AR Path="/61C32D75" Ref="#SUPPLY?"  Part="1" 
AR Path="/62EA3721/61C32D75" Ref="#SUPPLY?"  Part="1" 
AR Path="/61B961C0/61C32D75" Ref="#SUPPLY0120"  Part="1" 
F 0 "#SUPPLY0120" H 6550 850 45  0001 L BNN
F 1 "+3V3" H 6550 1021 45  0000 C CNN
F 2 "" H 6550 850 60  0001 C CNN
F 3 "" H 6550 850 60  0001 C CNN
	1    6550 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 2000 6550 2000
Wire Wire Line
	6550 1400 6550 1300
$Comp
L EcoApi_pcb:SW_PUSH S?
U 1 1 61C32D87
P 6550 1700
AR Path="/61C32D87" Ref="S?"  Part="1" 
AR Path="/62EA3721/61C32D87" Ref="S?"  Part="1" 
AR Path="/61B961C0/61C32D87" Ref="S1"  Part="1" 
F 0 "S1" V 6504 1812 50  0000 L CNN
F 1 "TARE" V 6595 1812 50  0000 L CNN
F 2 "EcoApi_pcb:SW_PUSH" H 6550 1700 50  0001 L BNN
F 3 "" H 6550 1700 50  0001 L BNN
F 4 "Compliant" H 6550 1700 50  0001 L BNN "EU_RoHS_Compliance"
F 5 "1571527-1" H 6550 1700 50  0001 L BNN "Comment"
	1    6550 1700
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61C32D9B
P 9800 2250
AR Path="/61C32D9B" Ref="#PWR?"  Part="1" 
AR Path="/62EA3721/61C32D9B" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/61C32D9B" Ref="#PWR0157"  Part="1" 
F 0 "#PWR0157" H 9800 2000 50  0001 C CNN
F 1 "GND" H 9805 2077 50  0000 C CNN
F 2 "" H 9800 2250 50  0001 C CNN
F 3 "" H 9800 2250 50  0001 C CNN
	1    9800 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 2250 9800 2100
Text GLabel 9350 1500 0    50   Output ~ 0
NRST
$Comp
L EcoApi_pcb:SW_PUSH S?
U 1 1 61C32DA5
P 9800 1800
AR Path="/61C32DA5" Ref="S?"  Part="1" 
AR Path="/62EA3721/61C32DA5" Ref="S?"  Part="1" 
AR Path="/61B961C0/61C32DA5" Ref="S3"  Part="1" 
F 0 "S3" V 9754 1912 50  0000 L CNN
F 1 "RESET" V 9845 1912 50  0000 L CNN
F 2 "EcoApi_pcb:SW_PUSH" H 9800 1800 50  0001 L BNN
F 3 "" H 9800 1800 50  0001 L BNN
F 4 "Compliant" H 9800 1800 50  0001 L BNN "EU_RoHS_Compliance"
F 5 "1571527-1" H 9800 1800 50  0001 L BNN "Comment"
	1    9800 1800
	0    1    1    0   
$EndComp
Text GLabel 9400 5500 2    50   Output ~ 0
SCALE_TYPE
$Comp
L Device:R R?
U 1 1 61C32DDA
P 9300 4450
AR Path="/61C32DDA" Ref="R?"  Part="1" 
AR Path="/62EA3721/61C32DDA" Ref="R?"  Part="1" 
AR Path="/61B961C0/61C32DDA" Ref="R68"  Part="1" 
F 0 "R68" H 9370 4496 50  0000 L CNN
F 1 "10k" H 9370 4405 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9230 4450 50  0001 C CNN
F 3 "~" H 9300 4450 50  0001 C CNN
	1    9300 4450
	1    0    0    -1  
$EndComp
Text GLabel 9400 4700 2    50   Output ~ 0
TRACE_EN
Wire Wire Line
	9400 4700 9300 4700
Wire Wire Line
	9300 4600 9300 4700
Connection ~ 9300 4700
Wire Wire Line
	10650 1500 10650 1400
$Comp
L power:GND #PWR?
U 1 1 61E97F07
P 10650 2250
AR Path="/61E97F07" Ref="#PWR?"  Part="1" 
AR Path="/62EA3721/61E97F07" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/61E97F07" Ref="#PWR0158"  Part="1" 
AR Path="/62CC297B/61E97F07" Ref="#PWR?"  Part="1" 
F 0 "#PWR0158" H 10650 2000 50  0001 C CNN
F 1 "GND" H 10655 2077 50  0000 C CNN
F 2 "" H 10650 2250 50  0001 C CNN
F 3 "" H 10650 2250 50  0001 C CNN
	1    10650 2250
	1    0    0    -1  
$EndComp
Text GLabel 10550 1500 0    50   Output ~ 0
BTN_PWR
Wire Wire Line
	10550 1500 10650 1500
Text GLabel 10550 1000 0    50   Input ~ 0
VBAT
$Comp
L Device:R R?
U 1 1 61E97F1A
P 10650 1250
AR Path="/61E97F1A" Ref="R?"  Part="1" 
AR Path="/62EA3721/61E97F1A" Ref="R?"  Part="1" 
AR Path="/61B961C0/61E97F1A" Ref="R73"  Part="1" 
AR Path="/62CC297B/61E97F1A" Ref="R?"  Part="1" 
F 0 "R73" H 10720 1296 50  0000 L CNN
F 1 "10k" H 10720 1205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10580 1250 50  0001 C CNN
F 3 "~" H 10650 1250 50  0001 C CNN
	1    10650 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10550 1000 10650 1000
Wire Wire Line
	10650 1000 10650 1100
$Comp
L power:GND #PWR?
U 1 1 61E97F22
P 7900 2500
AR Path="/61E97F22" Ref="#PWR?"  Part="1" 
AR Path="/62EA3721/61E97F22" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/61E97F22" Ref="#PWR0159"  Part="1" 
AR Path="/62CC297B/61E97F22" Ref="#PWR?"  Part="1" 
F 0 "#PWR0159" H 7900 2250 50  0001 C CNN
F 1 "GND" H 7905 2327 50  0000 C CNN
F 2 "" H 7900 2500 50  0001 C CNN
F 3 "" H 7900 2500 50  0001 C CNN
	1    7900 2500
	1    0    0    -1  
$EndComp
Text GLabel 7900 950  1    50   Input ~ 0
VBAT
$Comp
L EcoApi_pcb-rescue:+3V3-BluePillF4 #SUPPLY?
U 1 1 61E97F29
P 8150 850
AR Path="/61E97F29" Ref="#SUPPLY?"  Part="1" 
AR Path="/62CC297B/61E97F29" Ref="#SUPPLY?"  Part="1" 
AR Path="/61B961C0/61E97F29" Ref="#SUPPLY0123"  Part="1" 
F 0 "#SUPPLY0123" H 8150 850 45  0001 L BNN
F 1 "+3V3" H 8150 1020 45  0000 C CNN
F 2 "" H 8150 850 60  0001 C CNN
F 3 "" H 8150 850 60  0001 C CNN
	1    8150 850 
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 61E97F2F
P 7900 1150
AR Path="/61E97F2F" Ref="R?"  Part="1" 
AR Path="/62CC297B/61E97F2F" Ref="R?"  Part="1" 
AR Path="/61B961C0/61E97F2F" Ref="R64"  Part="1" 
F 0 "R64" H 7970 1196 50  0000 L CNN
F 1 "0R" H 7970 1105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7830 1150 50  0001 C CNN
F 3 "~" H 7900 1150 50  0001 C CNN
	1    7900 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 61E97F35
P 8150 1150
AR Path="/61E97F35" Ref="R?"  Part="1" 
AR Path="/62CC297B/61E97F35" Ref="R?"  Part="1" 
AR Path="/61B961C0/61E97F35" Ref="R66"  Part="1" 
F 0 "R66" H 8220 1196 50  0000 L CNN
F 1 "0R" H 8220 1105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8080 1150 50  0001 C CNN
F 3 "~" H 8150 1150 50  0001 C CNN
	1    8150 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 950  7900 1000
Wire Wire Line
	8150 1300 8150 1450
Wire Wire Line
	8150 850  8150 1000
$Comp
L Device:C C?
U 1 1 61E97F4A
P 7900 2250
AR Path="/61E97F4A" Ref="C?"  Part="1" 
AR Path="/62CC297B/61E97F4A" Ref="C?"  Part="1" 
AR Path="/61B961C0/61E97F4A" Ref="C30"  Part="1" 
F 0 "C30" V 8050 2050 50  0000 L CNN
F 1 "100nF" V 7950 2050 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7938 2100 50  0001 C CNN
F 3 "~" H 7900 2250 50  0001 C CNN
	1    7900 2250
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 61E97F50
P 7900 1750
AR Path="/61E97F50" Ref="R?"  Part="1" 
AR Path="/62CC297B/61E97F50" Ref="R?"  Part="1" 
AR Path="/61B961C0/61E97F50" Ref="R65"  Part="1" 
F 0 "R65" H 7970 1796 50  0000 L CNN
F 1 "1M" H 7970 1705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7830 1750 50  0001 C CNN
F 3 "~" H 7900 1750 50  0001 C CNN
	1    7900 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 1300 7900 1450
Wire Wire Line
	7900 1450 8150 1450
Text GLabel 7750 2000 0    50   Output ~ 0
MOTION_ALARM
Wire Wire Line
	7900 1450 7900 1600
Connection ~ 7900 1450
Wire Wire Line
	7900 1900 7900 2000
Wire Wire Line
	7900 2400 7900 2500
$Comp
L power:GND #PWR?
U 1 1 61E97F5D
P 8400 2500
AR Path="/61E97F5D" Ref="#PWR?"  Part="1" 
AR Path="/62EA3721/61E97F5D" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/61E97F5D" Ref="#PWR0160"  Part="1" 
AR Path="/62CC297B/61E97F5D" Ref="#PWR?"  Part="1" 
F 0 "#PWR0160" H 8400 2250 50  0001 C CNN
F 1 "GND" H 8405 2327 50  0000 C CNN
F 2 "" H 8400 2500 50  0001 C CNN
F 3 "" H 8400 2500 50  0001 C CNN
	1    8400 2500
	1    0    0    -1  
$EndComp
Connection ~ 7900 2000
Wire Wire Line
	7900 2000 7900 2100
Wire Wire Line
	7900 2000 7750 2000
Wire Wire Line
	8400 2000 7900 2000
Text GLabel 1350 5700 2    50   Output ~ 0
SW_CLK
Text GLabel 1350 5500 2    50   BiDi ~ 0
SW_DIO
Text GLabel 1350 5400 2    50   Output ~ 0
NRST
$Comp
L power:+3.3V #PWR?
U 1 1 61EC6DFB
P 1700 5800
AR Path="/61EC6DFB" Ref="#PWR?"  Part="1" 
AR Path="/62B8145A/61EC6DFB" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/61EC6DFB" Ref="#PWR0161"  Part="1" 
F 0 "#PWR0161" H 1700 5650 50  0001 C CNN
F 1 "+3.3V" H 1715 5973 50  0000 C CNN
F 2 "" H 1700 5800 50  0001 C CNN
F 3 "" H 1700 5800 50  0001 C CNN
	1    1700 5800
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61EC6E02
P 1700 5600
AR Path="/61EC6E02" Ref="#PWR?"  Part="1" 
AR Path="/62B8145A/61EC6E02" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/61EC6E02" Ref="#PWR0162"  Part="1" 
F 0 "#PWR0162" H 1700 5350 50  0001 C CNN
F 1 "GND" H 1705 5427 50  0000 C CNN
F 2 "" H 1700 5600 50  0001 C CNN
F 3 "" H 1700 5600 50  0001 C CNN
	1    1700 5600
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 61ED30F2
P 9550 3950
AR Path="/61ED30F2" Ref="R?"  Part="1" 
AR Path="/62CC297B/61ED30F2" Ref="R?"  Part="1" 
AR Path="/61B961C0/61ED30F2" Ref="R72"  Part="1" 
F 0 "R72" H 9620 3996 50  0000 L CNN
F 1 "1.5K" H 9620 3905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9480 3950 50  0001 C CNN
F 3 "~" H 9550 3950 50  0001 C CNN
	1    9550 3950
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_ALT D?
U 1 1 61ED3104
P 9150 3950
AR Path="/62CC297B/61ED3104" Ref="D?"  Part="1" 
AR Path="/61B961C0/61ED3104" Ref="D7"  Part="1" 
F 0 "D7" H 9150 4100 50  0000 C CNN
F 1 "LED_0603_BLUE" H 9150 4200 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 9150 3950 50  0001 C CNN
F 3 "~" H 9150 3950 50  0001 C CNN
	1    9150 3950
	1    0    0    -1  
$EndComp
$Comp
L EcoApi_pcb-rescue:+3V3-BluePillF4 #SUPPLY?
U 1 1 61C32DD4
P 9300 4250
AR Path="/61C32DD4" Ref="#SUPPLY?"  Part="1" 
AR Path="/62EA3721/61C32DD4" Ref="#SUPPLY?"  Part="1" 
AR Path="/61B961C0/61C32DD4" Ref="#SUPPLY0124"  Part="1" 
F 0 "#SUPPLY0124" H 9300 4250 45  0001 L BNN
F 1 "+3V3" H 9300 4421 45  0000 C CNN
F 2 "" H 9300 4250 60  0001 C CNN
F 3 "" H 9300 4250 60  0001 C CNN
	1    9300 4250
	1    0    0    -1  
$EndComp
Text GLabel 9900 3950 2    50   Input ~ 0
LED_INFO
Wire Wire Line
	9400 3950 9300 3950
$Comp
L EcoApi_pcb:+3V3-Ext #SUPPLY0126
U 1 1 61FA5C05
P 3250 2650
F 0 "#SUPPLY0126" H 3250 2650 45  0001 L BNN
F 1 "+3V3-Ext" H 3250 2821 45  0000 C CNN
F 2 "" H 3250 2650 60  0001 C CNN
F 3 "" H 3250 2650 60  0001 C CNN
	1    3250 2650
	0    1    1    0   
$EndComp
$Comp
L EcoApi_pcb:+3V3-Int #SUPPLY0129
U 1 1 6228415A
P 2200 4350
F 0 "#SUPPLY0129" H 2200 4350 45  0001 L BNN
F 1 "+3V3-Int" H 2200 4521 45  0000 C CNN
F 2 "" H 2200 4350 60  0001 C CNN
F 3 "" H 2200 4350 60  0001 C CNN
	1    2200 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 4700 8650 4700
Wire Wire Line
	8650 4600 8900 4600
Wire Wire Line
	8900 4600 8900 3950
Wire Wire Line
	8900 3950 9000 3950
Wire Wire Line
	8650 4800 8900 4800
Wire Wire Line
	8900 4800 8900 5500
Wire Wire Line
	8650 3800 8650 3450
Connection ~ 8150 3800
Wire Wire Line
	8150 3800 7950 3800
Wire Wire Line
	8150 4150 8150 3800
Wire Wire Line
	8250 4150 8150 4150
Wire Wire Line
	8650 3800 8550 3800
Connection ~ 8650 3800
Wire Wire Line
	8650 4150 8550 4150
Wire Wire Line
	8650 3800 8650 4150
$Comp
L Device:C C?
U 1 1 621F0FF4
P 8400 4150
AR Path="/621F0FF4" Ref="C?"  Part="1" 
AR Path="/62CC297B/621F0FF4" Ref="C?"  Part="1" 
AR Path="/61B961C0/621F0FF4" Ref="C31"  Part="1" 
F 0 "C31" V 8550 3950 50  0000 L CNN
F 1 "100nF" V 8450 3950 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8438 4000 50  0001 C CNN
F 3 "~" H 8400 4150 50  0001 C CNN
	1    8400 4150
	0    1    -1   0   
$EndComp
Wire Wire Line
	8150 3800 8250 3800
Text GLabel 8650 3450 1    50   Output ~ 0
BOOT0
$Comp
L power:GND #PWR?
U 1 1 621DB7BF
P 7950 3800
AR Path="/621DB7BF" Ref="#PWR?"  Part="1" 
AR Path="/62B8145A/621DB7BF" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/621DB7BF" Ref="#PWR0164"  Part="1" 
F 0 "#PWR0164" H 7950 3550 50  0001 C CNN
F 1 "GND" H 7955 3627 50  0000 C CNN
F 2 "" H 7950 3800 50  0001 C CNN
F 3 "" H 7950 3800 50  0001 C CNN
	1    7950 3800
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 621DB7B2
P 8400 3800
AR Path="/621DB7B2" Ref="R?"  Part="1" 
AR Path="/62B8145A/621DB7B2" Ref="R?"  Part="1" 
AR Path="/61B961C0/621DB7B2" Ref="R67"  Part="1" 
F 0 "R67" H 8470 3846 50  0000 L CNN
F 1 "10k" H 8470 3755 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8330 3800 50  0001 C CNN
F 3 "~" H 8400 3800 50  0001 C CNN
	1    8400 3800
	0    -1   -1   0   
$EndComp
Connection ~ 8650 4150
Wire Wire Line
	9900 3950 9700 3950
Text Notes 7150 6250 0    50   ~ 0
DSHP06TSGET\n1/12 : boot0 enable\n2/11 : Led info enable\n3/10 : Trace enable\n4/9 : Scale type (full or half)\n5/8 : Wdg delay 1\n6/7 : Wdg delay 2
Text GLabel 2600 2750 2    50   Output ~ 0
I2S_CK-MIC_GAIN
Text GLabel 2600 2850 2    50   Input ~ 0
I2S_WS
Text GLabel 2600 2950 2    50   Input ~ 0
I2S_SD-MIC_ANA
$Comp
L Device:C C?
U 1 1 629B6680
P 9500 1800
AR Path="/629B6680" Ref="C?"  Part="1" 
AR Path="/62CC297B/629B6680" Ref="C?"  Part="1" 
AR Path="/61B961C0/629B6680" Ref="C32"  Part="1" 
F 0 "C32" V 9650 1600 50  0000 L CNN
F 1 "100nF" V 9550 1600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9538 1650 50  0001 C CNN
F 3 "~" H 9500 1800 50  0001 C CNN
	1    9500 1800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9350 1500 9500 1500
Wire Wire Line
	9500 1500 9500 1650
Connection ~ 9500 1500
Wire Wire Line
	9500 1500 9800 1500
$Comp
L power:GND #PWR?
U 1 1 629BCD3A
P 9500 2250
AR Path="/629BCD3A" Ref="#PWR?"  Part="1" 
AR Path="/62EA3721/629BCD3A" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/629BCD3A" Ref="#PWR0176"  Part="1" 
F 0 "#PWR0176" H 9500 2000 50  0001 C CNN
F 1 "GND" H 9505 2077 50  0000 C CNN
F 2 "" H 9500 2250 50  0001 C CNN
F 3 "" H 9500 2250 50  0001 C CNN
	1    9500 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 1950 9500 2250
$Comp
L Device:R R?
U 1 1 629CC3FC
P 9500 1250
AR Path="/629CC3FC" Ref="R?"  Part="1" 
AR Path="/62EA3721/629CC3FC" Ref="R?"  Part="1" 
AR Path="/61B961C0/629CC3FC" Ref="R71"  Part="1" 
F 0 "R71" H 9570 1296 50  0000 L CNN
F 1 "10k" H 9570 1205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9430 1250 50  0001 C CNN
F 3 "~" H 9500 1250 50  0001 C CNN
	1    9500 1250
	1    0    0    -1  
$EndComp
$Comp
L EcoApi_pcb-rescue:+3V3-BluePillF4 #SUPPLY?
U 1 1 629CC402
P 9500 1000
AR Path="/629CC402" Ref="#SUPPLY?"  Part="1" 
AR Path="/62EA3721/629CC402" Ref="#SUPPLY?"  Part="1" 
AR Path="/61B961C0/629CC402" Ref="#SUPPLY0121"  Part="1" 
F 0 "#SUPPLY0121" H 9500 1000 45  0001 L BNN
F 1 "+3V3" H 9500 1171 45  0000 C CNN
F 2 "" H 9500 1000 60  0001 C CNN
F 3 "" H 9500 1000 60  0001 C CNN
	1    9500 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 1100 9500 1000
Wire Wire Line
	9500 1400 9500 1500
Text GLabel 9400 5750 2    50   Input ~ 0
WDG_DELAY1
Text GLabel 9400 5900 2    50   Input ~ 0
WDG_DELAY2
Wire Wire Line
	8800 5750 8800 4900
Wire Wire Line
	8800 4900 8650 4900
Wire Wire Line
	8700 5000 8650 5000
Wire Wire Line
	8650 4150 8650 4500
$Comp
L EcoApi_pcb:DSHP06TSGET S2
U 1 1 63C29548
P 7450 4500
F 0 "S2" H 8050 4765 50  0000 C CNN
F 1 "DSHP06TSGET" H 8050 4674 50  0000 C CNN
F 2 "EcoApi_pcb:DSHP06TSGET" H 8500 4600 50  0001 L CNN
F 3 "https://datasheet.lcsc.com/szlcsc/1810191313_Dongguan-Guangzhu-Industrial-DSHP06TSGET_C40737.pdf" H 8500 4500 50  0001 L CNN
F 4 "SPST 6 1.27mm, Half Slide (Standard) Black SMD DIP Switches" H 8500 4400 50  0001 L CNN "Description"
F 5 "2.6" H 8500 4300 50  0001 L CNN "Height"
F 6 "Dongguan Guangzhu Industrial" H 8500 4200 50  0001 L CNN "Manufacturer_Name"
F 7 "DSHP06TSGET" H 8500 4100 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "" H 8500 4000 50  0001 L CNN "Mouser Part Number"
F 9 "" H 8500 3900 50  0001 L CNN "Mouser Price/Stock"
F 10 "" H 8500 3800 50  0001 L CNN "Arrow Part Number"
F 11 "" H 8500 3700 50  0001 L CNN "Arrow Price/Stock"
	1    7450 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 4600 7300 4600
Wire Wire Line
	7300 4600 7300 4700
Wire Wire Line
	7300 5000 7450 5000
Wire Wire Line
	7450 4900 7300 4900
Connection ~ 7300 4900
Wire Wire Line
	7300 4900 7300 5000
Wire Wire Line
	7450 4800 7300 4800
Connection ~ 7300 4800
Wire Wire Line
	7300 4800 7300 4900
Wire Wire Line
	7450 4700 7300 4700
Connection ~ 7300 4700
Wire Wire Line
	7300 4700 7300 4800
$Comp
L power:GND #PWR?
U 1 1 63C3607F
P 7300 5200
AR Path="/63C3607F" Ref="#PWR?"  Part="1" 
AR Path="/62EA3721/63C3607F" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/63C3607F" Ref="#PWR0177"  Part="1" 
F 0 "#PWR0177" H 7300 4950 50  0001 C CNN
F 1 "GND" H 7305 5027 50  0000 C CNN
F 2 "" H 7300 5200 50  0001 C CNN
F 3 "" H 7300 5200 50  0001 C CNN
	1    7300 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 5000 7300 5200
Connection ~ 7300 5000
$Comp
L EcoApi_pcb-rescue:+3V3-BluePillF4 #SUPPLY?
U 1 1 63C3D869
P 7300 4300
AR Path="/63C3D869" Ref="#SUPPLY?"  Part="1" 
AR Path="/62B8145A/63C3D869" Ref="#SUPPLY?"  Part="1" 
AR Path="/61B961C0/63C3D869" Ref="#SUPPLY0122"  Part="1" 
F 0 "#SUPPLY0122" H 7300 4300 45  0001 L BNN
F 1 "+3V3" H 7300 4471 45  0000 C CNN
F 2 "" H 7300 4300 60  0001 C CNN
F 3 "" H 7300 4300 60  0001 C CNN
	1    7300 4300
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7450 4500 7300 4500
Wire Wire Line
	7300 4500 7300 4300
Wire Wire Line
	9300 4250 9300 4300
$Comp
L Device:R R?
U 1 1 63E4E2A3
P 9300 5250
AR Path="/63E4E2A3" Ref="R?"  Part="1" 
AR Path="/62EA3721/63E4E2A3" Ref="R?"  Part="1" 
AR Path="/61B961C0/63E4E2A3" Ref="R69"  Part="1" 
F 0 "R69" H 9370 5296 50  0000 L CNN
F 1 "10k" H 9370 5205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9230 5250 50  0001 C CNN
F 3 "~" H 9300 5250 50  0001 C CNN
	1    9300 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 5500 9300 5500
Wire Wire Line
	9300 5400 9300 5500
$Comp
L EcoApi_pcb-rescue:+3V3-BluePillF4 #SUPPLY?
U 1 1 63E4E2AD
P 9300 5050
AR Path="/63E4E2AD" Ref="#SUPPLY?"  Part="1" 
AR Path="/62EA3721/63E4E2AD" Ref="#SUPPLY?"  Part="1" 
AR Path="/61B961C0/63E4E2AD" Ref="#SUPPLY0127"  Part="1" 
F 0 "#SUPPLY0127" H 9300 5050 45  0001 L BNN
F 1 "+3V3" H 9300 5221 45  0000 C CNN
F 2 "" H 9300 5050 60  0001 C CNN
F 3 "" H 9300 5050 60  0001 C CNN
	1    9300 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 5050 9300 5100
Wire Wire Line
	8900 5500 9300 5500
Connection ~ 9300 5500
Wire Wire Line
	8800 5750 9400 5750
Wire Wire Line
	8700 5900 9400 5900
Wire Wire Line
	8700 5000 8700 5900
Text GLabel 1350 6350 2    50   Input ~ 0
UART_TX
Text GLabel 1350 6450 2    50   Output ~ 0
UART_RX
$Comp
L power:GND #PWR?
U 1 1 61E16B9F
P 1700 6550
AR Path="/61E16B9F" Ref="#PWR?"  Part="1" 
AR Path="/62B8145A/61E16B9F" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/61E16B9F" Ref="#PWR0178"  Part="1" 
F 0 "#PWR0178" H 1700 6300 50  0001 C CNN
F 1 "GND" H 1705 6377 50  0000 C CNN
F 2 "" H 1700 6550 50  0001 C CNN
F 3 "" H 1700 6550 50  0001 C CNN
	1    1700 6550
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x05_Male J5
U 1 1 61E1EB4D
P 1150 5600
F 0 "J5" H 1258 5981 50  0000 C CNN
F 1 "DBG" H 1258 5890 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 1150 5600 50  0001 C CNN
F 3 "~" H 1150 5600 50  0001 C CNN
	1    1150 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 5600 1350 5600
Wire Wire Line
	1700 5800 1350 5800
$Comp
L Connector:Conn_01x03_Male J6
U 1 1 61E4F280
P 1150 6450
F 0 "J6" H 1258 6731 50  0000 C CNN
F 1 "TRACE" H 1258 6640 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 1150 6450 50  0001 C CNN
F 3 "~" H 1150 6450 50  0001 C CNN
	1    1150 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 6550 1700 6550
$Comp
L EcoApi_pcb-rescue:+3V3-BluePillF4 #SUPPLY?
U 1 1 61E938BC
P 3250 3350
AR Path="/61E938BC" Ref="#SUPPLY?"  Part="1" 
AR Path="/62CC297B/61E938BC" Ref="#SUPPLY?"  Part="1" 
AR Path="/61B961C0/61E938BC" Ref="#SUPPLY0128"  Part="1" 
F 0 "#SUPPLY0128" H 3250 3350 45  0001 L BNN
F 1 "+3V3" H 3250 3520 45  0000 C CNN
F 2 "" H 3250 3350 60  0001 C CNN
F 3 "" H 3250 3350 60  0001 C CNN
	1    3250 3350
	0    1    1    0   
$EndComp
Wire Wire Line
	2600 2650 3250 2650
$Comp
L EcoApi_pcb:B5B-XH-A__LF__SN_ J8
U 1 1 61F018A0
P 2600 1750
F 0 "J8" H 2892 1085 50  0000 C CNN
F 1 "B5B-XH-A__LF__SN_" H 2892 1176 50  0000 C CNN
F 2 "EcoApi_pcb:SHDR5W64P0X250_1X5_1490X575X700P" H 3250 1850 50  0001 L CNN
F 3 "http://www.farnell.com/datasheets/1684882.pdf" H 3250 1750 50  0001 L CNN
F 4 "Shrouded hdr,XH,2.5mm pitch,straight,5w JST XH Series, 2.5mm Pitch 5 Way 1 Row Straight PCB Header, Solder Termination, 3A" H 3250 1650 50  0001 L CNN "Description"
F 5 "7" H 3250 1550 50  0001 L CNN "Height"
F 6 "JST (JAPAN SOLDERLESS TERMINALS)" H 3250 1450 50  0001 L CNN "Manufacturer_Name"
F 7 "B5B-XH-A (LF)(SN)" H 3250 1350 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "" H 3250 1250 50  0001 L CNN "Mouser Part Number"
F 9 "" H 3250 1150 50  0001 L CNN "Mouser Price/Stock"
F 10 "" H 3250 1050 50  0001 L CNN "Arrow Part Number"
F 11 "" H 3250 950 50  0001 L CNN "Arrow Price/Stock"
	1    2600 1750
	-1   0    0    1   
$EndComp
$Comp
L EcoApi_pcb:B8B-XH-A_LF__SN_ J9
U 1 1 61F02CAF
P 2600 3350
F 0 "J9" H 2892 2385 50  0000 C CNN
F 1 "B8B-XH-A_LF__SN_" H 2892 2476 50  0000 C CNN
F 2 "EcoApi_pcb:SHDR8W64P0X250_1X8_2240X575X700P" H 3250 3450 50  0001 L CNN
F 3 "https://datasheet.datasheetarchive.com/originals/distributors/Datasheets_SAMA/45bda73f655da0a850bf2ea3e93b2e86.pdf" H 3250 3350 50  0001 L CNN
F 4 "XH-2.5mm Header top entry 8 way JST XH Series, Series Number B8B, 2.5mm Pitch 8 Way 1 Row Straight PCB Header, Solder Termination, 3A" H 3250 3250 50  0001 L CNN "Description"
F 5 "7" H 3250 3150 50  0001 L CNN "Height"
F 6 "JST (JAPAN SOLDERLESS TERMINALS)" H 3250 3050 50  0001 L CNN "Manufacturer_Name"
F 7 "B8B-XH-A(LF)(SN)" H 3250 2950 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "" H 3250 2850 50  0001 L CNN "Mouser Part Number"
F 9 "" H 3250 2750 50  0001 L CNN "Mouser Price/Stock"
F 10 "B8B-XH-A(LF)(SN)" H 3250 2650 50  0001 L CNN "Arrow Part Number"
F 11 "https://www.arrow.com/en/products/b8b-xh-a-lf-sn/jst-manufacturing" H 3250 2550 50  0001 L CNN "Arrow Price/Stock"
	1    2600 3350
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61BB1D09
P 3300 3050
AR Path="/61BB1D09" Ref="#PWR?"  Part="1" 
AR Path="/62D38F6A/61BB1D09" Ref="#PWR?"  Part="1" 
AR Path="/62EA3721/61BB1D09" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/61BB1D09" Ref="#PWR0155"  Part="1" 
F 0 "#PWR0155" H 3300 2800 50  0001 C CNN
F 1 "GND" H 3305 2877 50  0000 C CNN
F 2 "" H 3300 3050 50  0001 C CNN
F 3 "" H 3300 3050 50  0001 C CNN
	1    3300 3050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2600 3350 3250 3350
Wire Wire Line
	2600 3050 3300 3050
Text GLabel 6450 2000 0    50   BiDi ~ 0
EN_AVBATT-BTN_TARE
Wire Wire Line
	6550 850  6550 1000
$Comp
L Connector:Conn_01x02_Male J?
U 1 1 63C32D33
P 10850 1800
AR Path="/63C32D33" Ref="J?"  Part="1" 
AR Path="/62D38F6A/63C32D33" Ref="J?"  Part="1" 
AR Path="/62CC297B/63C32D33" Ref="J?"  Part="1" 
AR Path="/61B961C0/63C32D33" Ref="J12"  Part="1" 
F 0 "J12" V 10950 2000 50  0000 R CNN
F 1 "POWER ON" V 10750 2050 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 10850 1800 50  0001 C CNN
F 3 "~" H 10850 1800 50  0001 C CNN
	1    10850 1800
	-1   0    0    1   
$EndComp
Wire Wire Line
	10650 1800 10650 2250
Wire Wire Line
	10650 1500 10650 1700
Connection ~ 10650 1500
$Comp
L Connector:Conn_01x02_Male J?
U 1 1 63DDD3C1
P 8600 2100
AR Path="/63DDD3C1" Ref="J?"  Part="1" 
AR Path="/62D38F6A/63DDD3C1" Ref="J?"  Part="1" 
AR Path="/62CC297B/63DDD3C1" Ref="J?"  Part="1" 
AR Path="/61B961C0/63DDD3C1" Ref="J10"  Part="1" 
F 0 "J10" V 8700 2300 50  0000 R CNN
F 1 "TILT" V 8500 2350 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8600 2100 50  0001 C CNN
F 3 "~" H 8600 2100 50  0001 C CNN
	1    8600 2100
	-1   0    0    1   
$EndComp
Wire Wire Line
	8400 2100 8400 2500
$Comp
L power:GND #PWR?
U 1 1 63E19E1A
P 3250 1800
AR Path="/63E19E1A" Ref="#PWR?"  Part="1" 
AR Path="/62D38F6A/63E19E1A" Ref="#PWR?"  Part="1" 
AR Path="/62EA3721/63E19E1A" Ref="#PWR?"  Part="1" 
AR Path="/61B961C0/63E19E1A" Ref="#PWR0188"  Part="1" 
F 0 "#PWR0188" H 3250 1550 50  0001 C CNN
F 1 "GND" H 3255 1627 50  0000 C CNN
F 2 "" H 3250 1800 50  0001 C CNN
F 3 "" H 3250 1800 50  0001 C CNN
	1    3250 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 1750 3250 1750
Wire Wire Line
	3250 1750 3250 1800
Wire Wire Line
	2600 1450 3250 1450
Wire Wire Line
	3250 1450 3250 1750
Connection ~ 3250 1750
$EndSCHEMATC
