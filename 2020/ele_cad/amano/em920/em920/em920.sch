EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR02
U 1 1 60FC137E
P 1800 2300
F 0 "#PWR02" H 1800 2050 50  0001 C CNN
F 1 "GND" H 1850 2100 50  0000 C CNN
F 2 "" H 1800 2300 50  0001 C CNN
F 3 "" H 1800 2300 50  0001 C CNN
	1    1800 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 750  1800 900 
Wire Wire Line
	1800 2100 1800 2300
$Comp
L power:+3V3 #PWR05
U 1 1 60FC4E42
P 3500 1050
F 0 "#PWR05" H 3500 900 50  0001 C CNN
F 1 "+3V3" H 3550 1250 50  0000 C CNN
F 2 "" H 3500 1050 50  0001 C CNN
F 3 "" H 3500 1050 50  0001 C CNN
	1    3500 1050
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR01
U 1 1 60FC5E35
P 1800 750
F 0 "#PWR01" H 1800 600 50  0001 C CNN
F 1 "+3V3" H 1850 950 50  0000 C CNN
F 2 "" H 1800 750 50  0001 C CNN
F 3 "" H 1800 750 50  0001 C CNN
	1    1800 750 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 60FC5745
P 3500 1900
F 0 "#PWR06" H 3500 1650 50  0001 C CNN
F 1 "GND" H 3550 1700 50  0000 C CNN
F 2 "" H 3500 1900 50  0001 C CNN
F 3 "" H 3500 1900 50  0001 C CNN
	1    3500 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 1050 3500 1200
Wire Wire Line
	3500 1200 3550 1200
Wire Wire Line
	3550 1600 3500 1600
Wire Wire Line
	3500 1600 3500 1900
Text Label 2500 1500 0    50   ~ 0
RX
Text Label 2500 1600 0    50   ~ 0
TX
Text Label 2500 1400 0    50   ~ 0
EM_ENABLE
Text Label 2500 1300 0    50   ~ 0
RP_ENABLE
$Comp
L power:+3V3 #PWR03
U 1 1 60FD0099
P 3500 2600
F 0 "#PWR03" H 3500 2450 50  0001 C CNN
F 1 "+3V3" H 3550 2800 50  0000 C CNN
F 2 "" H 3500 2600 50  0001 C CNN
F 3 "" H 3500 2600 50  0001 C CNN
	1    3500 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 2600 3500 2700
Wire Wire Line
	3350 1500 3350 2850
Wire Wire Line
	3350 2850 3500 2850
Wire Wire Line
	3500 3000 3200 3000
Wire Wire Line
	3200 3000 3200 1600
$Comp
L power:GND #PWR04
U 1 1 60FD4012
P 3500 3500
F 0 "#PWR04" H 3500 3250 50  0001 C CNN
F 1 "GND" H 3550 3300 50  0000 C CNN
F 2 "" H 3500 3500 50  0001 C CNN
F 3 "" H 3500 3500 50  0001 C CNN
	1    3500 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 3300 3500 3500
Wire Wire Line
	2400 1600 3200 1600
Wire Wire Line
	2400 1500 3350 1500
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 60FDFD4D
P 3750 1050
F 0 "#FLG0101" H 3750 1125 50  0001 C CNN
F 1 "PWR_FLAG" V 3750 1200 50  0000 L CNN
F 2 "" H 3750 1050 50  0001 C CNN
F 3 "~" H 3750 1050 50  0001 C CNN
	1    3750 1050
	0    1    1    0   
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 60FE07FE
P 3750 1900
F 0 "#FLG0102" H 3750 1975 50  0001 C CNN
F 1 "PWR_FLAG" V 3750 2050 50  0000 L CNN
F 2 "" H 3750 1900 50  0001 C CNN
F 3 "~" H 3750 1900 50  0001 C CNN
	1    3750 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	3500 1900 3750 1900
Connection ~ 3500 1900
Wire Wire Line
	3500 1050 3750 1050
Connection ~ 3500 1050
NoConn ~ 850  500 
Wire Wire Line
	3550 1200 4450 1200
Wire Wire Line
	4450 1200 4450 1300
Connection ~ 3550 1200
Wire Wire Line
	4450 1600 3550 1600
Connection ~ 3550 1600
NoConn ~ 3550 1300
NoConn ~ 3550 1500
$Comp
L Device:C C1
U 1 1 60FF4567
P 4450 1450
F 0 "C1" H 4600 1500 50  0000 L CNN
F 1 "0.1uF" H 4600 1450 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 4488 1300 50  0001 C CNN
F 3 "~" H 4450 1450 50  0001 C CNN
F 4 "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 4450 1450 50  0001 C CNN "Footprint"
F 5 "C1" H 4450 1450 50  0001 C CNN "Value"
	1    4450 1450
	1    0    0    -1  
$EndComp
$Comp
L MCU_Microchip_ATtiny:ATtiny85-20PU U1
U 1 1 60FBF9F7
P 1800 1500
F 0 "U1" H 1250 1550 50  0000 R CNN
F 1 "ATtiny85-20PU" H 1250 1500 50  0000 R CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 1800 1500 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-2586-avr-8-bit-microcontroller-attiny25-attiny45-attiny85_datasheet.pdf" H 1800 1500 50  0001 C CNN
	1    1800 1500
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J1
U 1 1 60FC3A2C
P 3750 1400
F 0 "J1" H 3850 1400 50  0000 L CNN
F 1 "Conn_01x06" H 3850 1300 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 3750 1400 50  0001 C CNN
F 3 "~" H 3750 1400 50  0001 C CNN
	1    3750 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 3150 3500 3150
Text Label 2500 1200 0    50   ~ 0
RESET
Wire Wire Line
	2400 1400 3550 1400
Wire Wire Line
	2400 1300 3400 1300
Wire Wire Line
	3400 1300 3400 1700
Wire Wire Line
	3400 1700 3550 1700
Wire Wire Line
	2400 1200 3100 1200
Wire Wire Line
	3100 1200 3100 3150
NoConn ~ 2400 1700
$Comp
L em920_lib:TY92SS U2
U 1 1 6123A86E
P 3500 2700
F 0 "U2" H 3950 2965 50  0000 C CNN
F 1 "TY92SS" H 3950 2874 50  0000 C CNN
F 2 "em920_lib:TY92SS" H 3500 2700 50  0001 C CNN
F 3 "" H 3500 2700 50  0001 C CNN
	1    3500 2700
	1    0    0    -1  
$EndComp
NoConn ~ 4400 2700
NoConn ~ 4400 2850
NoConn ~ 4400 3000
NoConn ~ 4400 3150
$EndSCHEMATC
