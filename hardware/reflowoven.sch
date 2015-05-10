EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Reflowoven"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_01X08 P?
U 1 1 554F614C
P 3630 3253
F 0 "P?" V 3626 2802 50  0000 C CNN
F 1 "DIGITAL" V 3730 3253 50  0000 C CNN
F 2 "" H 3630 3253 60  0000 C CNN
F 3 "" H 3630 3253 60  0000 C CNN
	1    3630 3253
	0    1    1    0   
$EndComp
$Comp
L CONN_01X08 P?
U 1 1 554F6227
P 4511 3253
F 0 "P?" V 4499 3701 50  0000 C CNN
F 1 "DIGITAL" V 4611 3253 50  0000 C CNN
F 2 "" H 4511 3253 60  0000 C CNN
F 3 "" H 4511 3253 60  0000 C CNN
	1    4511 3253
	0    1    1    0   
$EndComp
$Comp
L CONN_01X06 P?
U 1 1 554F6367
P 4645 4685
F 0 "P?" V 4651 4332 50  0000 C CNN
F 1 "ANALOG IN" V 4745 4685 50  0000 C CNN
F 2 "" H 4645 4685 60  0000 C CNN
F 3 "" H 4645 4685 60  0000 C CNN
	1    4645 4685
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X06 P?
U 1 1 554F63AE
P 3969 4685
F 0 "P?" V 3969 5048 50  0000 C CNN
F 1 "POWER" V 4069 4685 50  0000 C CNN
F 2 "" H 3969 4685 60  0000 C CNN
F 3 "" H 3969 4685 60  0000 C CNN
	1    3969 4685
	0    -1   -1   0   
$EndComp
Wire Notes Line
	2570 2881 2570 5097
Wire Notes Line
	2570 5097 5262 5097
Wire Notes Line
	5262 5097 5262 2881
Wire Notes Line
	5262 2881 2570 2881
Text Notes 2550 5063 1    60   ~ 0
Arduino Duemilanove
Wire Wire Line
	4019 4885 4019 5337
Wire Wire Line
	4119 4885 4119 4992
Wire Wire Line
	4119 4992 4019 4992
Connection ~ 4019 4992
$Comp
L GND #PWR?
U 1 1 554F68F1
P 4019 5337
F 0 "#PWR?" H 4019 5337 30  0001 C CNN
F 1 "GND" H 4019 5267 30  0001 C CNN
F 2 "" H 4019 5337 60  0000 C CNN
F 3 "" H 4019 5337 60  0000 C CNN
	1    4019 5337
	1    0    0    -1  
$EndComp
$Comp
L 3V3 #PWR?
U 1 1 554F694F
P 3475 5269
F 0 "#PWR?" H 3475 5369 40  0001 C CNN
F 1 "3V3" V 3478 5421 40  0000 C CNN
F 2 "" H 3475 5269 60  0000 C CNN
F 3 "" H 3475 5269 60  0000 C CNN
	1    3475 5269
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR?
U 1 1 554F6963
P 3477 5396
F 0 "#PWR?" H 3477 5486 20  0001 C CNN
F 1 "+5V" V 3469 5526 30  0000 C CNN
F 2 "" H 3477 5396 60  0000 C CNN
F 3 "" H 3477 5396 60  0000 C CNN
	1    3477 5396
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3475 5269 3819 5269
Wire Wire Line
	3819 5269 3819 4885
Wire Wire Line
	3477 5396 3919 5396
Wire Wire Line
	3919 5396 3919 4885
$Comp
L CONN_01X06 P?
U 1 1 554F6B91
P 9017 2473
F 0 "P?" H 9017 2823 50  0000 C CNN
F 1 "K-Type thermocouple module" V 9117 2473 50  0000 C CNN
F 2 "" H 9017 2473 60  0000 C CNN
F 3 "" H 9017 2473 60  0000 C CNN
	1    9017 2473
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P?
U 1 1 554F6CC5
P 7887 4621
F 0 "P?" H 7887 4771 50  0000 C CNN
F 1 "SSR" H 8026 4543 50  0000 C CNN
F 2 "" H 7887 4621 60  0000 C CNN
F 3 "" H 7887 4621 60  0000 C CNN
	1    7887 4621
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P?
U 1 1 554F6D5C
P 7887 5791
F 0 "P?" H 7887 5941 50  0000 C CNN
F 1 "SSR" H 8036 5713 50  0000 C CNN
F 2 "" H 7887 5791 60  0000 C CNN
F 3 "" H 7887 5791 60  0000 C CNN
	1    7887 5791
	1    0    0    -1  
$EndComp
Wire Wire Line
	7687 4671 7511 4671
Wire Wire Line
	7511 4671 7511 6137
Wire Wire Line
	7687 5841 7511 5841
Connection ~ 7511 5841
$Comp
L GND #PWR?
U 1 1 554F6F9E
P 7511 6137
F 0 "#PWR?" H 7511 6137 30  0001 C CNN
F 1 "GND" H 7511 6067 30  0001 C CNN
F 2 "" H 7511 6137 60  0000 C CNN
F 3 "" H 7511 6137 60  0000 C CNN
	1    7511 6137
	1    0    0    -1  
$EndComp
Wire Wire Line
	7687 4571 6989 4571
Wire Wire Line
	7687 5741 7687 5740
Wire Wire Line
	7687 5740 6998 5740
Wire Wire Line
	4461 3053 4461 2759
Text Label 4461 2809 1    60   ~ 0
SSR
Text Label 7163 4571 0    60   ~ 0
SSR
Text Label 7115 5740 0    60   ~ 0
SSR
$Comp
L R R?
U 1 1 554F77EE
P 7570 1925
F 0 "R?" V 7650 1925 40  0000 C CNN
F 1 "4.7K" V 7577 1926 40  0000 C CNN
F 2 "" V 7500 1925 30  0000 C CNN
F 3 "" H 7570 1925 30  0000 C CNN
	1    7570 1925
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 554F78A1
P 7570 2519
F 0 "R?" V 7650 2519 40  0000 C CNN
F 1 "2.2K" V 7577 2520 40  0000 C CNN
F 2 "" V 7500 2519 30  0000 C CNN
F 3 "" H 7570 2519 30  0000 C CNN
	1    7570 2519
	1    0    0    -1  
$EndComp
$Comp
L 3V3 #PWR?
U 1 1 554F79B1
P 8613 1889
F 0 "#PWR?" H 8613 1989 40  0001 C CNN
F 1 "3V3" H 8613 2014 40  0000 C CNN
F 2 "" H 8613 1889 60  0000 C CNN
F 3 "" H 8613 1889 60  0000 C CNN
	1    8613 1889
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 554F79C5
P 8520 2900
F 0 "#PWR?" H 8520 2900 30  0001 C CNN
F 1 "GND" H 8520 2830 30  0001 C CNN
F 2 "" H 8520 2900 60  0000 C CNN
F 3 "" H 8520 2900 60  0000 C CNN
	1    8520 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8613 1889 8613 2723
Wire Wire Line
	8613 2723 8817 2723
Wire Wire Line
	8817 2623 8520 2623
Wire Wire Line
	8520 2623 8520 2900
Wire Wire Line
	8817 2323 8465 2323
Text Label 8511 2323 2    60   ~ 0
MISO
NoConn ~ 8817 2423
Wire Wire Line
	3580 3053 3580 2799
Text Label 3580 2841 1    60   ~ 0
MISO
Wire Wire Line
	7570 2175 7570 2269
$Comp
L R R?
U 1 1 554F92CD
P 7797 3203
F 0 "R?" V 7877 3203 40  0000 C CNN
F 1 "4.7K" V 7804 3204 40  0000 C CNN
F 2 "" V 7727 3203 30  0000 C CNN
F 3 "" H 7797 3203 30  0000 C CNN
	1    7797 3203
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 554F92D3
P 7797 3797
F 0 "R?" V 7877 3797 40  0000 C CNN
F 1 "2.2K" V 7804 3798 40  0000 C CNN
F 2 "" V 7727 3797 30  0000 C CNN
F 3 "" H 7797 3797 30  0000 C CNN
	1    7797 3797
	1    0    0    -1  
$EndComp
Wire Wire Line
	7797 3453 7797 3547
Wire Wire Line
	8817 2223 7570 2223
Connection ~ 7570 2223
Wire Wire Line
	8817 2523 8223 2523
Wire Wire Line
	8223 2523 8223 3501
Wire Wire Line
	8223 3501 7797 3501
Connection ~ 7797 3501
Wire Wire Line
	3480 3054 3480 1469
Wire Wire Line
	3480 1469 7570 1469
Wire Wire Line
	7570 1469 7570 1675
Wire Wire Line
	3780 3053 3780 2019
Wire Wire Line
	3780 2019 6354 2019
Wire Wire Line
	6354 2019 6354 2810
Wire Wire Line
	6354 2810 7797 2810
Wire Wire Line
	7797 2810 7797 2953
Wire Wire Line
	7570 2769 7570 2882
$Comp
L GND #PWR?
U 1 1 554F9FE1
P 7570 2882
F 0 "#PWR?" H 7570 2882 30  0001 C CNN
F 1 "GND" H 7570 2812 30  0001 C CNN
F 2 "" H 7570 2882 60  0000 C CNN
F 3 "" H 7570 2882 60  0000 C CNN
	1    7570 2882
	1    0    0    -1  
$EndComp
Wire Wire Line
	7797 4047 7797 4129
$Comp
L GND #PWR?
U 1 1 554FA11F
P 7797 4129
F 0 "#PWR?" H 7797 4129 30  0001 C CNN
F 1 "GND" H 7797 4059 30  0001 C CNN
F 2 "" H 7797 4129 60  0000 C CNN
F 3 "" H 7797 4129 60  0000 C CNN
	1    7797 4129
	1    0    0    -1  
$EndComp
Text Label 8503 2223 2    60   ~ 0
R-SCK
Text Label 3480 2836 1    60   ~ 0
SCK-R
Text Label 3780 2835 1    60   ~ 0
/SS-R
Text Label 8501 2523 2    60   ~ 0
R-/SS
$EndSCHEMATC