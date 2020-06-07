EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "afterglow stern sam"
Date ""
Rev "1.3"
Comp "morbid cornflakes X coder cabana"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Conn_01x10_Male J1
U 1 1 59D5D70B
P 800 1800
F 0 "J1" H 800 2300 50  0000 C CNN
F 1 "J13_IN_C" H 900 1200 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-41791-9_9x3.96mm_Straight" H 800 1800 50  0001 C CNN
F 3 "" H 800 1800 50  0001 C CNN
	1    800  1800
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x10_Male J5
U 1 1 59D5EC41
P 2400 1800
F 0 "J5" H 2400 2300 50  0000 C CNN
F 1 "J13_OUT_C" H 2600 1200 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-41791-9_9x3.96mm_Straight" H 2400 1800 50  0001 C CNN
F 3 "" H 2400 1800 50  0001 C CNN
	1    2400 1800
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x12_Male J4
U 1 1 59D5EE35
P 2300 3100
F 0 "J4" H 2050 3100 50  0000 C CNN
F 1 "J12_OUT_R" H 1800 2550 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-6410-09_09x2.54mm_Straight" H 2300 3100 50  0001 C CNN
F 3 "" H 2300 3100 50  0001 C CNN
	1    2300 3100
	-1   0    0    -1  
$EndComp
$Comp
L Device:Fuse F1
U 1 1 59DA27BF
P 1700 750
F 0 "F1" V 1780 750 50  0000 C CNN
F 1 "5A SB" V 1625 750 50  0000 C CNN
F 2 "Fuse_Holders_and_Fuses:Fuseholder5x20_horiz_open_lateral_Type-II" V 1630 750 50  0001 C CNN
F 3 "" H 1700 750 50  0001 C CNN
	1    1700 750 
	0    1    1    0   
$EndComp
Text Label 1900 650  0    60   ~ 0
18V
$Comp
L Connector:Conn_01x02_Male J3
U 1 1 59DA2DDC
P 1200 750
F 0 "J3" H 1200 850 50  0000 C CNN
F 1 "Conn_Power" H 1200 550 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-41791-02_02x3.96mm_Straight" H 1200 750 50  0001 C CNN
F 3 "" H 1200 750 50  0001 C CNN
	1    1200 750 
	1    0    0    -1  
$EndComp
Text Label 2200 1400 2    60   ~ 0
J13_CO1
Text Label 2200 1600 2    60   ~ 0
J13_CO2
Text Label 2200 1700 2    60   ~ 0
J13_CO3
Text Label 2200 1800 2    60   ~ 0
J13_CO4
Text Label 2200 1900 2    60   ~ 0
J13_CO5
Text Label 2200 2000 2    60   ~ 0
J13_CO6
Text Label 2200 2100 2    60   ~ 0
J13_CO7
Text Label 2200 2200 2    60   ~ 0
J13_CO8
Text Label 2050 2600 2    60   ~ 0
J12_RO1
Text Label 2050 2700 2    60   ~ 0
J12_RO2
Text Label 2050 2800 2    60   ~ 0
J12_RO3
Text Label 2050 2900 2    60   ~ 0
J12_RO4
Text Label 2050 3000 2    60   ~ 0
J12_RO5
Text Label 2050 3100 2    60   ~ 0
J12_RO6
Text Label 2100 3300 2    60   ~ 0
J12_RO7
Text Label 2100 3400 2    60   ~ 0
J12_RO8
$Comp
L Device:R R17
U 1 1 59DABDB5
P 1150 1400
F 0 "R17" V 1050 1400 50  0000 C CNN
F 1 "1k" V 1150 1400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1080 1400 50  0001 C CNN
F 3 "" H 1150 1400 50  0001 C CNN
	1    1150 1400
	0    1    1    0   
$EndComp
$Comp
L Device:R R18
U 1 1 59DAC1A1
P 1150 1600
F 0 "R18" V 1230 1600 50  0000 C CNN
F 1 "1k" V 1150 1600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1080 1600 50  0001 C CNN
F 3 "" H 1150 1600 50  0001 C CNN
	1    1150 1600
	0    1    1    0   
$EndComp
$Comp
L Device:R R19
U 1 1 59DAC1E4
P 1150 1700
F 0 "R19" V 1230 1700 50  0000 C CNN
F 1 "1k" V 1150 1700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1080 1700 50  0001 C CNN
F 3 "" H 1150 1700 50  0001 C CNN
	1    1150 1700
	0    1    1    0   
$EndComp
$Comp
L Device:R R20
U 1 1 59DAC22A
P 1150 1800
F 0 "R20" V 1230 1800 50  0000 C CNN
F 1 "1k" V 1150 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1080 1800 50  0001 C CNN
F 3 "" H 1150 1800 50  0001 C CNN
	1    1150 1800
	0    1    1    0   
$EndComp
$Comp
L Device:R R21
U 1 1 59DAC277
P 1150 1900
F 0 "R21" V 1230 1900 50  0000 C CNN
F 1 "1k" V 1150 1900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1080 1900 50  0001 C CNN
F 3 "" H 1150 1900 50  0001 C CNN
	1    1150 1900
	0    1    1    0   
$EndComp
$Comp
L Device:R R22
U 1 1 59DAC2C3
P 1150 2000
F 0 "R22" V 1230 2000 50  0000 C CNN
F 1 "1k" V 1150 2000 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1080 2000 50  0001 C CNN
F 3 "" H 1150 2000 50  0001 C CNN
	1    1150 2000
	0    1    1    0   
$EndComp
$Comp
L Device:R R23
U 1 1 59DAC312
P 1150 2100
F 0 "R23" V 1230 2100 50  0000 C CNN
F 1 "1k" V 1150 2100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1080 2100 50  0001 C CNN
F 3 "" H 1150 2100 50  0001 C CNN
	1    1150 2100
	0    1    1    0   
$EndComp
$Comp
L Device:R R24
U 1 1 59DAC370
P 1150 2200
F 0 "R24" V 1230 2200 50  0000 C CNN
F 1 "1k" V 1150 2200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1080 2200 50  0001 C CNN
F 3 "" H 1150 2200 50  0001 C CNN
	1    1150 2200
	0    1    1    0   
$EndComp
Text Label 1300 1400 0    60   ~ 0
CI1
Text Label 1300 1600 0    60   ~ 0
CI2
Text Label 1300 1700 0    60   ~ 0
CI3
Text Label 1300 1800 0    60   ~ 0
CI4
Text Label 1300 1900 0    60   ~ 0
CI5
Text Label 1300 2000 0    60   ~ 0
CI6
Text Label 1300 2100 0    60   ~ 0
CI7
Text Label 1300 2200 0    60   ~ 0
CI8
$Comp
L afterglow_nano_smd_stern_sam-rescue:LM7805_TO220-Regulator_Linear-afterglow_nano_smd-rescue U3
U 1 1 59DE1BB0
P 3000 750
F 0 "U3" H 2850 875 50  0000 C CNN
F 1 "5V_Regulator" H 3000 875 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-220_Vertical" H 3000 975 50  0001 C CIN
F 3 "" H 3000 700 50  0001 C CNN
	1    3000 750 
	1    0    0    -1  
$EndComp
Text Label 3600 750  0    60   ~ 0
5V
$Comp
L Device:C_Small C2
U 1 1 59DE33FA
P 2600 950
F 0 "C2" H 2610 1020 50  0000 L CNN
F 1 "0.33" H 2610 870 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2600 950 50  0001 C CNN
F 3 "" H 2600 950 50  0001 C CNN
	1    2600 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 59DE35A1
P 3350 950
F 0 "C3" H 3360 1020 50  0000 L CNN
F 1 "0.1" H 3360 870 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3350 950 50  0001 C CNN
F 3 "" H 3350 950 50  0001 C CNN
	1    3350 950 
	1    0    0    -1  
$EndComp
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF9540N-Transistor_FET-afterglow_nano_smd-rescue Q1
U 1 1 59DF9BBD
P 8100 850
F 0 "Q1" H 8350 925 50  0000 L CNN
F 1 "IRFR9024" H 8350 850 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 8350 775 50  0001 L CIN
F 3 "" H 8100 850 50  0001 L CNN
	1    8100 850 
	1    0    0    1   
$EndComp
Text Label 7800 850  2    60   ~ 0
CO1
Text Label 8200 650  0    60   ~ 0
18V
Text Label 8200 1150 0    60   ~ 0
J137_CO1
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF9540N-Transistor_FET-afterglow_nano_smd-rescue Q2
U 1 1 59DFD6C1
P 8100 1600
F 0 "Q2" H 8350 1675 50  0000 L CNN
F 1 "IRFR9024" H 8350 1600 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 8350 1525 50  0001 L CIN
F 3 "" H 8100 1600 50  0001 L CNN
	1    8100 1600
	1    0    0    1   
$EndComp
Text Label 7800 1600 2    60   ~ 0
CO2
Text Label 8200 1400 0    60   ~ 0
18V
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF9540N-Transistor_FET-afterglow_nano_smd-rescue Q3
U 1 1 59DFE015
P 8100 2350
F 0 "Q3" H 8350 2425 50  0000 L CNN
F 1 "IRFR9024" H 8350 2350 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 8350 2275 50  0001 L CIN
F 3 "" H 8100 2350 50  0001 L CNN
	1    8100 2350
	1    0    0    1   
$EndComp
Text Label 7800 2350 2    60   ~ 0
CO3
Text Label 8200 2150 0    60   ~ 0
18V
Text Label 8200 2650 0    60   ~ 0
J137_CO3
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF9540N-Transistor_FET-afterglow_nano_smd-rescue Q4
U 1 1 59DFEFF0
P 8100 3100
F 0 "Q4" H 8350 3175 50  0000 L CNN
F 1 "IRFR9024" H 8350 3100 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 8350 3025 50  0001 L CIN
F 3 "" H 8100 3100 50  0001 L CNN
	1    8100 3100
	1    0    0    1   
$EndComp
Text Label 7800 3100 2    60   ~ 0
CO4
Text Label 8200 2900 0    60   ~ 0
18V
Text Label 8200 3400 0    60   ~ 0
J137_CO4
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF9540N-Transistor_FET-afterglow_nano_smd-rescue Q5
U 1 1 59DFF721
P 8100 3850
F 0 "Q5" H 8350 3925 50  0000 L CNN
F 1 "IRFR9024" H 8350 3850 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 8350 3775 50  0001 L CIN
F 3 "" H 8100 3850 50  0001 L CNN
	1    8100 3850
	1    0    0    1   
$EndComp
Text Label 7800 3850 2    60   ~ 0
CO5
Text Label 8200 3650 0    60   ~ 0
18V
Text Label 8200 4150 0    60   ~ 0
J137_CO5
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF9540N-Transistor_FET-afterglow_nano_smd-rescue Q6
U 1 1 59DFF742
P 8100 4600
F 0 "Q6" H 8350 4675 50  0000 L CNN
F 1 "IRFR9024" H 8350 4600 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 8350 4525 50  0001 L CIN
F 3 "" H 8100 4600 50  0001 L CNN
	1    8100 4600
	1    0    0    1   
$EndComp
Text Label 7800 4600 2    60   ~ 0
CO6
Text Label 8200 4400 0    60   ~ 0
18V
Text Label 8200 4900 0    60   ~ 0
J137_CO6
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF9540N-Transistor_FET-afterglow_nano_smd-rescue Q7
U 1 1 59DFF763
P 8100 5350
F 0 "Q7" H 8350 5425 50  0000 L CNN
F 1 "IRFR9024" H 8350 5350 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 8350 5275 50  0001 L CIN
F 3 "" H 8100 5350 50  0001 L CNN
	1    8100 5350
	1    0    0    1   
$EndComp
Text Label 7800 5350 2    60   ~ 0
CO7
Text Label 8200 5150 0    60   ~ 0
18V
Text Label 8200 5650 0    60   ~ 0
J137_CO7
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF9540N-Transistor_FET-afterglow_nano_smd-rescue Q8
U 1 1 59DFF784
P 8150 6100
F 0 "Q8" H 8400 6175 50  0000 L CNN
F 1 "IRFR9024" H 8400 6100 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 8400 6025 50  0001 L CIN
F 3 "" H 8150 6100 50  0001 L CNN
	1    8150 6100
	1    0    0    1   
$EndComp
Text Label 7850 6100 2    60   ~ 0
CO8
Text Label 8250 5900 0    60   ~ 0
18V
Text Label 8250 6400 0    60   ~ 0
J137_CO8
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF540N-Transistor_FET-afterglow_nano_smd-rescue Q9
U 1 1 59E31C3C
P 10200 850
F 0 "Q9" H 10450 850 50  0000 L CNN
F 1 "IRLR024" H 10450 750 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 10450 775 50  0001 L CIN
F 3 "" H 10200 850 50  0001 L CNN
	1    10200 850 
	1    0    0    -1  
$EndComp
Text Label 9600 850  2    60   ~ 0
RO1
Text Label 10550 750  0    60   ~ 0
J133_RO1
Text Label 9700 650  0    60   ~ 0
18V
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF540N-Transistor_FET-afterglow_nano_smd-rescue Q10
U 1 1 59E34EED
P 10200 1600
F 0 "Q10" H 10450 1600 50  0000 L CNN
F 1 "IRLR024" H 10450 1500 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 10450 1525 50  0001 L CIN
F 3 "" H 10200 1600 50  0001 L CNN
	1    10200 1600
	1    0    0    -1  
$EndComp
Text Label 9600 1600 2    60   ~ 0
RO2
Text Label 10550 1500 0    60   ~ 0
J133_RO2
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF540N-Transistor_FET-afterglow_nano_smd-rescue Q11
U 1 1 59E354D3
P 10200 2350
F 0 "Q11" H 10450 2350 50  0000 L CNN
F 1 "IRLR024" H 10450 2250 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 10450 2275 50  0001 L CIN
F 3 "" H 10200 2350 50  0001 L CNN
	1    10200 2350
	1    0    0    -1  
$EndComp
Text Label 9600 2350 2    60   ~ 0
RO3
Text Label 10550 2250 0    60   ~ 0
J133_RO3
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF540N-Transistor_FET-afterglow_nano_smd-rescue Q12
U 1 1 59E35940
P 10200 3100
F 0 "Q12" H 10450 3100 50  0000 L CNN
F 1 "IRLR024" H 10450 3000 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 10450 3025 50  0001 L CIN
F 3 "" H 10200 3100 50  0001 L CNN
	1    10200 3100
	1    0    0    -1  
$EndComp
Text Label 9600 3100 2    60   ~ 0
RO4
Text Label 10550 3000 0    60   ~ 0
J133_RO4
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF540N-Transistor_FET-afterglow_nano_smd-rescue Q13
U 1 1 59E360AF
P 10200 3850
F 0 "Q13" H 10450 3850 50  0000 L CNN
F 1 "IRLR024" H 10450 3750 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 10450 3775 50  0001 L CIN
F 3 "" H 10200 3850 50  0001 L CNN
	1    10200 3850
	1    0    0    -1  
$EndComp
Text Label 9600 3850 2    60   ~ 0
RO5
Text Label 10550 3750 0    60   ~ 0
J133_RO5
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF540N-Transistor_FET-afterglow_nano_smd-rescue Q14
U 1 1 59E360C1
P 10200 4600
F 0 "Q14" H 10450 4600 50  0000 L CNN
F 1 "IRLR024" H 10450 4500 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 10450 4525 50  0001 L CIN
F 3 "" H 10200 4600 50  0001 L CNN
	1    10200 4600
	1    0    0    -1  
$EndComp
Text Label 9600 4600 2    60   ~ 0
RO6
Text Label 10550 4500 0    60   ~ 0
J133_RO6
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF540N-Transistor_FET-afterglow_nano_smd-rescue Q15
U 1 1 59E360D3
P 10200 5350
F 0 "Q15" H 10450 5350 50  0000 L CNN
F 1 "IRLR024" H 10450 5250 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 10450 5275 50  0001 L CIN
F 3 "" H 10200 5350 50  0001 L CNN
	1    10200 5350
	1    0    0    -1  
$EndComp
Text Label 9600 5350 2    60   ~ 0
RO7
Text Label 10550 5250 0    60   ~ 0
J133_RO7
$Comp
L afterglow_nano_smd_stern_sam-rescue:IRF540N-Transistor_FET-afterglow_nano_smd-rescue Q16
U 1 1 59E360E5
P 10200 6100
F 0 "Q16" H 10450 6100 50  0000 L CNN
F 1 "IRLR024" H 10450 6000 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 10450 6025 50  0001 L CIN
F 3 "" H 10200 6100 50  0001 L CNN
	1    10200 6100
	1    0    0    -1  
$EndComp
Text Label 9600 6100 2    60   ~ 0
RO8
Text Label 10550 6000 0    60   ~ 0
J133_RO8
$Comp
L afterglow_nano_smd_stern_sam-rescue:PWR_FLAG-power-afterglow_nano_smd-rescue #FLG09
U 1 1 59E51401
P 2350 750
F 0 "#FLG09" H 2350 825 50  0001 C CNN
F 1 "PWR_FLAG" H 2350 900 50  0000 C CNN
F 2 "" H 2350 750 50  0001 C CNN
F 3 "" H 2350 750 50  0001 C CNN
	1    2350 750 
	1    0    0    -1  
$EndComp
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR010
U 1 1 59E53562
P 1650 1200
F 0 "#PWR010" H 1650 950 50  0001 C CNN
F 1 "GND" H 1650 1050 50  0000 C CNN
F 2 "" H 1650 1200 50  0001 C CNN
F 3 "" H 1650 1200 50  0001 C CNN
	1    1650 1200
	1    0    0    -1  
$EndComp
$Comp
L afterglow_nano_smd_stern_sam-rescue:PWR_FLAG-power-afterglow_nano_smd-rescue #FLG011
U 1 1 59E58D9D
P 700 800
F 0 "#FLG011" H 700 875 50  0001 C CNN
F 1 "PWR_FLAG" H 700 950 50  0000 C CNN
F 2 "" H 700 800 50  0001 C CNN
F 3 "" H 700 800 50  0001 C CNN
	1    700  800 
	1    0    0    -1  
$EndComp
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR012
U 1 1 59E58E59
P 700 800
F 0 "#PWR012" H 700 550 50  0001 C CNN
F 1 "GND" H 700 650 50  0000 C CNN
F 2 "" H 700 800 50  0001 C CNN
F 3 "" H 700 800 50  0001 C CNN
	1    700  800 
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74LS165 U1
U 1 1 59EB1A87
P 1900 4750
F 0 "U1" H 2050 4700 50  0000 C CNN
F 1 "74LS165" H 2050 4500 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-16_3.9x9.9mm_Pitch1.27mm" H 1900 4750 50  0001 C CNN
F 3 "" H 1900 4750 50  0001 C CNN
	1    1900 4750
	1    0    0    -1  
$EndComp
Text Label 1200 4250 0    60   ~ 0
CI1
Text Label 1200 4350 0    60   ~ 0
CI2
Text Label 1200 4450 0    60   ~ 0
CI3
Text Label 1200 4550 0    60   ~ 0
CI4
Text Label 1200 4650 0    60   ~ 0
CI5
Text Label 1200 4750 0    60   ~ 0
CI6
Text Label 1200 4850 0    60   ~ 0
CI7
Text Label 1200 4950 0    60   ~ 0
CI8
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR014
U 1 1 59EBBA9A
P 600 6600
F 0 "#PWR014" H 600 6350 50  0001 C CNN
F 1 "GND" H 550 6450 50  0000 C CNN
F 2 "" H 600 6600 50  0001 C CNN
F 3 "" H 600 6600 50  0001 C CNN
	1    600  6600
	1    0    0    -1  
$EndComp
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR015
U 1 1 59EBDD6B
P 1400 5450
F 0 "#PWR015" H 1400 5200 50  0001 C CNN
F 1 "GND" H 1400 5300 50  0000 C CNN
F 2 "" H 1400 5450 50  0001 C CNN
F 3 "" H 1400 5450 50  0001 C CNN
	1    1400 5450
	1    0    0    -1  
$EndComp
Text Label 2400 4150 0    60   ~ 0
IN_DATA
Text Label 1400 5350 2    60   ~ 0
IN_CLK
Text Label 1400 5150 2    60   ~ 0
IN_LOAD
NoConn ~ 2400 4250
$Comp
L 74xx:74HC595 U5
U 1 1 59EC0CD9
P 5750 3550
F 0 "U5" H 5900 4150 50  0000 C CNN
F 1 "74HC595" H 5750 2950 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-16_3.9x9.9mm_Pitch1.27mm" H 5750 3550 50  0001 C CNN
F 3 "" H 5750 3550 50  0001 C CNN
	1    5750 3550
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U6
U 1 1 59EC7FEA
P 5750 5300
F 0 "U6" H 5900 5900 50  0000 C CNN
F 1 "74HC595" H 5750 4700 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-16_3.9x9.9mm_Pitch1.27mm" H 5750 5300 50  0001 C CNN
F 3 "" H 5750 5300 50  0001 C CNN
	1    5750 5300
	1    0    0    -1  
$EndComp
Text Label 5350 5100 2    60   ~ 0
OUT_CLK
Text Label 5350 3350 2    60   ~ 0
OUT_CLK
Text Label 6150 5000 0    60   ~ 0
RO1
Text Label 6150 5100 0    60   ~ 0
RO2
Text Label 6150 5200 0    60   ~ 0
RO3
Text Label 6150 5300 0    60   ~ 0
RO4
Text Label 6150 5400 0    60   ~ 0
RO5
Text Label 6150 5500 0    60   ~ 0
RO6
Text Label 6150 5600 0    60   ~ 0
RO7
Text Label 5350 3150 2    60   ~ 0
OUT_DATA
Text Label 5350 3450 2    60   ~ 0
VCC
Text Label 5350 3650 2    60   ~ 0
OUT_LOAD
Text Label 5350 5400 2    60   ~ 0
OUT_LOAD
Text Label 5350 5200 2    60   ~ 0
VCC
$Comp
L Device:C_Small C4
U 1 1 59ECC15A
P 5900 2800
F 0 "C4" H 5910 2870 50  0000 L CNN
F 1 "0.33" H 5910 2720 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5900 2800 50  0001 C CNN
F 3 "" H 5900 2800 50  0001 C CNN
	1    5900 2800
	0    1    1    0   
$EndComp
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR018
U 1 1 59ECC295
P 6050 2800
F 0 "#PWR018" H 6050 2550 50  0001 C CNN
F 1 "GND" H 6050 2650 50  0000 C CNN
F 2 "" H 6050 2800 50  0001 C CNN
F 3 "" H 6050 2800 50  0001 C CNN
	1    6050 2800
	1    0    0    -1  
$EndComp
Text Label 3600 850  0    60   ~ 0
VCC
Text Label 950  2600 0    60   ~ 0
RI1
Text Label 950  2700 0    60   ~ 0
RI2
Text Label 950  3300 0    60   ~ 0
RI7
Text Label 950  2800 0    60   ~ 0
RI3
Text Label 950  2900 0    60   ~ 0
RI4
Text Label 950  3000 0    60   ~ 0
RI5
Text Label 950  3100 0    60   ~ 0
RI6
Text Label 950  3400 0    60   ~ 0
RI8
$Comp
L Device:D D1
U 1 1 5A3F72AD
P 2100 750
F 0 "D1" H 2100 850 50  0000 C CNN
F 1 "1N4004" H 2200 950 50  0000 C CNN
F 2 "Diodes_THT:D_5W_P10.16mm_Horizontal" H 2100 750 50  0001 C CNN
F 3 "" H 2100 750 50  0001 C CNN
	1    2100 750 
	-1   0    0    1   
$EndComp
$Comp
L Device:CP C1
U 1 1 5A3F768D
P 2350 950
F 0 "C1" H 2375 1050 50  0000 L CNN
F 1 "1000uF" H 2050 850 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P5.00mm" H 2388 800 50  0001 C CNN
F 3 "" H 2350 950 50  0001 C CNN
	1    2350 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:R R43
U 1 1 5A40325A
P 9850 850
F 0 "R43" V 9930 850 50  0000 C CNN
F 1 "220" V 9850 850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 850 50  0001 C CNN
F 3 "" H 9850 850 50  0001 C CNN
	1    9850 850 
	0    1    1    0   
$EndComp
$Comp
L Device:R R45
U 1 1 5A403819
P 9850 1600
F 0 "R45" V 9930 1600 50  0000 C CNN
F 1 "220" V 9850 1600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 1600 50  0001 C CNN
F 3 "" H 9850 1600 50  0001 C CNN
	1    9850 1600
	0    1    1    0   
$EndComp
$Comp
L Device:R R47
U 1 1 5A403969
P 9850 2350
F 0 "R47" V 9930 2350 50  0000 C CNN
F 1 "220" V 9850 2350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 2350 50  0001 C CNN
F 3 "" H 9850 2350 50  0001 C CNN
	1    9850 2350
	0    1    1    0   
$EndComp
$Comp
L Device:R R49
U 1 1 5A405403
P 9850 3100
F 0 "R49" V 9930 3100 50  0000 C CNN
F 1 "220" V 9850 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 3100 50  0001 C CNN
F 3 "" H 9850 3100 50  0001 C CNN
	1    9850 3100
	0    1    1    0   
$EndComp
$Comp
L Device:R R51
U 1 1 5A405617
P 9850 3850
F 0 "R51" V 9930 3850 50  0000 C CNN
F 1 "220" V 9850 3850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 3850 50  0001 C CNN
F 3 "" H 9850 3850 50  0001 C CNN
	1    9850 3850
	0    1    1    0   
$EndComp
$Comp
L Device:R R53
U 1 1 5A4058E0
P 9850 4600
F 0 "R53" V 9930 4600 50  0000 C CNN
F 1 "220" V 9850 4600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 4600 50  0001 C CNN
F 3 "" H 9850 4600 50  0001 C CNN
	1    9850 4600
	0    1    1    0   
$EndComp
$Comp
L Device:R R55
U 1 1 5A405A30
P 9850 5350
F 0 "R55" V 9930 5350 50  0000 C CNN
F 1 "220" V 9850 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 5350 50  0001 C CNN
F 3 "" H 9850 5350 50  0001 C CNN
	1    9850 5350
	0    1    1    0   
$EndComp
$Comp
L Device:R R57
U 1 1 5A405C77
P 9850 6100
F 0 "R57" V 9930 6100 50  0000 C CNN
F 1 "220" V 9850 6100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 6100 50  0001 C CNN
F 3 "" H 9850 6100 50  0001 C CNN
	1    9850 6100
	0    1    1    0   
$EndComp
Text Label 5500 2450 0    60   ~ 0
5V
$Comp
L Device:R R25
U 1 1 5A416845
P 5250 2450
F 0 "R25" V 5150 2450 50  0000 C CNN
F 1 "10k" V 5250 2450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5180 2450 50  0001 C CNN
F 3 "" H 5250 2450 50  0001 C CNN
	1    5250 2450
	0    1    1    0   
$EndComp
NoConn ~ 2450 1450
NoConn ~ 2450 3200
$Comp
L Device:R R1
U 1 1 5A4F652B
P 900 6600
F 0 "R1" V 950 6450 50  0000 C CNN
F 1 "1k" V 900 6600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 830 6600 50  0001 C CNN
F 3 "" H 900 6600 50  0001 C CNN
	1    900  6600
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5A4F6678
P 900 6700
F 0 "R2" V 950 6550 50  0000 C CNN
F 1 "1k" V 900 6700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 830 6700 50  0001 C CNN
F 3 "" H 900 6700 50  0001 C CNN
	1    900  6700
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5A4F6783
P 900 6800
F 0 "R3" V 950 6650 50  0000 C CNN
F 1 "1k" V 900 6800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 830 6800 50  0001 C CNN
F 3 "" H 900 6800 50  0001 C CNN
	1    900  6800
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5A4F68B6
P 900 6900
F 0 "R4" V 950 6750 50  0000 C CNN
F 1 "1k" V 900 6900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 830 6900 50  0001 C CNN
F 3 "" H 900 6900 50  0001 C CNN
	1    900  6900
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5A4F69CF
P 900 7000
F 0 "R5" V 950 6850 50  0000 C CNN
F 1 "1k" V 900 7000 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 830 7000 50  0001 C CNN
F 3 "" H 900 7000 50  0001 C CNN
	1    900  7000
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5A4F6AE6
P 900 7100
F 0 "R6" V 950 6950 50  0000 C CNN
F 1 "1k" V 900 7100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 830 7100 50  0001 C CNN
F 3 "" H 900 7100 50  0001 C CNN
	1    900  7100
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5A4F6BFB
P 900 7200
F 0 "R7" V 950 7050 50  0000 C CNN
F 1 "1k" V 900 7200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 830 7200 50  0001 C CNN
F 3 "" H 900 7200 50  0001 C CNN
	1    900  7200
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 5A4F6D0C
P 900 7300
F 0 "R8" V 950 7150 50  0000 C CNN
F 1 "1k" V 900 7300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 830 7300 50  0001 C CNN
F 3 "" H 900 7300 50  0001 C CNN
	1    900  7300
	0    1    1    0   
$EndComp
$Comp
L Device:R R35
U 1 1 5A541467
P 8050 650
F 0 "R35" V 8130 650 50  0000 C CNN
F 1 "2k" V 8050 650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 650 50  0001 C CNN
F 3 "" H 8050 650 50  0001 C CNN
	1    8050 650 
	0    1    1    0   
$EndComp
$Comp
L Device:R R36
U 1 1 5A541558
P 8050 1400
F 0 "R36" V 8130 1400 50  0000 C CNN
F 1 "2k" V 8050 1400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 1400 50  0001 C CNN
F 3 "" H 8050 1400 50  0001 C CNN
	1    8050 1400
	0    1    1    0   
$EndComp
$Comp
L Device:R R37
U 1 1 5A543C25
P 8050 2150
F 0 "R37" V 8130 2150 50  0000 C CNN
F 1 "2k" V 8050 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 2150 50  0001 C CNN
F 3 "" H 8050 2150 50  0001 C CNN
	1    8050 2150
	0    1    1    0   
$EndComp
$Comp
L Device:R R38
U 1 1 5A543D34
P 8050 2900
F 0 "R38" V 8130 2900 50  0000 C CNN
F 1 "2k" V 8050 2900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 2900 50  0001 C CNN
F 3 "" H 8050 2900 50  0001 C CNN
	1    8050 2900
	0    1    1    0   
$EndComp
$Comp
L Device:R R39
U 1 1 5A546EF3
P 8050 3650
F 0 "R39" V 8130 3650 50  0000 C CNN
F 1 "2k" V 8050 3650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 3650 50  0001 C CNN
F 3 "" H 8050 3650 50  0001 C CNN
	1    8050 3650
	0    1    1    0   
$EndComp
$Comp
L Device:R R40
U 1 1 5A546FF2
P 8050 4400
F 0 "R40" V 8130 4400 50  0000 C CNN
F 1 "2k" V 8050 4400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 4400 50  0001 C CNN
F 3 "" H 8050 4400 50  0001 C CNN
	1    8050 4400
	0    1    1    0   
$EndComp
$Comp
L Device:R R41
U 1 1 5A547103
P 8050 5150
F 0 "R41" V 8130 5150 50  0000 C CNN
F 1 "2k" V 8050 5150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 5150 50  0001 C CNN
F 3 "" H 8050 5150 50  0001 C CNN
	1    8050 5150
	0    1    1    0   
$EndComp
$Comp
L Device:R R42
U 1 1 5A54727F
P 8100 5900
F 0 "R42" V 8180 5900 50  0000 C CNN
F 1 "2k" V 8100 5900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8030 5900 50  0001 C CNN
F 3 "" H 8100 5900 50  0001 C CNN
	1    8100 5900
	0    1    1    0   
$EndComp
$Comp
L Device:R R44
U 1 1 5A5C9F75
P 9850 1050
F 0 "R44" V 9930 1050 50  0000 C CNN
F 1 "10k" V 9850 1050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 1050 50  0001 C CNN
F 3 "" H 9850 1050 50  0001 C CNN
	1    9850 1050
	0    1    1    0   
$EndComp
$Comp
L Device:R R46
U 1 1 5A5CA11C
P 9850 1800
F 0 "R46" V 9930 1800 50  0000 C CNN
F 1 "10k" V 9850 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 1800 50  0001 C CNN
F 3 "" H 9850 1800 50  0001 C CNN
	1    9850 1800
	0    1    1    0   
$EndComp
$Comp
L Device:R R48
U 1 1 5A5CA271
P 9850 2550
F 0 "R48" V 9930 2550 50  0000 C CNN
F 1 "10k" V 9850 2550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 2550 50  0001 C CNN
F 3 "" H 9850 2550 50  0001 C CNN
	1    9850 2550
	0    1    1    0   
$EndComp
$Comp
L Device:R R50
U 1 1 5A5CA3B4
P 9850 3300
F 0 "R50" V 9930 3300 50  0000 C CNN
F 1 "10k" V 9850 3300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 3300 50  0001 C CNN
F 3 "" H 9850 3300 50  0001 C CNN
	1    9850 3300
	0    1    1    0   
$EndComp
$Comp
L Device:R R52
U 1 1 5A5CA4DB
P 9850 4050
F 0 "R52" V 9930 4050 50  0000 C CNN
F 1 "10k" V 9850 4050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 4050 50  0001 C CNN
F 3 "" H 9850 4050 50  0001 C CNN
	1    9850 4050
	0    1    1    0   
$EndComp
$Comp
L Device:R R54
U 1 1 5A5CA602
P 9850 4800
F 0 "R54" V 9930 4800 50  0000 C CNN
F 1 "10k" V 9850 4800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 4800 50  0001 C CNN
F 3 "" H 9850 4800 50  0001 C CNN
	1    9850 4800
	0    1    1    0   
$EndComp
$Comp
L Device:R R56
U 1 1 5A5D036E
P 9850 5550
F 0 "R56" V 9930 5550 50  0000 C CNN
F 1 "10k" V 9850 5550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 5550 50  0001 C CNN
F 3 "" H 9850 5550 50  0001 C CNN
	1    9850 5550
	0    1    1    0   
$EndComp
$Comp
L Device:R R58
U 1 1 5A5D0499
P 9850 6300
F 0 "R58" V 9930 6300 50  0000 C CNN
F 1 "10k" V 9850 6300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9780 6300 50  0001 C CNN
F 3 "" H 9850 6300 50  0001 C CNN
	1    9850 6300
	0    1    1    0   
$EndComp
$Comp
L Device:R R59
U 1 1 5A5DC0C7
P 10050 650
F 0 "R59" V 10130 650 50  0000 C CNN
F 1 "10k" V 10050 650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9980 650 50  0001 C CNN
F 3 "" H 10050 650 50  0001 C CNN
	1    10050 650 
	0    1    1    0   
$EndComp
$Comp
L Device:R R60
U 1 1 5A5DC2BE
P 10050 1400
F 0 "R60" V 10130 1400 50  0000 C CNN
F 1 "10k" V 10050 1400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9980 1400 50  0001 C CNN
F 3 "" H 10050 1400 50  0001 C CNN
	1    10050 1400
	0    1    1    0   
$EndComp
$Comp
L Device:R R61
U 1 1 5A5DC3E5
P 10050 2150
F 0 "R61" V 10130 2150 50  0000 C CNN
F 1 "10k" V 10050 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9980 2150 50  0001 C CNN
F 3 "" H 10050 2150 50  0001 C CNN
	1    10050 2150
	0    1    1    0   
$EndComp
$Comp
L Device:R R62
U 1 1 5A5E1F26
P 10050 2900
F 0 "R62" V 10130 2900 50  0000 C CNN
F 1 "10k" V 10050 2900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9980 2900 50  0001 C CNN
F 3 "" H 10050 2900 50  0001 C CNN
	1    10050 2900
	0    1    1    0   
$EndComp
$Comp
L Device:R R63
U 1 1 5A5E207B
P 10050 3650
F 0 "R63" V 10130 3650 50  0000 C CNN
F 1 "10k" V 10050 3650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9980 3650 50  0001 C CNN
F 3 "" H 10050 3650 50  0001 C CNN
	1    10050 3650
	0    1    1    0   
$EndComp
$Comp
L Device:R R64
U 1 1 5A5E21C4
P 10050 4400
F 0 "R64" V 10130 4400 50  0000 C CNN
F 1 "10k" V 10050 4400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9980 4400 50  0001 C CNN
F 3 "" H 10050 4400 50  0001 C CNN
	1    10050 4400
	0    1    1    0   
$EndComp
$Comp
L Device:R R65
U 1 1 5A5E22FB
P 10050 5150
F 0 "R65" V 10130 5150 50  0000 C CNN
F 1 "10k" V 10050 5150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9980 5150 50  0001 C CNN
F 3 "" H 10050 5150 50  0001 C CNN
	1    10050 5150
	0    1    1    0   
$EndComp
$Comp
L Device:R R66
U 1 1 5A5E6730
P 10050 5900
F 0 "R66" V 10130 5900 50  0000 C CNN
F 1 "10k" V 10050 5900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9980 5900 50  0001 C CNN
F 3 "" H 10050 5900 50  0001 C CNN
	1    10050 5900
	0    1    1    0   
$EndComp
Text Label 9700 1400 0    60   ~ 0
18V
Text Label 9700 2150 0    60   ~ 0
18V
Text Label 9700 2900 0    60   ~ 0
18V
Text Label 9700 3650 0    60   ~ 0
18V
Text Label 9700 4400 0    60   ~ 0
18V
Text Label 9700 5150 0    60   ~ 0
18V
Text Label 9700 5900 0    60   ~ 0
18V
Wire Wire Line
	1400 850  1650 850 
Wire Wire Line
	1650 850  1650 1150
Connection ~ 1900 750 
Wire Wire Line
	3300 750  3350 750 
Wire Wire Line
	2600 750  2600 850 
Connection ~ 2600 750 
Wire Wire Line
	3350 750  3350 850 
Wire Wire Line
	3350 1150 3350 1050
Wire Wire Line
	1650 1150 2350 1150
Wire Wire Line
	2600 1050 2600 1150
Connection ~ 2600 1150
Wire Wire Line
	9700 850  9600 850 
Wire Wire Line
	10550 650  10550 750 
Wire Wire Line
	9700 1600 9600 1600
Wire Wire Line
	10550 1400 10550 1500
Wire Wire Line
	9700 2350 9600 2350
Wire Wire Line
	10550 2150 10550 2250
Wire Wire Line
	9700 3100 9600 3100
Wire Wire Line
	10550 2900 10550 3000
Wire Wire Line
	9700 3850 9600 3850
Wire Wire Line
	10550 3650 10550 3750
Wire Wire Line
	9700 4600 9600 4600
Wire Wire Line
	10550 4400 10550 4500
Wire Wire Line
	9700 5350 9600 5350
Wire Wire Line
	10550 5150 10550 5250
Wire Wire Line
	9700 6100 9600 6100
Wire Wire Line
	10550 5900 10550 6000
Connection ~ 1650 1150
Wire Wire Line
	1050 6600 1350 6600
Wire Wire Line
	1050 6700 1350 6700
Wire Wire Line
	1050 6800 1350 6800
Wire Wire Line
	1350 6900 1050 6900
Wire Wire Line
	1050 7000 1350 7000
Wire Wire Line
	1350 7100 1050 7100
Wire Wire Line
	1350 7200 1050 7200
Wire Wire Line
	1050 7300 1350 7300
Wire Wire Line
	600  6600 650  6600
Wire Wire Line
	1400 4150 1350 4150
Wire Wire Line
	1350 4150 1350 3950
Wire Wire Line
	5350 4900 5250 4900
Wire Wire Line
	5750 2800 5750 2950
Wire Wire Line
	5750 2800 5800 2800
Wire Wire Line
	6000 2800 6050 2800
Wire Wire Line
	3550 750  3550 850 
Wire Wire Line
	3550 850  3600 850 
Connection ~ 3550 750 
Wire Wire Line
	1550 750  1400 750 
Wire Wire Line
	2350 750  2350 800 
Connection ~ 2350 750 
Wire Wire Line
	2350 1100 2350 1150
Connection ~ 2350 1150
Wire Wire Line
	1850 750  1900 750 
Wire Wire Line
	5350 3750 5000 3750
Wire Wire Line
	4850 5500 5350 5500
Wire Wire Line
	5400 2450 5500 2450
Wire Wire Line
	2200 1400 2200 1350
Wire Wire Line
	2200 1350 2550 1350
Wire Wire Line
	2200 1600 2200 1550
Wire Wire Line
	2200 1550 2550 1550
Wire Wire Line
	2550 1650 2200 1650
Wire Wire Line
	2200 1650 2200 1700
Wire Wire Line
	2550 1750 2200 1750
Wire Wire Line
	2200 1750 2200 1800
Wire Wire Line
	2550 1850 2200 1850
Wire Wire Line
	2200 1850 2200 1900
Wire Wire Line
	2550 1950 2200 1950
Wire Wire Line
	2200 1950 2200 2000
Wire Wire Line
	2550 2150 2200 2150
Wire Wire Line
	2200 2150 2200 2200
Wire Wire Line
	2450 2650 2100 2650
Wire Wire Line
	2100 2650 2100 2700
Wire Wire Line
	2450 2750 2100 2750
Wire Wire Line
	2100 2750 2100 2800
Wire Wire Line
	2450 2950 2100 2950
Wire Wire Line
	2100 2950 2100 3000
Wire Wire Line
	2450 3050 2100 3050
Wire Wire Line
	2100 3050 2100 3100
Wire Wire Line
	2450 3250 2100 3250
Wire Wire Line
	2100 3250 2100 3300
Wire Wire Line
	2450 3350 2100 3350
Wire Wire Line
	2100 3350 2100 3400
Wire Wire Line
	2450 3450 2100 3450
Wire Wire Line
	2100 3450 2100 3500
Wire Wire Line
	2450 2700 2450 2650
Wire Wire Line
	2450 2800 2450 2750
Wire Wire Line
	2450 3000 2450 2950
Wire Wire Line
	2450 3100 2450 3050
Wire Wire Line
	2450 3300 2450 3250
Wire Wire Line
	2450 3400 2450 3350
Wire Wire Line
	2450 3500 2450 3450
Wire Wire Line
	2550 2200 2550 2150
Wire Wire Line
	2550 2000 2550 1950
Wire Wire Line
	2250 750  2350 750 
Wire Wire Line
	1900 650  1900 750 
Wire Wire Line
	750  6700 650  6700
Wire Wire Line
	650  6600 650  6700
Connection ~ 650  6600
Wire Wire Line
	650  6800 750  6800
Connection ~ 650  6700
Wire Wire Line
	650  6900 750  6900
Connection ~ 650  6800
Wire Wire Line
	650  7000 750  7000
Connection ~ 650  6900
Wire Wire Line
	650  7100 750  7100
Connection ~ 650  7000
Wire Wire Line
	650  7200 750  7200
Connection ~ 650  7100
Wire Wire Line
	650  7300 750  7300
Connection ~ 650  7200
Wire Wire Line
	10200 5900 10300 5900
Connection ~ 10300 5900
Wire Wire Line
	10200 5150 10300 5150
Connection ~ 10300 5150
Wire Wire Line
	10200 4400 10300 4400
Connection ~ 10300 4400
Wire Wire Line
	10200 3650 10300 3650
Connection ~ 10300 3650
Wire Wire Line
	10200 2900 10300 2900
Connection ~ 10300 2900
Wire Wire Line
	10200 2150 10300 2150
Connection ~ 10300 2150
Wire Wire Line
	10200 1400 10300 1400
Connection ~ 10300 1400
Wire Wire Line
	10200 650  10300 650 
Connection ~ 10300 650 
Wire Wire Line
	9700 650  9900 650 
Wire Wire Line
	9700 1400 9900 1400
Wire Wire Line
	9700 2150 9900 2150
Wire Wire Line
	10000 1050 10300 1050
Wire Wire Line
	9700 1050 9700 850 
Wire Wire Line
	9700 1800 9700 1600
Wire Wire Line
	10000 1800 10300 1800
Wire Wire Line
	9700 2550 9700 2350
Wire Wire Line
	10000 2550 10300 2550
Wire Wire Line
	9900 2900 9700 2900
Wire Wire Line
	9700 3100 9700 3300
Wire Wire Line
	10000 3300 10300 3300
Wire Wire Line
	9900 3650 9700 3650
Wire Wire Line
	9700 3850 9700 4050
Wire Wire Line
	10000 4050 10300 4050
Wire Wire Line
	9700 4400 9900 4400
Wire Wire Line
	9700 4600 9700 4800
Wire Wire Line
	10000 4800 10300 4800
Wire Wire Line
	10300 5550 10000 5550
Wire Wire Line
	9700 5150 9900 5150
Wire Wire Line
	9700 5900 9900 5900
Wire Wire Line
	9700 5550 9700 5350
Wire Wire Line
	9700 6300 9700 6100
Wire Wire Line
	10000 6300 10300 6300
$Comp
L afterglow_nano_smd_stern_sam-rescue:ULN2803A-Transistor_Array-afterglow_nano_smd-rescue U7
U 1 1 5A547A93
P 6800 3350
F 0 "U7" H 6800 3875 50  0000 C CNN
F 1 "ULN2803A" H 6800 3800 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-18W_7.5x11.6mm_Pitch1.27mm" H 6850 2700 50  0001 L CNN
F 3 "" H 6900 3250 50  0001 C CNN
	1    6800 3350
	1    0    0    -1  
$EndComp
Text Label 7200 3150 0    60   ~ 0
CO1
Text Label 7200 3250 0    60   ~ 0
CO2
Text Label 7200 3350 0    60   ~ 0
CO3
Text Label 7200 3450 0    60   ~ 0
CO4
Text Label 7200 3550 0    60   ~ 0
CO5
Text Label 7200 3650 0    60   ~ 0
CO6
Text Label 7200 3750 0    60   ~ 0
CO7
Text Label 7200 3850 0    60   ~ 0
CO8
Wire Wire Line
	6150 3150 6400 3150
Wire Wire Line
	6400 3250 6150 3250
Wire Wire Line
	6150 3350 6400 3350
Wire Wire Line
	6400 3450 6150 3450
Wire Wire Line
	6150 3550 6400 3550
Wire Wire Line
	6150 3650 6400 3650
Wire Wire Line
	6400 3750 6150 3750
Wire Wire Line
	6150 3850 6400 3850
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR019
U 1 1 5A54F7AA
P 6800 4050
F 0 "#PWR019" H 6800 3800 50  0001 C CNN
F 1 "GND" H 6800 3900 50  0000 C CNN
F 2 "" H 6800 4050 50  0001 C CNN
F 3 "" H 6800 4050 50  0001 C CNN
	1    6800 4050
	1    0    0    -1  
$EndComp
Text Label 7200 3050 0    60   ~ 0
18V
Text Label 8200 1900 0    60   ~ 0
J137_CO2
Wire Wire Line
	7800 850  7900 850 
Wire Wire Line
	7900 850  7900 650 
Wire Wire Line
	8200 1050 8200 1150
Wire Wire Line
	8200 1800 8200 1900
Wire Wire Line
	7900 1600 7800 1600
Wire Wire Line
	7900 1400 7900 1600
Wire Wire Line
	8200 2550 8200 2650
Wire Wire Line
	7900 2350 7800 2350
Wire Wire Line
	7900 2150 7900 2350
Wire Wire Line
	7900 3100 7800 3100
Wire Wire Line
	7900 2900 7900 3100
Wire Wire Line
	8200 3300 8200 3400
Wire Wire Line
	7900 3850 7800 3850
Wire Wire Line
	7900 3650 7900 3850
Wire Wire Line
	7800 4600 7900 4600
Wire Wire Line
	7900 4600 7900 4400
Wire Wire Line
	7800 5350 7900 5350
Wire Wire Line
	7900 5350 7900 5150
Wire Wire Line
	7950 6100 7850 6100
Wire Wire Line
	7950 5900 7950 6100
Wire Wire Line
	8200 4050 8200 4150
Wire Wire Line
	8200 4800 8200 4900
Wire Wire Line
	8200 5550 8200 5650
Wire Wire Line
	8250 6300 8250 6400
Wire Wire Line
	1900 750  1950 750 
Wire Wire Line
	2600 750  2700 750 
Wire Wire Line
	3350 750  3550 750 
Wire Wire Line
	1650 1150 1650 1200
Wire Wire Line
	3550 750  3600 750 
Wire Wire Line
	2350 750  2600 750 
Wire Wire Line
	2350 1150 2600 1150
Wire Wire Line
	650  6600 750  6600
Wire Wire Line
	650  6700 650  6800
Wire Wire Line
	650  6800 650  6900
Wire Wire Line
	650  6900 650  7000
Wire Wire Line
	650  7000 650  7100
Wire Wire Line
	650  7100 650  7200
Wire Wire Line
	650  7200 650  7300
Wire Wire Line
	10300 5900 10550 5900
Wire Wire Line
	10300 5150 10550 5150
Wire Wire Line
	10300 4400 10550 4400
Wire Wire Line
	10300 3650 10550 3650
Wire Wire Line
	10300 2900 10550 2900
Wire Wire Line
	10300 2150 10550 2150
Wire Wire Line
	10300 1400 10550 1400
Wire Wire Line
	10300 650  10550 650 
Wire Wire Line
	2600 1150 3000 1150
$Comp
L afterglow_nano_smd_stern_sam-rescue:Arduino_Nano_v3.x-MCU_Module-afterglow_nano_smd-rescue A1
U 1 1 5B9C7C51
P 4350 2350
F 0 "A1" H 3850 1250 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 3850 1150 50  0000 C CNN
F 2 "Modules:Arduino_Nano" H 4500 1400 50  0001 L CNN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 4350 1350 50  0001 C CNN
	1    4350 2350
	1    0    0    -1  
$EndComp
Connection ~ 3350 750 
Wire Wire Line
	4350 3350 4350 3450
Wire Wire Line
	4350 3450 4400 3450
Wire Wire Line
	4450 3450 4450 3350
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR0101
U 1 1 5BB48D39
P 4400 3450
F 0 "#PWR0101" H 4400 3200 50  0001 C CNN
F 1 "GND" H 4405 3277 50  0000 C CNN
F 2 "" H 4400 3450 50  0001 C CNN
F 3 "" H 4400 3450 50  0001 C CNN
	1    4400 3450
	1    0    0    -1  
$EndComp
Connection ~ 4400 3450
Wire Wire Line
	4400 3450 4450 3450
Text Label 3850 1950 2    60   ~ 0
IN_DATA
Text Label 3850 2050 2    60   ~ 0
IN_CLK
Text Label 3850 2150 2    60   ~ 0
IN_LOAD
Text Label 3850 2250 2    60   ~ 0
OUT_DATA
Text Label 3850 2350 2    60   ~ 0
OUT_CLK
Text Label 3850 2450 2    60   ~ 0
OUT_LOAD
NoConn ~ 4850 1750
NoConn ~ 4850 1850
NoConn ~ 4850 2550
NoConn ~ 4850 2650
NoConn ~ 4850 2750
NoConn ~ 4850 2850
NoConn ~ 4850 2950
NoConn ~ 4850 3050
NoConn ~ 3850 1750
NoConn ~ 3850 1850
NoConn ~ 4250 1350
NoConn ~ 4450 1350
Text Label 4550 1250 0    60   ~ 0
5V
Connection ~ 5750 2950
Wire Wire Line
	5750 2950 5750 3050
Wire Wire Line
	6250 4050 6150 4050
Wire Wire Line
	4550 1350 4550 1250
NoConn ~ 4850 2150
NoConn ~ 3850 2950
NoConn ~ 3850 3050
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR0102
U 1 1 5BA02CAB
P 5750 4250
F 0 "#PWR0102" H 5750 4000 50  0001 C CNN
F 1 "GND" H 5750 4100 50  0000 C CNN
F 2 "" H 5750 4250 50  0001 C CNN
F 3 "" H 5750 4250 50  0001 C CNN
	1    5750 4250
	1    0    0    -1  
$EndComp
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR0103
U 1 1 5BA02D84
P 5750 6000
F 0 "#PWR0103" H 5750 5750 50  0001 C CNN
F 1 "GND" H 5750 5850 50  0000 C CNN
F 2 "" H 5750 6000 50  0001 C CNN
F 3 "" H 5750 6000 50  0001 C CNN
	1    5750 6000
	1    0    0    -1  
$EndComp
Text Label 1950 3900 0    60   ~ 0
5V
Wire Wire Line
	5250 4900 5250 4550
Wire Wire Line
	5250 4550 6250 4550
Wire Wire Line
	6250 4550 6250 4050
Text Label 5750 4700 0    60   ~ 0
5V
Wire Wire Line
	5750 2800 5750 2700
Connection ~ 5750 2800
Text Label 5750 2700 0    60   ~ 0
5V
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR0105
U 1 1 5BAC07ED
P 1900 5750
F 0 "#PWR0105" H 1900 5500 50  0001 C CNN
F 1 "GND" H 1900 5600 50  0000 C CNN
F 2 "" H 1900 5750 50  0001 C CNN
F 3 "" H 1900 5750 50  0001 C CNN
	1    1900 5750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R26
U 1 1 5BACD2A1
P 5250 2000
F 0 "R26" V 5150 2000 50  0000 C CNN
F 1 "0.22" V 5250 2000 50  0000 C CNN
F 2 "Resistors_SMD:R_2512_HandSoldering" V 5180 2000 50  0001 C CNN
F 3 "" H 5250 2000 50  0001 C CNN
	1    5250 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	5050 2000 5100 2000
Wire Wire Line
	5400 2000 5500 2000
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR01
U 1 1 5BAFD7DF
P 5500 2000
F 0 "#PWR01" H 5500 1750 50  0001 C CNN
F 1 "GND" H 5505 1827 50  0000 C CNN
F 2 "" H 5500 2000 50  0001 C CNN
F 3 "" H 5500 2000 50  0001 C CNN
	1    5500 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 2000 5050 1850
Text Label 5050 1850 0    60   ~ 0
SUP_GND
Wire Wire Line
	10300 1050 10300 1150
Connection ~ 10300 1050
Wire Wire Line
	10300 1800 10300 1900
Connection ~ 10300 1800
Wire Wire Line
	10300 2550 10300 2650
Connection ~ 10300 2550
Wire Wire Line
	10300 3300 10300 3400
Connection ~ 10300 3300
Wire Wire Line
	10300 4050 10300 4150
Connection ~ 10300 4050
Wire Wire Line
	10300 4800 10300 4900
Connection ~ 10300 4800
Wire Wire Line
	10300 5550 10300 5650
Connection ~ 10300 5550
Wire Wire Line
	10300 6300 10300 6400
Connection ~ 10300 6300
Text Label 10300 1900 0    60   ~ 0
SUP_GND
Text Label 10300 2650 0    60   ~ 0
SUP_GND
Text Label 10300 3400 0    60   ~ 0
SUP_GND
Text Label 10300 4150 0    60   ~ 0
SUP_GND
Text Label 10300 4900 0    60   ~ 0
SUP_GND
Text Label 10300 5650 0    60   ~ 0
SUP_GND
Text Label 10300 6400 0    60   ~ 0
SUP_GND
Text Label 10300 1150 0    60   ~ 0
SUP_GND
Wire Wire Line
	3000 1050 3000 1150
Connection ~ 3000 1150
Wire Wire Line
	3000 1150 3350 1150
$Comp
L Device:R R27
U 1 1 5BA993C0
P 8050 1050
F 0 "R27" V 8130 1050 50  0000 C CNN
F 1 "10k" V 8050 1050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 1050 50  0001 C CNN
F 3 "" H 8050 1050 50  0001 C CNN
	1    8050 1050
	0    -1   -1   0   
$EndComp
Connection ~ 8200 1050
$Comp
L Device:R R28
U 1 1 5BA99EBD
P 8050 1800
F 0 "R28" V 8130 1800 50  0000 C CNN
F 1 "10k" V 8050 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 1800 50  0001 C CNN
F 3 "" H 8050 1800 50  0001 C CNN
	1    8050 1800
	0    -1   -1   0   
$EndComp
Connection ~ 8200 1800
$Comp
L Device:R R29
U 1 1 5BA99FCB
P 8050 2550
F 0 "R29" V 8130 2550 50  0000 C CNN
F 1 "10k" V 8050 2550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 2550 50  0001 C CNN
F 3 "" H 8050 2550 50  0001 C CNN
	1    8050 2550
	0    -1   -1   0   
$EndComp
Connection ~ 8200 2550
$Comp
L Device:R R30
U 1 1 5BA9A37A
P 8050 3300
F 0 "R30" V 8130 3300 50  0000 C CNN
F 1 "10k" V 8050 3300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 3300 50  0001 C CNN
F 3 "" H 8050 3300 50  0001 C CNN
	1    8050 3300
	0    -1   -1   0   
$EndComp
Connection ~ 8200 3300
$Comp
L Device:R R31
U 1 1 5BA9ADD5
P 8050 4050
F 0 "R31" V 8130 4050 50  0000 C CNN
F 1 "10k" V 8050 4050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 4050 50  0001 C CNN
F 3 "" H 8050 4050 50  0001 C CNN
	1    8050 4050
	0    -1   -1   0   
$EndComp
Connection ~ 8200 4050
$Comp
L Device:R R32
U 1 1 5BA9B262
P 8050 4800
F 0 "R32" V 8130 4800 50  0000 C CNN
F 1 "10k" V 8050 4800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 4800 50  0001 C CNN
F 3 "" H 8050 4800 50  0001 C CNN
	1    8050 4800
	0    -1   -1   0   
$EndComp
Connection ~ 8200 4800
$Comp
L Device:R R33
U 1 1 5BA9B374
P 8050 5550
F 0 "R33" V 8130 5550 50  0000 C CNN
F 1 "10k" V 8050 5550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 5550 50  0001 C CNN
F 3 "" H 8050 5550 50  0001 C CNN
	1    8050 5550
	0    -1   -1   0   
$EndComp
Connection ~ 8200 5550
$Comp
L Device:R R34
U 1 1 5BA9B947
P 8100 6300
F 0 "R34" V 8180 6300 50  0000 C CNN
F 1 "10k" V 8100 6300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8030 6300 50  0001 C CNN
F 3 "" H 8100 6300 50  0001 C CNN
	1    8100 6300
	0    -1   -1   0   
$EndComp
Connection ~ 8250 6300
$Comp
L Connector:TestPoint TP1
U 1 1 5BAEECC5
P 5300 900
F 0 "TP1" H 5358 1020 50  0000 L CNN
F 1 "TestPoint" H 5358 929 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 5500 900 50  0001 C CNN
F 3 "~" H 5500 900 50  0001 C CNN
	1    5300 900 
	1    0    0    -1  
$EndComp
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR013
U 1 1 5BAEF1E7
P 5300 900
F 0 "#PWR013" H 5300 650 50  0001 C CNN
F 1 "GND" H 5305 727 50  0000 C CNN
F 2 "" H 5300 900 50  0001 C CNN
F 3 "" H 5300 900 50  0001 C CNN
	1    5300 900 
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP2
U 1 1 5BAEF2C6
P 5950 900
F 0 "TP2" H 6008 1020 50  0000 L CNN
F 1 "TestPoint" H 6008 929 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 6150 900 50  0001 C CNN
F 3 "~" H 6150 900 50  0001 C CNN
	1    5950 900 
	1    0    0    -1  
$EndComp
Text Label 5950 900  2    60   ~ 0
5V
$Comp
L Connector:TestPoint TP3
U 1 1 5BB00A1A
P 6600 900
F 0 "TP3" H 6658 1020 50  0000 L CNN
F 1 "TestPoint" H 6658 929 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 6800 900 50  0001 C CNN
F 3 "~" H 6800 900 50  0001 C CNN
	1    6600 900 
	1    0    0    -1  
$EndComp
Text Label 6600 900  2    60   ~ 0
18V
Text Label 7900 1050 2    60   ~ 0
SUP_GND
Text Label 7900 1800 2    60   ~ 0
SUP_GND
Text Label 7900 2550 2    60   ~ 0
SUP_GND
Text Label 7900 3300 2    60   ~ 0
SUP_GND
Text Label 7900 4050 2    60   ~ 0
SUP_GND
Text Label 7900 4800 2    60   ~ 0
SUP_GND
Text Label 7900 5550 2    60   ~ 0
SUP_GND
Text Label 7950 6300 2    60   ~ 0
SUP_GND
$Comp
L afterglow_nano_smd_stern_sam-rescue:SW_DIP_x04-Switch-afterglow_nano_smd-rescue SW1
U 1 1 5BB93746
P 3550 2750
F 0 "SW1" H 3550 3217 50  0000 C CNN
F 1 "CFG" H 3550 2450 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_DIP_x4_W7.62mm_Slide" H 3550 2750 50  0001 C CNN
F 3 "" H 3550 2750 50  0001 C CNN
	1    3550 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2550 3100 2550
Wire Wire Line
	3100 2550 3100 2650
Wire Wire Line
	3250 2650 3100 2650
Connection ~ 3100 2650
Wire Wire Line
	3100 2650 3100 2750
Wire Wire Line
	3250 2750 3100 2750
Connection ~ 3100 2750
Wire Wire Line
	3100 2750 3100 2850
Wire Wire Line
	3250 2850 3100 2850
Connection ~ 3100 2850
Wire Wire Line
	3100 2850 3100 3100
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR0106
U 1 1 5BBDA8BB
P 3100 3100
F 0 "#PWR0106" H 3100 2850 50  0001 C CNN
F 1 "GND" H 3105 2927 50  0000 C CNN
F 2 "" H 3100 3100 50  0001 C CNN
F 3 "" H 3100 3100 50  0001 C CNN
	1    3100 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener D2
U 1 1 5BBE6766
P 1050 4250
F 0 "D2" V 1300 4200 50  0000 L CNN
F 1 "4.7V" V 1400 4150 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-323_HandSoldering" H 1050 4250 50  0001 C CNN
F 3 "~" H 1050 4250 50  0001 C CNN
	1    1050 4250
	-1   0    0    1   
$EndComp
Text Label 1350 6600 0    60   ~ 0
CI1
Text Label 1350 6700 0    60   ~ 0
CI2
Text Label 1350 6800 0    60   ~ 0
CI3
Text Label 1350 6900 0    60   ~ 0
CI4
Text Label 1350 7000 0    60   ~ 0
CI5
Text Label 1350 7100 0    60   ~ 0
CI6
Text Label 1350 7200 0    60   ~ 0
CI7
Text Label 1350 7300 0    60   ~ 0
CI8
Wire Wire Line
	1200 4250 1400 4250
Wire Wire Line
	1400 4350 1200 4350
Wire Wire Line
	1400 4450 1200 4450
Wire Wire Line
	1200 4550 1400 4550
Wire Wire Line
	1400 4650 1200 4650
Wire Wire Line
	1200 4750 1400 4750
Wire Wire Line
	1400 4850 1200 4850
Wire Wire Line
	1200 4950 1400 4950
$Comp
L Device:D_Zener D3
U 1 1 5BD03ED9
P 1050 4350
F 0 "D3" V 1300 4300 50  0000 L CNN
F 1 "4.7V" V 1400 4250 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-323_HandSoldering" H 1050 4350 50  0001 C CNN
F 3 "~" H 1050 4350 50  0001 C CNN
	1    1050 4350
	-1   0    0    1   
$EndComp
$Comp
L Device:D_Zener D4
U 1 1 5BD03FC9
P 1050 4450
F 0 "D4" V 1300 4400 50  0000 L CNN
F 1 "4.7V" V 1400 4350 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-323_HandSoldering" H 1050 4450 50  0001 C CNN
F 3 "~" H 1050 4450 50  0001 C CNN
	1    1050 4450
	-1   0    0    1   
$EndComp
$Comp
L Device:D_Zener D5
U 1 1 5BD040B7
P 1050 4550
F 0 "D5" V 1300 4500 50  0000 L CNN
F 1 "4.7V" V 1400 4450 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-323_HandSoldering" H 1050 4550 50  0001 C CNN
F 3 "~" H 1050 4550 50  0001 C CNN
	1    1050 4550
	-1   0    0    1   
$EndComp
$Comp
L Device:D_Zener D6
U 1 1 5BD041A7
P 1050 4650
F 0 "D6" V 1300 4600 50  0000 L CNN
F 1 "4.7V" V 1400 4550 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-323_HandSoldering" H 1050 4650 50  0001 C CNN
F 3 "~" H 1050 4650 50  0001 C CNN
	1    1050 4650
	-1   0    0    1   
$EndComp
$Comp
L Device:D_Zener D7
U 1 1 5BD0429B
P 1050 4750
F 0 "D7" V 1300 4700 50  0000 L CNN
F 1 "4.7V" V 1400 4650 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-323_HandSoldering" H 1050 4750 50  0001 C CNN
F 3 "~" H 1050 4750 50  0001 C CNN
	1    1050 4750
	-1   0    0    1   
$EndComp
$Comp
L Device:D_Zener D8
U 1 1 5BD04393
P 1050 4850
F 0 "D8" V 1300 4800 50  0000 L CNN
F 1 "4.7V" V 1400 4750 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-323_HandSoldering" H 1050 4850 50  0001 C CNN
F 3 "~" H 1050 4850 50  0001 C CNN
	1    1050 4850
	-1   0    0    1   
$EndComp
$Comp
L Device:D_Zener D9
U 1 1 5BD04491
P 1050 4950
F 0 "D9" V 1300 4900 50  0000 L CNN
F 1 "4.7V" V 1400 4850 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-323_HandSoldering" H 1050 4950 50  0001 C CNN
F 3 "~" H 1050 4950 50  0001 C CNN
	1    1050 4950
	-1   0    0    1   
$EndComp
Wire Wire Line
	900  4250 900  4350
Connection ~ 900  4350
Wire Wire Line
	900  4350 900  4450
Connection ~ 900  4450
Wire Wire Line
	900  4450 900  4550
Connection ~ 900  4550
Wire Wire Line
	900  4550 900  4650
Connection ~ 900  4650
Wire Wire Line
	900  4650 900  4750
Connection ~ 900  4750
Wire Wire Line
	900  4750 900  4850
Connection ~ 900  4850
Wire Wire Line
	900  4850 900  4950
Connection ~ 900  4950
Wire Wire Line
	900  4950 900  5050
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR02
U 1 1 5BD18694
P 900 5050
F 0 "#PWR02" H 900 4800 50  0001 C CNN
F 1 "GND" H 850 4900 50  0000 C CNN
F 2 "" H 900 5050 50  0001 C CNN
F 3 "" H 900 5050 50  0001 C CNN
	1    900  5050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x12_Male J2
U 1 1 59EB3EB6
P 750 3100
F 0 "J2" H 723 3124 50  0000 R CNN
F 1 "J12_IN_R" H 1450 3050 50  0000 R CNN
F 2 "Connectors_Molex:Molex_KK-6410-09_09x2.54mm_Straight" H 750 3100 50  0001 C CNN
F 3 "" H 750 3100 50  0001 C CNN
	1    750  3100
	1    0    0    -1  
$EndComp
NoConn ~ 950  3200
Text Label 950  3500 0    60   ~ 0
RI9
Text Label 950  3600 0    60   ~ 0
RI10
NoConn ~ 950  3700
Text Label 1700 3500 0    60   ~ 0
J12_RO9
NoConn ~ 2050 3150
Text Label 1650 3600 0    60   ~ 0
J12_RO10
NoConn ~ 2050 3700
Wire Wire Line
	2100 2900 2100 2850
Wire Wire Line
	2100 2850 2450 2850
Wire Wire Line
	2450 2850 2450 2900
Wire Wire Line
	2100 3600 2100 3550
Wire Wire Line
	2100 3550 2450 3550
Wire Wire Line
	2450 3550 2450 3600
NoConn ~ 2450 3700
NoConn ~ 1000 1500
$Comp
L Device:R R67
U 1 1 5F238BBD
P 1150 2300
F 0 "R67" V 1230 2300 50  0000 C CNN
F 1 "1k" V 1150 2300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1080 2300 50  0001 C CNN
F 3 "" H 1150 2300 50  0001 C CNN
	1    1150 2300
	0    1    1    0   
$EndComp
Text Label 1300 2300 0    60   ~ 0
CI9
NoConn ~ 2150 1450
Text Label 1800 2300 0    60   ~ 0
J13_CO9
Wire Wire Line
	2200 2100 2200 2050
Wire Wire Line
	2200 2050 2550 2050
Wire Wire Line
	2550 2050 2550 2100
Wire Wire Line
	2200 2300 2200 2250
Wire Wire Line
	2200 2250 2550 2250
Wire Wire Line
	2550 2250 2550 2300
$Comp
L Device:R R68
U 1 1 5F30A699
P 900 7400
F 0 "R68" V 950 7250 50  0000 C CNN
F 1 "1k" V 900 7400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 830 7400 50  0001 C CNN
F 3 "" H 900 7400 50  0001 C CNN
	1    900  7400
	0    1    1    0   
$EndComp
Wire Wire Line
	1050 7400 1350 7400
Wire Wire Line
	650  7400 750  7400
Wire Wire Line
	650  7300 650  7400
Text Label 1350 7400 0    60   ~ 0
CI9
Connection ~ 650  7300
Wire Wire Line
	1350 3950 4300 3950
Text Label 3700 4100 0    60   ~ 0
5V
Wire Wire Line
	3050 4900 3200 4900
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR0104
U 1 1 5B9D488A
P 3700 6000
F 0 "#PWR0104" H 3700 5750 50  0001 C CNN
F 1 "GND" H 3700 5850 50  0000 C CNN
F 2 "" H 3700 6000 50  0001 C CNN
F 3 "" H 3700 6000 50  0001 C CNN
	1    3700 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 5100 2650 5200
Wire Wire Line
	2650 5000 2650 5100
Wire Wire Line
	2650 4900 2650 5000
Wire Wire Line
	2650 4800 2650 4900
Wire Wire Line
	2650 4700 2650 4800
Wire Wire Line
	2650 4600 2650 4700
Connection ~ 2650 5100
Wire Wire Line
	2650 5200 2750 5200
Connection ~ 2650 5000
Wire Wire Line
	2650 5100 2750 5100
Connection ~ 2650 4900
Wire Wire Line
	2650 5000 2750 5000
Connection ~ 2650 4800
Wire Wire Line
	2650 4900 2750 4900
Connection ~ 2650 4700
Wire Wire Line
	2650 4800 2750 4800
Wire Wire Line
	2650 4700 2750 4700
Wire Wire Line
	2750 4600 2650 4600
Wire Wire Line
	3350 5200 3200 5200
Wire Wire Line
	3050 5100 3200 5100
Wire Wire Line
	3350 5000 3200 5000
Wire Wire Line
	3350 4800 3200 4800
Wire Wire Line
	3050 4700 3200 4700
Wire Wire Line
	3350 4600 3200 4600
$Comp
L Device:R R16
U 1 1 5A4F41D4
P 2900 5200
F 0 "R16" V 2950 5000 50  0000 C CNN
F 1 "1k" V 2900 5200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2830 5200 50  0001 C CNN
F 3 "" H 2900 5200 50  0001 C CNN
	1    2900 5200
	0    1    1    0   
$EndComp
$Comp
L Device:R R15
U 1 1 5A4F40D7
P 2900 5100
F 0 "R15" V 2950 4900 50  0000 C CNN
F 1 "1k" V 2900 5100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2830 5100 50  0001 C CNN
F 3 "" H 2900 5100 50  0001 C CNN
	1    2900 5100
	0    1    1    0   
$EndComp
$Comp
L Device:R R14
U 1 1 5A4F3FA0
P 2900 5000
F 0 "R14" V 2950 4800 50  0000 C CNN
F 1 "1k" V 2900 5000 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2830 5000 50  0001 C CNN
F 3 "" H 2900 5000 50  0001 C CNN
	1    2900 5000
	0    1    1    0   
$EndComp
$Comp
L Device:R R13
U 1 1 5A4F3EAF
P 2900 4900
F 0 "R13" V 2950 4700 50  0000 C CNN
F 1 "1k" V 2900 4900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2830 4900 50  0001 C CNN
F 3 "" H 2900 4900 50  0001 C CNN
	1    2900 4900
	0    1    1    0   
$EndComp
$Comp
L Device:R R12
U 1 1 5A4F3D98
P 2900 4800
F 0 "R12" V 2950 4600 50  0000 C CNN
F 1 "1k" V 2900 4800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2830 4800 50  0001 C CNN
F 3 "" H 2900 4800 50  0001 C CNN
	1    2900 4800
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 5A4F3CC5
P 2900 4700
F 0 "R11" V 2950 4500 50  0000 C CNN
F 1 "1k" V 2900 4700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2830 4700 50  0001 C CNN
F 3 "" H 2900 4700 50  0001 C CNN
	1    2900 4700
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 5A4F3AD2
P 2900 4600
F 0 "R10" V 2950 4400 50  0000 C CNN
F 1 "1k" V 2900 4600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2830 4600 50  0001 C CNN
F 3 "" H 2900 4600 50  0001 C CNN
	1    2900 4600
	0    1    1    0   
$EndComp
Text Label 3200 5400 2    60   ~ 0
IN_LOAD
Text Label 3200 5600 2    60   ~ 0
IN_CLK
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR016
U 1 1 59EBDE39
P 3200 5700
F 0 "#PWR016" H 3200 5450 50  0001 C CNN
F 1 "GND" H 3200 5550 50  0000 C CNN
F 2 "" H 3200 5700 50  0001 C CNN
F 3 "" H 3200 5700 50  0001 C CNN
	1    3200 5700
	1    0    0    -1  
$EndComp
Text Label 3050 5200 0    60   ~ 0
RI7
Text Label 3050 5100 0    60   ~ 0
RI6
Text Label 3050 5000 0    60   ~ 0
RI5
Text Label 3050 4900 0    60   ~ 0
RI4
Text Label 3050 4800 0    60   ~ 0
RI3
Text Label 3050 4700 0    60   ~ 0
RI2
Text Label 3050 4600 0    60   ~ 0
RI1
Text Label 3050 4500 0    60   ~ 0
CI9
Text Label 2650 4600 2    60   ~ 0
5V
$Comp
L 74xx:74LS165 U2
U 1 1 59EB4783
P 3700 5000
F 0 "U2" H 3850 4950 50  0000 C CNN
F 1 "74LS165" H 3850 4750 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-16_3.9x9.9mm_Pitch1.27mm" H 3700 5000 50  0001 C CNN
F 3 "" H 3700 5000 50  0001 C CNN
	1    3700 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 5500 4850 3750
Wire Wire Line
	4200 4400 4300 4400
Wire Wire Line
	4300 4400 4300 3950
NoConn ~ 4200 4500
Text Label 2800 5750 0    60   ~ 0
5V
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR?
U 1 1 5F58A29A
P 2800 7650
F 0 "#PWR?" H 2800 7400 50  0001 C CNN
F 1 "GND" H 3000 7650 50  0000 C CNN
F 2 "" H 2800 7650 50  0001 C CNN
F 3 "" H 2800 7650 50  0001 C CNN
	1    2800 7650
	1    0    0    -1  
$EndComp
Text Label 2300 7050 2    60   ~ 0
IN_LOAD
Text Label 2300 7250 2    60   ~ 0
IN_CLK
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR?
U 1 1 5F58A2D9
P 2300 7350
F 0 "#PWR?" H 2300 7100 50  0001 C CNN
F 1 "GND" H 2300 7200 50  0000 C CNN
F 2 "" H 2300 7350 50  0001 C CNN
F 3 "" H 2300 7350 50  0001 C CNN
	1    2300 7350
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74LS165 U8
U 1 1 5F58A2E7
P 2800 6650
F 0 "U8" H 2950 6600 50  0000 C CNN
F 1 "74LS165" H 2950 6400 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-16_3.9x9.9mm_Pitch1.27mm" H 2800 6650 50  0001 C CNN
F 3 "" H 2800 6650 50  0001 C CNN
	1    2800 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 4400 2250 4400
Wire Wire Line
	2250 4400 2250 5900
Wire Wire Line
	2250 5900 3300 5900
Wire Wire Line
	3300 5900 3300 6050
NoConn ~ 3300 6150
$Comp
L Device:D_Zener D10
U 1 1 5F6D602E
P 2700 4500
F 0 "D10" V 2950 4450 50  0000 L CNN
F 1 "4.7V" V 3050 4400 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-323_HandSoldering" H 2700 4500 50  0001 C CNN
F 3 "~" H 2700 4500 50  0001 C CNN
	1    2700 4500
	-1   0    0    1   
$EndComp
Wire Wire Line
	2850 4500 3050 4500
Connection ~ 3200 4600
Wire Wire Line
	3200 4600 3050 4600
Connection ~ 3200 4700
Wire Wire Line
	3200 4700 3350 4700
Connection ~ 3200 4800
Wire Wire Line
	3200 4800 3050 4800
Connection ~ 3200 4900
Wire Wire Line
	3200 4900 3350 4900
Connection ~ 3200 5000
Wire Wire Line
	3200 5000 3050 5000
Connection ~ 3200 5100
Wire Wire Line
	3200 5100 3350 5100
Connection ~ 3200 5200
Wire Wire Line
	3200 5200 3050 5200
Wire Wire Line
	2550 4500 2400 4500
Wire Wire Line
	2400 4500 2400 4800
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR?
U 1 1 5F7854D2
P 2400 4800
F 0 "#PWR?" H 2400 4550 50  0001 C CNN
F 1 "GND" H 2350 4650 50  0000 C CNN
F 2 "" H 2400 4800 50  0001 C CNN
F 3 "" H 2400 4800 50  0001 C CNN
	1    2400 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 6150 1750 6250
Wire Wire Line
	1750 6050 1750 6150
Connection ~ 1750 6150
Wire Wire Line
	1750 6250 1850 6250
Wire Wire Line
	1750 6150 1850 6150
Wire Wire Line
	1750 6050 1850 6050
Wire Wire Line
	2150 6150 2300 6150
$Comp
L Device:R R71
U 1 1 5F79F439
P 2000 6250
F 0 "R71" V 2050 6050 50  0000 C CNN
F 1 "1k" V 2000 6250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1930 6250 50  0001 C CNN
F 3 "" H 2000 6250 50  0001 C CNN
	1    2000 6250
	0    1    1    0   
$EndComp
$Comp
L Device:R R70
U 1 1 5F79F43F
P 2000 6150
F 0 "R70" V 2050 5950 50  0000 C CNN
F 1 "1k" V 2000 6150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1930 6150 50  0001 C CNN
F 3 "" H 2000 6150 50  0001 C CNN
	1    2000 6150
	0    1    1    0   
$EndComp
$Comp
L Device:R R69
U 1 1 5F79F445
P 2000 6050
F 0 "R69" V 2050 5850 50  0000 C CNN
F 1 "1k" V 2000 6050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1930 6050 50  0001 C CNN
F 3 "" H 2000 6050 50  0001 C CNN
	1    2000 6050
	0    1    1    0   
$EndComp
Text Label 2150 6250 0    60   ~ 0
RI10
Text Label 2150 6150 0    60   ~ 0
RI9
Text Label 2150 6050 0    60   ~ 0
RI8
Wire Wire Line
	2300 6050 2150 6050
Wire Wire Line
	2300 6250 2150 6250
NoConn ~ 2300 6350
NoConn ~ 2300 6450
NoConn ~ 2300 6550
NoConn ~ 2300 6650
NoConn ~ 2300 6750
NoConn ~ 2300 6850
Wire Wire Line
	4850 2350 5050 2350
Wire Wire Line
	5050 2350 5050 2000
Connection ~ 5050 2000
Wire Wire Line
	4850 2450 5000 2450
Wire Wire Line
	5000 2450 5000 3750
Connection ~ 5000 2450
Wire Wire Line
	5000 2450 5100 2450
Connection ~ 5000 3750
Wire Wire Line
	5000 3750 4850 3750
$Comp
L 74xx:74HC595 U8
U 1 1 5F9D24A6
P 5050 6750
F 0 "U8" H 5200 7350 50  0000 C CNN
F 1 "74HC595" H 5050 6150 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-16_3.9x9.9mm_Pitch1.27mm" H 5050 6750 50  0001 C CNN
F 3 "" H 5050 6750 50  0001 C CNN
	1    5050 6750
	1    0    0    -1  
$EndComp
Text Label 4650 6550 2    60   ~ 0
OUT_CLK
Text Label 5450 6350 0    60   ~ 0
RO8
Text Label 5450 6450 0    60   ~ 0
RO9
Text Label 5450 6550 0    60   ~ 0
R10
NoConn ~ 5450 7250
Text Label 4650 6850 2    60   ~ 0
OUT_LOAD
Text Label 4650 6650 2    60   ~ 0
VCC
Wire Wire Line
	4650 6350 4550 6350
Wire Wire Line
	4150 6950 4650 6950
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR?
U 1 1 5F9D24BA
P 5050 7450
F 0 "#PWR?" H 5050 7200 50  0001 C CNN
F 1 "GND" H 5050 7300 50  0000 C CNN
F 2 "" H 5050 7450 50  0001 C CNN
F 3 "" H 5050 7450 50  0001 C CNN
	1    5050 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 6350 4550 6000
Text Label 5050 6150 0    60   ~ 0
5V
Wire Wire Line
	4150 6950 4150 5500
Wire Wire Line
	4150 5500 4850 5500
Connection ~ 4850 5500
Wire Wire Line
	6150 5800 6450 5800
Wire Wire Line
	6450 5800 6450 6150
Wire Wire Line
	6450 6150 5300 6150
Wire Wire Line
	5300 6150 5300 6000
Wire Wire Line
	5300 6000 4550 6000
Text Label 6150 4900 0    60   ~ 0
CO9
$Comp
L afterglow_nano_smd_stern_sam-rescue:ULN2803A-Transistor_Array-afterglow_nano_smd-rescue U9
U 1 1 5FAAD8EF
P 6850 5050
F 0 "U9" H 6850 5575 50  0000 C CNN
F 1 "ULN2803A" H 6850 5500 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-18W_7.5x11.6mm_Pitch1.27mm" H 6900 4400 50  0001 L CNN
F 3 "" H 6950 4950 50  0001 C CNN
	1    6850 5050
	1    0    0    -1  
$EndComp
Text Label 7250 4850 0    60   ~ 0
CO9
$Comp
L afterglow_nano_smd_stern_sam-rescue:GND-power-afterglow_nano_smd-rescue #PWR?
U 1 1 5FAAD8FD
P 6850 5750
F 0 "#PWR?" H 6850 5500 50  0001 C CNN
F 1 "GND" H 6850 5600 50  0000 C CNN
F 2 "" H 6850 5750 50  0001 C CNN
F 3 "" H 6850 5750 50  0001 C CNN
	1    6850 5750
	1    0    0    -1  
$EndComp
Text Label 7250 4750 0    60   ~ 0
18V
NoConn ~ 7250 4950
NoConn ~ 7250 5050
NoConn ~ 7250 5150
NoConn ~ 7250 5250
NoConn ~ 7250 5350
NoConn ~ 7250 5450
NoConn ~ 7250 5550
NoConn ~ 6450 5550
NoConn ~ 6450 5450
NoConn ~ 6450 5350
NoConn ~ 6450 5250
NoConn ~ 6450 5150
NoConn ~ 6450 5050
NoConn ~ 6450 4950
Wire Wire Line
	6150 4900 6350 4900
Wire Wire Line
	6350 4900 6350 4850
Wire Wire Line
	6350 4850 6450 4850
NoConn ~ 5450 6650
NoConn ~ 5450 6750
NoConn ~ 5450 6850
NoConn ~ 5450 6950
NoConn ~ 5450 7000
$EndSCHEMATC
