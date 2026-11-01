
Note! GPS L5 signals are pre-operational
and not used by default. Refer to the
Overview section for more information.
NEO-F10N
Standard precision GNSS module
Professional grade
Data sheet
Abstract
This Data sheet describes the NEO-F10N module, an L1/L5 dual-band GNSS receiver for meter-level
accuracy in urban environment.
UBX-23002117 - R05
 C1-Public www.u-blox.com
NEO-F10N - Data sheet
Document information
Title NEO-F10N
Subtitle Standard precision GNSS module
Document type Data sheet
Document number UBX-23002117
Revision and date R05 09-Oct-2025
Disclosure restriction C1-Public
Product status Corresponding content status
Functional sample Draft For functional testing. Revised and supplementary data will be published
later.
In development /
prototype
Objective speciﬁcation Target values. Revised and supplementary data will be published later.
Engineering sample Advance information Data based on early testing. Revised and supplementary data will be
published later.
Initial production Early production information Data from product veriﬁcation. Revised and supplementary data may be
published later.
Mass production /
End of life
Production information Document contains the ﬁnal product speciﬁcation.
This document applies to the following products:
Product name Type number FW version IN/PCN reference Product status
NEO-F10N NEO-F10N-00B-00 EXT SPG 6.00 UBXDOC-304424225-18225
UBXDOC-963802114-12646
Mass production
NEO-F10N NEO-F10N-00B-20 EXT SPG 6.00 UBXDOC-304424225-18225
UBXDOC-963802114-12646
Mass production
u-blox or third parties may hold intellectual property rights in the products, names, logos and designs included in this
document. Copying, reproduction, or modiﬁcation of this document or any part thereof is only permitted with the express
written permission of u-blox. Disclosure to third parties is permitted for clearly public documents only.
The information contained herein is provided "as is" and u-blox assumes no liability for its use. No warranty, either express
or implied, is given, including but not limited to, with respect to the accuracy, correctness, reliability and ﬁtness for a
particular purpose of the information. This document may be revised by u-blox at any time without notice. For the most recent
documents and product statuses, visit www.u-blox.com.
Copyright © 2025, u-blox AG.
UBX-23002117 - R05
 
Document information Page 2 of 22
C1-Public
NEO-F10N - Data sheet
Contents
Document information............................................................................................................ 2
Contents.......................................................................................................................................3
1 Functional description......................................................................................................... 4
1.1 Overview.................................................................................................................................................... 4
1.2 Performance............................................................................................................................................. 4
1.3 Supported GNSS constellations.......................................................................................................... 5
1.4 Broadcast navigation data and satellite signal measurements................................................... 6
1.5 Supported protocols............................................................................................................................... 6
1.6 Firmware features...................................................................................................................................6
2 Block diagram......................................................................................................................... 8
3 Pin deﬁnition........................................................................................................................... 9
3.1 Pin assignment........................................................................................................................................9
3.2 Pin state..................................................................................................................................................10
4 Electrical speciﬁcations.................................................................................................... 11
4.1 Absolute maximum ratings................................................................................................................ 11
4.2 Operating conditions............................................................................................................................11
4.3 Indicative power requirements...........................................................................................................12
5 Communication interfaces...............................................................................................14
5.1 UART........................................................................................................................................................14
5.2 Default interface settings...................................................................................................................14
6 Mechanical speciﬁcations................................................................................................ 15
7 Qualiﬁcations and approvals........................................................................................... 16
8 Product handling................................................................................................................. 17
8.1 Packaging............................................................................................................................................... 17
8.1.1 Reels................................................................................................................................................17
8.1.2 Tapes...............................................................................................................................................17
8.2 Soldering................................................................................................................................................. 17
9 Product marking and ordering information................................................................ 18
9.1 Product marking....................................................................................................................................18
9.2 Product identiﬁers................................................................................................................................ 18
9.3 Ordering codes...................................................................................................................................... 19
Related documents................................................................................................................ 20
Revision history....................................................................................................................... 21
Contact.......................................................................................................................................22
UBX-23002117 - R05
 
Contents Page 3 of 22
C1-Public
NEO-F10N - Data sheet
1 Functional description
1.1 Overview
The NEO-F10N is built on the u-blox F10 dual-band GNSS technology using the L1 and L5 band
signals. The proprietary dual-band multipath mitigation technology enables the u-blox F10 to use
the best signals from the L1 and L5 bands providing a solid meter-level position accuracy in urban
environment.
NEO-F10N-00B-20 supports GNSS broadcast data and raw carrier-phase measurements.
At the time of writing, the GPS L5 signals remain pre-operational and are set as unhealthy
until suﬃcient monitoring capability is established. This is an operational issue concerning the
satellites / space segment and not a limitation of u-blox products.
Due to the pre-operational status, the GPS L5 signals are not used for the navigation solution
by default. However, it is possible to evaluate the GPS L5 signals before they become fully
operational by changing the receiver conﬁguration to override the GPS L5 health status. Refer
to the Integration manual [1] for details.
1.2 Performance
Parameter Speciﬁcation Value
Receiver type u-blox F10 dual-band receiver
RMS 30 nsAccuracy of time pulse signal
99% 60 ns
Frequency of time pulse signal Default 1PPS (0.25 Hz to 10 MHz conﬁgurable)
Dynamics ≤ 4 g
Altitude 80,000 m
Operational limits 1
Velocity 500 m/s
Velocity accuracy  2 0.05 m/s
Dynamic heading accuracy 2 0.3 deg
Table 1: NEO-F10N speciﬁcations
Table 2 shows typical performance values in multi-GNSS conﬁgurations3. SBAS is enabled in all
measurements.
Parameter GPS+GAL
+BDS (Default) GPS+BDS GPS+GAL GPS+NavIC Unit
Max navigation update rate  4 10 10 10 10 Hz
Position accuracy (CEP)  5 1 1 1 1 m
Cold start 28 28 27 27 sTime To First Fix
(TTFF)  6
Hot start 2 2 2 2 s
1 Assuming Airborne 4 g platform.
2 50% at 30 m/s for dynamic operation.
3 The GPS L5 signal is included in the performance ﬁgures. Note that this signal is not fully operational yet and using it
requires a conﬁguration change.
4 Minimum 98% ﬁx rate under typical conditions.
5 CEP, 50%, 24 hours static, –130 dBm, > 6 SVs for each GNSS system.
6 Commanded starts. All satellites signals at -130 dBm. Measured at room temperature.
UBX-23002117 - R05
 
1 Functional description Page 4 of 22
C1-Public
NEO-F10N - Data sheet
Parameter GPS+GAL
+BDS (Default) GPS+BDS GPS+GAL GPS+NavIC Unit
AssistNow Online  7 2 2 2 2 s
AssistNow Oﬄine  8 3 3 3 3 s
AssistNow Autonomous  9 4 4 4 4 s
Tracking and navigation –167 –167 –167 –167 dBm
Reacquisition –159 –159 –159 –159 dBm
Cold Start –148 –148 –148 –148 dBm
Sensitivity  10
Hot start –159 –159 –159 –159 dBm
Table 2: NEO-F10N typical performance in multi-GNSS conﬁgurations
Table 3 shows typical performance values in single-GNSS conﬁgurations3. SBAS is enabled in all
measurements.
Parameter GPS BDS Unit
Max navigation update rate 4 20 20 Hz
Position accuracy (CEP) 5 1.5 1 m
Cold start 27 42 s
Hot start 2 2 s
Time To First Fix
(TTFF) 6
AssistNow Online 7 2 N/A s
Tracking and navigation –167 –163 dBm
Reacquisition –159 –156 dBm
Cold Start –148 –137 dBm
Sensitivity 10
Hot start –159 –157 dBm
Table 3: NEO-F10N typical performance in single-GNSS conﬁgurations
1.3 Supported GNSS constellations
NEO-F10N is a concurrent GNSS receiver that can receive and track multiple GNSS systems. The
dual-band RF front-end architecture enables concurrent reception of multiple dual frequency GNSS
constellations. To achieve lower power consumption, the receiver can be conﬁgured for a subset of
GNSS constellations.
The default conﬁguration on NEO-F10N is concurrent reception of GPS, Galileo and BeiDou with
SBAS (EGNOS, GAGAN, MSAS, and WAAS) enabled.
Single-band operation is not supported. Both L1 and L5 signals must be enabled or disabled for
GPS, Galileo, BeiDou, and QZSS.
The following GNSS and their signals are supported:
System Signals
GPS / QZSS L1C/A (1575.42 MHz), L5 (1176.450 MHz)
Galileo E1-B/C (1575.42 MHz), E5a (1176.450 MHz)
BeiDou B1C (1575.42 MHz) , B2a (1176.450 MHz)
7 Depends on the speed and latency of the aiding data connection, commanded starts.
8 Using seven days old AssistNow Oﬄine data. External memory may be required.
9 Using two days old orbital predicted data. External memory may be required.
10 Demonstrated with a good external LNA. Measured at room temperature.
UBX-23002117 - R05
 
1 Functional description Page 5 of 22
C1-Public
NEO-F10N - Data sheet
System Signals
NavIC SPS-L5 (1176.450 MHz)
Table 4: Supported GNSS and signals on NEO-F10N
The following GNSS assistance services are supported:
Service Support
AssistNow Online GPS L1C/A, Galileo E1, QZSS L1C/A
AssistNow Oﬄine GPS L1C/A, Galileo E1, QZSS L1C/A
AssistNow Autonomous GPS L1C/A, Galileo E1
Table 5: Supported Assisted GNSS (A-GNSS) services
The following augmentation systems are supported:
System Support
SBAS11 EGNOS, GAGAN, MSAS, WAAS, BDSBAS, KASS and SouthPAN
QZSS L1S (SLAS), L1Sb (SBAS)
Table 6: Supported augmentation systems
The QZSS augmentation system can be enabled only if GPS operation is also enabled.
1.4 Broadcast navigation data and satellite signal
measurements
The GNSS broadcast data is available in the UBX-RXM-SFRBX message for all supported GNSS
signals as well as the QZSS and SBAS augmentation services. The satellite signal information is
compatible with the Radio Resource LCS Protocol (RRLP) [4].
The full raw data including code phase, doppler, pseudorange, carrier phase, and measurement
quality information is available in the UBX-RXM-RAWX message for all supported GNSS signals. The
UBX-RXM-RAWX message follows the conventions of a multi-GNSS RINEX 3 observation ﬁle.
This applies only to the product NEO-F10N-00B-20.
The raw measurement data is available after the receiver has established data bit
synchronization and time-of-week.
1.5 Supported protocols
NEO-F10N supports the following interface protocols:
Protocol Type
UBX Input/output, binary, u-blox proprietary
NMEA versions 2.1, 2.3, 4.0, 4.10 and 4.11 (default) Input/output, ASCII
Table 7: Supported protocols
1.6 Firmware features
Feature Description
Assisted GNSS AssistNow Online, AssistNow Oﬄine and AssistNow Autonomous
11 Ionospheric correction service is the only SBAS service supported by NEO-F10N
UBX-23002117 - R05
 
1 Functional description Page 6 of 22
C1-Public
NEO-F10N - Data sheet
Feature Description
Backup modes Hardware backup mode and software standby mode
Protection level Real-time position accuracy estimate with 95% conﬁdence level12
Galileo return link messages Galileo search and rescue (SAR) return link messages (RLM) via Galileo satellite signal
Carrier phase output13 GNSS broadcast data and raw carrier-phase measurements
Odometer Measure traveled distance with support for diﬀerent user proﬁles
Table 8: Firmware features
Feature Description
Anti-jamming RF interference and jamming detection and reporting
Anti-spooﬁng Spooﬁng detection and reporting
Conﬁguration lockdown Receiver conﬁguration can be locked by command
Message integrity All messages can be cryptographically signed
Secure boot Only signed ﬁrmware images are executed
Table 9: Security features
12 Veriﬁed for automotive environment only.
13 Applies only to the product NEO-F10N-00B-20.
UBX-23002117 - R05
 
1 Functional description Page 7 of 22
C1-Public
NEO-F10N - Data sheet
2 Block diagram
Figure 1: NEO-F10N block diagram
UBX-23002117 - R05
 
2 Block diagram Page 8 of 22
C1-Public
NEO-F10N - Data sheet
3 Pin deﬁnition
3.1 Pin assignment
Figure 2: NEO-F10N pin assignment
Pin no. Name I/O Description
1 SAFEBOOT_N I Safeboot mode. If not used, leave open.14
2 Reserved - Not connected
3 TIMEPULSE O Time pulse signal (shared with SAFEBOOT_N pin)14
4 EXTINT I External interrupt
5 Reserved - Not connected
6 Reserved - Not connected
7 Reserved - Not connected
8 RESET_N I RESET (active low)
9 VCC_RF O Output voltage RF section
10 GND - Ground
11 RF_IN I GNSS signal input
12 GND - Ground
13 GND - Ground
14 The receiver enters safeboot mode if SAFEBOOT_N pin is low at start up. The SAFEBOOT_N pin is internally connected
to TIMEPULSE pin through a 1 kΩ series resistor.
UBX-23002117 - R05
 
3 Pin deﬁnition Page 9 of 22
C1-Public
NEO-F10N - Data sheet
Pin no. Name I/O Description
14 LNA_EN O On/Oﬀ internal LNAs and an optional external LNA or an active
antenna
15 Reserved - Not connected
16 Reserved - Not connected
17 Reserved - Not connected
18 Reserved - Not connected
19 Reserved - Not connected
20 TXD O UART TX
21 RXD I UART RX
22 V_BCKP I Backup voltage supply
23 VCC I Supply voltage
24 GND - Ground
Table 10: NEO-F10N pin assignment
3.2 Pin state
Table 11 deﬁnes the state of the interface pins in diﬀerent modes.
Pin no. Function Continuous mode Software standby mode Safe boot mode
21 RXD Input pull-up Input pull-up Input pull-up
20 TXD Output Input pull-up Output
1 SAFEBOOT_N 14 Output Input pull-up Output (low)
3 TIMEPULSE Output Input pull-up Output (low)
8 RESET_N Input pull-up Input pull-up Input pull-up
4 EXTINT Input pull-up Input pull-up Input pull-up
14 LNA_EN 15 Output (high) Output (low) Output (high)
Table 11: Pin state
In the reset mode (RESET_N = low), all interface pins are conﬁgured as input pull-ups.
Do not drive pins in the hardware backup mode (VCC = 0 V).
15 LNA_EN signal is connected through a buﬀer circuit with 100 kΩ pull-down resistor to GND.
UBX-23002117 - R05
 
3 Pin deﬁnition Page 10 of 22
C1-Public
NEO-F10N - Data sheet
4 Electrical speciﬁcations
4.1 Absolute maximum ratings
CAUTION. Risk of device damage. Exceeding the absolute maximum ratings may aﬀect the
lifetime and reliability of the device or permanently damage it. Do not exceed the absolute
maximum ratings.
This product is not protected against overvoltage or reversed voltages. Use appropriate
protection to avoid device damage from voltage spikes exceeding the speciﬁed boundaries.
Symbol Parameter Min Max Unit
VCC Main supply voltage –0.3 3.6 V
Voltage ramp on VCC  16 25 35000 µs/V
V_BCKP Backup supply voltage –0.3 3.6 V
V_PIO Input voltage on RESET_N and digital pins –0.3 VCC + 0.3
(max 3.6)
V
I_PIO Max source / sink current, digital pins  17 –10 10 mA
ICC_RF Max source current, VCC_RF 250 mA
V_DCrﬁn DC voltage at RF_IN –5.5 +5.5 V
Prﬁn RF input power at RF_IN  18, 19 0 dBm
Tamb Ambient temperature –40 +85 °C
Ts Storage temperature –40 +85 °C
Table 12: Absolute maximum ratings
4.2 Operating conditions
Table 13 shows the general operating conditions. Table 14 shows the electrical parameters for
digital I/O.
Symbol Parameter Min Typical Max Unit
VCC Main supply voltage 2.7 3.0 3.6 V
V_BCKP Supply voltage, backup domain 1.65 3.6 V
VCCSWITCH VCC voltage threshold to switch an internal supply for
the backup domain from VCC to V_BCKP
1.45 V
VCC_RF VCC_RF output voltage VCC - 0.1 V
ICC_RF VCC_RF output current 50 mA
Zin
20 Input impedance at RF_IN 50 Ω
NFtot Receiver chain noise ﬁgure (L1) 3 dB
Receiver chain noise ﬁgure (L5) 3 dB
Ext_gain  21 External gain at RF_IN, low gain mode (default) 25 dB
16 Exceeding the voltage ramp speed may permanently damage the device.
17 The SAFEBOOT_N pin has an internal 1 kΩ series resistor.
18 Test conditions: source impedance = 50 Ω, continuous wave.
19 +15 dBm for outband; 0 dBm for inband
20 The RF_IN input integrates a built-in DC block.
21 The internal LNA gain is conﬁgurable.
UBX-23002117 - R05
 
4 Electrical speciﬁcations Page 11 of 22
C1-Public
NEO-F10N - Data sheet
Symbol Parameter Min Typical Max Unit
Topr Operating temperature -40 +85 °C
Table 13: General operating conditions
Symbol Parameter Min Typical Max Unit
Ileak Leakage current input pins  22 25 nA
Vin Input pin voltage range 0 VCC V
Vil Low-level input voltage 0.63 V
Vih High-level input voltage 0.68 x VCC V
Vol Low-level output voltage, Iout = -2 mA  15,  23 0.4 V
Voh High-level output voltage, Iout = 2 mA 15,  23 VCC - 0.4 V
Rpu, IO Pull-up resistance, Digital IO 8 18 40 kΩ
Rpd, IO Pull-down resistance, Digital IO 21 80 180 kΩ
Rpu, SAFEBOOT_N Pull-up resistance, SAFEBOOT_N  24 6 17 72 kΩ
Rpu, RESET_N Pull-up resistance, RESET_N 7 10 13 kΩ
Table 14: Digital IO
4.3 Indicative power requirements
This section provides examples of typical current requirements. They have been characterized on
samples using a cold start command. The actual power requirements may vary depending on the
ﬁrmware version used, the external circuitry, the number of satellites tracked, the signal strength,
the type and time of start, duration, internal LNA gain mode, and the test conditions.
All values in Table 15 and Table 16 have been measured at 25 °C ambient temperature with the
default conﬁguration unless otherwise stated. SBAS is active in all measurements.
Table 15 shows indicative current consumption for VCC with a 3.0 V supply.
Symbol
(Parameter)
Conditions GPS+GAL
+BDS
(Default)
GPS+BDS GPS+GAL GPS
+NavIC
GPS BDS Unit
Acquisition 26 26 26 22 21 20 24 mAIVCC 25
(VCC current) Tracking 21 20 19 18 18 19 mA
Table 15: Typical currents for 3.0 V supply at VCC
The inrush current can go up to 100 mA at startup. Ensure that the external power supply is
able to deliver up to 100 mA.
Table 16 shows current consumption for backup modes.
Symbol Parameter Conditions Typical Unit
IV_BCKP  27 Total current in hardware backup mode V_BCKP = 3.0 V; VCC  = 0 V 31 µA
22 Vin = VCC, at room temperature.
23 TIMEPULSE has 4 mA current drive/sink capability.
24 The SAFEBOOT_N pin has an additional 1 kΩ series resistor.
25 1 Hz navigation update rate. Simulated signals using power levels of -130 dBm.
26 Average current from start-up until the ﬁrst ﬁx.
27 IV_BCKP current in normal operation  (V_BCKP = 3.0 V) is ~3 µA.
UBX-23002117 - R05
 
4 Electrical speciﬁcations Page 12 of 22
C1-Public
NEO-F10N - Data sheet
Symbol Parameter Conditions Typical Unit
IVCC Total current in software standby mode VCC = 3.0 V 49 µA
Table 16: Backup currents
Extreme operating temperatures can signiﬁcantly impact the speciﬁed values. If an application
operates near the min or max temperature limits, ensure the speciﬁed values are not exceeded.
UBX-23002117 - R05
 
4 Electrical speciﬁcations Page 13 of 22
C1-Public
NEO-F10N - Data sheet
5 Communication interfaces
The receiver supports communication over the UART only.
All the inputs have an internal pull-up resistor in normal operation and can be left open if not used.
The voltage level at the PIO pins is related to the VCC supply voltage.
5.1 UART
The UART interface supports conﬁgurable baud rates. Hardware ﬂow control is not supported. UART
speciﬁcations are described in Table 17.
Symbol Parameter Min Max Unit
Ru Baud rate 9600 921600 bit/s
ΔTx Tx baud rate accuracy -1% +1% -
ΔRx Rx baud rate tolerance -2.5% +2.5% -
Table 17: UART speciﬁcations
5.2 Default interface settings
Interface Settings
UART • 38400 baud28, 8 bits, no parity bit, 1 stop bit.
• Input messages: NMEA and UBX
• Output messages: NMEA GGA, GLL, GSA, GSV, RMC, VTG and TXT
Table 18: Default interface settings
28 9600 baud in the safe boot mode.
UBX-23002117 - R05
 
5 Communication interfaces Page 14 of 22
C1-Public
NEO-F10N - Data sheet
6 Mechanical speciﬁcations
Figure 3: NEO-F10N mechanical drawing
Symbol Min (mm) Typical (mm) Max (mm)
A 15.9 16.0 16.1
B 12.1 12.2 12.3
C 2.2 2.4 2.6
D 0.9 1.0 1.1
E 1.0 1.1 1.2
F 2.9 3.0 3.1
G 0.9 1.0 1.1
H - 0.82 -
K 0.7 0.8 0.9
M 0.8 0.9 1.0
N 0.4 0.5 0.6
P* 0.0 - 0.5 The de-paneling residual tabs may be on either
side (not both).
Weight 1.0 g
Table 19: NEO-F10N mechanical dimensions
The mechanical picture of the de-paneling residual tabs (P*) is an approximate representation,
shape and position may vary.
Component keep-out area must consider that the de-paneling residual tabs can be on either
side (not both).
UBX-23002117 - R05
 
6 Mechanical speciﬁcations Page 15 of 22
C1-Public
NEO-F10N - Data sheet
7 Qualiﬁcations and approvals
Type Description
Quality and reliability
Product qualiﬁcation Qualiﬁed according to u-blox qualiﬁcation policy, based on a subset of AEC-
Q104
Manufacturing Manufactured at IATF 16949 certiﬁed sites
Environmental
RoHS compliance Yes
Moisture sensitivity level (MSL)29, 30 4
Type approvals
European RED certiﬁcation (CE) Declaration of Conformity (DoC) is available on the u-blox website.
UK conformity assessment (UKCA) Yes
Table 20: Qualiﬁcations and approvals
29 For MSL standard see IPC/JEDEC J-STD-020 and J-STD-033 [6].
30 For more information regarding moisture sensitivity levels, labeling, storage, and drying, see the Product packaging
reference guide [5].
UBX-23002117 - R05
 
7 Qualiﬁcations and approvals Page 16 of 22
C1-Public
NEO-F10N - Data sheet
8 Product handling
8.1 Packaging
The NEO-F10N modules are delivered as hermetically sealed, reeled tapes to enable eﬃcient
production, production lot set-up and tear-down. For more information, see the Product packaging
reference guide [5].
8.1.1 Reels
NEO-F10N modules are deliverable in quantities of 500 pieces on a reel. They are shipped on reel
type A3, as speciﬁed in the Product packaging reference guide [5].
8.1.2 Tapes
Figure 4 shows the feed direction, orientation and dimensions of the NEO-F10N modules on the tape
(measurements in mm).
Figure 4: Tape speciﬁcation
8.2 Soldering
Reﬂow soldering is described in the IPC/JEDEC J-STD-020 standard [6].
UBX-23002117 - R05
 
8 Product handling Page 17 of 22
C1-Public
NEO-F10N - Data sheet
9 Product marking and ordering information
This section provides information about product marking and ordering.
9.1 Product marking
The product marking provides information on NEO-F10N and its revision, as in Figure 5. For a
description of the product marking, see Table 21
The product marking is done by laser etching on the NEO-F10N module metal shield cover.
Figure 5: Example of NEO-F10N product marking
Code Meaning Example
PPP Form factor NEO
TGG Platform F10 = u-blox F10
V Variant N = Standard precision, ﬂash, TCXO, SAW ﬁlter, and LNA
NN Major product version 00, 01, 02, ...
Q Product grade A = Automotive, B = Professional
XX Revision Hardware and ﬁrmware versions
(R) Production site No character = global, 1 = regional variant
XXXXXXXXXXX Serial number Alphanumeric characters, e.g. BN600001181
YYWW Production date Year/week, e.g. 2404
Table 21: Description of product marking
9.2 Product identiﬁers
The NEO-F10N marking features three product identiﬁers: product name, ordering code and type
number. The product name identiﬁes all u-blox products, independent of packaging and product
grade, and it is used in documentation such as this data sheet. The ordering code includes the major
product version and product grade, while the type number additionally includes the hardware and
ﬁrmware versions.
Table 22 describes the three diﬀerent product identiﬁers used in the NEO-F10N module product
marking.
Identiﬁer Format Example
Product name PPP-TGGV NEO-F10N
UBX-23002117 - R05
 
9 Product marking and
ordering information
Page 18 of 22
C1-Public
NEO-F10N - Data sheet
Identiﬁer Format Example
Ordering code PPP-TGGV-NNQ NEO-F10N-00B
Type number PPP-TGGV-NNQ-XX NEO-F10N-00B-00
Table 22: Product identiﬁers
9.3 Ordering codes
Ordering code Product Remark
NEO-F10N-00B u-blox F10 multi-band GNSS receiver
module, 24 pin LCC, professional grade
NEO-F10N-00B-20 u-blox F10 multi-band GNSS receiver
module, 24 pin LCC, professional grade
Carrier phase raw data
Table 23: Product ordering codes
u-blox provides information on product changes aﬀecting the form factor, size or function of the
product. For the Product change notiﬁcations (PCNs), see our website at: https://www.u-blox.com/
en/product-resources.
UBX-23002117 - R05
 
9 Product marking and
ordering information
Page 19 of 22
C1-Public
NEO-F10N - Data sheet
Related documents
[1] NEO-F10N Integration manual, UBXDOC-963802114-12193
[2] u-blox F10 SPG 6.00 Interface description, UBX-23002975
[3] u-blox F10 SPG 6.00 Release note, UBXDOC-963802114-12318
[4] Radio Resource LCS Protocol (RRLP), (3GPP TS 44.031 version 11.0.0 Release 11)
[5] Product packaging reference guide, UBX-14001652
[6] Joint IPC/JEDEC standard, www.jedec.org
For regular updates to u-blox documentation and to receive product change notiﬁcations please
register on our homepage https://www.u-blox.com.
UBX-23002117 - R05
 
Related documents Page 20 of 22
C1-Public
NEO-F10N - Data sheet
Revision history
Revision Date Comments
R01 27-Jun-2023 Initial release
R02 01-Nov-2023 Engineering sample
Updated:
• KPIs in section Performance
• Supported signals in Assisted GNSS services in section Supported GNSS constellations
• Description of Reserved pins in section Pin deﬁnition
• Pin state of SAFEBOOT_N and TIMPULSE in section Pin state
• Typical and backup currents in section Indicative power requirements
R03 22-Dec-2023 Initial production
Added:
• Approvals section
Updated:
• Overview section with information related to GPS L5
• Pin state of SAFEBOOT_N and TIMPULSE in section Pin state
• Operating conditions section with input impedance at RF_IN
R04 04-Apr-2025 Mass production
Added:
• Broadcast navigation data and satellite signal measurements
• Qualiﬁcations and approvals
• Packaging
Updated sections:
• Document information: added NEO-F10N-00B-20
• Pin state: added LNA_EN pin state
• Firmware features: added carrier phase output feature for NEO-F10N-00B-20
• Absolute maximum ratings: added outband and inband limits
• Updated the reel type, delivery quantity and tape dimensions in sections Reels and Tapes
• Ordering codes: added NEO-F10N-00B-20
• Related documents: added Radio Resource LCS Protocol (RRLP)
Change in document structure
• Moisture sensitivity level (MSL) included in chapter Qualiﬁcations and approvals
R05 09-Oct-2025 Mass production
Updated sections:
• Updated module weight in table Mechanical speciﬁcations
• Updated section Product marking
UBX-23002117 - R05
 
Revision history Page 21 of 22
C1-Public
NEO-F10N - Data sheet
Contact
u-blox AG
Address: Zürcherstrasse 68
8800 Thalwil
Switzerland
For further support and contact information, visit us at www.u-blox.com/support.
UBX-23002117 - R05
 
Page 22 of 22
C1-Public