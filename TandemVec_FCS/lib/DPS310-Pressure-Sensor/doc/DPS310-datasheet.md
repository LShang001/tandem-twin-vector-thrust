<!-- page 1 of 41 -->

infineon

## DPS310

## Digital XENSIV™ Barometric Pressure Sensor for Portable Devices

## Product Description

The DPS310 is a miniaturized Digital Barometric Air Pressure Sensor with a high accuracy and a low current consumption, capable of measuring both pressure and temperature. The pressure sensor element is based on a capacitive sensing principle which guarantees high precision during temperature changes. The small package makes the DPS310 ideal for mobile applications and wearable devices.

![Image block](doc:440d2b0/tier:standard/page:1/block:6)

The internal signal processor converts the output from the pressure and temperature sensor elements to 24 bit results. Each unit is individually calibrated, the calibration coefficients calculated during this process are stored in the calibration registers. The coefficients are used in the application to convert the measurement results to high accuracy pressure and temperature values. The result FIFO can store up to 32 measurement results, allowing for a reduced host processor polling rate. Sensor measurements and calibration coefficients are available through the serial I²C or SPI interface. The measurement status is indicated by status bits or interrupts on the SDO pin.

## Features

\- Operation range: Pressure: 300 –1200 hPa. Temperature: -40 – 85 °C.

• Pressure sensor precision: ± 0.002 hPa (or ±0.02 m) (high precision mode).

![Image block](doc:440d2b0/tier:standard/page:1/block:11)

• Relative accuracy: ± 0.06 hPa (or ±0.5 m)

• Absolute accuracy: ±1 hPa (or ±8 m)

• Temperature accuracy: ± 0.5°C.

• Pressure temperature sensitivity: 0.5Pa/K

• Measurement time: Typical: 27.6 ms for standard mode (16x). Minimum: 3.6 ms for low precision mode.

• Average current consumption: 1.7 μA for Pressure Measurement, 1.5uA for Temperature measurement @1Hz sampling rate, Standby: 0.5 μA.

• Supply voltage: VDDIO: 1.2 – 3.6 V, VDD: 1.7 – 3.6 V.

\- Operating modes: Command (manual), Background (automatic), and Standby.

• Calibration: Individually calibrated with coefficients for measurement correction.

• FIFO: Stores up to 32 pressure or temperature measurements.

\- Interface: $I^{2}C$ and SPI (both with optional interrupt)

• Package dimensions: 8-pin LGA, 2.0 mm x 2.5 mm x 1.0 mm.

• Green Product (RoHS) Compliant

## Typical Applications

\- Indoor Navigation (floor detection e.g. in shopping malls and parking garages)

• Health and Sports (accurate elevation gain and vertical speed)

• Outdoor Navigation (GPS start-up time and accuracy improvement, dead-reckoning e.g. in tunnels)

• Weather Station('Micro-weather' and local forecasts)

• HDD drivers, (leak rate detection in hard disk drives)

• Drones (flight stability and height control)

Please read the Important Notice and Warnings at the end of this document

V1.1

www.infineon.com

2019-07-11

<!-- page 2 of 41 -->

DPS310
Digital XENSIV™ Barometric Pressure Sensor for Portable Devices

infineon

## Table of contents

## Table of contents

- Product Description....1
- Features....1
- Typical Applications....1
- Table of contents....2
- 1 Definitions, acronyms and abbreviations....4
- 1.1 Definitions....4
- 2 Pin Configuration and Block Diagram....5
- 2.1 Pin Configuration and Description....5
- 2.2 Block Diagram....6
- 3 Specifications....6
- 3.1 Operating Range....6
- 3.2 Absolute Maximum Ratings....7
- 3.3 Current Consumption....7
- 3.4 Temperature Transfer Function....8
- 3.5 Pressure Transfer Function....9
- 3.6 Timing Characteristics....10
- 4 Functional Description....10
- 4.1 Operating Modes....10
- 4.2 Mode transition diagram....10
- 4.3 Start-up sequence....11
- 4.4 Measurement Precision and Rate....11
- 4.5 Sensor Interface....12
- 4.6 Interrupt....12
- 4.7 Result Register Operation....13
- 4.8 FIFO Operation....13
- 4.9 Calibration and Measurement Compensation....13
- 4.9.1 How to Calculate Compensated Pressure Values....14
- 4.9.2 How to Calculate Compensated Temperature Values....14
- 4.9.3 Compensation Scale Factors....15
- 4.9.4 Pressure and Temperature calculation flow....15
- 5 Applications....17
- 5.1 Measurement Settings and Use Case Examples....17
- 5.2 Application Circuit Example....18
- 5.3 IIR filtering....19
- 6 Digital interfaces....19
- 6.1 I2C Interface....20
- 6.2 SPI Interface....20

2

2019-07-11

<!-- page 3 of 41 -->

DPS310
Digital XENSIV™ Barometric Pressure Sensor for Portable Devices

infineon

## Table of contents

- 6.3 Interface parameters specification 22
- 6.3.1 General interface parameters 22
- 6.3.1.1 I2C timings 22
- 6.3.1.2 SPI timings 24
- 7 Register Map 25
- 8 Register description 25
- 8.1 Pressure Data (PRS\_Bn) 25
- 8.1.1 PRS\_B2 26
- 8.1.2 PRS\_B1 26
- 8.1.3 PRS\_B0 27
- 8.2 Temperature Data (TMP\_Tn) 27
- 8.2.1 TMP\_B2 27
- 8.2.2 TMP\_B1 28
- 8.2.3 TMP\_B0 28
- 8.3 Pressure Configuration (PRS\_CFG) 29
- 8.4 Temperature Configuration(TMP\_CFG) 31
- 8.5 Sensor Operating Mode and Status (MEAS\_CFG) 32
- 8.6 Interrupt and FIFO configuration (CFG\_REG) 33
- 8.7 Interrupt Status (INT\_STS) 34
- 8.8 FIFO Status (FIFO\_STS) 35
- 8.9 Soft Reset and FIFO flush (RESET) 36
- 8.10 Product and Revision ID (ID) 36
- 8.11 Calibration Coefficients (COEF) 37
- 8.12 Coefficient Source 38
- 9 Package Dimensions 38
- 10 Package Handling 38
- 11 Reflow soldering and board assembly 38
- Revision History 40
- Known Issues List 40
- Disclaimer 41

3

V1.1

2019-07-11

<!-- page 4 of 41 -->

DPS310
Digital XENSIV™ Barometric Pressure Sensor for Portable Devices

infineon

1 Definitions, acronyms and abbreviations

## 1 Definitions, acronyms and abbreviations

1.1 Definitions

An explanation of terms and definitions used in this datasheet.

## Table 1

| Term | Definition/explanation |
| --- | --- |
| Absolute accuracy | The absolute measurement accuracy over the entire measurement range. |
| Digital bit depth | The total bit depth used for conversion of the sensor input to the digital output. Measured in bits. |
| Digital resolution | The pressure value represented by the LSB change in output. This value should be much smaller than the sensor noise. |
| Full Scale Range (FSR) | The peak-to-peak measurement range of the sensor. |
| LSB | Least Significant Bit |
| Measurement time | The time required to acquire one sensor output result. This value determines the maximum measurement rate. |
| MSB | Most Significant Bit |
| Non-linearity | The deviation of measured output from the best-fit straight line, relative to 1000 hPa and 25 °C. |
| Output compensation | The process of applying an algorithm to the sensor output to improve the absolute accuracy of the sensor across temperature and to minimize unit to unit output variation. This algorithm makes use of both the temperature sensor readings and the individual calibration coefficients. |
| Precision (noise) | The smallest measurable change, expressed as rms, after sensor oversampling. |
| Pressure temperature coefficient | The pressure measurement deviation, after compensation, from expected measurement value due to temperature change from 25 °C. Measured in Pa/K. |
| Sensor calibration | The process, during the production test, where the sensor's measurement results are compared against reference values, and a set of calibration coefficients are calculated from the deviation. The coefficients are stored in the sensor's memory and are used in the output compensation. |
| Sensor oversampling rate (OSR) | Specifies the number of sensor measurements used internally to generate one sensor output result. |

4

2019-07-11

V1.1

<!-- page 5 of 41 -->

DPS310
Digital XENSIV™ Barometric Pressure Sensor for Portable Devices

infineon

2 Pin Configuration and Block Diagram

## 2

## Pin Configuration and Block Diagram

## 2.1

## Pin Configuration and Description

![Image block](doc:440d2b0/tier:standard/page:5/block:8)

Figure 1 Pin configuration (top view, figure not to scale)

Table 2 Pin description

<table><tr><td>Pin</td><td>Name</td><td>SPI 3-wire</td><td>SPI 3-wire with interrupt</td><td>SPI 4-wire</td><td> $I^2C$ </td><td> $I^2C$  with interrupt</td></tr><tr><td>1</td><td>GND</td><td colspan="5">Ground</td></tr><tr><td>2</td><td>CSB</td><td>Chip select - active low</td><td>Chip select - active low</td><td>Chip select - active low</td><td>Not used - open (internal pull-up) or tie to VDDIO</td><td>Not used - open (internal pull-up) or tie to VDDIO</td></tr><tr><td>3</td><td>SDI</td><td>Serial data in/out</td><td>Serial data in/out</td><td>Serial data in</td><td>Serial data in/out</td><td>Serial data in/out</td></tr><tr><td>4</td><td>SCK</td><td colspan="5">Serial Clock</td></tr><tr><td>5</td><td>SDO</td><td>Not used</td><td>Interrupt</td><td>Serial data out</td><td>Least significant bit in the device address.</td><td>Interrupt pin and least significant bit in the device address.</td></tr><tr><td>6</td><td>VDDIO</td><td colspan="5">Digital supply voltage for digital blocks and I/O interface</td></tr><tr><td>7</td><td>GND</td><td colspan="5">Ground</td></tr><tr><td>8</td><td>VDD</td><td colspan="5">Supply voltage for analog blocks</td></tr></table>

5

V1.1

2019-07-11

<!-- page 6 of 41 -->

DPS310
Digital XENSIV™ Barometric Pressure Sensor for Portable Devices

infineon

## 3 Specifications

## 2.2

Block Diagram

![Image block](doc:440d2b0/tier:standard/page:6/block:6)

Figure 2

3 Specifications

3.1 Operating Range

The following operating conditions must not be exceeded in order to ensure correct operation of the device. All parameters specified in the following sections refer to these operating conditions, unless noted otherwise.

Table 3

Operating Range

<table><tr><td rowspan="2">Parameter</td><td rowspan="2">Symbol</td><td colspan="3">Values</td><td rowspan="2">Unit</td><td rowspan="2">Note / Test Condition</td></tr><tr><td>Min.</td><td>Typ.</td><td>Max.</td></tr><tr><td>Pressure</td><td> $P_a$ </td><td>300</td><td></td><td>1200</td><td>hPa</td><td></td></tr><tr><td>Temperature</td><td> $T_a$ </td><td>-40</td><td></td><td>85</td><td>°C</td><td></td></tr><tr><td>Supply voltage</td><td> $V_{DD}$ </td><td>1.7</td><td></td><td>3.6</td><td>V</td><td></td></tr><tr><td>Supply voltage IO</td><td> $V_{DDIO}$ </td><td>1.2</td><td></td><td>3.6</td><td>V</td><td></td></tr><tr><td>Supply voltage ramp-up time</td><td> $t_{vddup}$ </td><td>0.001</td><td></td><td>5</td><td>ms</td><td>Time for supply voltage to reach 90% of final value.</td></tr><tr><td>Solder drift1)</td><td></td><td></td><td>0.8</td><td></td><td>hPa</td><td>Minimum solder height 50um.</td></tr><tr><td>Long term stability</td><td></td><td></td><td>±1</td><td></td><td>hPa</td><td>Depending on environmental conditions.</td></tr></table>

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">Effects of solder drift can be eliminated by one point calibration. See AN487.</span></small>

6

V1.1

2019-07-11

<!-- page 7 of 41 -->

DPS310
Digital XENSIV™ Barometric Pressure Sensor for Portable Devices

infineon

## 3 Specifications

## 3.2 Absolute Maximum Ratings

Maximum ratings are absolute ratings. Exceeding any one of these values may cause irreversible damage to the integrated circuit.

Attention: Stresses above the values listed as "Absolute Maximum Ratings" may cause permanent damage to the devices. Exposure to absolute maximum rating conditions for extended periods may affect device reliability.

Table 4 Absolute Maximum Ratings

<table><tr><td rowspan="2">Parameter</td><td rowspan="2">Symbol</td><td colspan="3">Values</td><td rowspan="2">Unit</td><td rowspan="2">Note / Test Condition</td></tr><tr><td>Min.</td><td>Typ.</td><td>Max.</td></tr><tr><td> $V_{DD}\ and\ V_{DDIO}$ </td><td> $V_{DDxx\_max}$ </td><td></td><td></td><td>4</td><td>V</td><td></td></tr><tr><td>Voltage on any pin</td><td> $V_{max}$ </td><td></td><td></td><td>4</td><td>V</td><td></td></tr><tr><td>Storage temperature</td><td> $T_s$ </td><td>-40</td><td></td><td>125</td><td>°C</td><td></td></tr><tr><td>Pressure</td><td> $P_{max}$ </td><td></td><td></td><td>10,000</td><td>hPa</td><td></td></tr><tr><td>ESD</td><td> $V_{ESD\_HBM}$ </td><td>-2</td><td></td><td>2</td><td>KV</td><td>HBM (JS001)</td></tr></table>

## 3.3 Current Consumption

Test conditions (unless otherwise specified in the table):  $V_{DD}=1.8V$  and  $V_{DDIO}=1.8V$ . Typ. values ( $P_{A}=1000hPa$  and  $T_{A}=25^{\circ}C$ ). Max./Min. values ( $P_{A}=950-1050hPa$  and  $T_{A}=0...+65^{\circ}C$ ).

<table><tr><td rowspan="2">Parameter</td><td rowspan="2">Symbol</td><td colspan="3">Values</td><td rowspan="2">Unit</td><td rowspan="2">Note / Test Condition</td></tr><tr><td>Min.</td><td>Typ.</td><td>Max.</td></tr><tr><td rowspan="2">Peak Current Consumption</td><td rowspan="2"> $I_{peak}$ </td><td></td><td>345</td><td></td><td>μA</td><td>during Pressure measurement</td></tr><tr><td></td><td>280</td><td></td><td>μA</td><td>during Temperature measurement</td></tr><tr><td>Standby Current Consumption</td><td> $I_{stb}$ </td><td></td><td>0.5</td><td></td><td>μA</td><td></td></tr><tr><td rowspan="3">Current Consumption.(1 pressure and temperature measurements per second.)</td><td rowspan="3"> $I_{1Hz}$ </td><td></td><td>2.1</td><td></td><td rowspan="3">μA</td><td>Low precision</td></tr><tr><td></td><td>11</td><td></td><td>Standard precision</td></tr><tr><td></td><td>38</td><td></td><td>High precision</td></tr></table>

Note: The current consumption depends on both pressure measurement precision and rate. Please refer to the Pressure Configuration (PRS\_CFG) register description for an overview of the current consumption in different combinations of measurement precision and rate.

7

2019-07-11

V1.1

<!-- page 8 of 41 -->

DPS310
Digital XENSIV™ Barometric Pressure Sensor for Portable Devices

infineon

## 3 Specifications

## 3.4

## Temperature Transfer Function

Test conditions (unless otherwise specified in the table):  $V_{DD}=1.8V$  and  $V_{DDIO}=1.8V$ . Typ. values ( $P_{A}=1000hPa$  and  $T_{A}=25^{\circ}C$ ). Max./Min. values ( $P_{A}=950-1050hPa$  and  $T_{A}=0...+65^{\circ}C$ ).

Table 6

Temperature Transfer Function

<table><tr><td rowspan="2">Parameter</td><td rowspan="2">Symbol</td><td colspan="3">Values</td><td rowspan="2">Unit</td><td rowspan="2">Note / Test Condition</td></tr><tr><td>Min.</td><td>Typ.</td><td>Max.</td></tr><tr><td>Temperature accuracy</td><td> $A_t$ </td><td></td><td>+/-0.5</td><td></td><td>°C</td><td></td></tr><tr><td>Temperature data resolution</td><td> $A_{t\_res}$ </td><td></td><td>0.01</td><td></td><td>°C</td><td></td></tr><tr><td>Temperature measurement rate</td><td>f</td><td>1</td><td></td><td>128</td><td>Hz</td><td></td></tr></table>

8

2019-07-11

V1.1

<!-- page 9 of 41 -->

DPS310
Digital XENSIV™ Barometric Pressure Sensor for Portable Devices

infineon

## 3 Specifications

## 3.5 Pressure Transfer Function

Test conditions (unless otherwise specified in the table):  $V_{DD}=1.8V$  and  $V_{DDIO}=1.8V$ . Typ. values ( $P_{A}=1000hPa$  and  $T_{A}=25^{\circ}C$ ).

Pressure Transfer Function

<table><tr><td rowspan="2">Parameter</td><td rowspan="2">Symbol</td><td colspan="3">Values</td><td rowspan="2">Unit</td><td rowspan="2">Note / Test Condition</td></tr><tr><td>Min.</td><td>Typ.</td><td>Max.</td></tr><tr><td>Absolute pressure accuracy</td><td> $A_{p\_abs}$ </td><td></td><td>+/-100</td><td></td><td>Pa</td><td>Pa=300 - 1200hPa; $T_A$ =0..+65°C; Excluding solder effects</td></tr><tr><td>Relative pressure accuracy</td><td> $A_{p\_rel}$ </td><td></td><td>+/-6</td><td></td><td>Pa</td><td>Any Δ1hPa in the range Pa=800 - 1200hP; Any constant temperature in the range  $T_A$ =20..+60°C</td></tr><tr><td rowspan="3">Pressure precision</td><td rowspan="3"> $A_{p\_prc}$ </td><td></td><td>1.0</td><td></td><td rowspan="3"> $Pa_{RMS}$ </td><td>Low Power</td></tr><tr><td></td><td>0.35</td><td></td><td>Standard</td></tr><tr><td></td><td>0.2</td><td></td><td>High Precision</td></tr></table>

Note: Pressure precision is measured as the average standard deviation. Please refer to the Pressure Configuration (PRS\_CFG) register description for all precision mode options.

<table><tr><td>Power supply rejection</td><td> $A_{p\_psr}$ </td><td></td><td></td><td>0.063</td><td> $Pa_{RMS}$ </td><td>Measured with 217Hz square wave and broad band noise, 100mVpp</td></tr><tr><td>Pressure temperature sensitivity of calibrated measurements</td><td> $A_{p\_tmp}$ </td><td></td><td>0.5</td><td></td><td>Pa/K</td><td>1000hPa, 25...+65°C.</td></tr><tr><td>Pressure data resolution</td><td> $A_{p\_res}$ </td><td></td><td></td><td>0.06</td><td> $Pa_{RMS}$ </td><td></td></tr><tr><td>Pressure measurement rate</td><td>f</td><td>1</td><td></td><td>128</td><td>Hz</td><td></td></tr><tr><td rowspan="3">Pressure measurement time</td><td rowspan="3">t</td><td></td><td>5.2</td><td></td><td rowspan="3">ms</td><td>Low Power</td></tr><tr><td></td><td>27.6</td><td></td><td>Standard</td></tr><tr><td></td><td>105</td><td></td><td>High Precision</td></tr></table>

Note: The pressure measurement time (and thus the maximum rate) depends on the pressure measurement precision. Please refer to the Pressure Configuration (PRS\_CFG) register description for an overview of the possible combinations of measurement precision and rate.

9

2019-07-11

V1.1

<!-- page 10 of 41 -->

Digital XENSIV™ Barometric Pressure Sensor for Portable Devices

infineon

4 Functional Description

## 3.6 Timing Characteristics

Table 8 Timing Characteristics

<table><tr><td rowspan="2">Parameter</td><td rowspan="2">Symbol</td><td colspan="3">Values</td><td rowspan="2">Unit</td><td rowspan="2">Note / Test Condition</td></tr><tr><td>Min.</td><td>Typ.</td><td>Max.</td></tr></table>

| Time to sensor ready | $T_{Sensor\_rdy}$ |  |  | 12 | ms | The SENSOR_RDY bit in the Measurement Configuration register will be set when the sensor is ready. |
| --- | --- | --- | --- | --- | --- | --- |
| Time to coefficients are available. | $T_{Coef\_rdy}$ |  |  | 40 | ms | The COEF_RDY bit in the Measurement Configuration register will be set when the coefficients can be read out. |

Note: Start-up timing is measured from VDD > 1.2V & VDDIO > 0.6V or Soft Reset.

| I2C Clock. | fI2C |  |  | 3.4 | MHz |  |
| --- | --- | --- | --- | --- | --- | --- |
| SPI Clock | fSPI |  |  | 10 | MHz |  |

## 4 Functional Description

## 4.1 Operating Modes

The DPS310 supports 3 different modes of operation: Standby, Command, and Background mode.

• Standby Mode

\- Default mode after power on or reset. No measurements are performed.

\- All registers and compensation coefficients are accessible.

• Command Mode

\- One temperature or pressure measurement is performed according to the selected precision.

\- The sensor will return to Standby Mode when the measurement is finished, and the measurement result will be available in the data registers.

• Background Mode

\- Pressure and/or temperature measurements are performed continuously according to the selected measurement precision and rate. The temperature measurement is performed immediately after the pressure measurement.

\- The FIFO can be used to store 32 measurement results and minimize the number of times the sensor must be accessed to read out the results.

Note: Operation mode and measurement type are set in the Sensor Operating Mode and Status (MEAS\_CFG) register.

## 4.2 Mode transition diagram

The mode transition diagram is shown below.

10

2019-07-11

V1.1