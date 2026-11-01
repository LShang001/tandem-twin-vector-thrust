<!-- page 1 of 22 -->



## NEO-F10N

**Standard precision GNSS module Professional grade Data sheet**

blox

NEO-F10N

## Abstract

This Data sheet describes the NEO-F10N module, an L1/L5 dual-band GNSS receiver for meter-level accuracy in urban environment.

**Note!** GPS L5 signals are pre-operational and not used by default. Refer to the Overview section for more information.

Cblox

UBX-23002117 - R05 C1-Public

www.u-blox.com

<!-- page 2 of 22 -->

Cblox

NEO-F10N - Data sheet

Document information

| Title | NEO-F10N |
| --- | --- |
| Subtitle | StandardprecisionGNSSmodule |
| Documenttype | Datasheet |
| Documentnumber | UBX-23002117 |
| Revisionanddate | R05 09-Oct-2025 |
| Disclosurerestriction | C1-Public |

| Product status | Corresponding content status |  |
| --- | --- | --- |
| Functional sample | Draft | For functional testing. Revised and supplementary data will be published later. |
| In development/ | Objective specification | Target values. Revised and supplementary data will be published later. |
| prototype |  |  |
| Engineering sample | Advance information | Data based on early testing. Revised and supplementary data will be published later. |
| Initial production | Early production information | Data from product verification. Revised and supplementary data may be published later. |
| Mass production/ | Production information | Document contains the final product specification. |
| End of life |  |  |

This document applies to the following products:

| Product name | Type number | FW version | IN/PCN reference Product status |
| --- | --- | --- | --- |
| NEO-F10N | NEO-F10N-00B-00 | EXT SPG 6.00 | UBXDOC-304424225-18225 Mass production UBXDOC-963802114-12646 |
| NEO-F10N | NEO-F10N-00B-20 | EXT SPG 6.00 | UBXDOC-304424225-18225 Mass production UBXDOC-963802114-12646 |

u-blox or third parties may hold intellectual property rights in the products, names, logos and designs included in this document. Copying, reproduction, or modification of this document or any part thereof is only permitted with the express written permission of u-blox. Disclosure to third parties is permitted for clearly public documents only.

The information contained herein is provided "as is" and u-blox assumes no liability for its use. No warranty, either express or implied, is given, including but not limited to, with respect to the accuracy, correctness, reliability and fitness for a particular purpose of the information. This document may be revised by u-blox at any time without notice. For the most recent documents and product statuses, visit www.u-blox.com.

Copyright © 2025, u-blox AG.

UBX-23002117 - R05 C1-Public

Document information

Page 2 of 22

<!-- page 3 of 22 -->

Cblox

NEO-F10N - Data sheet

## Contents

- Document information....2
- Contents....3
- 1 Functional description....4
- 1.1 Overview....4
- 1.2 Performance....4
- 1.3 Supported GNSS constellations....5
- 1.4 Broadcast navigation data and satellite signal measurements....6
- 1.5 Supported protocols....6
- 1.6 Firmware features....6
- 2 Block diagram....8
- 3 Pin definition....9
- 3.1 Pin assignment....9
- 3.2 Pin state....10
- 4 Electrical specifications....11
- 4.1 Absolute maximum ratings....11
- 4.2 Operating conditions....11
- 4.3 Indicative power requirements....12
- 5 Communication interfaces....14
- 5.1 UART....14
- 5.2 Default interface settings....14
- 6 Mechanical specifications....15
- 7 Qualifications and approvals....16
- 8 Product handling....17
- 8.1 Packaging....17
- 8.1.1 Reels....17
- 8.1.2 Tapes....17
- 8.2 Soldering....17
- 9 Product marking and ordering information....18
- 9.1 Product marking....18
- 9.2 Product identifiers....18
- 9.3 Ordering codes....19
- Related documents....20
- Revision history....21
- Contact....22

UBX-23002117 - R05 C1-Public

Contents

Page 3 of 22

<!-- page 4 of 22 -->

Cblox

NEO-F10N - Data sheet

## 1 Functional description

## 1.1 Overview

The NEO-F10N is built on the u-blox F10 dual-band GNSS technology using the L1 and L5 band signals. The proprietary dual-band multipath mitigation technology enables the u-blox F10 to use the best signals from the L1 and L5 bands providing a solid meter-level position accuracy in urban environment.

NEO-F10N-00B-20 supports GNSS broadcast data and raw carrier-phase measurements.

At the time of writing, the GPS L5 signals remain pre-operational and are set as unhealthy until sufficient monitoring capability is established. This is an operational issue concerning the satellites / space segment and not a limitation of u-blox products.

Due to the pre-operational status, the GPS L5 signals are not used for the navigation solution by default. However, it is possible to evaluate the GPS L5 signals before they become fully operational by changing the receiver configuration to override the GPS L5 health status. Refer to the Integration manual [1] for details.

## 1.2 Performance

| Parameter | Specification | Value |
| --- | --- | --- |
| Receiver type |  | u-blox F10 dual-band receiver |
| Accuracy of time pulse signal | RMS99% | 30 ns 60 ns |
| Frequency of time pulse signal |  | Default 1PPS (0.25 Hz to 10 MHz configurable) |
| Operational limits <sup>1</sup> | Dynamics Altitude Velocity | ≤ 4 g 80,000 m 500 m/s |
| Velocity accuracy <sup>2</sup> |  | 0.05 m/s |
| Dynamic heading accuracy <sup>2</sup> |  | 0.3 deg |

Table 1: NEO-F10N specifications

Table 2 shows typical performance values in multi-GNSS configurations<sup>3</sup>. SBAS is enabled in all measurements.

| Parameter | GPS+GAL +BDS (Default) | GPS+BDS | GPS+GAL | GPS+NavIC | Unit |
| --- | --- | --- | --- | --- | --- |
| Max navigation update rate <sup>4</sup> | 10 | 10 | 10 | 10 | Hz |
| Position accuracy (CEP) <sup>5</sup> | 1 | 1 | 1 | 1 | m |
| Time To First Fix Cold start | 28 | 28 | 27 | 27 | s |
| (TTFF) <sup>6</sup> Hot start | 2 | 2 | 2 | 2 | s |

1 Assuming Airborne 4 g platform.

2 50% at 30 m/s for dynamic operation.

3 The GPS L5 signal is included in the performance figures. Note that this signal is not fully operational yet and using it requires a configuration change.

4 Minimum 98% fix rate under typical conditions.

5 CEP, 50%, 24 hours static, –130 dBm, > 6 SVs for each GNSS system.

6 Commanded starts. All satellites signals at -130 dBm. Measured at room temperature.

UBX-23002117 - R05 C1-Public

1 Functional description

Page 4 of 22

<!-- page 5 of 22 -->

Cblox

NEO-F10N - Data sheet

| Parameter | GPS+GAL +BDS (Default) | GPS+BDS | GPS+GAL | GPS+NavIC | Unit |
| --- | --- | --- | --- | --- | --- |
| AssistNow Online <sup>7</sup> | 2 | 2 | 2 | 2 | s |
| AssistNow Offline <sup>8</sup> | 3 | 3 | 3 | 3 | s |
| AssistNow Autonomous <sup>9</sup> | 4 | 4 | 4 | 4 | s |
| Sensitivity <sup>10</sup> Tracking and navigation | -167 | -167 | -167 | -167 | dBm |
| Reacquisition | -159 | -159 | -159 | -159 | dBm |
| Cold Start | -148 | -148 | -148 | -148 | dBm |
| Hot start | -159 | -159 | -159 | -159 | dBm |

Table 2: NEO-F10N typical performance in multi-GNSS configurations

Table 3 shows typical performance values in single-GNSS configurations<sup>3</sup>. SBAS is enabled in all measurements.

| Parameter | GPS | BDS | Unit |
| --- | --- | --- | --- |
| Max navigation update rate <sup>4</sup> | 20 | 20 | Hz |
| Position accuracy (CEP) <sup>5</sup> | 1.5 | 1 | m |
| Time To First Fix Cold start | 27 | 42 | s |
| (TTFF) <sup>6</sup> Hot start | 2 | 2 | s |
| AssistNow Online <sup>7</sup> | 2 | N/A | s |
| Sensitivity <sup>10</sup> Tracking and navigation | -167 | -163 | dBm |
| Reacquisition | -159 | -156 | dBm |
| Cold Start | -148 | -137 | dBm |
| Hot start | -159 | -157 | dBm |

Table 3: NEO-F10N typical performance in single-GNSS configurations

## 1.3 Supported GNSS constellations

NEO-F10N is a concurrent GNSS receiver that can receive and track multiple GNSS systems. The dual-band RF front-end architecture enables concurrent reception of multiple dual frequency GNSS constellations. To achieve lower power consumption, the receiver can be configured for a subset of GNSS constellations.

The default configuration on NEO-F10N is concurrent reception of GPS, Galileo and BeiDou with SBAS (EGNOS, GAGAN, MSAS, and WAAS) enabled.

![Image block](doc:19a6131/tier:standard/page:5/block:11)

Single-band operation is not supported. Both L1 and L5 signals must be enabled or disabled for GPS, Galileo, BeiDou, and QZSS.

The following GNSS and their signals are supported:

| System | Signals |
| --- | --- |
| GPS/QZSS | L1C/A (1575.42 MHz), L5 (1176.450 MHz) |
| Galileo | E1-B/C (1575.42 MHz), E5a (1176.450 MHz) |
| BeiDou | B1C (1575.42 MHz) , B2a (1176.450 MHz) |

7 Depends on the speed and latency of the aiding data connection, commanded starts.

8 Using seven days old AssistNow Offline data. External memory may be required.

9 Using two days old orbital predicted data. External memory may be required.

10 Demonstrated with a good external LNA. Measured at room temperature.

UBX-23002117 - R05 C1-Public

1 Functional description

Page 5 of 22

<!-- page 6 of 22 -->

Cblox

NEO-F10N - Data sheet

| System | Signals |
| --- | --- |
| NavIC | SPS-L5 (1176.450 MHz) |

Table 4: Supported GNSS and signals on NEO-F10N

The following GNSS assistance services are supported:

| Service | Support |
| --- | --- |
| AssistNow Online | GPS L1C/A, Galileo E1, QZSS L1C/A |
| AssistNow Offline | GPS L1C/A, Galileo E1, QZSS L1C/A |
| AssistNow Autonomous | GPS L1C/A, Galileo E1 |

**Table 5: Supported Assisted GNSS (A-GNSS) services**

The following augmentation systems are supported:

| System | Support |
| --- | --- |
| SBAS<sup>11</sup> | EGNOS, GAGAN, MSAS, WAAS, BDSBAS, KASS and SouthPAN |
| QZSS | L1S (SLAS), L1Sb (SBAS) |

Table 6: Supported augmentation systems

![Image block](doc:19a6131/tier:standard/page:6/block:11)

The QZSS augmentation system can be enabled only if GPS operation is also enabled.

## 1.4 Broadcast navigation data and satellite signal measurements

The GNSS broadcast data is available in the UBX-RXM-SFRBX message for all supported GNSS signals as well as the QZSS and SBAS augmentation services. The satellite signal information is compatible with the Radio Resource LCS Protocol (RRLP) [4].

The full raw data including code phase, doppler, pseudorange, carrier phase, and measurement quality information is available in the UBX-RXM-RAWX message for all supported GNSS signals. The UBX-RXM-RAWX message follows the conventions of a multi-GNSS RINEX 3 observation file.

This applies only to the product NEO-F10N-00B-20.

![Image block](doc:19a6131/tier:standard/page:6/block:17)

The raw measurement data is available after the receiver has established data bit synchronization and time-of-week.

## 1.5 Supported protocols

NEO-F10N supports the following interface protocols:

| Protocol | Type |
| --- | --- |
| UBX | Input/output, binary, u-blox proprietary |
| NMEA versions 2.1, 2.3, 4.0, 4.10 and 4.11 (default) | Input/output, ASCII |

Table 7: Supported protocols

## 1.6 Firmware features

| Feature | Description |
| --- | --- |
| Assisted GNSS | AssistNow Online, AssistNow Offline and AssistNow Autonomous |

11 Ionospheric correction service is the only SBAS service supported by NEO-F10N

UBX-23002117 - R05 C1-Public

1 Functional description

Page 6 of 22

<!-- page 7 of 22 -->

Cblox

NEO-F10N - Data sheet

| Feature | Description |
| --- | --- |
| Backup modes | Hardware backup mode and software standby mode |
| Protection level | Real-time position accuracy estimate with 95% confidence level<sup>12</sup> |
| Galileo return link messages | Galileo search and rescue (SAR) return link messages (RLM) via Galileo satellite signal |
| Carrier phase output<sup>13</sup> | GNSS broadcast data and raw carrier-phase measurements |
| Odometer | Measure traveled distance with support for different user profiles |
| Table 8: Firmware features |  |
| Feature | Description |
| Anti-jamming | RF interference andjamming detection and reporting |
| Anti-spoofing | Spoofing detection and reporting |
| Configuration lockdown | Receiver configuration can be locked by command |
| Message integrity | All messages can be cryptographically signed |
| Secure boot | Only signed firmware images are executed |

**Table 9: Security features**

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">12 Verified for automotive environment only.</span></small>

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">13 Applies only to the product NEO-F10N-00B-20.</span></small>

UBX-23002117 - R05 C1-Public

1 Functional description

Page 7 of 22

<!-- page 8 of 22 -->

Cblox

NEO-F10N - Data sheet

## 2 Block diagram

![Image block](doc:19a6131/tier:standard/page:8/block:4)

Figure 1: NEO-F10N block diagram

UBX-23002117 - R05 C1-Public

2 Block diagram

Page 8 of 22

<!-- page 9 of 22 -->

Cblox

NEO-F10N - Data sheet

## 3 Pin definition

## 3.1 Pin assignment

![Image block](doc:19a6131/tier:standard/page:9/block:5)

Figure 2: NEO-F10N pin assignment

| Pin no. | Name | I/O | Description |
| --- | --- | --- | --- |
| 1 | SAFEBOOT_N | I | Safeboot mode. If not used, leave open.<sup>14</sup> |
| 2 | Reserved | - | Not connected |
| 3 | TIMEPULSE | O | Time pulse signal (shared with SAFEBOOT_N pin)<sup>14</sup> |
| 4 | EXTINT | I | External interrupt |
| 5 | Reserved | - | Not connected |
| 6 | Reserved | - | Not connected |
| 7 | Reserved | - | Not connected |
| 8 | RESET_N | I | RESET (active low) |
| 9 | VCC_RF | O | Output voltage RF section |
| 10 | GND | - | Ground |
| 11 | RF_IN | I | GNSS signal input |
| 12 | GND | - | Ground |
| 13 | GND | - | Ground |

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">14 The receiver enters safeboot mode if SAFEBOOT\_N pin is low at start up. The SAFEBOOT\_N pin is internally connected to TIMEPULSE pin through a 1 kΩ series resistor.</span></small>

UBX-23002117 - R05 C1-Public

3 Pin definition

Page 9 of 22

<!-- page 10 of 22 -->

Cblox

NEO-F10N - Data sheet

| Pin no. | Name | I/O | Description |
| --- | --- | --- | --- |
| 14 | LNA_EN | O | On/Off internal LNAs and an optional external LNA or an active antenna |
| 15 | Reserved | - | Not connected |
| 16 | Reserved | - | Not connected |
| 17 | Reserved | - | Not connected |
| 18 | Reserved | - | Not connected |
| 19 | Reserved | - | Not connected |
| 20 | TXD | O | UART TX |
| 21 | RXD | I | UART RX |
| 22 | V_BCKP | I | Backup voltage supply |
| 23 | VCC | I | Supply voltage |
| 24 | GND | - | Ground |

Table 10: NEO-F10N pin assignment

## 3.2 Pin state

Table 11 defines the state of the interface pins in different modes.

| Pin no | . Function | Continuous mode | Software standby mode | Safe boot mode |
| --- | --- | --- | --- | --- |
| 21 | RXD | Input pull-up | Input pull-up | Input pull-up |
| 20 | TXD | Output | Input pull-up | Output |
| 1 | SAFEBOOT_N <sup>14</sup> | Output | Input pull-up | Output (low) |
| 3 | TIMEPULSE | Output | Input pull-up | Output (low) |
| 8 | RESET_N | Input pull-up | Input pull-up | Input pull-up |
| 4 | EXTINT | Input pull-up | Input pull-up | Input pull-up |
| 14 | LNA_EN <sup>15</sup> | Output (high) | Output (low) | Output (high) |

Table 11: Pin state

心 In the reset mode $\mathsf { ( R E S E T \_ N = I o w ) }$ , all interface pins are configured as input pull-ups.

Do not drive pins in the hardware backup mode $(VCC = 0V)$

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">15 LNA\_EN signal is connected through a buffer circuit with 100 kΩ pull-down resistor to GND.</span></small>

UBX-23002117 - R05 C1-Public

3 Pin definition

Page 10 of 22