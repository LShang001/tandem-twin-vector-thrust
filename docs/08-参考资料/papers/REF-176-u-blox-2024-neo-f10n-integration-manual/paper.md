<!-- page 1 of 72 -->



## NEO-F10N

**Standard precision GNSS module Professional grade**

**Integration manual**

![Image block](doc:7d59532/tier:standard/page:1/block:5)

Cblox

NEO-F10N

## Abstract

This document describes the features and application of the u-blox NEO-F10N module, an L1/L5 dual-band GNSS receiver for meter-level accuracy in urban environment.

**Note!** GPS L5 signals are pre-operational and not used by default. Refer to the Overview section for more information.

www.u-blox.com

UBXDOC-963802114-12193 - R04 C1-Public

Cblox

<!-- page 2 of 72 -->

Cblox

NEO-F10N - Integration manual

Document information

| Title | NEO-F10N |
| --- | --- |
| Subtitle | Standard precision GNSS module |
| Document type | Integration manual |
| Document number | UBXDOC-963802114-12193 |
| Revision and date | R04 12-Dec-2024 |
| Disclosure restriction | C1-Public |

This document applies to the following products:

<table><tbody><tr><td colspan="2">Product name Type number</td><td>FW version</td><td>IN/PCN reference</td><td>RN reference</td></tr><tr><td>NEO-F10N</td><td>NEO-F10N-00B-00</td><td>EXT SPG 6.00</td><td>UBXDOC-963802114-12320</td><td>UBXDOC-963802114-12318</td></tr></tbody></table>

u-blox or third parties may hold intellectual property rights in the products, names, logos and designs included in this document. Copying, reproduction, or modification of this document or any part thereof is only permitted with the express written permission of u-blox. Disclosure to third parties is permitted for clearly public documents only.

The information contained herein is provided "as is" and u-blox assumes no liability for its use. No warranty, either express or implied, is given, including but not limited to, with respect to the accuracy, correctness, reliability and fitness for a particular purpose of the information. This document may be revised by u-blox at any time without notice. For the most recent documents and product statuses, visit www.u-blox.com.

Copyright © 2024, u-blox AG.

UBXDOC-963802114-12193 - R04 C1-Public

Page 2 of 72

<!-- page 3 of 72 -->

Cblox

NEO-F10N - Integration manual

## Contents

- 1 System description....6
- 1.1 Overview....6
- 1.2 Architecture....6
- 1.2.1 Block diagram....6
- 1.3 Pin definition....7
- 1.3.1 Pin assignment....7
- 2 Receiver configuration....9
- 2.1 Basic receiver configuration....10
- 2.1.1 Basic hardware configuration....10
- 2.1.2 Internal LNA mode configuration....10
- 2.1.3 GNSS signal configuration....10
- 2.1.4 GPS L5 signal health status configuration....11
- 2.1.5 Communication interface configuration....11
- 2.1.6 Message output configuration....12
- 2.2 Navigation configuration....13
- 2.2.1 Dynamic platform....13
- 2.2.2 Navigation input filters....13
- 2.2.3 Navigation output filters....14
- 2.2.4 Odometer filters....14
- 2.2.5 Static hold....15
- 2.2.6 Freezing the course over ground....16
- 3 Receiver functionality....18
- 3.1 Augmentation systems....18
- 3.1.1 SBAS....18
- 3.1.2 QZSS SLAS....20
- 3.2 Communication interfaces and PIOs....21
- 3.2.1 UART....21
- 3.2.2 PIOs....22
- 3.3 Forcing receiver reset....23
- 3.4 Security....24
- 3.4.1 GNSS receiver integrity....24
- 3.4.2 Jamming and spoofing detection....25
- 3.5 Power management....28
- 3.5.1 Continuous mode....28
- 3.5.2 Backup modes....28
- 3.6 Time....29
- 3.6.1 Receiver local time....29
- 3.6.2 GNSS time bases....29
- 3.6.3 Navigation epochs....30
- 3.6.4 iTow timestamps....31
- 3.6.5 Time validity....31
- 3.6.6 UTC representation....32
- 3.6.7 Leap seconds....32
- 3.6.8 Date ambiguity....33

UBXDOC-963802114-12193 - R04 C1-Public

Contents

Page 3 of 72

<!-- page 4 of 72 -->

Cblox

NEO-F10N - Integration manual

- 3.7 Time mark....34
- 3.8 Time pulse....35
- 3.8.1 Recommendations....35
- 3.8.2 Time pulse configuration....36
- 3.9 Time maintenance....37
- 3.9.1 Real-time clock....37
- 3.9.2 Time assistance....38
- 3.9.3 Frequency assistance....38
- 3.9.4 Clock drift assistance....38
- 3.10 Protection level....38
- 3.10.1 Introduction....38
- 3.10.2 Interface....39
- 3.10.3 Validity requirements....40
- 3.10.4 Expected behavior....41
- 3.11 Multiple GNSS assistance (MGA)....41
- 3.11.1 Authorization....42
- 3.11.2 Preserving MGA and operational data during power-off....42
- 3.11.3 AssistNow Online....42
- 3.11.4 AssistNow Offline....42
- 3.11.5 AssistNow Autonomous....43
- 3.12 Save-on-shutdown....46
- 3.13 Firmware update....47
- 4 Hardware integration....48
- 4.1 Power supply....48
- 4.1.1 VCC....48
- 4.1.2 V\_BCKP....48
- 4.2 RF interference....48
- 4.2.1 In-band interference....49
- 4.2.2 Out-of-band interference....49
- 4.2.3 Spectrum analyzer....49
- 4.3 RF front-end....51
- 4.3.1 Internal LNA modes....51
- 4.3.2 Out-of-band blocking immunity....51
- 4.3.3 Interference coupling....52
- 4.3.4 Antenna power supply....52
- 4.4 Layout....53
- 4.4.1 Package footprint, copper and solder mask....55
- 5 Product handling....57
- 5.1 Safety....57
- 5.1.1 ESD precautions....57
- 5.1.2 Safety precautions....58
- 5.2 Soldering....58
- Appendix....62
- A Migration....62
- A.1 Hardware differences....62
- A.2 Firmware changes....63
- B Reference designs....65
- B.1 Typical design....65
- B.2 Active antenna design....66

UBXDOC-963802114-12193 - R04 C1-Public

Contents

Page 4 of 72

<!-- page 5 of 72 -->

Cblox

NEO-F10N - Integration manual

- C External components....68
- C.1 Antenna....68
- C.2 Standard capacitors....68
- C.3 Standard resistors....68
- C.4 Inductors....69
- C.5 Switch transistors for antenna supply....69
- Related documents....70
- Revision history....71

UBXDOC-963802114-12193 - R04 C1-Public

Contents

Page 5 of 72

<!-- page 6 of 72 -->

Cblox

NEO-F10N - Integration manual

## 1 System description

This document is an important source of information for all aspects of NEO-F10N software and hardware design. The purpose of this document is to provide guidelines for a successful integration of the receiver with the customer's end product.

## 1.1 Overview

NEO-F10N is built on the u-blox F10 dual-band GNSS technology using the L1 and L5 band signals. The proprietary dual-band multipath mitigation technology enables the u-blox F10 to use the best signals from the L1 and L5 bands providing a solid meter-level position accuracy in urban environment.

At the time of writing, the GPS L5 signals remain pre-operational and are set as unhealthy until sufficient monitoring capability is established. This is an operational issue concerning the satellites / space segment and not a limitation of u-blox products.

Due to the pre-operational status, the GPS L5 signals are not used for the navigation solution by default. However, it is possible to evaluate the GPS L5 signals before they become fully operational by changing the receiver configuration to override the GPS L5 health status. Refer to the section GPS L5 signal health status configuration for details.

## 1.2 Architecture

The NEO-F10N receiver provides all the necessary RF and baseband processing to enable dual-band operation. The block diagram below shows the key functionality.

## 1.2.1 Block diagram

![Image block](doc:7d59532/tier:standard/page:6/block:12)

Figure 1: NEO-F10N block diagram

UBXDOC-963802114-12193 - R04 C1-Public

1 System description

Page 6 of 72

<!-- page 7 of 72 -->

Cblox

NEO-F10N - Integration manual

## 1.3 Pin definition

## 1.3.1 Pin assignment

![Image block](doc:7d59532/tier:standard/page:7/block:5)

Figure 2: NEO-F10N pin assignment

| Pin no. | Name | I/O | Description |
| --- | --- | --- | --- |
| 1 | SAFEBOOT_N | I | Safeboot mode. If not used, leave open.<sup>1</sup> |
| 2 | Reserved | - | Not connected |
| 3 | TIMEPULSE | O | Time pulse signal (shared with SAFEBOOT_N pin)<sup>1</sup> |
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
| 14 | LNA_EN | O | On/Off internal LNAs and an optional external LNA or an active antenna |
| 15 | Reserved | - | Not connected |

1 The receiver enters safeboot mode if SAFEBOOT\_N pin is low at start up. The SAFEBOOT\_N pin is internally connected to TIMEPULSE pin through a 1 kΩ series resistor.

UBXDOC-963802114-12193 - R04 C1-Public

1 System description

Page 7 of 72

<!-- page 8 of 72 -->

Cblox

NEO-F10N - Integration manual

| Pin no. | Name | I/O | Description |
| --- | --- | --- | --- |
| 16 | Reserved | - | Not connected |
| 17 | Reserved | - | Not connected |
| 18 | Reserved | - | Not connected |
| 19 | Reserved | - | Not connected |
| 20 | TXD | O | UART TX |
| 21 | RXD | I | UART RX |
| 22 | V_BCKP | I | Backup voltage supply |
| 23 | VCC | I | Supply voltage |
| 24 | GND | - | Ground |

Table 1: NEO-F10N pin assignment

UBXDOC-963802114-12193 - R04 C1-Public

1 System description

Page 8 of 72

<!-- page 9 of 72 -->

Cblox

NEO-F10N - Integration manual

## 2 Receiver configuration

The configuration determines all aspects of the GNSS receiver operation and therefore, information in this section is essential for the successful integration of NEO-F10N.

NEO-F10N is configured using UBX configuration interface keys. The configuration database in the receiver's RAM holds the current configuration, which is used by the receiver at runtime. It is constructed at the receiver startup from several sources of configuration. For more information on the receiver configuration, see the Interface description [3].

The configuration can be stored in the RAM, battery-backed RAM (BBR), and flash memory. It is recommended to store the configuration in the flash memory only one time during production, not at every start-up. Nevertheless, sporadic configuration changes on the run are permitted.

The permanence of the stored configuration and the actions to clear it in each memory are listed in Table 2.

| Memory | Permanence of storage | Clearing actions |
| --- | --- | --- |
| RAM | Settings remain effective until power-down | Activating the RESET_N pinA UBX-CFG-RST message excluding GNSS stop (resetMode 0x08) and GNSS start (resetMode 0x09)Entering software standby mode |
| BBR | The receiver retains the settings stored as long as the backup power supply remains | Activating the RESET_N pinA UBX-CFG-RST message with reset mode set to a hardware reset (resetMode 0x00 and 0x04) |
| Flash | Permanent storage of the configuration settings | A UBX-CFG-CFG message to erase the configuration on the flash layer |

Table 2: Permanence of storage and clearing actions for each memory

For more information about the UBX-CFG-RST message, refer to Forcing receiver reset.

**CAUTION** The configuration interface has changed from earlier u-blox positioning receivers. Users must adopt the configuration interface described in this document.

The configuration interface settings are stored in a database consisting of separate configuration items. An item is made up of a pair consisting of a key ID and a value. Related items are grouped together and identified under a common group name: CFG-GROUP-\*; a convention used in u-center 2 and within this document. Within u-center 2, a configuration group is identified as "Group name" and the configuration item is identified as the "item name" in the "Device configuration" window.

The UBX messages available to change or poll the configurations are the UBX-CFG-VALSET, UBX-CFG-VALGET, and UBX-CFG-VALDEL messages. For more information about these messages and the configuration keys, see the configuration interface section in the Interface description [3].

UBXDOC-963802114-12193 - R04 C1-Public

2 Receiver configuration

Page 9 of 72

<!-- page 10 of 72 -->

Cblox

NEO-F10N - Integration manual

## 2.1 Basic receiver configuration

This section summarizes the most commonly used, basic receiver configurations.

## 2.1.1 Basic hardware configuration

The NEO-F10N receiver is preconfigured in module production and is fully operational after connecting a proper power supply, the communication interfaces with the host application device, and a suitable antenna signal.

## 2.1.2 Internal LNA mode configuration

u-blox 10 receivers feature an internal low-noise amplifier (LNA) with three operational modes: normal gain, low gain and bypass mode. The NEO-F10N default is the normal mode. With a highgain external active antenna, use the low gain or the bypass mode to save power.

The internal LNA mode can be configured at run time in the flash, BBR and RAM memory using the configuration item CFG-HW-RF\_LNA\_MODE and applying a suitable software reset by sending a UBX-CFG-RST message. The suitable software reset type depends on the configured memory layer. For more information, refer to Forcing receiver reset.

The default gain mode is pre-configured in the receiver and does not require configuration in production. The configuration strings for setting the internal LNA mode in the flash memory is given in Table 3.

| Internal LNA mode | Configuration string |
| --- | --- |
| Normal gain (default) | B5 62 06 8A 09 00 01 04 00 00 57 00 A3 20 00 B8 2E |
| Low gain | B5 62 06 8A 09 00 01 04 00 00 57 00 A3 20 01 B9 2F |
| Bypass | B5 62 06 8A 09 00 01 04 00 00 57 00 A3 20 02 BA 30 |

Table 3: Internal LNA mode configuration in the flash memory

Configuring the internal LNA mode in the flash memory:

**1.** Power up the system.

**2.** Test the communication interface by polling the UBX-MON-VER message.

**3.** Send the desired configuration string in Table 3.

**4.** Power cycle the receiver or apply a hardware reset by sending a UBX-CFG-RST message. The configured internal LNA setting is applied at startup.

**5.** Verify that the configuration item is correctly set by polling CFG-HW-RF\_LNA\_MODE at RAM layer using the UBX-CFG-VALGET message.

## 2.1.3 GNSS signal configuration

NEO-F10N supports reception of GPS, Galileo, BeiDou and QZSS L1/L5 dual-band signals plus NavIC L5 and SBAS L1. The default configuration is concurrent reception of GPS (L1C/A, L5), Galileo (E1- B/C, E5a) and BeiDou (B1C, B2a) with SBAS enabled.

Each GNSS constellation can be enabled or disabled independently except for QZSS and NAVIC L5, which is functional only with GPS. In addition to the configuration key for each constellation, there is a configuration key for each signal supported by the firmware. For constellations with dual-band support, it is not possible to disable one of the bands. Both bands must be enabled and disabled at once. Alternatively, the CFG-SIGNAL-\* configuration group for that constellation can be enabled and disabled. For example, if CFG-SIGNAL-GPS\_ENA is set to zero, the GPS constellation is disabled.

Unsupported combinations are rejected with a UBX-ACK-NAK message, and the warning "inv sig cfg" is sent via UBX-INF and NMEA-TXT messages (if enabled).

UBXDOC-963802114-12193 - R04 C1-Public

2 Receiver configuration

Page 10 of 72

<!-- page 11 of 72 -->

Cblox

NEO-F10N - Integration manual

Any change to the signal configuration items triggers a restart of the GNSS subsystem. During the restart, the host application should wait for message acknowledgement and a margin of 0.5 seconds prior to sending any further commands.

In Japan and surrounding regions, it is recommended to enable QZSS when GPS is enabled to mitigate possible cross-correlation issues between the signals.

For more information on the CFG-SIGNAL-\* configuration group, refer to the Interface description [3].

## 2.1.4 GPS L5 signal health status configuration

NEO-F10N supports both GPS L1 C/A and L5 signals. Broadcasting of Civil Navigation (CNAV) messages on the L5 signal began in April 2014. At the time of writing, GPS L5 signals remain [pre-operational](https://www.gps.gov/systems/gps/modernization/civilsignals/) which are set unhealthy until sufficient monitoring capability is established.

To evaluate GPS L5 signals before they become fully operational, the receiver can be configured to ignore the GPS L5 health status by overriding it with the respective GPS L1 C/A signal status.

4 Do not use unhealthy, pre-operational GPS L5 signals for safety-of-life or other critical purposes. This is an operational issue concerning the satellites / space segment and not a limitation or specific configuration of u-blox products.

To ignore the GPS L5 signal health status and override it with the respective GPS L1 signal health status, send the configuration string given in Table 4. The configuration can be stored in RAM, battery-backed RAM (BBR), and flash layers. Stored in the RAM layer, the device returns the UBX-ACK-ACK message if the configuration is sent successfully and it is applied immediately without a configuration reset. To apply the configuration stored in the BBR and flash layers, send the UBX-CFG-RST message with resetMode 0x01.

To revert back to the default configuration, send the configuration string given in Table 5. The device returns the UBX-ACK-ACK message if the configuration is sent successfully and it is applied immediately without a reset in the RAM layer. To apply the configuration stored in the BBR and flash layers, send the UBX-CFG-RST message with resetMode 0x01.

4 Customers who choose to ignore the GPS L5 signal health status in their production system do so at their own risk and must be fully aware of the implications. The system should also include a mechanism to revert to the mode where the L5 signal health status is respected.

| Configuration layer | Configuration string |
| --- | --- |
| RAM | B5 62 06 8A 09 00 01 01 00 00 01 00 32 10 01 DF F6 |
| BBR | B5 62 06 8A 09 00 01 02 00 00 01 00 32 10 01 E0 FE |
| FLASH | B5 62 06 8A 09 00 01 04 00 00 01 00 32 10 01 E2 0E |

Table 4: UBX binary string to override GPS L5 signal health status with GPS L1 health status

| Configuration layer | Configuration string |
| --- | --- |
| RAM | B5 62 06 8A 09 00 01 01 00 00 01 00 32 10 00 DE F5 |
| BBR | B5 62 06 8A 09 00 01 02 00 00 01 00 32 10 00 DF FD |
| FLASH | B5 62 06 8A 09 00 01 04 00 00 01 00 32 10 00 E1 0D |

Table 5: UBX binary strings to revert the GPS L5 signal health status monitoring to default

## 2.1.5 Communication interface configuration

Several configuration groups allow configuring the operation mode of the communication interfaces. These include parameters for the data framing, transfer rate and enabled input/output

UBXDOC-963802114-12193 - R04 C1-Public

2 Receiver configuration

Page 11 of 72

<!-- page 12 of 72 -->

Cblox

NEO-F10N - Integration manual

protocols. See Communication interfaces and PIOs section for details. The configuration groups available for each interface are:

<table><tr><td>Interface</td><td>Configuration groups</td></tr><tr><td rowspan="3">UART</td><td>CFG-UART1-*</td></tr><tr><td>CFG-UART1INPROT-*</td></tr><tr><td>CFG-UART1OUTPROT-*</td></tr></table>

Table 6: Interface configuration

## 2.1.6 Message output configuration

The receiver supports two protocols for output messages: industry-standard NMEA and u-blox UBX. Any message type can be enabled or disabled individually and the output rate is configurable.

The message output rate is related to the frequency of an event. For example, the output message UBX-NAV-PVT (position, velocity, and time solution) is related to the navigation event, which generates a navigation epoch. In this case, the rate for each navigation epoch is defined by the configuration keys CFG-RATE-MEAS and CFG-RATE-NAV. For example, a value of 1000 ms in CFG-RATE-MEAS indicates that a measurement is done every second. If CFG-RATE-NAV is set to one (1), the solution is calculated for every measurement. This means that a navigation epoch is calculated every 1000 ms. If the rate is set to two (2), only the second measurement is used and the navigation epoch is calculated every two seconds. The same result is obtained if CFG-RATE-MEAS is set to 2000 ms, and CFG-RATE-NAV is set to one (1). Every 2000 ms a measurement is done, and in every measurement, a navigation epoch is calculated. However, this second option demands fewer resources and is the correct procedure when the navigation rate is changed. Setting a navigation rate value higher than one (1) is only needed when it is required that the raw measurement data is output at a higher rate than the navigation data.

The output rate for each message is defined in the CFG-MSGOUT-\* configuration group. If the output rate of the message is set to one (1) on the UART interface, CFG-MSGOUT-UBX\_NAV\_PVT\_UART1 = 1, the message is output for every navigation epoch. If the rate is set to two (2), the message is output for every other navigation epoch. If the rate is zero (0), then corresponding message is not output. As seen in this example, the rates of the output messages are individually configurable per communication interface.

Some messages, such as UBX-MON-VER, are non-periodic and are only output as an answer to a poll request.

The UBX-INF-\* and NMEA-Standard-TXT information messages are non-periodic output messages that do not have a message rate configuration. Instead they can be enabled for each communication interface via the CFG-INFMSG-\* configuration group.

All message output is additionally subject to the protocol configuration of the communication interfaces. Messages of a given protocol are not output unless the protocol is enabled for output on the interface. See Communication interface configuration for details.

UBXDOC-963802114-12193 - R04 C1-Public

2 Receiver configuration

Page 12 of 72

<!-- page 13 of 72 -->

Cblox

NEO-F10N - Integration manual

## 2.2 Navigation configuration

This section presents various configuration options related to the navigation engine. These options can be configured through CFG-NAVSPG-\* configuration keys.

## 2.2.1 Dynamic platform

The dynamic platform model can be configured through the CFG-NAVSPG-DYNMODEL configuration item. For the supported dynamic platform models and their details, see Table 7 and Table 8.

| Platform | Description |
| --- | --- |
| Portable | Applications with low acceleration, e.g. portable devices. Suitable for most situations. |
| Stationary | Used in timing applications (antenna must be stationary) or other stationary applications. Velocity restricted to 0 m/s. Zero dynamics assumed. |
| Pedestrian | Applications with low acceleration and speed, e.g. how a pedestrian would move. Low acceleration assumed. |
| Automotive | Used for applications with equivalent dynamics to those of a passenger car. Low vertical acceleration assumed. |
| At sea | Recommended for applications at sea, with zero vertical velocity. Zero vertical velocity assumed. Sea level assumed. |
| Airborne &lt;1g | Used for applications with a higher dynamic range and greater vertical acceleration than a passenger car. No 2D position fixes supported. |
| Airborne &lt;2g | Recommended for typical airborne environments. No 2D position fixes supported. |
| Airborne &lt;4g | Only recommended for extremely dynamic environments. No 2D position fixes supported. |
| Wrist | Only recommended for wrist-worn applications. Receiver will filter out arm motion. |

| Platform | Max altitude [m] | Max horizontal velocity [m/s] | Max vertical [m/s] | velocity Sanity check type | Max position deviation |
| --- | --- | --- | --- | --- | --- |
| Portable | 12000 | 310 | 50 | Altitude and velocity | Medium |
| Stationary | 9000 | 10 | 6 | Altitude and velocity | Small |
| Pedestrian | 9000 | 30 | 20 | Altitude and velocity | Small |
| Automotive | 6000 | 100 | 15 | Altitude and velocity | Medium |
| At sea | 500 | 25 | 5 | Altitude and velocity | Medium |
| Airborne &lt;1g | 80000 | 100 | 6400 | Altitude | Large |
| Airborne &lt;2g | 80000 | 250 | 10000 | Altitude | Large |
| Airborne &lt;4g | 80000 | 500 | 20000 | Altitude | Large |
| Wrist | 9000 | 30 | 20 | Altitude and velocity | Medium |

Table 8: Dynamic platform model details

Applying dynamic platform models designed for high acceleration systems (e.g. airborne <2g) can result in a higher standard deviation in the reported position.

If a sanity check against the limit of the dynamic platform model fails, the position solution becomes invalid. Table 8 shows the types of sanity checks which are applied for a particular dynamic platform model.

## 2.2.2 Navigation input filters

The navigation input filters in the CFG-NAVSPG-\* configuration group control how the navigation engine handles the input data that comes from the satellite signal.

UBXDOC-963802114-12193 - R04 C1-Public

2 Receiver configuration

Page 13 of 72

<!-- page 14 of 72 -->

Cblox

NEO-F10N - Integration manual

| Configuration item | Description |
| --- | --- |
| CFG-NAVSPG-FIXMODE | By default, the receiver calculates a 3D position fix if possible but it reverts to 2D position if necessary (auto 2D/3D). The receiver can be configured to only calculate 2D (2D only) or 3D (3D only) positions. |
| CFG-NAVSPG-CONSTR_ALT, CFG- | The fixed altitude is used if fixMode is set to 2D only. A variance greater than zero |
| NAVSPG-CONSTR_ALTVAR | must also be supplied. |
| CFG-NAVSPG-INFIL_MINELEV | Minimum elevation of a satellite above the horizon to be used in the navigation solution. Low-elevation satellites may provide degraded accuracy, due to the long signal path through the atmosphere. |
| CFG-NAVSPG-INFIL_MINSVS, CFG- | Minimum and maximum number of satellites to use in the navigation solution. |
| NAVSPG-INFIL_MAXSVS | There is an absolute maximum limit of 32 satellites that can be used for navigation. |
| CFG-NAVSPG-INFIL_NCNOTHRS, | A navigation solution will only be attempted if there is at least the given number of |
| CFG-NAVSPG-INFIL_CNOTHRS | satellites with signals at least as strong as the given threshold. |

Table 9: Navigation input filter parameters

If the receiver has only three satellites for calculating a position, the navigation algorithm uses a constant altitude to compensate for the missing fourth satellite. This is called a 2D fix. The constant altitude value is taken from the last successful 3D fix using a minimum of four available satellites.

![Image block](doc:7d59532/tier:standard/page:14/block:6)

u-blox receivers do not calculate any navigation solution with fewer than three satellites.

## 2.2.3 Navigation output filters

The result of a navigation solution is initially classified by the fix type (as detailed in the fixType field of the UBX-NAV-PVT message). This distinguishes between failures to obtain a fix ("No Fix") and cases where a fix has been achieved, which are further subdivided into specific types of fixes (for example, 2D, 3D).

Where a fix has been achieved, the fix is checked to determine whether it is valid or not. A fix is only valid if it passes the navigation output filters as defined in CFG-NAVSPG-OUTFIL. In particular, both PDOP and accuracy values must be below the respective limits.

![Image block](doc:7d59532/tier:standard/page:14/block:11)

Important: Users are recommended to check the gnssFixOK flag in the UBX-NAV-PVT or the NMEA valid flag. Fixes not marked as valid should not be used.

UBX-NAV-STATUS message also reports whether a fix is valid in the gpsFixOK flag. These messages have only been retained for backwards compatibility and it is recommended to use the UBX-NAV-PVT message.

## 2.2.4 Odometer filters

## 2.2.4.1 Speed (3D) low-pass filter

The CFG-ODO-OUTLPVEL configuration item activates a speed (3D) low-pass filter. The output of the speed low-pass filter is available in the UBX-NAV-VELNED message (speed field). The filtering level can be set via the CFG-ODO-VELLPGAIN configuration item and must be between 0 (heavy lowpass filtering) and 255 (weak low-pass filtering).

![Image block](doc:7d59532/tier:standard/page:14/block:17)

The internal filter gain is computed as a function of speed. Therefore, the level defines the nominal filtering level for speeds below 5 m/s, as defined in the CFG-ODO-VELLPGAIN configuration item.

## 2.2.4.2 Course over ground low-pass filter

The CFG-ODO-OUTLPCOG configuration item activates a course over ground low-pass filter when the speed is below 8 m/s. The output of the course over ground (also named heading of motion 2D) low-pass filter is available in the UBX-NAV-PVT message (headMot field), UBX-NAV-VELNED

UBXDOC-963802114-12193 - R04 C1-Public

2 Receiver configuration

Page 14 of 72

<!-- page 15 of 72 -->

Cblox

NEO-F10N - Integration manual

message (heading field), NMEA-RMC message (cog field), and NMEA-VTG message (cogt field). The filtering level can be set via the CFG-ODO-COGLPGAIN configuration item and must be between 0 (heavy low-pass filtering) and 255 (weak low-pass filtering).

![Image block](doc:7d59532/tier:standard/page:15/block:4)

The filtering level defines the filter gain for speeds below 8 m/s, as defined in the CFG-ODO-COGLPGAIN configuration item. If the speed is 8 m/s or higher, no course over ground low-pass filtering is performed.

## 2.2.4.3 Low-speed course over ground filter

The CFG-ODO-USE\_COG configuration item activates this feature and the CFG-ODO-COGMAXSPEED, CFG-ODO-COGMAXPOSACC configuration items are used to configure a lowspeed course over ground filter (also named heading of motion 2D). This filter derives the course over ground from position at very low speed. The output of the low-speed course over ground filter is available in the UBX-NAV-PVT message (headMot field), UBX-NAV-VELNED message (heading field), NMEA-RMC message (cog field) and NMEA-VTG message (cogt field). If the low-speed course over ground filter is not configured, then the course over ground is computed as described in section Freezing the course over ground.

## 2.2.5 Static hold

The static hold mode allows the navigation algorithms to decrease the noise in the position output when the velocity is below a predefined "Static Hold Threshold" level. This reduces the position wander caused by environmental factors such as multi-path and improves position accuracy especially in stationary applications. By default, the static hold mode is disabled.

The CFG-MOT-GNSSSPEED\_THRS configuration item defines the static hold speed threshold. If the speed drops below the defined "Static Hold Threshold", static hold mode is activated. Once static hold mode is active, the position output is kept static and the velocity is set to zero until there is evidence of movement again. Such evidence can be velocity, acceleration, changes of the valid flag (for example, position accuracy estimate exceeding the position accuracy mask, see also section Navigation output filters), position displacement, etc.

The CFG-MOT-GNSSDIST\_THRS configuration item defines the static hold distance threshold. If the distance between the estimated position and the static hold position exceeds the defined threshold, the static hold mode is suspended or deactivated until there is evidence of no movement.

![Image block](doc:7d59532/tier:standard/page:15/block:12)

Figure 3: Position output in static hold mode

UBXDOC-963802114-12193 - R04 C1-Public

2 Receiver configuration

Page 15 of 72

<!-- page 16 of 72 -->

Cblox

NEO-F10N - Integration manual

![Image block](doc:7d59532/tier:standard/page:16/block:3)

Figure 4: Flowchart of static hold mode

## 2.2.6 Freezing the course over ground

If the low-speed course over ground filter is deactivated or inactive (see section Low-speed course over ground filter), the receiver derives the course over ground from the GNSS velocity information. If the velocity cannot be calculated with sufficient accuracy (for example, with bad signals) or if the absolute speed value is very low (under 0.1 m/s), the course over ground value becomes inaccurate too. In this case the course over ground value is frozen, that is, the previous value is kept and its accuracy degrades over time. These frozen values will not be output in the NMEA messages NMEA-RMC and NMEA-VTG unless the NMEA protocol is explicitly configured to do so (see NMEA protocol configuration in the applicable interface description [3]).

UBXDOC-963802114-12193 - R04 C1-Public

2 Receiver configuration

Page 16 of 72

<!-- page 17 of 72 -->

Cblox

NEO-F10N - Integration manual

![Image block](doc:7d59532/tier:standard/page:17/block:3)

Figure 5: Flowchart of course over ground freezing

UBXDOC-963802114-12193 - R04 C1-Public

2 Receiver configuration

Page 17 of 72

<!-- page 18 of 72 -->

Cblox

NEO-F10N - Integration manual

## 3 Receiver functionality

This chapter describes the NEO-F10N operational features and their configuration.

## 3.1 Augmentation systems

## 3.1.1 SBAS

NEO-F10N is capable of receiving multiple Satellite Based Augmentation System (SBAS) signals concurrently, even from different SBAS systems (WAAS, EGNOS, etc.). SBAS signals are recommended to be used only for correction data. These signals can also be used for navigation, but with their low weighting, they only have a minor impact on the navigation solution.

![Image block](doc:7d59532/tier:standard/page:18/block:8)

SBAS is recommended to use, as the accuracy of an L1/L5 receiver is highly affected by ionospheric delays.

For receiving correction data, NEO-F10N automatically chooses the best SBAS satellite as its primary source. It selects only one satellite since the information received from other SBAS satellites is redundant and could be inconsistent. The selection strategy is determined by the proximity of the satellites, the services offered by the satellite, the configuration of the receiver (test mode allowed/ disallowed, integrity enabled/disabled) and the signal link quality to the satellite.

If corrections are available from the chosen SBAS satellite and used in the navigation solution, the differential correction status is indicated in several output messages such as UBX-NAV-PVT, UBX-NAV-STATUS, UBX-NAV-SAT, UBX-NAV-SIG, NMEA-GGA, NMEA-GLL, NMEA-RMC, and NMEA-GNS. The UBX-NAV-SBAS message provides detailed information about the corrections available and applied. Refer to the Interface description [3] for a detailed description of the messages.

The most important SBAS feature for accuracy improvement is the ionosphere correction parameters. The measured data from regional Ranging and Integrity Monitoring Stations (RIMS) are combined to make a Total Electron Content (TEC) map. This map is transferred to the receiver via SBAS satellites to allow a correction of the ionosphere delays on each received signal.

| Message type | Message content | Source |
| --- | --- | --- |
| 0(0/2) | Test mode | All |
| 1 | PRN mask assignment | Primary |
| 2, 3, 4, 5 | Fast corrections | Primary |
| 6 | Integrity | Primary |
| 7 | Fast correction degradation | Primary |
| 9 | Satellite navigation (ephemeris) | All |
| 10 | Degradation | Primary |
| 12 | Time offset | Primary |
| 17 | Satellite almanac | All |
| 18 | Ionosphere grid point assignment | Primary |
| 24 | Mixed fast/long-term corrections | Primary |
| 25 | Long-term corrections | Primary |
| 26 | Ionosphere delays | Primary |

Table 10: Supported SBAS messages

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 18 of 72

<!-- page 19 of 72 -->

Cblox

NEO-F10N - Integration manual

Each satellite serves a specific region and its correction signal is only useful within that region. Planning is crucial to determine the best possible configuration, especially in areas where signals from different SBAS systems can be received:

**Example 1 - SBAS receiver in North America:** In eastern parts of North America, make sure that EGNOS satellites do not take preference over WAAS satellites. The satellite signals from the EGNOS system should be disallowed by using the PRN scan mask (configuration key CFG-SBAS-PRNSCANMASK).

**Example 2 - SBAS receiver in Europe:** Some WAAS satellite signals can be received in some parts of western Europe and GAGAN SBAS satellites in other parts of Europe. Therefore, it is recommended that satellites from all but the EGNOS system are disallowed using the PRN scan mask.

Although u-blox receivers try to select the best available SBAS correction data, it is recommended to configure them to exclude the unwanted SBAS satellites.

To configure the SBAS functionality, use the CFG-SBAS-\* configuration group.

| Parameter | Description |
| --- | --- |
| CFG-SIGNAL-SBAS_ENA | Enabled/disabled status of the SBAS subsystem |
| CFG-SBAS-USE_TESTMODE | Allow/disallow SBAS usage from satellites in test mode (enable when BDSBAS is used) |
| CFG-SBAS-USE_RANGING | Use the SBAS satellites for navigation (ranging) |
| CFG-SBAS-USE_DIFFCORR | Combined enable/disable switch for fast, long-term, and ionosphere corrections |
| CFG-SBAS-USE_INTEGRITY | Apply integrity information data |
| CFG-SBAS-PRNSCANMASK | Allows selectively enabling/disabling SBAS satellites (BDSBAS disabled by default) |

Table 11: SBAS configuration parameters

When SBAS integrity data is applied, the navigation engine stops using all signals for which no integrity data is available (including all non-GPS signals). It is not recommended to enable SBAS integrity on borders of SBAS service regions in order not to inadvertently restrict the number of available signals.

SBAS integrity information is required for at least five GPS satellites. If this condition is not met, SBAS integrity data will not be applied.

G When the receiver switches from a solution using correction data to a standard position solution, the reference frame of the output position switches as well. For an SBAS solution, the reference frame is aligned within a few centimeters of WGS84 (and modern ITRF realizations).

Other SBAS systems (such as KASS) can be enabled by adding the corresponding PRNs to the CFG-SBAS-PRNSCANMASK list.

## 3.1.1.1 BeiDou SBAS configuration

BeiDou satellite based augmentation system (BDSBAS) provides SBAS services to China and surrounding regions. BDSBAS is integrated in the BeiDou system and uses BDS-3 type satellites to broadcast SBAS L1/L5 signal, providing augmentation for BeiDou and GPS systems.

BeiDou SBAS is in testing mode and it is not enabled by default. Enabling Beidou SBAS may improve navigation solution by its correction data. To enable the BeiDou SBAS functionality, configure the following configuration items.

| Configuration item | Value |
| --- | --- |
| CFG-SBAS-ACCEPT_NOT_IN_PRNMASK | BDSBAS |
| CFG-SBAS-PRNSCANMASK | 0x0000000001800400 |
| CFG-SBAS-USE_DIFFCORR | TRUE (default) |

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 19 of 72

<!-- page 20 of 72 -->

Cblox

NEO-F10N - Integration manual

| Configuration item | Value |
| --- | --- |
| CFG-SBAS-USE_INTEGRITY | FALSE (default) |
| CFG-SBAS-USE_IONOONLY | FALSE (default) |
| CFG-SBAS-USE_RANGING | FALSE |
| CFG-SBAS-USE_TESTMODE | TRUE |

Table 12: BeiDou SBAS configuration item

BeiDou SBAS is still in testing mode and it has not been officially released for operational use. Do not use it for safety related applications.

The CFG-SBAS-PRNSCANMASK in Table 12 includes only BeiDou SBAS PRNs. Alternatively, the BeiDou SBAS PRNs can be added to the default list of PRNs enabled in the firmware or modified list of PRNs that is configured. This allows other SBAS systems to be used if BeiDou SBAS is not available.

## 3.1.2 QZSS SLAS

QZSS SLAS (Sub-meter Level Augmentation Service) is an augmentation technology, which provides correction data for pseudoranges of GPS, QZSS, and other major GNSS satellites. The correction stream is transmitted on the L1S signal at the L1 frequency (1575.42 MHz).

For more information on QZSS SLAS, visit [qzss.go.jp/en/](http://qzss.go.jp/en/).

Multiple QZSS SLAS signals can be received simultaneously. When receiving QZSS SLAS correction data, NEO-F10N will autonomously select the best QZSS satellite. The selection strategy is determined by the quality of the QZSS L1S signals, the receiver configuration (test mode allowed or not), and the location of the receiver with respect to the QZSS SLAS coverage area. When outside of this coverage area, the receiver will likely fall back to using SBAS corrections.

If QZSS SLAS corrections are used in the navigation solution, the differential status will be indicated in several output messages such as UBX-NAV-PVT, UBX-NAV-STATUS, UBX-NAV-SAT, NMEA-GGA, NMEA-GLL, NMEA-RMC, and NMEA-GNS. The UBX-NAV-SLAS message provides detailed information about which corrections are available and applied. Refer to the interface description [3] for a detailed description of the messages.

| Message type | Message content |
| --- | --- |
| 0 | Test mode |
| 47 | Monitoring station information |
| 48 | PRN mask |
| 49 | Data issue number |
| 50 | DGPS correction |
| 51 | Satellite health |

Table 13: Supported QZSS L1S SLAS messages for navigation enhancement

Use the configuration key CFG-SIGNAL-QZSS\_L1S\_ENA to enable QZSS L1S signal. For further QZSS SLAS functionality, use the CFG-QZSS-USE\_SLAS\* configuration keys.

| Parameter | Description |
| --- | --- |
| CFG-QZSS-USE_SLAS_DGNSS | Apply QZSS SLAS corrections |
| CFG-QZSS-USE_SLAS_TESTMODE | Allow the correction provided by QZSS satellites that are in test mode |

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 20 of 72

<!-- page 21 of 72 -->

Cblox

NEO-F10N - Integration manual

| Parameter | Description |
| --- | --- |
| CFG-QZSS-USE_SLAS_RAIM_UNCORR | If this configuration is set, the receiver will try to estimate the position by using only corrected measurements; if all corrected measurements are not available, it will not use any corrections. If this configuration is not set, the receiver will mix corrected and uncorrected measurements for the navigation solution. |

Table 14: QZSS SLAS configuration parameters

![Image block](doc:7d59532/tier:standard/page:21/block:5)

If the RAIM option is set, QZSS is the only GNSS time system that measurements can observe.

## 3.2 Communication interfaces and PIOs

NEO-F10N supports communication over UART and I2C interfaces for communication with a host CPU. UBX and NMEA protocols can be enabled simultaneously with individual interface settings, e.g. for baud rate, message rates, and so on.

## 3.2.1 UART

NEO-F10N supports a Universal Asynchronous Receiver/Transmitter (UART) port consisting of an RX and a TX line. The UART can be used as a host interface which supports a configurable baud rate and protocol selection.

![Image block](doc:7d59532/tier:standard/page:21/block:11)

心 The UART interface does not support handshaking signals or hardware flow control signals.

The UART baud rate can be configured for selected speeds. Different rates than these speeds are not supported for transmission and reception.

The UART RX interface is disabled when more than 100 frame errors are detected during a onesecond period. This can happen if the wrong baud rate is used or the UART RX pin is grounded. An error message appears when the UART RX interface is re-enabled at the end of the one-second period.

| Baud rate | Data bits | Parity | Stop bits |
| --- | --- | --- | --- |
| 9600 | 8 | none | 1 |
| 19200 | 8 | none | 1 |
| 38400 | 8 | none | 1 |
| 57600 | 8 | none | 1 |
| 115200 | 8 | none | 1 |
| 230400 | 8 | none | 1 |
| 460800 | 8 | none | 1 |
| 921600 | 8 | none | 1 |

Table 15: Possible UART interface configurations

Allow a short time delay of typically 100 ms between sending a baud rate change message and providing input data at the new rate. Otherwise some input characters may be ignored or the port could be disabled until the interface is able to process the new baud rate.

![Image block](doc:7d59532/tier:standard/page:21/block:18)

The default baud rate is 38400 baud. Using a lower baud rate may cause buffering problems.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 21 of 72

<!-- page 22 of 72 -->

Cblox

NEO-F10N - Integration manual

If there is too much data for the interface's bandwidth, the output buffer will fill up. Once the buffer space is exceeded, new messages to be sent will be dropped. To prevent message loss, the baud rate and the number of enabled messages should be selected carefully.

## 3.2.2 PIOs

This section describes the PIOs supported by NEO-F10N. All PIO active voltage levels are related to the VCC supply voltage. All the inputs have internal pull-up resistors in normal operation and can be left open if unused.

![Image block](doc:7d59532/tier:standard/page:22/block:6)

When assigning a different function to a PIO, ensure that the default function is disabled where applicable. For example, disable the I2C interface with the CFG-I2C-ENABLED configuration key if I2C pins are used for antenna supervisor functions.

## 3.2.2.1 RESET\_N

NEO-F10N provides a RESET\_N pin to reset the receiver. The RESET\_N pin is input-only with an internal pull-up resistor to VCC and should be left open for normal operation. Driving RESET\_N low for at least 1 ms triggers a receiver reset. The RESET\_N complies with the VCC level and can be actively driven high.

心 Use RESET\_N only in critical situations to recover the receiver. RESET\_N resets the receiver and clears the BBR content including receiver configuration, real-time clock (RTC), and GNSS orbit data, triggering a cold start.

No capacitor should be placed at RESET\_N to GND, otherwise it could trigger a reset on every startup.

## 3.2.2.2 SAFEBOOT\_N

The SAFEBOOT\_N pin is for future service, updates and reconfiguration.

![Image block](doc:7d59532/tier:standard/page:22/block:14)

The SAFEBOOT\_N pin is internally connected to the TIMEPULSE pin through a 1 kΩ series resistor.

## 3.2.2.3 TIMEPULSE

The NEO-F10N features one time pulse output at the TIMEPULSE pin. This can only be configured in PIO4. The details about this feature are explained in the section Time pulse.

![Image block](doc:7d59532/tier:standard/page:22/block:18)

The TIMEPULSE and SAFEBOOT\_N functions share the same internal IC function. If this pin is low at receiver startup, the receiver will enter safeboot mode. However, in normal operation the pin outputs the time pulse signal. Make sure this pin has no load that could pull it low at startup.

## 3.2.2.4 LNA\_EN

The LNA\_EN signal can be used to turn on and off an optional external LNA and an active antenna supply to optimize the power consumption in the backup modes. The LNA and the active antenna supply are turned on when the LNA\_EN signal is "high".

The LNA\_EN signal is also used internally in NEO-F10N to control the integrated LNAs. The polarity cannot be changed.

## 3.2.2.5 EXTINT

NEO-F10N supports external interrupts at the EXTINT pin. The EXTINT pin has a fixed input voltage threshold with respect to VCC.

The EXTINT pin can also be configured for another functionality.

![Image block](doc:7d59532/tier:standard/page:22/block:26)

EXTINT functionality is only available at the EXTINT pin.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 22 of 72

<!-- page 23 of 72 -->

Cblox

NEO-F10N - Integration manual

## 3.3 Forcing receiver reset

GNSS receivers typically make a distinction between cold, warm, and hot start based on the type of valid information the receiver has during the restart.

**Cold start:** in the cold start mode, the receiver has no information from the last position (e.g. time, velocity, frequency etc.) at startup. Therefore, the receiver must search the full time and frequency space, and all possible satellite numbers. If a satellite signal is found, it is tracked to decode the ephemeris (18-36 seconds under strong signal conditions), while the other channels continue to search satellites. Once there is a sufficient number of satellites with valid ephemeris, the receiver can calculate position and velocity data. Other GNSS receiver manufacturers call this the Factory startup mode.

**Warm start:** in the warm start mode, the receiver has approximate information for time, position, and coarse satellite position data (Almanac). In this mode, the receiver normally needs to download ephemeris after power-up before it can calculate position and velocity data. As the ephemeris data is usually outdated after 4 hours, the receiver typically starts with a warm start if it has been powered down for more than 4 hours. In this scenario, several augmentations are possible. See Multiple GNSS assistance.

**Hot start:** in the hot start mode, the receiver has been powered down only for a short time (4 hours or less), so that its ephemeris is still valid. Since the receiver does not need to download ephemeris again, this is the fastest startup method.

Using the UBX-CFG-RST message, you can force the receiver to reset and clear data, in order to see the effects of maintaining/losing such data between restarts. For this purpose, use the navBbrMask field in the UBX-CFG-RST message to initiate hot, warm, and cold starts, or a combination of startup modes.

The reset type can also be specified. This is not related to GNSS, but to the way the software restarts the system.

• **Hardware reset** uses the on-chip watchdog to electrically reset the chip. This is an immediate asynchronous reset. No stop events are generated.

**Controlled software reset** terminates all running processes in an orderly manner. Once the system is idle, restarts the receiver operation, reloads its configuration and starts to acquire and track GNSS satellites.

• **Controlled software reset (GNSS only)** only restarts the GNSS tasks, without reinitializing the full system or reloading any stored configuration.

• **Hardware reset (after shutdown)** uses the on-chip watchdog to reset the receiver after shutdown.

• **Controlled GNSS stop** stops all GNSS tasks. The receiver is not restarted, but stops any GNSS-related processing.

• **Controlled GNSS start** starts all GNSS tasks.

Table 16 below contains an overview of the different reset types and the data that is cleared.

| Reset type | Clears RAM | Clears BBR | Clears Flash |
| --- | --- | --- | --- |
| 0x00 - Hardware reset (immediately), | Yes | Yes | No |
| 0x04 - Hardware reset (after shutdown) |  |  |  |
| 0x01 - Controlled Software reset | Yes | No | No |
| 0x02 - Controlled Software reset (GNSS only), | No | No | No |
| 0x08 - Controlled GNSS stop, |  |  |  |
| 0x09 - Controlled GNSS start |  |  |  |
| RESET_N pin | Yes | Yes | No |

Table 16: Overview of the available reset types

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 23 of 72

<!-- page 24 of 72 -->

Cblox

NEO-F10N - Integration manual

After using any reset type that clears the BBR, the TTFF is similar to performing a cold start.

The code RAM is not cleared when using any of the reset modes in the CFG-RST message, but it is cleared when the **RESET\_N** pin is used to initiate a reset. Hence, the firmware image is reloaded from the flash memory or it needs to be uploaded from the host to the code RAM after the **RESET\_N** pin is pulled low.

## 3.4 Security

The security concept of NEO-F10N covers:

• The integrity of the receiver

• Communication between the receiver and the GNSS satellites

Some security functions monitor and detect threats and report them to the host system. Other functions mitigate threats and allow the receiver to operate normally.

## 3.4.1 GNSS receiver integrity

This section describes receiver security features implemented with the NEO-F10N:

• Secure boot

• Secure firmware update

• Receiver configuration lock

## 3.4.1.1 Secure boot

The NEO-F10N boots only with firmware images that are signed by u-blox. This prevents the execution of non-genuine firmware images on the receiver.

## 3.4.1.2 Secure firmware update

The firmware image is signed by u-blox. The NEO-F10N verifies the signature during the firmware update.

![Image block](doc:7d59532/tier:standard/page:24/block:19)

Firmware update applies to products supporting an external firmware.

## 3.4.1.3 Receiver configuration lock

The receiver configuration lock feature ensures that no configuration changes are possible once the feature is enabled. The configuration lock is enabled by setting the configuration item CFG-SEC-CFG\_LOCK to "true".

The configuration lock can be applied to different configuration layers including the RAM, BBR, and flash memory. At startup, the receiver constructs the configuration database from different configuration layers and maintains it in the run-time RAM memory. When the configuration lock is set in the run-time RAM, the receiver configuration cannot be changed on any configuration layer.

For more information on the configuration layers including the order of priority they are applied in, see the applicable Interface description [3].

The configuration lock set on the RAM or BBR configuration layer is removed when the memory is cleared. However, the configuration lock set in non-volatile memory (flash memory) is permanent unless the flash memory is erased. Flash memory is also erased during the firmware update, causing the configuration lock to be cleared. Refer to Firmware update for more information on firmware update.

To test the lock functionality, set it on the RAM configuration layer. After a power cycle, the information on RAM layer is cleared and the lock is no longer set.

![Image block](doc:7d59532/tier:standard/page:24/block:27)

It is recommended to apply the configuration lock on the same layer the configuration is stored.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 24 of 72

<!-- page 25 of 72 -->

Cblox

NEO-F10N - Integration manual

An example of use case is that the host application locks the receiver configuration. A user communicating with the NEO-F10N through any of the available interfaces can poll, enable or send messages, but cannot change the configuration by sending UBX configuration messages.

## 3.4.2 Jamming and spoofing detection

## 3.4.2.1 Jamming and RF interference detection and monitoring

Intentional jamming signals and/or unintentional interference generated by nearby electronics can degrade the quality of GNSS signals and the receiver performance. The receiver has two independent mechanisms to detect and report the presence of RF interference or intentional jamming signals: jamming indicator and jamming and interference monitor (ITFM).

## Jamming indicator

The jamming indicator detects narrow-band continuous wave (CW) signals over the configured frequency bands. The status is reported in the UBX-MON-RF message, cwSuppression flag for each frequency band. The value is always relative to the base level reported in an unjammed environment. A significant increase in the jamming indicator value indicates presence of a jamming signal. The jamming indicator is always enabled.

## Jamming and interference monitor (ITFM)

Jamming and interference monitor detects any waveform over the configured frequency bands. The receiver monitors the background noise and looks for significant changes in the spectrum. The monitor status is reported in the UBX-SEC-SIG message, jammingState flag. The monitor is disabled by default.

The monitor is configured with the CFG-ITFM-\* configuration group. The configuration keys are summarized in Table 17.

| Configuration key | Description |
| --- | --- |
| CFG-ITFM-ENABLE | Set to 1 to enable ITFM |
| CFG-ITFM-BBTHRESHOLD | The threshold level for broadband interference detection. The value is given in decibels (dB) above the level of the reference spectrum. |
| CFG-ITFM-CWTHRESHOLD | The threshold level for CW interference detection. The value is given in decibels (dB) above the level of the reference spectrum. |
| CFG-ITFM-ANTSETTING | The type of antenna (active or passive) used in the design. |

Table 17: CFG-ITFM-\* group configuration keys

The receiver measures the reference spectrum at the start-up after obtaining a good fix. Until then, the monitor reports "Unknown". Once the reference spectrum is available, the receiver measures the signal spectrum and compares it against the reference while applying the detection thresholds. The receiver also internally monitors other factors including changes in the average C/N0 level to determine the jamming state. The reported monitor states are summarized in section Messages related to jamming/RF interference and spoofing detection and monitoring.

![Image block](doc:7d59532/tier:standard/page:25/block:15)

It is not recommended to restart the receiver when it is indicating jamming.

## Evaluation

The detection of jamming or RF interference depends both on the type of jamming signal and the signal environment. It may not be always possible to detect jamming or RF interference signals. If the GNSS performance is degraded or the fix is completely lost, jamming or RF interference reported in cwSuppression and/or jammingState is a likely cause.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 25 of 72

<!-- page 26 of 72 -->

Cblox

NEO-F10N - Integration manual

RF interference generated by the device itself or coupled from external sources is common and may be reported by the receiver. If jamming is reported but the C/N0 level and GNSS performance are not affected, the receiver may be able to mitigate the impact of jamming.

The jamming and RF interference detection feature can be evaluated by applying jamming signals relevant for the application and signal environment and observing the receiver behaviour.

## 3.4.2.2 Spoofing detection and monitoring

Spoofing is the process where a counterfeit GNSS signal is transmitted locally to deceive the receiver/user and produce an erroneous position fix and/or time solution. The detection algorithm monitors GNSS signals for implausible changes or inconsistencies. These are evaluated with regards to spoofing.

A detection is successful when a signal is observed to transition from an initially genuine one to a spoofed version. Hence detection is not possible if the receiver is started under spoofing conditions. The detection algorithms also rely on availability of signals from multiple GNSS constellations to improve the spoofing detection capabilities. The spoofing detector is always enabled.

The detection of spoofing requires a transition from initially genuine GNSS signals to the introduction of spoofed signals. Detection is therefore not possible if the spoofing signals are already present when the receiver starts up. Detection is most likely at the time when the spoofing signal is introduced, but it may take some time until spoofing is reported. The spoofing status is reported in the UBX-SEC-SIG message, spoofingState flag. The reported states are summarized in section Messages related to jamming/RF interference and spoofing detection and monitoring.

## It is not recommended to restart the receiver when it is indicating spoofing.

The detection of spoofing signals depends on the type of spoofing but also on the signal environment. It may not always be possible to detect spoofing attacks. However, for some spoofing scenarios the receiver may reject the inconsistent signals from the navigation solution, and in such case the receiver may not report detection of spoofing.

To evaluate the spoofing detection feature, apply spoofing signals relevant for the application and signal environment and observe the receiver behaviour.

## 3.4.2.3 Messages related to jamming/RF interference and spoofing detection and monitoring

The information about jamming/RF interference and spoofing detection is reported in two messages: UBX-SEC-SIG and UBX-SEC-SIGLOG. For further information, see the Interface description [3].

## Signal security status (UBX-SEC-SIG)

The UBX-SEC-SIG message provides information related to the security, that is availability and integrity, of the signals. It also provides high-level jamming/interference and spoofing detector information, a direct method for detecting the current security status at each navigation epoch to alert the host about potential jamming/RF interference or spoofing events.

Fields related to jamming and spoofing states of UBX-SEC-SIG message are described in Table 18.

<table><tr><td>Message fields</td><td>Jamming/spoofing state</td><td>Description</td></tr><tr><td>jamDetEnabled</td><td>0/1</td><td>Flag indicates whether jamming/RF interference detection is enabled or not. If 0, it is disabled and if 1, it is enabled.</td></tr><tr><td rowspan="2">jammingState</td><td>0: Unknown</td><td>Monitor is not enabled, monitor is uninitialized, or the antenna is disconnected</td></tr><tr><td>1: No jamming indicated</td><td>No jamming or RF interference is detected</td></tr></table>

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 26 of 72

<!-- page 27 of 72 -->

Cblox

NEO-F10N - Integration manual

<table><tr><td>Message fields</td><td>Jamming/spoofing state</td><td>Description</td></tr><tr><td rowspan="2"></td><td>2: Warning; jamming indicated but fix OK</td><td>Position OK but jamming or RF interference is visible (above the thresholds)</td></tr><tr><td>3: Critical; jamming indicated and no fix</td><td>No reliable position fix and jamming or RF interference is visible (above the thresholds); jamming/RF interference is a probable reason for no position fix</td></tr><tr><td>spfDetEnabled</td><td>0/1</td><td>Flag indicates whether spoofing detection is enabled or not. If 0, it is disabled and if 1, it is enabled.</td></tr><tr><td rowspan="4">spoofingState</td><td>0: Unknown</td><td>Monitor is not enabled, monitor is uninitialized, or the antenna is disconnected</td></tr><tr><td>1: No spoofing indicated</td><td>No spoofing detector indicates spoofing</td></tr><tr><td>2: Spoofing indicated</td><td>Spoofing detectors indicate spoofing</td></tr><tr><td>3: Spoofing affirmed</td><td>The indicated spoofing has been confirmed</td></tr></table>

Table 18: Fields related to jamming and spoofing states of UBX-SEC-SIG message

Note that the spoofing state value only reflects the detector state for the current navigation epoch. That is, 1: No spoofing indicated does not mean that the receiver is not being spoofed; it simply states that during this epoch, the detector has not not been triggered.

## Signal security logfile (UBX-SEC-SIGLOG)

The UBX-SEC-SIGLOG message provides a log of past events triggered by jamming/interference or spoofing detection.

Each event is a combination of a detection type and an event type, where the event type 'indication started' and 'indication stopped' and also the event type 'indication triggered' and 'indication timeout' form a pair.

A maximum of 16 events are logged and new events take precedence over the past events in the log. Power cycles and restarts of the receiver reset the log, deleting its content.

Fields related to jamming and spoofing detection states of UBX-SEC-SIGLOG message are described in Table 19.

| Message fields Signal security log state | Description |
| --- | --- |
| ttag | Shows the time tag in millisecond. |
| detectionType Type of the spoofing or |  |
| jamming detection: |  |
| 0 = simulated signal | Signal from simulator with changed navigation data |
| 1 = abnormal signal | Not supported |
| 2 = INS/GNSS mismatch | Not supported |
| 3 = abrupt changes in GNSS | Abrupt changes in GNSS signal level and time offset |
| signal |  |
| eventType Type of the event: |  |
| 0 = indication started |  |
| 1 = indication stopped |  |
| 2 = indication triggered |  |
| 3 = indication timed-out |  |

Table 19: Fields related to jamming and spoofing detection states of UBX-SEC-SIGLOG message

Single epoch events, caused by abrupt changes due to switching from the real to the spoofing signal or vice versa are handled as time-out events. This means that the time-out event is reported after a certain cooling-off period which is not related to any observations in the signal. The other detection types make use of 'start' and 'stop' event types.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 27 of 72

<!-- page 28 of 72 -->

Cblox

NEO-F10N - Integration manual

## 3.5 Power management

u-blox receivers support different operating modes. These modes represent strategies of controlling the acquisition and tracking engines to achieve either the best possible performance or good performance with reduced power consumption.

## 3.5.1 Continuous mode

NEO-F10N uses dedicated signal processing engines optimized for signal acquisition and tracking. The acquisition engine actively searches for and acquires signals during cold starts or when insufficient signals are available during navigation. The tracking engine continuously tracks and downloads all the almanac data and acquires new signals as they become available during navigation. The tracking engine consumes less power than the acquisition engine.

The current consumption is lower when a valid position is obtained quickly after the start of the receiver navigation, the entire almanac has been downloaded, and the ephemeris for each satellite in view is valid. If these conditions are not met, the search for the available satellites takes more time and consumes more power.

## 3.5.2 Backup modes

A backup mode is an inactive state where the power consumption is reduced to a fraction of that in operating modes. The receiver maintains time information and navigation data to speed up the receiver restart after backup or standby mode.

NEO-F10N supports the following backup modes: hardware backup mode and software standby mode.

## 3.5.2.1 Hardware backup mode

The hardware backup mode allows entering a backup state and resuming operation by switching the main power supply on and off while maintaining a V\_BCKP supply via, e.g. a battery.

V\_BCKP must be supplied to maintain the backup domain (BBR and RTC) to allow better TTFF, accuracy, availability and power consumption at the next startup compared with a cold start. As VCC is not supplied, the PIOs cannot be driven by an external host processor. If driving of the PIOs cannot be avoided, buffers are required for isolating the PIOs.

## 3.5.2.2 Software standby mode

Software standby mode is entered using the UBX-RXM-PMREQ message. The VCC supply, which must be maintained during standby is switched internally to supply the battery-backed RAM (BBR), RTC, and PIO sections to save power.

Entering the software standby mode clears the RAM memory including the receiver configuration. To maintain the configuration, store it on BBR or flash layers. For more information on permanence of the stored configuration, refer to Receiver configuration.

The software standby mode can be set for a specific duration, or until the receiver is woken up by a signal at a wake-up source defined in UBX-RXM-PMREQ. The possible wake-up sources are UART RX and/or EXTINT pin. Refer to the Interface description [3] for more information on the UBX-RXM-PMREQ message. A system reset with the RESET\_N signal also terminates the software standby mode, clears the BBR content and restarts the receiver.

As VCC is supplied, the PIOs can be driven by an external host processor. No buffers are required for isolating the PIOs, which reduces cost.

The LNA\_EN signal is set to the "LOW" state during the software standby mode.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 28 of 72

<!-- page 29 of 72 -->

Cblox

NEO-F10N - Integration manual

V\_BCKP should be left open if not used.

## 3.6 Time

Maintaining receiver local time and keeping it synchronized with GNSS time is essential for proper timing and positioning functionality. This section explains how the receiver maintains local time and introduces the supported GNSS time bases.

## 3.6.1 Receiver local time

The receiver is dependent on a local oscillator for both the operation of its radio parts and also for timing within its signal processing. No matter what nominal frequency the local oscillator has, u-blox receivers subdivide the oscillator signal to provide a 1-kHz reference clock signal, which is used to drive many of the receiver's processes. In particular, the measurement of satellite signals is arranged to be synchronized with the "ticking" of this 1-kHz clock signal.

When the receiver first starts, it has no information about how these clock ticks relate to other time systems; it can only count time in 1 millisecond steps. However, as the receiver derives information from the satellites it is tracking or from aiding messages, it estimates the time that each 1-kHz clock tick takes in the time base of the chosen GNSS system. This estimate of GNSS time based on the local 1-kHz clock is called receiver local time.

As receiver local time is a mapping of the local 1-kHz reference onto a GNSS time base, it may experience occasional discontinuities, especially when the receiver first starts up and the information it has about the time base is changing. Indeed, after a cold start, the receiver local time initially indicates the length of time that the receiver has been running. However, when the receiver obtains some credible timing information from a satellite or an aiding message, it jumps to an estimate of GNSS time.

## 3.6.2 GNSS time bases

GNSS receivers must handle a variety of different time bases as each GNSS has its own reference system time. What is more, although each GNSS provides a model for converting their system time into UTC, they all support a slightly different variant of UTC. So, for example, GPS supports a variant of UTC as defined by the US National Observatory, while BeiDou uses UTC from the National Time Service Center, China (NTSC). While the different UTC variants are normally closely aligned, they can differ by as much as a few hundreds of nanoseconds.

Although u-blox receivers can combine a variety of different GNSS times internally, the user must choose a single type of GNSS time and, separately, a single type of UTC for input (on EXTINT pins) and output (via the TIMEPULSE pin) and the parameters reported in corresponding messages.

The CFG-TP-TIMEGRID\_TP\* configuration item allows the user to choose between any of the supported GNSS (GPS, GLONASS, BeiDou, etc.) time bases and UTC. Also, the CFG-NAVSPG-UTCSTANDARD configuration item allows the user to select which variant of UTC the receiver should use. This includes an "automatic" option which causes the receiver to select an appropriate UTC version itself, based on the GNSS configuration, using, in order of preference, USNO if GPS is enabled, SU if GLONASS is enabled, NTSC if BeiDou is enabled, NPLI if NAVIC is enabled, NICT when QZSS is enabled, finally, European if Galileo is enabled.

The receiver assumes that an input time pulse uses the same GNSS time base as specified for the time pulse output. So if the user selects GLONASS time for time pulse output, any time pulse input must also be aligned to GLONASS time (or to the separately chosen variant of UTC). Where UTC is selected for time pulse output, any GNSS time pulse input will be assumed to be aligned to GPS time.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 29 of 72

<!-- page 30 of 72 -->

Cblox

NEO-F10N - Integration manual

The receiver allows users to independently choose GNSS signals used in the receiver (using CFG-SIGNAL-\*) and the input/output time base (using CFG-TP-\*). For example it is possible to instruct the receiver to use GPS and GLONASS satellite signals to generate BeiDou time. This practice compromises time pulse accuracy if the receiver cannot measure the timing difference between the constellations directly and is therefore not recommended.

The information that allows GNSS times to be converted to the associated UTC times is only transmitted by the GNSS at relatively infrequent periods. For example GPS transmits UTC(USNO) information only once every 12.5 minutes. Therefore, if a time pulse is configured to use a variant of UTC time, after a cold start, substantial delays before the receiver has sufficient information to start outputting the time pulse can be expected.

Each GNSS has its own time reference for which detailed and reliable information is provided in the messages listed in the table below.

| Time reference | Message |
| --- | --- |
| GPS time | UBX-NAV-TIMEGPS |
| BeiDou time | UBX-NAV-TIMEBDS |
| GLONASS time | UBX-NAV-TIMEGLO |
| Galileo time | UBX-NAV-TIMEGAL |
| NavIC time | UBX-NAV-TIMENAVIC |
| QZSS time | UBX-NAV-TIMEQZSS |
| UTC time | UBX-NAV-TIMEUTC |

Table 20: GNSS time messages

## 3.6.3 Navigation epochs

Each navigation solution is triggered by the tick of the 1 kHz clock nearest to the desired navigation solution time. This tick is referred to as a navigation epoch. If the navigation solution attempt is successful, one of the results is an accurate measurement of time in the time base of the chosen GNSS system, called GNSS system time. The difference between the calculated GNSS system time and receiver local time is called the clock bias (and the clock drift is the rate at which this bias is changing).

In practice the receiver's local oscillator is not as stable as the atomic clocks to which GNSS systems are referenced and consequently clock bias tends to accumulate. However, when selecting the next navigation epoch, the receiver always tries to use the 1 kHz clock tick which it estimates to be closest to the desired fix period as measured in GNSS system time. Consequently, the number of 1 kHz clock ticks between fixes occasionally varies. This means that when producing one fix per second, there are normally 1000 clock ticks between fixes, but sometimes, to correct drift away from the GNSS system time, there are 999 or 1001 ticks.

The GNSS system time calculated in the navigation solution is always converted to a time in both the GPS and UTC time bases for output.

Clearly when the receiver has chosen to use the GPS time base for its GNSS system time, conversion to GPS time requires no work at all, but conversion to UTC requires knowledge of the number of leap seconds since GPS time started (and other minor correction terms). The relevant GPS-to-UTC conversion parameters are transmitted periodically (every 12.5 minutes) by GPS satellites, but can also be supplied to the receiver via the UBX-MGA-GPS-UTC aiding message. By contrast, when the receiver has chosen to use the GLONASS time base as its GNSS system time, conversion to GPS time is more difficult as it requires knowledge of the difference between the two time bases, but as GLONASS time is closely linked to UTC, conversion to UTC is easier.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 30 of 72

<!-- page 31 of 72 -->

Cblox

NEO-F10N - Integration manual

When insufficient information is available for the receiver to perform any of these time base conversions precisely, predefined default offsets are used. Consequently, plausible times are nearly always generated, but they may be wrong by a few seconds (especially shortly after receiver start). Depending on the configuration of the receiver, such "invalid" times may well be output, but with flags indicating their state (e.g. the "valid" flags in UBX-NAV-PVT).

To support multiple GNSS systems concurrently, u-blox receivers employ multiple GNSS system times and/or receiver local times. For reporting GNSS system time or the receiver local time, users are recommended to use messages that report UTC time instead of using UBX messages. Other messages are retained only to support backwards compatibility.

## 3.6.4 iTow timestamps

The original designers of GPS chose to express time/date as an integer week number (starting with the first full week in January 1980) and a time of week (TOW) expressed in seconds. Manipulating time/date in this form is far easier for digital systems than the more conventional year/month/day, hour/minute/second representation. Therefore, most GNSS receivers use this time representation internally, and convert it to a more conventional form at external interfaces. In many UBX messages, the iTOW field provides an externally visible example of the internal time representation.

All the main UBX-NAV messages (and some other messages) contain an iTOW field to indicate the GPS time when the navigation epoch occurred. Messages with the same iTOW value can be assumed to have come from the same navigation solution, and therefore, iTOW can be used to synchronize between these UBX messages. However, the iTOW values may not be valid (i.e., they may have been generated with insufficient conversion data). Therefore, it is not recommended to use the iTOW field for any other purpose.

If reliable absolute time information is required, use the UTC time related fields in the UBX-NAV-PVT message. Additionally, the UBX-NAV-PVT message contains information about the validity and the accuracy of the provided UTC time. See the section Time validity for further information.

心 iTOW is always referenced to GPS time, and it should not be confused with the UTC representation.

The iTOW timestamps are not compensated for the Leap seconds.

## 3.6.5 Time validity

Information about the validity of the time solution is given in the following form:

**Time validity**: Information about time validity is provided in the valid flags (e.g. validDate and validTime flags in the UBX-NAV-PVT message). If these flags are set, the time is known and considered valid for use.

**Time validity confirmation**: Information about confirmed validity is provided in the confirmedDate and confirmedTime flags in the UBX-NAV-PVT message. If these flags are set, the time validity can be confirmed by using an additional independent source, meaning that the probability of the time to be correct is very high. Note that information about time validity confirmation is only available if the confirmedAvai bit in the UBX-NAV-PVT message is set.

validDate means that the receiver has knowledge of the current date. However, it must be noted that this date might be wrong for various reasons. Only when the confirmedDate flag is set, the probability of the incorrect date information drops significantly.

心 validTime means that the receiver has knowledge of the current time. However, it must be noted that this time might be wrong for various reasons. Only when the confirmedTime flag is set, the probability of incorrect time information drops significantly.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 31 of 72

<!-- page 32 of 72 -->

Cblox

NEO-F10N - Integration manual

fullyResolved means that the UTC time is known without full seconds ambiguity. When deriving UTC time from GNSS time the number of leap seconds must be known, with the exception of GLONASS. It might take several minutes to obtain such information from the GNSS payload. When the one second ambiguity has not been resolved, the time accuracy is usually in the range of \~20s.

## 3.6.6 UTC representation

UTC time is used in many NMEA and UBX messages. In NMEA messages, time is always rounded to the nearest hundredth of a second and it is normally reported with two decimal places (e.g. 124923.52). Although compatibility mode (selected using CFG-NMEA-COMPAT) requires three decimal places, rounding to the nearest hundredth of a second remains, so the extra digit is always 0.

UTC time is also reported within some UBX messages, such as UBX-NAV-TIMEUTC and UBX-NAV-PVT. In these messages date and time are separated into seven distinct integer fields. Six of these (year, month, day, hour, min. and sec.) have fairly obvious meanings and are all guaranteed to match the corresponding values in NMEA messages generated by the same navigation epoch. This facilitates simple synchronization between associated UBX and NMEA messages.

The seventh field is called nano and it contains the number of nanoseconds by which the rest of the time and date fields need to be corrected to get the precise time. So, for example, the UTC time 12:49:23.521 would be reported as: hour: 12, min: 49, sec: 23, nano: 521000000.

It is however important to note that the first six fields are the result of rounding to the nearest hundredth of a second. Consequently the nano value can range from -5000000 (i.e. -5 ms) to +994999999 (i.e. nearly 995 ms).

When the nano field is negative, the number of seconds (and maybe minutes, hours, days, months or even years) have been rounded up. Therefore, some or all of them must be adjusted to get the correct time and date. Thus in an extreme example, the UTC time 23:59:59.9993 on 31st December 2011 would be reported as: year: 2012, month: 1, day: 1, hour: 0, min: 0, sec: 0, nano: -700000.

If a resolution of one hundredth of a second is adequate, negative nano values can simply be rounded up to 0 and effectively ignored.

The UBX-NAV-TIMEUTC message gives information about the UTC time reference clock.

The preferred variant of UTC time can be specified using the CFG-NAVSPG-UTCSTANDARD configuration item. The UTC time variant configured must correspond to a GNSS that is currently enabled. Otherwise the reported UTC time is inaccurate.

## 3.6.7 Leap seconds

Due to the slightly uneven spin rate of the Earth, UTC time gradually moves out of alignment with the mean solar time (that is, the sun no longer appears directly overhead at 0 longitude at midday). Occasionally, a "leap second" is announced to bring UTC back into close alignment with the mean solar time. Usually this means adding an extra second to the last minute of the year, but this can also happen on 30th June. When this happens, UTC clocks are expected to go from 23:59:59 to 23:59:60, and only then on to 00:00:00.

It is also possible to have a negative leap second, in which case there will only be 59 seconds in a minute and 23:59:58 will be followed by 00:00:00.

u-blox receivers are designed to handle leap seconds in their UTC output and consequently applications processing UTC times from either NMEA or UBX messages should be prepared to handle minutes that are either 59 or 61 seconds long.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 32 of 72

<!-- page 33 of 72 -->

Cblox

NEO-F10N - Integration manual

Leap second information can be polled from the receiver with the message UBX-NAV-TIMELS.

## 3.6.8 Date ambiguity

Each navigation satellite transmits information about the current date and time in the data message. The time of week (TOW) indicates the elapsed number of seconds since the start of the week (midnight Saturday/Sunday). The week number (WN) indicates the elapsed number of weeks since the particular GNSS system was started. By combining these two values the current date and time can be known. Modern GPS satellites use a 13-bit value for the week number. As GPS system was started in 1980, it allows the week number to represent dates up to year 2137. Unfortunately, at the time when the commonly used GPS L1C/A data message was designed the signal had only 10 bits available for the week number. The top bits of the full week number had to be left out. The 10 bottom bits of the week number are not sufficient to yield a completely unambiguous date as every 1024 weeks (a bit less than 20 years), the transmitted week number value "rolls over" back to zero. Consequently, the information in GPS L1 message does not differentiate between, for example, 1980, 1999, or 2019. GPS L1 receivers must thus use additional methods to calculate the full week number.

Although BeiDou and Galileo have similar representations of time, they still transmit sufficient bits for the week number to be unambiguous for the foreseeable future (the first ambiguity will be in 2078 for Galileo, and not until 2163 for BeiDou). GLONASS presents the time and date in different way and transmits sufficient information to avoid any ambiguity during the expected lifetime of the system (the first ambiguous date will be in 2124). Therefore, the receiver regards the date information transmitted by GLONASS, BeiDou, and Galileo to be unambiguous and, where necessary, uses this information to resolve any ambiguity in the GPS date.

If the receiver is connected to a simulator, note that GPS time is referenced to 6th January 1980, GLONASS to 1 January 1996, Galileo to 22 August 1999 and BeiDou to 1 January 2006. The receiver doesn't work reliably with signals simulated before these dates.

## 3.6.8.1 GPS-only date resolution

If only GPS L1C/A signals are available, the receiver establishes the date by assuming that all week numbers must be at least as large as the reference rollover week number. The default value for the reference rollover week number is selected at the compile time of the receiver firmware and is normally set to a value of a few weeks before the software is completed. The value can be overridden by CFG-NAVSPG-WKNROLLOVER configuration item.

## The following example illustrates how this works:

Assume that the reference rollover week number set in the firmware at compile time is 2148 (which corresponds to a week in calendar year 2021, but is transmitted by the satellites as 100). In this case, if the receiver sees transmissions containing week numbers in the range of 100 ... 1023, they are interpreted as week numbers 2148 ... 3071 (calendar years 2021 ... 2038), whereas transmissions with week numbers from 0 to 99 are interpreted as week numbers 3072 ... 3171 (calendar years 2038 ... 2040).

It is important to set the reference rollover week number correctly when supplying the receiver with simulated signals, especially when the scenarios are in the past.

G The GPS L5 signal contains the full 13-bit week number. Ambiquity in rollover week number is resolved once GPS L5 signals are acquired.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 33 of 72

<!-- page 34 of 72 -->

Cblox

NEO-F10N - Integration manual

## 3.7 Time mark

The receiver can be used to provide an accurate measurement of the time at which a pulse was detected on the external interrupt pin. The reference time can be chosen by setting the time source parameter to UTC, GPS, GLONASS, BeiDou, Galileo, NAVIC or local time in the CFG-TP-\* configuration group. The UTC standard can be set in the CFG-NAVSPG-\* configuration group. The delay figures defined with CFG-TP-\* are also applied to the results output in the UBX-TIM-TM2 message.

A UBX-TIM-TM2 message is output at the next epoch if

• The UBX-TIM-TM2 message is enabled, and

• a rising or falling edge was triggered since last epoch on the EXTINT pin.

The UBX-TIM-TM2 messages includes the time of the last time mark, new rising/falling edge indicator, time source, validity, number of marks and an accuracy estimate.

![Image block](doc:7d59532/tier:standard/page:34/block:9)

Only the last rising and falling edge detected between two epochs is reported since the output rate of the UBX-TIM-TM2 message corresponds to the measurement rate configured with CFG-RATE-MEAS (see Figure 6 below).

![Chart block](doc:7d59532/tier:standard/page:34/block:11)

Figure 6: Time mark

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 34 of 72

<!-- page 35 of 72 -->

Cblox

NEO-F10N - Integration manual

## 3.8 Time pulse

The receiver includes a time pulse feature providing clock pulses with configurable duration and frequency. The time pulse function can be configured using the CFG-TP-\* configuration group. The UBX-TIM-TP message provides time information for the next pulse and the time source.

![Image block](doc:7d59532/tier:standard/page:35/block:5)

Figure 7: Time pulse

## 3.8.1 Recommendations

The time pulse can be aligned to a wide variety of GNSS times or to variants of UTC derived from them. For further information, see GNSS time bases. However, it is strongly recommended that the choice of time base is aligned with the available GNSS signals (for example, to produce GPS time or UTC(USNO), ensure GPS signals are available, and for GLONASS time or UTC(SU) ensure the presence of GLONASS signals etc.). This involves coordinating the setting of CFG-SIGNAL-\* configuration group with the choice of time pulse time base.

When using time pulse for precision timing applications it is recommended to calibrate the antenna cable delay against a reference timing source.

• To get the best timing accuracy with the antenna, a fixed and accurate position is needed.

If relative time accuracy between multiple receivers is required, do not mix receivers of different product families. If this is required, the receivers must be calibrated accordingly, by setting cable delay and user delay.

The recommended configuration when using the UBX-TIM-TP message is to set both the measurement rate (CFG-RATE-MEAS) and the time pulse frequency (CFG-TP-\*) to 1 Hz.

The sequential order of the signal present at the TIMEPULSE pin and the respective output message for the simple case of 1 pulse per second (1PPS) is shown in the following figure.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 35 of 72

<!-- page 36 of 72 -->

Cblox

NEO-F10N - Integration manual

![Image block](doc:7d59532/tier:standard/page:36/block:3)

Figure 8: Time pulse and TIM-TP

## 3.8.2 Time pulse configuration

The time pulse (TIMEPULSE) signal has configurable pulse period, length and polarity (rising or falling edge).

It is possible to define different signal behavior (i.e. output frequency and pulse length) depending on whether or not the receiver is locked to reliable time source.

The configuration group CFG-TP-\* can be used to change the time pulse settings, and includes the following parameters defining the pulse:

• **time pulse enable** - If this item is set, the time pulse is active.

• **frequency/period type** - Determines whether the time pulse is interpreted as frequency or period.

• **length/ratio type** - Determines whether the time pulse length is interpreted as length [us] or pulse ratio [%].

**antenna cable delay** - Signal delay due to the cable between the antenna and the receiver.

• **pulse frequency/period** - Frequency or pulse time period when locked mode is not configured or not active.

**pulse frequency/period lock** - Frequency or pulse time period for locked mode. In use as soon as the receiver has calculated a valid time from a received signal. Only used if the corresponding item is set to use another setting in locked mode.

• **pulse length/ratio** - Length or duty cycle of the generated pulse, specifies either time or ratio for the pulse to be on/off.

**pulse length/ratio lock** - Length or duty cycle of the generated pulse for locked mode. In use as soon as the receiver has calculated a valid time from a received signal. Only used if the corresponding item is set to use another setting in locked mode.

• **user delay** - The cable delay from the receiver to the user device plus signal delay of any user application.

• **lock to GNSS freq** - If this item is set, uses the frequency gained from the GNSS signal information rather than the local oscillator's frequency.

**locked other setting** - If this item is set, the alternative setting is used as soon as the receiver can calculate a valid time. This mode can be used, for example, to disable time pulse if the time is not locked, or to indicate a lock with different duty cycles.

• **align to TOW** - If this item is set, pulses are aligned to the top of a second.

• **polarity** - If set, the first edge of the pulse is a rising edge (pulse polarity: rising).

• **grid UTC/GNSS** - Selection between UTC (0), GPS (1), GLONASS (2), BeiDou (3), (4) Galileo and NAVIC (5) time grid. Also affects the time output by UBX-TIM-TP message.

• **drive strength** - Selection of time pulse drive strength (available options: 2mA, 4mA, 8mA, 12mA)

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 36 of 72

<!-- page 37 of 72 -->

Cblox

NEO-F10N - Integration manual

The maximum pulse length cannot exceed the pulse period.

![Image block](doc:7d59532/tier:standard/page:37/block:4)

The high and the low period of the output cannot be less than 50 ns, otherwise pulses can be lost.

## 3.8.2.1 Example

The example below shows the 1PPS TIMEPULSE signal generated on the time pulse output according to the specific parameters of the CFG-TP-\* configuration group:

• **CFG-TP-TP1_ENA** = 1
• **CFG-TP-PULSE_DEF** = 0 (PERIOD)
• **CFG-TP-PULSE_LENGTH_DEF** = 1 (LENGTH)
• **CFG-TP-PERIOD_TP1** = 1 000 000 µs
• **CFG-TP-LEN_TP1** = 100 000 µs
• **CFG-TP-TIMEGRID_TP1** = 1 (GPS)
• **CFG-TP-ALIGN_TO_TOW_TP1** = 1
• **CFG-TP-USE_LOCKED_TP1** = 1
• **CFG-TP-POL_TP1** = 1
• **CFG-TP-PERIOD_LOCK_TP1** = 1 000 000 µs
• **CFG-TP-LEN_LOCK_TP1** = 100 000 µs
• **CFG-TP-DRSTR_TP1** = 1 (4 mA drive strength)

The 1 Hz output is maintained whether or not the receiver is locked to GPS time. The alignment to TOW can only be maintained when GPS time is locked.

![Image block](doc:7d59532/tier:standard/page:37/block:21)

Figure 9: Time pulse signal with the example parameters

## 3.9 Time maintenance

Maintaining accurate time can improve the speed and performance of the receiver restart. Estimate of GNSS time can be maintained by a real-time clock, or it can be provided to the receiver by the host. Estimate of the clock drift of the receiver local oscillator or an external reference frequency can also be provided to improve the startup performance.

## 3.9.1 Real-time clock

The receiver contains a real-time clock (RTC). The RTC section is located in the backup domain and can keep time while the receiver is otherwise powered off. When the receiver powers up, it attempts to use the RTC to initialize receiver local time and in most cases this leads to considerably faster and more accurate first fixes.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 37 of 72

<!-- page 38 of 72 -->

Cblox

NEO-F10N - Integration manual

NEO-F10N also provides improved time to first fix (TTFF) when RTC is not available and backup domain is supplied. The receiver uses the time of the last position that is stored in the BBR before the receiver is powered off to estimate the current time at the next startup.

## 3.9.2 Time assistance

The host can deliver time assistance to the receiver using UBX-MGA-INI-TIME\_UTC or UBX-MGA-INI-TIME\_GNSS for better startup performance.

The current GNSS time can be supplied to the receiver as a coarse value via the standard communication interfaces. This method suffers from communication latency and unpredictable delays so the accuracy of the supplied time is poor. Accuracy of the supplied time can be improved greatly if the host system has a very good sense of the current time and can deliver an exactly timed pulse to the EXTINT pin. This pulse informs the receiver when the supplied time assistance data is to be applied.

UTC time leap seconds and GPS-to-UTC conversion parameters are transmitted periodically by GPS satellites, but that happens only every 12.5 minutes. The receiver can normally calculate the correct leap seconds value from other GNSS systems immediately, but in some situations that is not possible. If the leap seconds information or the difference of time between GPS and UTC system is important for the host application, the information can be supplied to the receiver via the UBX-MGA-GPS-UTC aiding message.

## 3.9.3 Frequency assistance

To supply hardware frequency assistance, connect a periodic rectangular signal with a frequency of up to 500 kHz to the EXTINT pin. The frequency can have an arbitrary duty cycle but the low/high phase duration must not be shorter than 50 ns. The applied frequency value must be submitted to the receiver using the UBX-MGA-INI-FREQ message.

Frequency assistance can improve the cold start speed in crystal-based designs. For TCXO-based designs, the frequency assistance has only minimal impact as the receiver is quick to acquire accurate frequency from satellite transmissions. A stable external reference frequency can be used to speed up receiver testing in production test setup. The host system may also be able to provide the reference frequency to improve the cold start speed.

## 3.9.4 Clock drift assistance

Estimate of the clock drift of the local oscillator can also be fetched from the receiver using the UBX-NAV-CLOCK message. This estimate can then be sent back to the receiver using the UBX-MGA-INI-CLKD message.

## 3.10 Protection level

## 3.10.1 Introduction

Critical applications need to know how much trust they can place in their GNSS receiver's output at any given moment. Computed by the GNSS receiver in real time, the protection level (PL) quantifies the reliability of the position information to allow systems to change their mode of operation and improve the efficiency and quality of the tasks being performed.

The GNSS receiver's protection level describes the maximum likely position error to a specified degree of confidence. For example, if a GNSS receiver determines its position with a 95% protection level of one meter, there is only a 5% chance that the reported position is more than one meter away from its true position. Like the accuracy estimate of the GNSS receiver, the protection level

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 38 of 72

<!-- page 39 of 72 -->

Cblox

NEO-F10N - Integration manual

constantly fluctuates, influenced by all the common error sources that affect GNSS solutions. Unlike the accuracy estimate, the confidence level of the protection level is much higher and is validated against specific operating scenarios to ensure that the output bounds the true error.

了 The maximum navigation update rate for protection level is limited to 1 Hz. Higher baud rate (>38400 baud) is required if UBX messages are enabled in addition to the default NMEA messages.

![Image block](doc:7d59532/tier:standard/page:39/block:5)

The protection level feature is disabled by default.

![Image block](doc:7d59532/tier:standard/page:39/block:7)

Figure 10: PL bounding true position error

## 3.10.2 Interface

The protection level bounds the true position error with a target misleading information risk (TMIR), for example 5% [MI/epoch] (read: 5% probability of having an MI per epoch). The target misleading information risk describes the probability per epoch of having misleading information (MI), meaning that it is not possible to bound the true position error because it is larger than the protection level (see Figure 11).

![Image block](doc:7d59532/tier:standard/page:39/block:11)

Figure 11: Misleading information

The output of the protection level is published through the UBX-NAV-PL message.

![Image block](doc:7d59532/tier:standard/page:39/block:14)

TMIR is specified in one dimension for PL. It is not specified as a horizontal 2D or 3D value.

![Image block](doc:7d59532/tier:standard/page:39/block:16)

The protection level values (UBX-NAV-PL.plPos1/2/3) are confidence intervals around the reported position (for example, UBX-NAV-PVT or UBX-NAV-HPPOSLLH).

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 39 of 72

<!-- page 40 of 72 -->

Cblox

NEO-F10N - Integration manual

The target misleading information risk is provided in exponential notation (UBX-NAV-PL.tmirCoeff and UBX-NAV-PL.tmirExp), for example UBX-NAV-PL.tmirCoeff = 5 and UBX-NAV-PL.tmirExp = 0 results in 5e0 (= 5).

The true position error is generally unknown, unless a very accurate and reliable truth positioning system is reporting an estimate for the true position.

When the GNSS environment deviates significantly from the normal mode of operation as compared to scenarios where the PL has been validated, a validity flag is set to false to indicate these conditions. These conditions tend to be binary in nature, such as jamming has been detected, or the minimum number of satellites is being observed. UBX-NAV-PL reports a PL validity flag (see UBX-NAV-PL.plPosValid), which indicates whether the PL is usable.

## 3.10.3 Validity requirements

The protection level performance depends on many external and internal factors. Some external factors such as a harsh GNSS environment may lead to degraded PL performance.

| PL validity values | Description |
| --- | --- |
| UBX-NAV-PL.plPosValid = 1 | PL values are valid and can be used |
| UBX-NAV-PL.plPosValid = 0 | PL values are invalid and shall not be used |

Table 21: PL validity

![Image block](doc:7d59532/tier:standard/page:40/block:10)

The protection level validity flag and the misleading information are two separate, non-related parameters.

The required receiver configuration for using the PL feature is shown in Table 22.

| Parameter | Details or required configuration key value |
| --- | --- |
| GPS system is enabled and used for | CFG-SIGNAL-GPS_ENA = 1 |
| navigation |  |
| Minimum 2 GNSS systems are enabled | Galileo and/or BeiDou is enabled in addition to GPS |
| 2 | CFG-SIGNAL-GAL_ENA = 1 and/or CFG-SIGNAL-BDS_ENA = 1 |
| Automotive and portable dynamic | CFG-NAVSPG-DYNMODEL = 0 or 4 |
| models |  |
| Continuous mode | CFG-PM-OPERATEMODE = 0 |
| Static hold is disabled (optional) | CFG-MOT-GNSSSPEED_THRS = 0, CFG-MOT-GNSSSPEED_THRS = 0. Optional, disabling ensures that the static hold mode is not activated. |
| AssistNow Autonomous or AssistNow | CFG-ANA-USE_ANA = 0, AssistNow Offline data is not used. Optional, disabling |
| Offline are not used (optional) | ensures that the predicted orbits are not used for the navigation solution. This typically occurs during start-up before the ephemerides are decoded. |

Table 22: Required configuration for using the PL feature

心 If the configuration requirements are not met, the PL values are invalid and shall not be used. The PL values are valid and can be used provided the conditions in Table 23 are met.

| Parameter | Condition |
| --- | --- |
| 3D position fix | fixType = 3 in UBX-NAV-PVT message |
| Valid position fix flag | gnssfixOK = 1 UBX-NAV-PVT message |
| Nojamming reported | jammingState flag in UBX-SEC-SIG |
| No spoofing reported | spoofingState flag in UBX-SEC-SIG |
| Valid and resolved time and date | validTime = 1, validDate = 1, and fullyResolved = 1 in UBX-NAV-PVT |

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">2 Refer to the data sheet [1] for the supported GNSS combinations.</span></small>

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 40 of 72

<!-- page 41 of 72 -->

Cblox

NEO-F10N - Integration manual

| Parameter | Condition |
| --- | --- |
| Static hold mode is not activated | The static hold flag is not raised |
| Orbit prediction algorithm | AssistNow Autonomous or AssistNow Offline are not used for the navigation solution. |

Table 23: Navigation solution requirements

## 3.10.4 Expected behavior

For each navigation epoch and for each coordinate axis, a PL value is provided. For example, if the coordinate frame reported is North/East/Down, then the UBX-NAV-PL contents can be interpreted as follows:

| PL values | Description |
| --- | --- |
| UBX-NAV-PL.plPos1 | 1 stands for the north axis |
| UBX-NAV-PL.plPos2 | 2 stands for the east axis |
| UBX-NAV-PL.plPos3 | 3 stands for the down axis |

Table 24: Position PL values

If the PL coordinate frame is set to invalid (UBX-NAV-PL.plPosFrame = 0), then the PL values shall not be used. If the PL validity flag is cleared (UBX-NAV-PL.plValid = 0), the PL values shall not be used. Both of these cases must be checked.

Only if the PL is set to valid (UBX-NAV-PL.plPosValid), the PL values (UBX-NAV-PL.plPos1/2/3) can be used and are reliable with respect to the target misleading information risk.

## 3.11 Multiple GNSS assistance (MGA)

u-blox AssistNow is a multiple GNSS assistance (MGA) service. It provides a proprietary implementation of an assisted GNSS (A-GNSS) protocol compatible with the u-blox GNSS receivers. The MGA services consist of AssistNow Online and Offline variants delivered by the HTTP or HTTPS protocols.

AssistNow Online optionally provides immediate satellite ephemerides, health information and time aiding data suitable for GNSS receiver systems with direct internet access.

The AssistNow Offline service benefits u-blox GNSS receivers that only have occasional internet access. In addition, there is an MGA feature called AssistNow Autonomous, which does not need an internet connection and runs entirely on the receiver.

Do not use BeiDou AssistNow aiding data because NEO-F10N only supports the B1C signal, not B1I. Using BeiDou aiding data can force the receiver to search for B1I satellites (B1-B18) that do not transmit B1C signals, potentially degrading the receiver performance.

For further details on setting up and using AssistNow, refer to the AssistNow user guide [5].

Table 25 below contains an overview of the different MGA services u-blox provides. Refer to the NEO-F10N Data sheet for the supported GNSS signals by each MGA service [1].

| Requirements | AssistNow Online | AssistNow Offline | AssistNow Autonomous |
| --- | --- | --- | --- |
| Requires external flash memory | No | Optional | Optional |
| Requires internet connection | Permanently | Sporadically | No |
| Amount of internet data | Medium | High | None |
| Ephemeris in data | Yes | No | No |

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 41 of 72

<!-- page 42 of 72 -->

Cblox

NEO-F10N - Integration manual

| Requirements | AssistNowOnline | AssistNowOffline | AssistNowAutonomous |
| --- | --- | --- | --- |
| Almanacindata | Yes | Yes | Yes |

Table 25: AssistNow service overview

## 3.11.1 Authorization

To use the AssistNow services, customers will need to obtain an authorization token from u-blox. Go to [https://www.u-blox.com/en/solution/services/assistnow](https://www.u-blox.com/en/solution/services/assistnow) or contact your local technical support to get more information and to request access to the service.

## 3.11.2 Preserving MGA and operational data during power-off

The time-to-fix after a receiver power interruption is dependent on the amount of operational data available at startup. Satellite broadcast information and an estimate of accurate time can be fetched form the AssistNow service. In addition, the following techniques can restore the data that was stored prior to powering down.

**Battery-backed RAM:**The receiver operational state stored in this RAM can be maintained during power outages by connecting the V\_BCKP pin to an independent supply, e.g. a battery. This is a recommended method as it will maintain all MGA-related information, any user configuration, calibration data, and an estimate of time via the real-time clock. See V\_BCKP for more information.

**Save-on-shutdown:**The receiver can be instructed to dump its current state to flash memory as part of the shutdown procedure; this data is then automatically retrieved when the receiver is restarted. For more information, see section Save-on-shutdown.

**Database dump:** The receiver can be made to dump the state of its navigation database in the form of a sequence of UBX messages reported to the host; these messages can be stored by the host and sent back to the receiver when it has been restarted. For more information, see the description of the UBX-MGA-DBD messages in the Interface description [3].

## 3.11.3 AssistNow Online

AssistNow Online provides satellite ephemerides, health information and time aiding data suitable for GNSS receiver systems with direct internet access. If this service is used, the host system delivers the necessary data to the receiver at each startup.

AssistNow Online data must be sent immediately at the receiver start-up.

Do not use AssistNow Online aiding data close to its expiration. The validity period of the aiding data depends on the GNSS constellations used. For information on the GNSS signals that AssistNow Online service supports, see the NEO-F10N Data sheet [1].

For further details on setting up and using AssistNow Online, refer to the AssistNow user guide [5].

## 3.11.4 AssistNow Offline

AssistNow Offline is targeted at receivers that only have occasional internet access, and therefore AssistNow Online cannot be used. AssistNow Offline speeds up time to first fix (TTFF), typically to considerably less than 10 s.

The AssistNow Offline Service uses a simple, stateless, HTTP interface. Therefore, it works on all standard mobile communication networks that support internet access, including GPRS, UMTS and Wireless LAN. No special arrangements need to be made with mobile network operators to enable AssistNow Offline.

![Image block](doc:7d59532/tier:standard/page:42/block:20)

AssistNow Offline currently supports GPS, Galileo, and QZSS, and GLONASS. u-blox intends to expand the AssistNow Offline Service to support other GNSS (such as BeiDou) in due course.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 42 of 72

<!-- page 43 of 72 -->

Cblox

NEO-F10N - Integration manual

![Image block](doc:7d59532/tier:standard/page:43/block:3)

Do not use more than 1 week old AssistNow Offline data when high navigation rate (i.e. more than 1 Hz) is used.

The downloaded AssistNow Offline data is encoded in a sequence of UBX-MGA-ANO messages, one for every satellite for every day of the period covered. For example, collecting data from all GPS satellites over a four-week period would result in more than 900 distinct messages, occupying approximately 70 kilobytes, which would not fit into the available BBR memory in the receiver.

If the receiver has flash storage, all the data can be downloaded from the server and stored in the flash until it is needed. In this case, the receiver will automatically select the most appropriate data to use at any time.

If the receiver has no flash storage, either a smaller amount of data must be requested, or the host system must store the AssistNow Offline data until the receiver needs it and then upload only the appropriate part for immediate use.

For further details on setting up and using AssistNow Offline, refer to the AssistNow user guide [5].

## 3.11.5 AssistNow Autonomous

The assistance scenarios covered by AssistNow Online and AssistNow Offline require an online connection and a host that can use this connection to download aiding data and provide this to the receiver when required.

The AssistNow Autonomous feature provides a functionality similar to AssistNow Offline without the need for a host and a connection. Based on a broadcast ephemeris downloaded from the satellite (or obtained by AssistNow Online), the receiver can autonomously (i.e. without any host interaction or online connection) generate an accurate satellite orbit representation ("AssistNow Autonomous data") that is usable for navigation much longer than the underlying broadcast ephemeris was intended for. This makes downloading new ephemeris or aiding data for the first fix unnecessary for subsequent startups of the receiver.

![Image block](doc:7d59532/tier:standard/page:43/block:12)

The AssistNow Autonomous feature is disabled by default. It can be enabled using the CFG-ANA-USE\_ANA configuration item.

## 3.11.5.1 Concept

The figure below illustrates the AssistNow Autonomous concept in a graphical way. Note that the figure is a qualitative illustration and is not to scale.

A broadcast ephemeris downloaded from the satellite is a precise representation of a part (for GPS nominally four hours) of the satellite's true orbit (trajectory). It is not usable for positioning beyond this validity period because it diverges dramatically from the true orbit afterwards.

The AssistNow Autonomous orbit is an extension of one or more broadcast ephemerides. It provides a long-term orbit for the satellite for several revolutions. Although this orbit is not perfectly precise, it is a sufficiently accurate representation of the true orbit to be used for navigation.

The AssistNow Autonomous data is automatically and autonomously generated from downloaded (or assisted) ephemerides. The data is stored automatically in the on-chip batterybacked memory (BBR). Optionally, the data can be backed up in external flash memory or on the host. The number of satellites for which data can be stored depends on the receiver configuration and may change during operation.

If no broadcast ephemeris is available for navigation, AssistNow Autonomous automatically generates the required parts of the orbits suitable for navigation from the stored data. The data is also automatically kept current in order to minimize the calculation time once the navigation engine needs orbits.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 43 of 72

<!-- page 44 of 72 -->

Cblox

NEO-F10N - Integration manual

The operation of the AssistNow Autonomous feature is transparent to the user and the operation of the receiver. All calculations are done in the background and do not affect the normal operation of the receiver.

• The AssistNow Autonomous subsystem automatically invalidates data that has become too old and that would introduce unacceptable positioning errors. This threshold is configurable.

The prediction quality will be automatically improved if the satellite has been observed multiple times. However, this requires the availability of a suitable flash memory. Improved prediction quality also extends the maximum usability period of the data.

AssistNow Autonomous considers GPS, GLONASS, Galileo and BeiDou satellites only. It will not consider satellites on orbits with an eccentricity of >0.05 (e.g., Galileo E18). For GLONASS support, a suitable flash memory is mandatory because a single GLONASS broadcast ephemeris contains information only for approximately 30 minutes. This is not long enough to extend it in a usable way. Orbit information of each GLONASS satellite must be collected at least for four hours to generate data.

![Image block](doc:7d59532/tier:standard/page:44/block:7)

Non-volatile storage options for AssistNow Autonomous data

N.B. qualitative illustration, not to scale!

## 3.11.5.2 Interface

Several UBX protocol messages provide interfaces to the AssistNow Autonomous feature:

The CFG-ANA-USE\_ANA item is used to enable or disable the AssistNow Autonomous feature. When enabled, the receiver will automatically produce AssistNow Autonomous data for newly received broadcast ephemerides and, if that data is available, automatically provide the navigation subsystem with orbits when necessary and adequate.

The CFG-ANA-\* configuration group also allows for a configuration of the maximum acceptable orbit error. See the next section for an explanation of this feature. It is recommended to use the firmware default value that corresponds to a default orbit data validity of approximately three days (for GPS satellites observed once) and up to six days (for GPS and GLONASS satellites observed multiple times over a period of at least half a day).

If the receiver uses flash memory, disabling the AssistNow Autonomous feature will delete all previously collected satellite observation data from the flash memory.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 44 of 72

<!-- page 45 of 72 -->

Cblox

NEO-F10N - Integration manual

The UBX-NAV-AOPSTATUS message provides information on the current state of the AssistNow Autonomous subsystem. The status indicates whether the AssistNow Autonomous subsystem is currently idle (or not enabled) or busy generating data or orbits. Hosts should monitor this information and only power off the receiver when the subsystem is idle (that is, when the status field shows a steady zero).

• The UBX-NAV-SAT message indicates the use of AssistNow Autonomous orbits for individual satellites.

• The UBX-NAV-ORB message indicates the availability of AssistNow Autonomous orbits for individual satellites.

The UBX-MGA-DBD message provides a means to retrieve the AssistNow Autonomous data from the receiver in order to preserve the data in power-off mode where no battery backup is available. Note that the receiver requires the absolute time (i.e. full date and time) to calculate AssistNow Autonomous orbits. For the best performance, it is therefore recommended to supply this information to the receiver using the UBX-MGA-INI-TIME\_UTC message in this scenario.

• The Save-on-Shutdown (SOS) feature preserves AssistNow Autonomous data. For more information about the SOS feature, see the interface description [3].

## 3.11.5.3 Benefits and drawbacks

AssistNow Autonomous can provide quicker startup times by lowering the TTFF, provided that data is available for enough visible satellites. This is particularly true under weak signal conditions where it might not be possible to download broadcast ephemerides at all and therefore, no fix would be possible without AssistNow Autonomous (or A-GNSS). It is however required that the receiver roughly knows the absolute time, either from an RTC or from time-aiding (see the Interface section), and that it knows which satellites are visible, either from the almanac or from tracking the respective signals.

The AssistNow Autonomous orbit (satellite position) accuracy depends on various factors, such as the particular type of satellite, the accuracy of the underlying broadcast ephemeris, or the orbital phase of the satellite and Earth, and the age of the data (errors add up over time).

AssistNow Autonomous will typically extend a broadcast ephemeris from three up to six days. The CFG-ANA-ORBMAXERR item allows changing this threshold by setting the «maximum acceptable modeled orbit error» (in meters). Note that this number does not reflect the true orbit error introduced by extending the ephemeris. It is a statistical value that represents a certain expected upper limit based on a number of parameters. A rough approximation that relates the maximum extension time to this setting is: maxError [m] = maxAge [d] \* f, where the factor f is 30 for data derived from satellites seen once and 16 for data derived for satellites seen multiple times during a long enough time period (see the Concept section).

There is no direct relation between (true and statistical) orbit accuracy and positioning accuracy. The positioning accuracy depends on various factors, such as the satellite position accuracy, the number of visible satellites, and the geometry (DOP) of the visible satellites. Position fixes that include AssistNow Autonomous orbit information may be significantly worse than fixes using only broadcast ephemerides. Therefore, it might be necessary to adjust the limits of the navigation output filters (CFG-NAVSPG-OUTFIL\_XXXX).

Unknown future events form a fundamental deficiency of any system and can prevent precise satellite orbit predictions. Hence, the receiver will not be able to know about satellites that will have become unhealthy, have undergone a clock swap, or have had a maneuver. This means that the navigation engine might rarely mistake a wrong satellite position as the true satellite position. However, provided that there are enough other good satellites, the navigation algorithms will eventually eliminate a defective orbit from the navigation solution.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 45 of 72

<!-- page 46 of 72 -->

Cblox

NEO-F10N - Integration manual

The repeatability of the satellite constellation is a potential pitfall for the use of the AssistNow Autonomous feature. For a given location on Earth, the (GPS) constellation (geometry of visible satellites) repeats every 24 hours. Hence, when the receiver «learned» about a number of satellites at some point in time, the same satellites will in most places not be visible 12 hours later, and the available AssistNow Autonomous data will not be of any help. However, after another 12 hours, usable data would be available because it was generated 24 hours ago.

The longer a receiver observes the sky, the more satellites it will have seen. At the equator, and with full sky view, approximately ten (GPS) satellites will show up in a one-hour window. After four hours of observation approx. 16 satellites (i.e. half the constellation), after 10 hours approx. 24 satellites (2/3rd of the constellation), and after approx. 16 hours the full constellation will have been observed (and AssistNow Autonomous data generated). Lower sky visibility reduces these figures (i.e. the number of satellites seen). Further away from the equator, the numbers improve because the satellites can be seen twice a day. For example, at 47 degrees north the full constellation can be observed in approx. 12 hours with full sky view.

The calculations required for AssistNow Autonomous are carried out on the receiver. This requires energy and users may therefore occasionally see increased power consumption during short periods (several seconds, rarely more than 60 seconds) when such calculations are running. Ongoing calculations will automatically prevent the power save mode from entering the power-off state. The power-down will be delayed until all calculations are done.

AssistNow Autonomous should be enabled if the system has sporadic access to the AssistNow Offline service. In this case, the receiver will intelligently choose the more reliable orbit predictions for each satellite. This way the autonomous prediction can provide performance improvements if the offline data becomes old or gets outdated.

## 3.12 Save-on-shutdown

The save-on-shutdown feature (SOS) enables the u-blox receiver to store the contents of the battery-backed RAM to an external flash memory and restore it upon startup. This allows the ublox receiver to preserve some of the features available only with a battery backup (preserving configuration and satellite orbit knowledge) without having a battery backup supply present. However, the receiver does not preserve any kind of time knowledge. Save-on-shutdown must be commanded by the host. Refer to the NEO-F10N Interface description [3] for more information on the required UBX-UPD-SOS messages. The restoring of data on startup is automatically done if the corresponding data is present in the flash. Data expiration is not checked.

The following outlines the suggested shutdown procedure when using the SOS feature:

With the UBX-CFG-RST message, the host commands the u-blox receiver to stop, specifying reset mode 0x08 ("Controlled GNSS stop") and a BBR mask of 0 ("Hotstart").

• The host commands the saving of the contents of BBR to the flash memory using the UBX-UPD-SOS-BACKUP message.

• For a valid request the u-blox receiver reports on the success of the backup operation with a UBX-UPD-SOS-ACK message.

• The host powers off the u-blox receiver.

The startup procedure is as follows:

• The host powers on the u-blox receiver.

The u-blox receiver detects the previously stored data in the flash. It restores the corresponding memory and reports the success of the operation with a UBX-UPD-SOS-RESTORED message on the port on which it had received the save command message (if the output protocol filter on that port allows it). It does not report anything if no stored data has been detected.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 46 of 72

<!-- page 47 of 72 -->

Cblox

NEO-F10N - Integration manual

Additionally the u-blox receiver outputs a UBX-INF-NOTICE and/or a NMEA-TXT message with the contents RESTORED in the boot screen (depends on the configuration of the port and information messages) upon success.

• Optionally the host can deliver coarse time assistance using UBX-MGA-INI-TIME\_UTC for better startup performance.

![Image block](doc:7d59532/tier:standard/page:47/block:5)

It is recommended to delete the stored data using a UBX-UPD-SOS-CLEAR message once the u-blox receiver has started up. The u-blox receiver responds with a UBX-ACK-ACK / UBX-ACK-NAK message.

## 3.13 Firmware update

u-blox may release updated firmware images containing, for example, security fixes, enhancements, bug fixes, etc. Therefore, it is important to implement a firmware update mechanism in the host system.

Contact u-blox for more information on firmware update.

UBXDOC-963802114-12193 - R04 C1-Public

3 Receiver functionality

Page 47 of 72

<!-- page 48 of 72 -->

Cblox

NEO-F10N - Integration manual

## 4 Hardware integration

This chapter explains how the receiver can be integrated into an application design.

## 4.1 Power supply

The NEO-F10N has the following power supply pins: VCC and V\_BCKP.

A power supply at VCC must be present for normal operation. A supply at V\_BCKP is optional. If present, it enables the hardware backup mode when the VCC supply is off.

Refer to the NEO-F10N Data sheet [1] for absolute maximum ratings, operating conditions, and power requirements.

## 4.1.1 VCC

VCC provides power to the core and RF domains and must be supplied during normal operation. For low power consumption, the VCC pin supplies power to the core via an internal DCDC converter. A filtered VCC supply is available on the VCC\_RF pin. The VCC\_RF output voltage is derived from the VCC supply and is available whenever VCC is supplied.

VCC also supplies all the digital IOs, clock, and the backup domain. The current drawn at VCC depends on the activity and loading of the PIOs and the main oscillator.

![Image block](doc:7d59532/tier:standard/page:48/block:12)

Do not add series resistance greater than 0.2 Ω on the supply line to avoid voltage ripple due to the dynamic current conditions.

## 4.1.2 V\_BCKP

Power supply at V\_BCKP is optional. If the power supply at VCC is interrupted, but the V\_BCKP pin is supplied, the receiver enters the hardware backup mode. In this mode, the RTC time and the GNSS orbit data in the BBR are maintained. Valid time and GNSS orbit data at startup improves positioning performance by enabling hot starts, warm starts, and AssistNow Autonomous. This ensures faster TTFF when VCC is supplied again. To make these features available, connect an independent power supply to V\_BCKP to ensure backup domain supply when VCC is not supplied.

Designs using an external battery as a power source at the V\_BCKP pin must consider the battery capacity. That is, the GNSS satellite ephemeris data is typically valid for up to 4 hours for hot starts. Furthermore, for products supporting AssistNow Offline and Autonomous, the assistance data is valid up to few days for warm starts .

![Image block](doc:7d59532/tier:standard/page:48/block:17)

Avoid high resistance on the V\_BCKP line. During the switch to V\_BCKP supply, a short current adjustment peak may cause a high voltage drop at the pin.

![Image block](doc:7d59532/tier:standard/page:48/block:19)

If the hardware backup mode is not used, leave the V\_BCKP pin open.

## 4.2 RF interference

The GNSS signal power received at the antenna is very low compared to other wireless communication signals. The received nominal –130 dBm GNSS signal strength makes the GNSS receiver susceptible to interference from any kind of nearby RF sources.

As an example, cellular applications emit signals with power levels of approximately +30 dBm, while the GNSS signal is less than –130 dBm when reaching the antenna. By simply comparing these

UBXDOC-963802114-12193 - R04 C1-Public

4 Hardware integration

Page 48 of 72

<!-- page 49 of 72 -->

Cblox

NEO-F10N - Integration manual

numbers, it is obvious that interference issues must be seriously considered during the design phase.

## 4.2.1 In-band interference

Although the radio communications standards prevent intentional RF signal sources from interfering the GNSS frequencies, many devices emit RF power into the GNSS band at levels much higher than the GNSS signal itself.

One reason is that the frequency band above 1 GHz is not well regulated with regards to EMI, and even if permitted, signal levels are much higher than the GNSS signal power. In particular, all types of digital equipment, such as PCs, digital cameras, LCD screens, etc. tend to emit a broad frequency spectrum up to several GHz of frequency. Also wireless transmitters may generate spurious emissions that fall into the GNSS band.

![Image block](doc:7d59532/tier:standard/page:49/block:7)

The Layout section defines measures against in-band interference during the design phase of the application.

## 4.2.2 Out-of-band interference

Out-of-band interference is caused by signal frequencies that are different from the GNSS carrier frequency. The main sources are wireless communication systems such as LTE, GSM, CDMA, WCDMA, Wi-Fi, BT, etc. Typically, these systems may emit their specified maximum transmit power in close proximity to the GNSS receiving antenna, especially if such a system is integrated with the GNSS receiver. Even at reasonable antenna selectivity, destructive power levels may reach the RF input of the GNSS receiver. In addition, larger signal interferers may generate intermodulation products inside the GNSS receiver front-end that fall into the GNSS band and contribute to in-band interference.

Measures against out-of-band interference include maintaining a good grounding concept in the design and adding a GNSS band-pass filter into the antenna input line to the receiver.

![Image block](doc:7d59532/tier:standard/page:49/block:12)

Sections Out-of-band blocking immunity and Interference coupling provide more information about the RF immunity of the NEO-F10N module.

## 4.2.3 Spectrum analyzer

The UBX-MON-SPAN message can be enabled in u-center 2 to provide a low-resolution spectrum analyzer sufficient to identify noise or jammers in the reception band. Once enabled, u-center 2 includes a real-time chart that is updated once per second with the message data. See Figure 12 for an example.

The design or device environment can generate interference at the in-band that can be analyzed from the spectrum in the UBX-MON-SPAN message. Hence, the shape of the spectrum as well as visible peaks help to identify in-band interference. Out-of-band interference can also cause peaks that appear in the in-band. However, there can be out-of-band interference that is not visible within the span of the spectrum. The presence of out-of-band interference may be seen as reduction in the PGA value.

The vertical axis compares the power level in dB for each frequency. A good spectrum shape is characterized by an even noise floor along with the GNSS band. For example, if any unwanted interference peak stands out, the vertical axis gives a rough approximation of the power level in dB compared to the noise floor.

UBXDOC-963802114-12193 - R04 C1-Public

4 Hardware integration

Page 49 of 72

<!-- page 50 of 72 -->

Cblox

NEO-F10N - Integration manual

Next to the chart, the center frequency, span, and resolution values set for the spectrum, and the PGA value are also displayed. The PGA value represents the internal gain set by the receiver, which depends on the external amplification of the GNSS input signal.

The vertical discontinuous lines in the chart area represent the offset to the center frequency in MHz. This helps to estimate the frequency of any spurious emission seen.

In addition, u-center 2 includes three functions commonly found in any spectrum analyzer. These features support the RF front-end design and help to spot out any jammer present during the application operation.

Hold: if selected, the current spectrum shape freezes in a colored line. This allows for a comparison between the time the spectrum was frozen and the real-time spectrum. This is particularly helpful in assessing the impact of running other onboard components.

Average: if selected, a colored line shows the averaged spectrum for each frequency. This supports the analysis over time and obtaining a less noisy shape.

Max hold: if selected, a colored line shows the maximum amplitude measured at each frequency. This option helps to spot out any jammer over a period of time.

Figure 12 shows the spectrum view in u-center 2 with the hold, average and max hold options selected. The green, yellow and red lines represent the frozen hold, average and max hold spectrums, while the blue line represents the current continuous spectrum.

![Chart block](doc:7d59532/tier:standard/page:50/block:10)

Figure 12: Spectrum analyzer view in u-center 2

By changing the enabled GNSS constellations, the span widens or narrows. This has a direct impact on the spectrum resolution, as the number of measured values is fixed to 256. For further details about this message and how to calculate each frequency, see the Interface description [3].

A peak may be visible around the center frequency. The signal comes internally from the receiver and it does not cause any degradation in the performance.

UBXDOC-963802114-12193 - R04 C1-Public

4 Hardware integration

Page 50 of 72

<!-- page 51 of 72 -->

Cblox

NEO-F10N - Integration manual

## 4.3 RF front-end

GNSS receivers operate with very low signal levels, ranging from –130 dBm to approximately –167 dBm. This alone is a challenge for the GNSS application design. Out-of-band sources of interference such as GSM, CDMA, WCDMA, LTE, Wi-Fi, or Bluetooth wireless systems with a much higher signal level require additional specific measures. The goal of the RF front-end design is to receive the inband signal with minimum loss and added noise while suppressing the out-of-band interference.

The NEO-F10N RF front-end is designed for highest immunity against RF interference and is suitable for designs with a cellular transmitter. The integrated RF circuit includes an L1, L5 dualband diplexer SAW filter followed by a low-noise amplifier (LNA) and a second SAW filter stage at both GNSS bands. There is also an LTE B13 notch filter in front of the LNA at L1 band. The dualband SAW–LNA–SAW circuit with built-in LTE Band 13 suppression efficiently rejects out-of-band blocking signals and amplifies the GNSS signal enhancing the sensitivity of the receiver. The RF input is matched to 50 Ω and includes a built-in DC block.

Refer to the Block diagram for an overview of the RF front-end.

## 4.3.1 Internal LNA modes

In addition to the L1 and L5 band LNAs integrated in the RF front-end circuit in NEO-F10N, there are internal LNAs in the u-blox F10 receiver.

The internal L1 and L5 band LNAs in the receiver have three operating modes: normal gain, low gain, and bypass mode.

By default, the internal L1 and L5 band LNAs are configured for the normal-gain mode for optimized sensitivity. For designs with external gain above 25 dB, the low-gain or bypass mode must be used to maintain the dynamic range.

• The low-gain mode can be used up to 35 dB total external gain. There are also minor improvements in immunity and power consumption in the low-gain mode.

The bypass mode can be used up to 45 dB external gain. There are further improvements in immunity and power consumption in the bypass mode.

The internal LNA mode can be configured at run time in the RAM, BBR and flash layers using the configuration item CFG-HW-RF\_LNA\_MODE and applying a reset. Once in production, the flash layer can be configured to automatically apply the configuration at every startup. For more information, refer to Internal LNA mode configuration .

For information on RF parameters, refer to the NEO-F10N Data sheet [1].

## 4.3.2 Out-of-band blocking immunity

Out-of-band RF interference may degrade the quality and availability of the navigation solution. Out-of-band immunity limit describes the maximum power allowed at the receiver RF input with no degradation in performance. Minor violation of the immunity limit may reduce C/N0 of the received signals but does not necessarily affect the overall receiver performance. However, a significant violation may reduce receiver sensitivity or cause a complete loss of signal reception. The severity of the interference depends on the repetition rate, frequency, signal level, modulation, and bandwidth of the signal.

A typical out-of-band immunity limit at the NEO-F10N RF input for the normal gain (default) mode is +13 dBm at 400–1070 MHz, 1280–1460 MHz, and 1710–3300 MHz. The immunity decreases closer to the GNSS in-band. The limit is defined at room temperature using a test signal with 64QAM modulation and 10 MHz bandwidth similar to an LTE signal.

UBXDOC-963802114-12193 - R04 C1-Public

4 Hardware integration

Page 51 of 72

<!-- page 52 of 72 -->

Cblox

NEO-F10N - Integration manual

![Image block](doc:7d59532/tier:standard/page:52/block:3)

If the out-of-band immunity limit is exceeded, it is recommended to verify that the receiver performance is not affected or is at an acceptable level in the presence of interference.

## 4.3.3 Interference coupling

RF interference is typically first coupled into the antenna and subsequently conducted into the receiver input. Typical out-of-band interference sources include transmitting antennas of other radio systems. Estimation of the RF interference level coupled into the receiver antenna is a starting point for RF front-end design.

For designs with other radio systems, the maximum power coupled into the antenna can be estimated from the maximum transmission power and the isolation between the antennas. Practical values for antenna isolation can range from 15–20 dB down to 6–10 dB for very small devices. RF interference may also couple from external sources such as nearby mobile devices or base stations.

A simplified test board can be used to estimate the isolation between two antennas. The size of the board and the placement of the antennas must match the final design. Connect the RF cables to the antenna inputs and measure S21 over the frequency band of interest with a vector network analyzer (VNA).

It is more difficult to estimate RF interference from other parts of the design. One option is to measure the interference level at the receiver input using a spectrum analyzer. Interference within the design is primarily a problem at the receiver in-band, where it cannot be addressed by filtering on the RF path. Outside the GNSS band, the required filtering is determined by the estimated interference level and the immunity of the receiver.

The maximum power coupled into the receiver RF input is compared against the immunity limit of the receiver defined in Out-of-band blocking immunity.

## 4.3.4 Antenna power supply

Figure 13 shows an active antenna supply network to connect the antenna supply to the RF signal line. The inductance L4 connects the antenna power supply to the RF signal line. The capacitance C14 filters out high-frequency interference from the power supply and the resistor R8 limits the short-circuit current.

The type and value of L4 is selected to have a resonance peak at GNSS frequencies. This provides a high series impedance above 500 Ω at GNSS L1 and L5 frequencies, creating an impedance mismatch with respect to the 50 Ω RF signal line. This minimizes the effect of the feed point on the RF signal line, and isolates the antenna supply from the RF signal line at GNSS frequencies. Both R8 and L4 must have sufficient current and power rating to withstand the short-circuit current. Example component values for the antenna supply network are given in Standard resistors, Standard capacitors, and Inductors.

The VCC\_RF pin can be used to supply an active antenna. VCC\_RF is a RF filtered supply voltage derived from the VCC supply. Refer to the Data sheet[1] for the VCC\_RF specification.

The LNA\_EN pin can be used to control the active antenna supply.

UBXDOC-963802114-12193 - R04 C1-Public

4 Hardware integration

Page 52 of 72

<!-- page 53 of 72 -->

Cblox

NEO-F10N - Integration manual

![Image block](doc:7d59532/tier:standard/page:53/block:3)

Figure 13: Antenna supply network

## 4.4 Layout

GNSS signals on the surface of the earth have a very low signal strength and are about 15 dB below the thermal noise floor. When integrating a GNSS receiver into a PCB, the placement of the components, as well as grounding, shielding, and interference from other digital devices are crucial issues that need to be considered very carefully.

An important factor in achieving high GNSS performance is the placement of the receiver with respect to other components on the PCB.

To minimize signal loss on the RF connection from the antenna to the receiver input and to avoid possible coupled interference, the connection to the antenna must be kept short while keeping some distance from the antenna to other electronic components.

The RF section should not be subject to noisy digital supply currents running through its GND plane. Make sure that critical RF circuits are clearly separated from any other digital circuits on the system board. To achieve this, position the receiver digital part towards the digital section of the system PCB and place the RF section and antenna as far away as possible from the other digital circuits on the board. Keep at least a 5 mm distance to any RF component and ensure proper grounding.

![Image block](doc:7d59532/tier:standard/page:53/block:10)

For applications using cellular antennas, increase the distance between both antennas as much as possible.

Another very important factor in GNSS applications is the grounding concept. Ensure good ground reference to the host ground by increasing the number of GND vias. The GND vias will improve the GND reference between all the layers, and the pads will serve as thermal relief.

Any stubs at the ground planes must be avoided or ended with a via to the reference ground. Otherwise, they could pick up and propagate interference.

For the RF signal line, it is best to use the co-planar waveguide with ground on the second layer. All the RF parts need a solid GND plane underneath in order to achieve the targeted impedance in the RF signal line.

UBXDOC-963802114-12193 - R04 C1-Public

4 Hardware integration

Page 53 of 72

<!-- page 54 of 72 -->

Cblox

NEO-F10N - Integration manual

The length and geometry in the RF signal line must be carefully analyzed. The impedance of the RF signal line must be 50 Ω. Select the stack-up, copper, and dielectric properties of the PCB accordingly to fulfill this condition. The RF signal line should be as short as possible and the ground plane around should be filled with GND vias.

The RF trace must be shielded by vias to ground along the entire length of the trace and the NEO-F10N RF\_IN pad should be surrounded by vias as shown in Figure 14.

![Image block](doc:7d59532/tier:standard/page:54/block:5)

Figure 14: RF input trace

The RF\_IN trace on the top layer should be referenced to a suitable ground layer.

The VCC pad for the NEO-F10N Standard precision GNSS module needs to have as low an impedance as possible with large vias to the lower power layer of the PCB. The VCC pad needs a large pad and the decoupling capacitor must be placed as close as possible. This is shown in Figure 15.

![Image block](doc:7d59532/tier:standard/page:54/block:9)

Figure 15: VCC pad

Be careful when placing the receiver in proximity to circuitry that can emit heat. Temperaturesensitive components inside the module, like TCXOs and crystals, are sensitive to sudden changes in ambient temperature which can adversely impact satellite signal tracking. Sources can include co-located power devices, cooling fans or thermal conduction via the PCB.

The GND planes can conduct heat to other elements, but they can act as heat dissipators as well. Increasing the number of GND vias helps to decrease sudden temperature changes.

UBXDOC-963802114-12193 - R04 C1-Public

4 Hardware integration

Page 54 of 72

<!-- page 55 of 72 -->

Cblox

NEO-F10N - Integration manual

![Image block](doc:7d59532/tier:standard/page:55/block:3)

High temperature drift and air vents can affect the GNSS performance. For best performance, avoid high temperature drift and air vents near the module.

## 4.4.1 Package footprint, copper and solder mask

The mechanical specification is available in the Data sheet [1].

Figure 16 and Figure 17 describe the footprint and provide recommendations for the paste mask. Note that the copper and solder masks have the same size and position.

![Chart block](doc:7d59532/tier:standard/page:55/block:8)

Figure 16: Recommended copper land and solder mask opening for NEO-F10N

To improve the wetting of the half vias, reduce the amount of solder paste under the module and increase it outside of the module by defining the dimensions of the paste mask to form a T-shape (or equivalent) extending beyond the copper mask.

Recommended stencil thickness is 150 µm.

UBXDOC-963802114-12193 - R04 C1-Public

4 Hardware integration

Page 55 of 72

<!-- page 56 of 72 -->

blox

NEO-F10N - Integration manual

![Chart block](doc:7d59532/tier:standard/page:56/block:3)

Figure 17: Recommended paste mask pattern for NEO-F10N

UBXDOC-963802114-12193 - R04 C1-Public

4 Hardware integration

Page 56 of 72

<!-- page 57 of 72 -->

Cblox

NEO-F10N - Integration manual

## 5 Product handling

## 5.1 Safety

## 5.1.1 ESD precautions

![Image block](doc:7d59532/tier:standard/page:57/block:6)

CAUTION! Risk of electrostatic discharge (ESD) damage. u-blox chips and modules are electrostatic sensitive devices containing highly sensitive electronic circuitry. A discharge of static electricity may damage the device or reduce the life expectancy of the device. To avoid ESD damage, adhere to the standard guidelines for handling ESD devices.

Consider the following:

## Preventing electrostatic discharge

• Keep components in their original packages during transport.

• Open the package within an ESD-protected area (EPA), as in Figure 18.

• At a workstation, store components in an EPA.

• Place ESD sensitive devices inside of shielding packaging or containers when transported outside of an EPA.

Use protective clothing and proper personnel grounding at all necessary points when touching electrostatic sensitivedevice or assembly. For instance, wear ESD-safe clothing and shoes and wear an ESD wrist strap connected to a groundedworkstation. Use heel straps when standing on conductive floors or dissipating floor mats.

• Hold the devices by the edges and avoid touching component contacts, pins, or circuitry

## Product handling

• When handling RF transceivers and patch antennas, work in an EPA.

When connecting test equipment or any other electronics to the module (as a standalone or PCB-mounted device), the first point of contact must always be between the local ground and the PCB ground.

• Before mounting a ceramic patch antenna, connect the device to ground.

When handling the RF pin, do not touch any charged capacitors. Be especially careful when handling materials likepatch antennas (\~10 pF), coaxial cables (\~50-80 pF/m), soldering irons, or any other materials that can develop charges.

• If there is any risk of touching an exposed antenna area in a non-ESD protected work area, implement proper ESDprotection measures in the design.

• When soldering RF connectors and patch antennas to the receiver's RF pin, use an ESD-safe soldering iron (tip)

UBXDOC-963802114-12193 - R04 C1-Public

5 Product handling

Page 57 of 72

<!-- page 58 of 72 -->

Cblox

NEO-F10N - Integration manual

![Image block](doc:7d59532/tier:standard/page:58/block:3)

Figure 18: Standard workstation setup for safe handling of ESD-sensitive devices

## 5.1.2 Safety precautions

The NEO-F10N modules must be supplied by an external limited power source in compliance with the clause 2.5 of the standard IEC 60950-1. In addition to external limited power source, only Separated or Safety Extra-Low Voltage (SELV) circuits are to be connected to the module including interfaces and antennas.

![Image block](doc:7d59532/tier:standard/page:58/block:7)

For more information about SELV circuits see section 2.2 in Safety standard IEC 60950-1.

## 5.2 Soldering

Reflow soldering procedures are described in the IPC/JEDEC J-STD-020 standard [4].

![Image block](doc:7d59532/tier:standard/page:58/block:11)

When populating the modules, make sure that the pick and place machine is aligned to the copper pins of the module instead of the module edge.

## Soldering paste

Use of “no clean” soldering paste is highly recommended, as it does not require cleaning after the soldering process. For instance, the following paste meets these criteria.

• Soldering paste: OM338 SAC405 / Nr.143714 (Cookson Electronics)

• Alloy specification: Sn 95.5/ Ag 4/ Cu 0.5 (95.5% tin/ 4% silver/ 0.5% copper)

• Melting temperature: 217 °C

• Stencil: The exact geometry, distances, stencil thicknesses and solder paste volumes must be adapted to the customer's specific production processes.

## Reflow soldering

![Image block](doc:7d59532/tier:standard/page:58/block:20)

CAUTION. Risk of device damage. Exceeding the peak temperature of the recommended soldering profile may permanently damage the device.

The final soldering temperature chosen at the factory depends on additional external factors such as the choice of soldering paste, size, thickness and properties of the base board, etc.

UBXDOC-963802114-12193 - R04 C1-Public

5 Product handling

Page 58 of 72

<!-- page 59 of 72 -->

Cblox

NEO-F10N - Integration manual

As a reference, see “IPC-7530 Guidelines for temperature profiling for mass soldering (reflow and wave) processes”, published in 2001.

A convection-type soldering oven is highly recommended over the infrared-type radiation oven. Convection-heated ovens allow precise control of the temperature, and all parts will heat up evenly, regardless of material properties, thickness of components and surface color.

CAUTION. Risk of device damage. Modules must not be soldered with a damp heat process.

To avoid falling off, the modules should be placed on the topside of the board during soldering.

For the recommended soldering profile and conditions, see Figure 19 and Table 26

| Preheating | Heating | Cooling |
| --- | --- | --- |

![Chart block](doc:7d59532/tier:standard/page:59/block:9)

Elapsed time [s]

Typical lead-free soldering profile Liquidus temperature

Figure 19: Recommended soldering profile

| Phase | Value | Details |
| --- | --- | --- |
| Preheating |  | During the initial heating of component leads and balls, residualhumidityisdriedout.Notethatthepreheatingphase does not replace prior baking procedures. |
| Temperature rise rate | Max 3 °C/s | If the temperature rise is too rapid in the preheat phase, excessive slumping may be caused. |
| Time | 60 - 120 s | If the preheating is insufficient, rather large solder balls tend to be generated. Conversely, if performed excessively, fine balls and large balls will be generated in clusters. |
| End temperature | 150 - 200 °C | Ifthetemperatureistoolow,non-meltingtendstobecaused in areas containing large heat capacity. |
| Heating - reflow |  |  |

UBXDOC-963802114-12193 - R04 C1-Public

5 Product handling

Page 59 of 72

<!-- page 60 of 72 -->

Cblox

NEO-F10N - Integration manual

| Phase | Value | Details |
| --- | --- | --- |
| Time limit above 217 °C liquidus temperature | 40 - 60 s | Thetemperaturerisesabovetheliquidustemperatureof217 °C. Avoid a sudden rise in temperature as the slump of the paste could become worse. |
| Peak reflow temperature | 245 °C |  |
| Cooling Temperature fall rate | Max 4 °C/s | A controlled cooling prevents negative metallurgical effects of the solder (solder becomes more brittle) and possible mechanicaltensionsintheproducts.Controlledcoolinghelps to achieve bright solder fillets with a good shape and low contact angle. |

Table 26: Recommended conditions for reflow soldering

## Optical inspection

After soldering the module, consider optical inspection.

## Cleaning

Do not clean with water, solvent, or ultrasonic cleaner:

Cleaning with water leads to capillary effects where water is absorbed into the gap between the baseboard and the module. The combination of residues of soldering flux and encapsulated water leads to short circuits or resistor-like interconnections between neighboring pins.

Cleaning with alcohol or other organic solvents can result in soldering flux residues flowing underneath the module into areas that are not accessible for post-cleaning inspections. The solvent also damages the sticker and the printed text on the module.

CAUTION. Risk of device damage. Ultrasonic cleaning permanently damages the module, in particular the quartz oscillators.

The best approach is to use a “no clean” soldering paste to eliminate the cleaning step after the soldering.

## Repeated reflow soldering

G Repeated reflow soldering processes or soldering the module upside down are not recommended.

A board that is populated with components on both sides may require more than one reflow soldering cycle. In such a case, the process should ensure the module is only placed on the board submitted for a single final upright reflow cycle. A module placed on the underside of the board may detach during a reflow soldering cycle due to lack of adhesion.

The module can also tolerate an additional reflow cycle for rework purposes.

## Wave soldering

Base boards with combined through-hole technology (THT) components and surface-mount technology (SMT) devices require wave soldering to solder the THT components. Only a single wave soldering process is encouraged for boards populated with modules.

## Rework

CAUTION. Risk of device damage. Using a hot air gun is an uncontrolled process. It can lead to overheating and severely damage the module. Always avoid overheating the module.

After the module is removed from the oven, clean the pins before reapplying the solder paste, placing the module in the oven and proceeding with the reflow soldering of a new module.

UBXDOC-963802114-12193 - R04 C1-Public

5 Product handling

Page 60 of 72

<!-- page 61 of 72 -->

Cblox

NEO-F10N - Integration manual

Never attempt to alter the module itself, e.g. by replacing individual components. Such actions immediately void the warranty.

## Conformal coating

Certain applications employ a conformal coating of the PCB using HumiSeal® or other related coating products. These materials affect the RF properties of the GNSS module and it is important to prevent them from flowing into the module. The RF shields do not provide 100% protection for the module from coating liquids with low viscosity. Apply the coating carefully.

## Conformal coating of the module voids the warranty.

## Casting

If casting is required, use viscose or another type of silicon pottant. The OEM is strongly advised to qualify that such processes are suitable for the module before implementing them in the production.

## Casting voids the warranty.

## Grounding metal covers

Attempts to improve grounding by soldering ground cables, wick or other forms of metal strips directly onto the EMI covers is done at customer’s own risk. The numerous ground pins should be sufficient to provide optimum immunity to interferences and noise.

u-blox provides no warranty for damages to the module caused by soldering metal cables or any other forms of metal strips directly onto the EMI covers.

## Use of ultrasonic processes

Some components on the module are sensitive to ultrasonic waves.

CAUTION. Risk of device damage. Use of any ultrasonic processes (cleaning, welding etc.) may cause damage to the receiver.

u-blox provides no warranty against damages to the module caused by ultrasonic processes.

UBXDOC-963802114-12193 - R04 C1-Public

5 Product handling

Page 61 of 72

<!-- page 62 of 72 -->

Cblox

NEO-F10N - Integration manual

## Appendix

## A Migration

u-blox is committed to ensuring that products in the same form factor are backwards compatible over several technology generations. This section describes important differences to consider when migrating from the u-blox 7/8/M8 to F10.

## A.1 Hardware differences

Table 27 lists the key hardware-related changes between NEO-F10N and NEO-7/8Q/NEO-M8 modules.

| Feature | Change | Action needed / Remarks |
| --- | --- | --- |
| Absolute maximum ratings | The max input voltage for PIOs changed to VCC + 0.3 V in NEO-F10N. | A HW change is required if the main supply voltage source is not within the specified range. |
| VCC supply range | NEO-F10N: 2.7 V - 3.6 VNEO-7M/M8M: 1.65 V - 3.6 V | A HW change is required when migrating from a NEO-7M/NEO-M8M design that uses less than 2.7 V VCC supply.Refer to the minimum and maximum VCC ramp requirements in the NEO-F10N Data sheet [1]. |
| V_BCKP supply range | NEO-F10N: 1.65 V - 3.6 VNEO-7/8Q/M8: 1.4 V - 3.6 V | A HW change is required when migrating from a NEO-7 or NEO-8Q/M8 design that uses less than 1.65 V V_BCKP supply. |
| Backup current | NEO-F10N: I_BCKP = 31 μANEO-7/8Q/M8: I_BCKP = 15 μA | In NEO-F10N, the hardware backup current into V_BCKP is higher than in the NEO-7 and NEO-8Q/M8 modules.Check that the backup battery capacity is still suitable for the target design/application requirements. |
| Software standby current | NEO-F10N: I_SWBCKP = 49 μANEO-M8N/Q/J/M: I_SWBCKP = 30 μANEO-8Q: I_SWBCKP = 22, 30 μANEO-7N: I_SWBCKP = 35 μANEO-7M: I_SWBCKP = 20 μA | In NEO-F10N, the software standby current into VCC is higher than in NEO-7 and NEO-8Q/M8 modules. |
| Power consumption | NEO-F10N peak current: 100 mANEO-7/8Q/M8 peak current: 67 mA | In u-blox F10, the average current consumption of acquisition and tracking state is smaller than in u-blox 7/8/M8. However, in terms of peak current, I_VCC is slightly higher in u-blox F10 than in u-blox 7/8/M8.Ensure that the power supply is still able to provide clean and stable voltage with the higher current demand in NEO-F10N. |
| Pin-out | 1:1 pin-out mapping with NEO-7N/8Q/M8N/Q/J/M modules. | NEO-7M/NEO-M8M (pin 14) is reserved and NEO-F10N (pin 14) is LNA_EN. NEO-7M/NEO-M8M designs can be updated to use pin 14 on NEO-F10N for external LNA and antenna control.ANT_ON (pin 14) name in NEO-7N changed to LNA_EN in NEO-8Q/M8 and NEO-F10N modules.VDD_USB (pin 7) name changed to Reserved in NEO-F10N since a USB interface is not available. |
| Communication interfaces | Due to the hardware implementation, only UART interface is available on NEO-F10N. | Communication with host can only be done by UART interface. |

UBXDOC-963802114-12193 - R04 C1-Public

Appendix

Page 62 of 72

<!-- page 63 of 72 -->

Cblox

NEO-F10N - Integration manual

| Feature | Change | Action needed / Remarks |
| --- | --- | --- |
| Dual-band L1+L5SAW-LNA-SAWRF front-end | NEO-F10N has two SAW filters and one LNA on the RF path for both L1 and L5 along with a Band 13 notch filter for L1, which allows the use of a passive antenna.NEO-F10N noise figure: 3 dB for L1 and L5NEO-7N/8Q/M8 noise figure: 2 dBNEO-7M noise figure: 3.5 dB | The Band 13 notch filter and the SAW filters used in NEO-F10N provide improved out-of-band immunity compared to NEO-7 and NEO-8Q/M8 modules. |
| Antenna, ExternalRF path | NEO-F10N supports dual-band L1/L5 GNSS signals. | An antenna supporting dual-band L1/L5 is required with NEO-F10N.Check that the RF path (antenna and filtering) is suitable for all of the GNSS signals in use. |
| Absolute maxRF input power | NEO-F10N: +15 dBmNEO-7 modules: +13 dBm | Check if the slight increase in NEO-F10N affects the target design/application requirements. |
| RESET_N | NEO-F10N: clears the RTC time and BBR contents (GNSS orbit data).NEO-7/8Q/M8: clears only the RTC time. | After resetting NEO-F10N with the RESET_N pin, the TTFF is similar to performing a cold start.After resetting the NEO-7/8Q/M8 modules, the TTFF is typically around 6 s in open-sky conditions. |
| Digital IO | External IO isolation is required.The output current of the digital IOs is 2 mA compared to 4 mA in NEO-7 and NEO-8Q/M8 modules, except for the TIMEPULSE pin, which still provides 4 mA drive strength. | Caution! Risk of receiver damage. If the IO pins are not isolated, the GNSS receiver may get permanently damaged when high current is drawn from the IO pins. Use external IO isolation to avoid receiver damage.Do not drive IOs when VCC is not supplied. Removing the VCC supply does not isolate the module IO pins from the the host device IO pins.Additional external pull-up resistors may be required to increase the drive strength of the digital IO output pin or adjust the load accordingly. |

Table 27: NEO-F10N hardware features compared to NEO-7/8Q/M8 modules

## A.2 Firmware changes

Table 28 presents a summary of the key firmware-related changes between u-blox F10 and u-blox 7/8/M8.

<table><tr><td>Feature</td><td>Change</td><td>Action needed / Remarks</td></tr><tr><td>Default baud rate</td><td>NEO-F10N: 38400 baudNEO-7N/7M/8Q/M8N/Q/J/M: 9600 baud</td><td>Code change (optional)</td></tr><tr><td colspan="3">Signals</td></tr><tr><td>Default GNSS configuration</td><td>NEO-F10N: Dual-band L1/L5 signals for GPS, Galileo, BeiDou B1C/B2a and SBAS.NEO-M8N/Q/M/J: Single-band L1 signals GPS, GLONASS, QZSS and SBAS.NEO-7/8: Single-band L1 signals GPS, QZSS and SBAS.</td><td>Code change (optional)HW change (optional): requires antenna selection or antenna frequency tuning and any additional filtering in the previous NEO-7/8/M8 designs to support L5 frequency band.</td></tr><tr><td>GPS L5</td><td>New signal</td><td>Code change (optional)</td></tr><tr><td>BeiDou B1C</td><td>New signal . BeiDou satellite IDs up to 63 supported.</td><td>Code change (optional)</td></tr><tr><td>BeiDou B1I</td><td>Not supported</td><td>Code change (optional)</td></tr><tr><td>BeiDou B2a</td><td>New signal. BeiDou satellite IDs up to 63 supported.</td><td>Code change (optional)</td></tr></table>

UBXDOC-963802114-12193 - R04 C1-Public

Appendix

Page 63 of 72

<!-- page 64 of 72 -->

Cblox

NEO-F10N - Integration manual

<table><tr><td>Feature</td><td>Change</td><td>Action needed / Remarks</td></tr><tr><td>Galileo E5a</td><td>New signal</td><td>Code change (optional)</td></tr><tr><td>GLONASS</td><td>Not supported</td><td>Code change</td></tr><tr><td>SBAS</td><td>New SBAS PRN selection : 123, 126-129, 131, 133, 136-138.</td><td>Code change (optional)</td></tr><tr><td>QZSS L1S</td><td>SLAS corrections are now applied for navigation.</td><td>Code change (optional)</td></tr><tr><td>QZSS IMES</td><td>Not supported</td><td>Code change (optional)</td></tr><tr><td>NavIC SPS L5</td><td>New signal</td><td>Code change (optional)</td></tr><tr><td colspan="3">Protocols</td></tr><tr><td>NMEA</td><td>Supports NMEA V4.11, V4.10, V4.0, V2.3, and V2.1. NMEA V4.11 is enabled by default.In M10 and F10, the NMEA GSV messages for zero signal C/NO level are grouped separately as compared to u-blox 7/8/M8. If the signal&#x27;s C/NO level is 0, the FW treats it as an unknown signal for not being able to track it and sets the &quot;signal ID&quot; to zero. The firmware creates two separate GSV message groups. The first group contains GSV messages for the tracked signals with non-zero C/NO and the second group contains GSV messages for the untracked signals with zero C/NO. In the untracked signals message group, the C/NO field is blank (i.e., ), There is no configuration to revert this to the u-blox 7/8/M8 format.</td><td>Code change (optional)</td></tr><tr><td>RTCM</td><td>Not supported</td><td>Code change</td></tr><tr><td colspan="3">General</td></tr><tr><td>Configuration concept</td><td>New configuration concept using UBX-CFG-VALSET and UBX-CFG-VALGET messages.</td><td>Code change</td></tr><tr><td>Navigation update rate</td><td>Navigation update rate up to 20 Hz for single GNSS in F10 and 10 Hz for multi-GNSS constellations.</td><td>Code change (optional)</td></tr><tr><td>Power save mode (PSM)</td><td>PSM is not supported.Hardware backup and software standby modes are supported.</td><td>Code change (optional)</td></tr><tr><td>Altitude limit</td><td>The maximum altitude limit increased to 80 000 m.</td><td>-</td></tr><tr><td>AssistNow</td><td>Simultaneous operation of AssistNow Online, Offline and Autonomous. New AssistNow Offline data download options are available. The time period can be limited to a resolution of days to reduce the data package size.</td><td>Code change (optional)</td></tr><tr><td>Geofencing</td><td>Not supported</td><td>Code change</td></tr><tr><td>Data logging</td><td>Not supported</td><td>Code change</td></tr><tr><td>Galileo return link message</td><td>Galileo search and rescue (SAR) return link message UBX-RXM-RLM.</td><td>Code change (optional)</td></tr><tr><td>RF spectrum view</td><td>New message. UBX-MON-SPAN shows in-band RF spectrum around the GNSS band. This can be used to identify potential in-band RF interference sources in the design.</td><td>Code change (optional)</td></tr><tr><td>Protection level</td><td>New feature. Real-time position accuracy estimate with 95% confidence level with the message UBX-NAV-PL.</td><td>Code change (optional)</td></tr><tr><td>CloudLocate GNSS</td><td>Not supported</td><td>Code change (optional)</td></tr><tr><td>Data batching</td><td>Not supported</td><td>Code change (optional)</td></tr><tr><td>Super-S</td><td>Not supported</td><td>Code change (optional)</td></tr><tr><td>Sky view signal masking</td><td>New feature. Includes CFG-NAVMASK-SV_MASK_* configuration keys for excluding certain satellites from navigation that are known to introduce signal distortions and performance degradation due to multi-path effects. Recommended to be used for receivers that are stationary.</td><td>Code change (optional)</td></tr></table>

UBXDOC-963802114-12193 - R04 C1-Public

Appendix

Page 64 of 72

<!-- page 65 of 72 -->

Cblox

NEO-F10N - Integration manual

| Feature | Change | Action needed/Remarks |
| --- | --- | --- |
| BeiDou SBAS | BeiDou SBAS provides SBAS services to China and nearby regions using BDS-3 type satellites. It's in testing mode and not enabled by default. | Code change (optional) |
| Security |  |  |
| Message integrity | New feature. Authentication of data output based on private/ public key pair. | Code change (optional) |
| Unique chip identifier | A unique chip identifier output in boot screen and in the UBX-SEC-UNIQID message. | Code change (optional) |
| Configuration lock | New security feature that is enabled with CFG-SEC-CFG_LOCK message for locking the receiver configuration. | Code change (optional) |
| Additional changes | Support for multiple GNSS and new signals: Galileo E1, BeiDou | Code change (optional) |
| for u-blox-7 migration | B1I, and GLONASS L1OF. Multiple GNSS assistance (MGA) with AssistNow services. Spoofing detection added. Wrist dynamic model added. Odometer to measure traveled distance with support for different user profiles. Time pulse and time mark reference changed from GPS to UTC. |  |

Table 28: NEO-F10N firmware-related changes

For more information on the supported features and messages in the u-blox F10 receiver, refer to the SPG 6.00 Release note [2] and Interface description [3].

## B Reference designs

The External components section provides the specification and recommendations for the external components that are shown in each reference design.

## B.1 Typical design

Here are some key features for a NEO-F10N typical design:

The internal RF front design in the NEO-F10N makes the module suitable for passive antenna designs where external gain is not required. It is importnat to use a good quality passive antenna to avoid performance degradation. Figure 20 shows a design using a dual band passive antenna with single feed.

• NEO-F10N has an internal SAW filter diplexer, two SAW filters and an LTE band 13 notch filter which provides high out-of-band immunity against RF interference from other sources.

Applications that require maximum sensitivity can integrate an external LNA with very low noise figure (NF). The maximum external gain must not exceed 25 dB. The external LNA can be supplied with the VCC\_RF output from NEO-F10N and controlled by the LNA\_EN signal, switching off the LNA when entering hardware backup mode, and software standby mode.

V\_BCKP supply is optional. If present, the hardware backup mode is supported. This mode maintains the RTC time and GNSS orbit data in the battery-backed RAM memory if the main supply is switched off.

If there is no backup supply, time aiding with the UBX-MGA-INI-TIME\_UTC message (optionally with a timing signal at the EXTINT pin) and the GNSS orbit data from the AssistNow services or stored on the host controller can be used to reduce the TTFF.

• For an absolute minimum design, all the PIOs (RESET\_N, EXTINT, TIMEPULSE, LNA\_EN, and SAFEBOOT\_N) can be left open.

UBXDOC-963802114-12193 - R04 C1-Public

Appendix

Page 65 of 72

<!-- page 66 of 72 -->

Cblox

NEO-F10N - Integration manual

![Image block](doc:7d59532/tier:standard/page:66/block:3)

**Figure 20: Typical design**

## B.2 Active antenna design

Here are some key features for a NEO-F10N design with an active antenna:

Designs using active antennas must not exceed 25 dB gain. For antenna gain above 25 dB, the internal LNA mode can be changed to low-gain or bypass mode.

The active antenna can be supplied either with the VCC\_RF output from NEO-F10N as shown in Figure 22 or from an external supply as shown in Figure 21. In both figures the supply is controlled by the LNA\_EN signal. The LNA\_EN signal combined with the dual MOSFET switches on/off the antenna power supply when VCC supply is removed or during software standby mode, saving the antenna power consumption during the off periods.

Designs using the VCC\_RF output to supply the GNSS antenna require a current limiter circuitry in case of short circuit on the antenna side. Figure 22 shows the required components. The R17 resistor value defines the maximum current in case of antenna short circuit.

UBXDOC-963802114-12193 - R04 C1-Public

Appendix

Page 66 of 72

<!-- page 67 of 72 -->

blox

NEO-F10N - Integration manual

![Image block](doc:7d59532/tier:standard/page:67/block:3)

Figure 21: Design with active antenna controlled by LNA\_EN signal

![Image block](doc:7d59532/tier:standard/page:67/block:5)

Figure 22: Design with active antenna supplied by VCC\_RF and controlled by LNA\_EN signal

UBXDOC-963802114-12193 - R04 C1-Public

Appendix

Page 67 of 72

<!-- page 68 of 72 -->

Cblox

NEO-F10N - Integration manual

## C External components

This section lists the recommended values for the external components in the reference designs.

## C.1 Antenna

The antenna specification in Table 29 is recommended for optimal GNSS performance.

| Parameter | Specification | Condition |
| --- | --- | --- |
| **Passive antenna element** | | |
| L1 Operational frequency band | 1559 - 1606 MHz | |
| L5 Operational frequency band | 1166 - 1186 MHz | |
| Impedance | 50 Ω | |
| VSWR | Max 2 | |
| Gain | Typical 3 dBic (at Zenith) | |
| Axial ratio | Typical 3 dB (at Zenith) | |
| Polarization | RHCP | |
| **Optional amplifier section** | | |
| L1 Operational frequency band | 1559 - 1606 MHz | |
| L5 Operational frequency band | 1166 - 1186 MHz | |
| Impedance | 50 Ω | Amplifier section input and output |
| VSWR | Max 2 | Amplifier section input and output |
| Total gain | Max 25 dB | Internal LNA normal gain (default) |
| Gain pass-band ripple | Max 3 dB | |
| Total noise figure | Typical 3 dB | Includes LNA and SAW pre-filter section |
| Group delay variation | 10 ns within each GNSS signal band, max 50 ns inter signal | |

Table 29: Antenna specifications

## C.2 Standard capacitors

Table 30 presents the recommended capacitor values for NEO-F10N.

| Name | Use | Type/Value |
| --- | --- | --- |
| C14 | RF Bias-T capacitor | 10 nF, 10%, 16 V, X7R |

Table 30: Standard capacitors

## C.3 Standard resistors

Table 31 presents the recommended resistor values for NEO-F10N.

| Name | Use | Type/Value |
| --- | --- | --- |
| R7 | Pull-up resistor at antenna switch transistor | 100 kΩ, 5%, 0.1 W |
| R8 | Antenna supply current limiter/shunt resistor | 10 Ω, 5%, 0.25 W |
| R17 | Current limiter to supply antenna through VCC_RF | 10 Ω, 5%, 0.1 W |
| R18 | Current limiter to supply antenna through VCC_RF | 220 Ω, 5%, 0.1 W |

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">3 Covers GPS L1 C/A, Galileo E1, BeiDou B1C, BeiDou B1I, and GLONASS L1OF signals. The bandwidth can be reduced if BeiDou B1I and/or GLONASS L1OF is not used.</span></small>

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">4 Covers GPS L5 CM, Galileo E5a, BeiDou B2A.</span></small>

UBXDOC-963802114-12193 - R04 C1-Public

Appendix

Page 68 of 72

<!-- page 69 of 72 -->

Cblox

NEO-F10N - Integration manual

| Name | Use | Type / Value |
| --- | --- | --- |
| R20 | Current limiter to supply antenna through VCC_RF | 2.2 kΩ, 5%, 0.1 W |

Table 31: Standard resistors

## C.4 Inductors

Table 32 presents the recommended inductor values for NEO-F10N.

| Name | Use | Type / Value | Recommended component |
| --- | --- | --- | --- |
| L4 | RF Bias-T inductor | 47 nH, 5% | Murata LQG15HS47NJ02Johanson Technology L-07W seriesAny other inductor with impedance > 500 Ω at GNSS L1 and L5 frequencies and current rating above 300 mA. |

Table 32: Recommended inductors

## C.5 Switch transistors for antenna supply

Table 33 presents the recommended switch transistors for NEO-F10N.

| Name | Manufacturer | Order no. | Comments |
| --- | --- | --- | --- |
| T1, T2 | Vishay | Si1016X-T1-GE3 | p-channel, n-channel MOSFET |
| T6, T7 | - | BC857C | BJT PNP transistor |

Table 33: Recommended parts list for the antenna supply switch transistors

UBXDOC-963802114-12193 - R04 C1-Public

Appendix

Page 69 of 72

<!-- page 70 of 72 -->

Cblox

NEO-F10N - Integration manual

## Related documents

**[1]** NEO-F10N Data sheet, [UBX-23002117](https://www.u-blox.com/docs/UBX-23002117)

**[2]** u-blox F10 SPG 6.00 Release notes, [UBXDOC-963802114-12318](https://www.u-blox.com/docs/UBXDOC-963802114-12318)

**[3]** u-blox F10 SPG 6.00 Interface description, [UBX-23002975](https://www.u-blox.com/docs/UBX-23002975)

**[4]** Joint IPC/JEDEC standard, [www.jedec.org](http://www.jedec.org/)

**[5]** AssistNow user guide, [developer.thingstream.io/guides/location-services/assistnow-userguide](https://developer.thingstream.io/guides/location-services/assistnow-user-guide)

For regular updates to u-blox documentation and to receive product change notifications please register on our homepage [https://www.u-blox.com](https://www.u-blox.com).

UBXDOC-963802114-12193 - R04 C1-Public

Related documents

Page 70 of 72

<!-- page 71 of 72 -->

Cblox

NEO-F10N - Integration manual

Revision history

| Revision | Date | Status/comments |
| --- | --- | --- |
| R01 | 05-Sep-2023 | Initial release. |
| R02 | 30-Jan-2024 | Added sections: • GPS L5 signal health status configuration • LNA_EN • Time Updated sections: • Block diagram • Pin assignment • Internal LNA modes • Migration |
| R03 | 22-May-2024 | Added sections: • BeiDou SBAS configuration • OTP memory configuration • Security • Time mark • Time pulse • Time maintenance • Protection level • Multiple GNSS assistance (MGA) • Save-on-shutdown • RF interference Updated sections: • Overview with information related to GPS L5 • RF front-end • Augmentation systems • Migration |
| R04 | 12-Dec-2024 | Added sections: • AssistNow Online • Antenna Updated section: • Multiple GNSS assistance (MGA) • Jamming and spoofing detection Removed section • OTP memory configuration • Packaging (moved to NEO-F10N Data sheet) |

UBXDOC-963802114-12193 - R04 C1-Public

Revision history

Page 71 of 72

<!-- page 72 of 72 -->

Cblox

NEO-F10N - Integration manual

## Contact

**u-blox AG**

Address: Zürcherstrasse 68

8800 Thalwil

Switzerland

For further support and contact information, visit us at [www.u-blox.com/support](https://www.u-blox.com/support).

UBXDOC-963802114-12193 - R04 C1-Public

Page 72 of 72