<!-- page 1 of 181 -->



## u-blox F10 SPG 6.00

**Standard precision GNSS firmware Protocol version 40.00**

**Interface description**

## Abstract

This document describes the interface of the u-blox F10 SPG 6.00 firmware.

www.u-blox.com

UBX-23002975 - R02 C1-Public

Cblox

<!-- page 2 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## Document information

| Title | u-bloxF10SPG6.00 |
| --- | --- |
| Subtitle | StandardprecisionGNSSfirmware |
| Documenttype | Interfacedescription |
| Documentnumber | UBX-23002975 |
| Revisionanddate | R02 13-Jun-2024 |
| Disclosurerestriction | C1-Public |

u-blox or third parties may hold intellectual property rights in the products, names, logos and designs included in this document. Copying, reproduction, or modification of this document or any part thereof is only permitted with the express written permission of u-blox. Disclosure to third parties is permitted for clearly public documents only.

The information contained herein is provided "as is" and u-blox assumes no liability for its use. No warranty, either express or implied, is given, including but not limited to, with respect to the accuracy, correctness, reliability and fitness for a particular purpose of the information. This document may be revised by u-blox at any time without notice. For the most recent documents, visit www.u-blox.com.

Copyright © 2024, u-blox AG.

UBX-23002975 - R02 C1-Public

Page 2 of 181

<!-- page 3 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## Contents

- 1 General information....10
- 1.1 Document overview....10
- 1.2 Firmware and protocol versions....10
- 1.3 Receiver configuration....12
- 1.4 Message naming....12
- 1.5 GNSS, satellite, and signal identifiers....13
- 1.5.1 Overview....13
- 1.5.2 GNSS identifiers....13
- 1.5.3 Satellite identifiers....14
- 1.5.4 Signal identifiers....15
- 1.6 Message types....16
- 2 NMEA protocol....17
- 2.1 NMEA frame structure....17
- 2.2 NMEA protocol configuration....17
- 2.3 NMEA-proprietary messages....18
- 2.4 NMEA multi-GNSS operation....19
- 2.5 NMEA data fields....19
- 2.5.1 NMEA Talker ID....19
- 2.5.2 NMEA extra fields....20
- 2.5.3 NMEA latitude and longitude format....20
- 2.5.4 NMEA GNSS, satellite, and signal numbering....20
- 2.5.5 NMEA position fix flags....20
- 2.5.6 NMEA output of invalid or unknown data....21
- 2.6 NMEA messages overview....22
- 2.7 Standard messages....22
- 2.7.1 DTM....22
- 2.7.1.1 Datum reference....23
- 2.7.2 GAQ....23
- 2.7.2.1 Poll a standard message (Talker ID GA)....23
- 2.7.3 GBQ....24
- 2.7.3.1 Poll a standard message (Talker ID GB)....24
- 2.7.4 GBS....24
- 2.7.4.1 GNSS satellite fault detection....24
- 2.7.5 GGA....25
- 2.7.5.1 Global positioning system fix data....25
- 2.7.6 GLL....26
- 2.7.6.1 Latitude and longitude, with time of position fix and status....26
- 2.7.7 GLQ....26
- 2.7.7.1 Poll a standard message (Talker ID GL)....26
- 2.7.8 GNQ....27
- 2.7.8.1 Poll a standard message (Talker ID GN)....27
- 2.7.9 GNS....27
- 2.7.9.1 GNSS fix data....27
- 2.7.10 GPQ....28
- 2.7.10.1 Poll a standard message (Talker ID GP)....28

UBX-23002975 - R02 C1-Public

Contents

Page 3 of 181

<!-- page 4 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

- 2.7.11 GQQ....28
- 2.7.11.1 Poll a standard message (Talker ID GQ)....29
- 2.7.12 GRS....29
- 2.7.12.1 GNSS range residuals....29
- 2.7.13 GSA....30
- 2.7.13.1 GNSS DOP and active satellites....30
- 2.7.14 GST....30
- 2.7.14.1 GNSS pseudorange error statistics....30
- 2.7.15 GSV....31
- 2.7.15.1 GNSS satellites in view....31
- 2.7.16 RLM....32
- 2.7.16.1 Return link message (RLM)....32
- 2.7.17 RMC....32
- 2.7.17.1 Recommended minimum data....33
- 2.7.18 TXT....33
- 2.7.18.1 Text transmission....33
- 2.7.19 VLW....34
- 2.7.19.1 Dual ground/water distance....34
- 2.7.20 VTG....35
- 2.7.20.1 Course over ground and ground speed....35
- 2.7.21 ZDA....35
- 2.7.21.1 Time and date....35
- 2.8 PUBX messages....36
- 2.8.1 CONFIG (PUBX,41)....36
- 2.8.1.1 Set protocols and baud rate....36
- 2.8.2 POSITION (PUBX,00)....36
- 2.8.2.1 Poll a PUBX,00 message....37
- 2.8.2.2 Lat/Long position data....37
- 2.8.3 RATE (PUBX,40)....38
- 2.8.3.1 Set NMEA message output rate....38
- 2.8.4 SVSTATUS (PUBX,03)....39
- 2.8.4.1 Poll a PUBX,03 message....39
- 2.8.4.2 Satellite status....39
- 2.8.5 TIME (PUBX,04)....40
- 2.8.5.1 Poll a PUBX,04 message....40
- 2.8.5.2 Time of day and clock information....40
- UBX protocol....42
- 3.1 UBX protocol key features....42
- 3.2 UBX frame structure....42
- 3.3 UBX payload definition rules....43
- 3.3.1 UBX structure packing....43
- 3.3.2 UBX reserved elements....43
- 3.3.3 UBX undefined values....43
- 3.3.4 UBX conditional values....43
- 3.3.5 UBX data types....43
- 3.3.6 UBX fields scale and unit....44
- 3.3.7 UBX repeated fields....44
- 3.3.8 UBX payload decoding....45
- 3.4 UBX checksum....45
- 3.5 UBX message flow....45

UBX-23002975 - R02 C1-Public

Contents

Page 4 of 181

<!-- page 5 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

- 3.5.1 UBX acknowledgement....45
- 3.5.2 UBX polling mechanism....45
- 3.6 GNSS, satellite, and signal numbering....46
- 3.7 UBX message example....46
- 3.8 UBX messages overview....47
- 3.9 UBX-ACK (0x05)....49
- 3.9.1 UBX-ACK-ACK (0x05 0x01)....49
- 3.9.1.1 Message acknowledged....49
- 3.9.2 UBX-ACK-NAK (0x05 0x00)....50
- 3.9.2.1 Message not acknowledged....50
- 3.10 UBX-CFG (0x06)....50
- 3.10.1 UBX-CFG-CFG (0x06 0x09)....50
- 3.10.1.1 Clear, save and load configurations....50
- 3.10.2 UBX-CFG-RST (0x06 0x04)....51
- 3.10.2.1 Reset receiver / Clear backup data structures....51
- 3.10.3 UBX-CFG-VALDEL (0x06 0x8c)....52
- 3.10.3.1 Delete configuration item values....52
- 3.10.3.2 Delete configuration item values (with transaction)....53
- 3.10.4 UBX-CFG-VALGET (0x06 0x8b)....54
- 3.10.4.1 Get configuration items....54
- 3.10.4.2 Configuration items....55
- 3.10.5 UBX-CFG-VALSET (0x06 0x8a)....56
- 3.10.5.1 Set configuration item values....56
- 3.10.5.2 Set configuration item values (with transaction)....57
- 3.11 UBX-INF (0x04)....58
- 3.11.1 UBX-INF-DEBUG (0x04 0x04)....58
- 3.11.1.1 ASCII output with debug contents....58
- 3.11.2 UBX-INF-ERROR (0x04 0x00)....58
- 3.11.2.1 ASCII output with error contents....58
- 3.11.3 UBX-INF-NOTICE (0x04 0x02)....59
- 3.11.3.1 ASCII output with informational contents....59
- 3.11.4 UBX-INF-TEST (0x04 0x03)....59
- 3.11.4.1 ASCII output with test contents....59
- 3.11.5 UBX-INF-WARNING (0x04 0x01)....59
- 3.11.5.1 ASCII output with warning contents....59
- 3.12 UBX-MGA (0x13)....60
- 3.12.1 UBX-MGA-ACK (0x13 0x60)....60
- 3.12.1.1 Multiple GNSS acknowledge message....60
- 3.12.2 UBX-MGA-ANO (0x13 0x20)....60
- 3.12.2.1 Multiple GNSS AssistNow Offline assistance....60
- 3.12.3 UBX-MGA-BDS (0x13 0x03)....61
- 3.12.3.1 BeiDou ephemeris assistance for satellites svld 1..37....61
- 3.12.3.2 BeiDou almanac assistance....62
- 3.12.3.3 BeiDou health assistance....63
- 3.12.3.4 BeiDou UTC assistance....63
- 3.12.3.5 BeiDou ionosphere assistance....64
- 3.12.4 UBX-MGA-DBD (0x13 0x80)....64
- 3.12.4.1 Poll the navigation database....64
- 3.12.4.2 Navigation database dump entry....65
- 3.12.5 UBX-MGA-FLASH (0x13 0x21)....65

UBX-23002975 - R02 C1-Public

Contents

Page 5 of 181

<!-- page 6 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

- 3.12.5.1 Transfer MGA-ANO data block to flash....65
- 3.12.5.2 Finish flashing MGA-ANO data....66
- 3.12.5.3 Acknowledge last FLASH-DATA or -STOP....66
- 3.12.6 UBX-MGA-GAL (0x13 0x02)....66
- 3.12.6.1 Galileo ephemeris assistance....67
- 3.12.6.2 Galileo almanac assistance....68
- 3.12.6.3 Galileo GPS time offset assistance....69
- 3.12.6.4 Galileo UTC assistance....69
- 3.12.7 UBX-MGA-GPS (0x13 0x00)....70
- 3.12.7.1 GPS ephemeris assistance....70
- 3.12.7.2 GPS almanac assistance....71
- 3.12.7.3 GPS health assistance....72
- 3.12.7.4 GPS UTC assistance....72
- 3.12.7.5 GPS ionosphere assistance....73
- 3.12.8 UBX-MGA-INI (0x13 0x40)....73
- 3.12.8.1 Initial position assistance....73
- 3.12.8.2 Initial position assistance....74
- 3.12.8.3 Initial time assistance....74
- 3.12.8.4 Initial time assistance....75
- 3.12.8.5 Initial clock drift assistance....76
- 3.12.8.6 Initial frequency assistance....77
- 3.12.8.7 Earth orientation parameters assistance....77
- 3.12.9 UBX-MGA-QZSS (0x13 0x05)....78
- 3.12.9.1 QZSS ephemeris assistance....78
- 3.12.9.2 QZSS almanac assistance....79
- 3.12.9.3 QZSS health assistance....79
- 3.13 UBX-MON (0x0a)....80
- 3.13.1 UBX-MON-COMMS (0x0a 0x36)....80
- 3.13.1.1 Communication port information....80
- 3.13.2 UBX-MON-GNSS (0x0a 0x28)....81
- 3.13.2.1 Information message major GNSS selection....81
- 3.13.3 UBX-MON-HW3 (0x0a 0x37)....82
- 3.13.3.1 I/O pin status....82
- 3.13.4 UBX-MON-PATCH (0x0a 0x27)....83
- 3.13.4.1 Installed patches....83
- 3.13.5 UBX-MON-RCVRSTAT (0x0a 0x40)....84
- 3.13.5.1 Status of the configurable receiver features....84
- 3.13.6 UBX-MON-RF (0x0a 0x38)....91
- 3.13.6.1 RF information....91
- 3.13.7 UBX-MON-RXR (0x0a 0x21)....92
- 3.13.7.1 Receiver status information....93
- 3.13.8 UBX-MON-SPAN (0x0a 0x31)....93
- 3.13.8.1 Signal characteristics....93
- 3.13.9 UBX-MON-VER (0x0a 0x04)....94
- 3.13.9.1 Poll receiver and software version....94
- 3.13.9.2 Receiver and software version....94
- 3.14 UBX-NAV (0x01)....94
- 3.14.1 UBX-NAV-AOPSTATUS (0x01 0x60)....94
- 3.14.1.1 AssistNow Autonomous status....95
- 3.14.2 UBX-NAV-CLOCK (0x01 0x22)....95

UBX-23002975 - R02 C1-Public

Contents

Page 6 of 181

<!-- page 7 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

- 3.14.2.1 Clock solution....95
- 3.14.3 UBX-NAV-COV (0x01 0x36)....95
- 3.14.3.1 Covariance matrices....95
- 3.14.4 UBX-NAV-DOP (0x01 0x04)....96
- 3.14.4.1 Dilution of precision....96
- 3.14.5 UBX-NAV-EOE (0x01 0x61)....97
- 3.14.5.1 End of epoch....97
- 3.14.6 UBX-NAV-ODO (0x01 0x09)....97
- 3.14.6.1 Odometer solution....97
- 3.14.7 UBX-NAV-ORB (0x01 0x34)....97
- 3.14.7.1 GNSS orbit database info....98
- 3.14.8 UBX-NAV-PL (0x01 0x62)....99
- 3.14.8.1 Protection level information....99
- 3.14.9 UBX-NAV-POSECEF (0x01 0x01)....101
- 3.14.9.1 Position solution in ECEF....101
- 3.14.10 UBX-NAV-POSLLH (0x01 0x02)....101
- 3.14.10.1 Geodetic position solution....102
- 3.14.11 UBX-NAV-PVT (0x01 0x07)....102
- 3.14.11.1 Navigation position velocity time solution....102
- 3.14.12 UBX-NAV-RESETODO (0x01 0x10)....105
- 3.14.12.1 Reset odometer....105
- 3.14.13 UBX-NAV-SAT (0x01 0x35)....105
- 3.14.13.1 Satellite information....105
- 3.14.14 UBX-NAV-SBAS (0x01 0x32)....107
- 3.14.14.1 SBAS status data....107
- 3.14.15 UBX-NAV-SIG (0x01 0x43)....108
- 3.14.15.1 Signal information....108
- 3.14.16 UBX-NAV-SLAS (0x01 0x42)....110
- 3.14.16.1 QZSS L1S SLAS status data....110
- 3.14.17 UBX-NAV-STATUS (0x01 0x03)....111
- 3.14.17.1 Receiver navigation status....111
- 3.14.18 UBX-NAV-TIMEBDS (0x01 0x24)....112
- 3.14.18.1 BeiDou time solution....112
- 3.14.19 UBX-NAV-TIMEGAL (0x01 0x25)....113
- 3.14.19.1 Galileo time solution....113
- 3.14.20 UBX-NAV-TIMEGPS (0x01 0x20)....113
- 3.14.20.1 GPS time solution....114
- 3.14.21 UBX-NAV-TIMELS (0x01 0x26)....114
- 3.14.21.1 Leap second event information....114
- 3.14.22 UBX-NAV-TIMENAVIC (0x01 0x63)....115
- 3.14.22.1 NavIC time solution....116
- 3.14.23 UBX-NAV-TIMEQZSS (0x01 0x27)....116
- 3.14.23.1 QZSS time solution....116
- 3.14.24 UBX-NAV-TIMEUTC (0x01 0x21)....117
- 3.14.24.1 UTC time solution....117
- 3.14.25 UBX-NAV-VELECEF (0x01 0x11)....118
- 3.14.25.1 Velocity solution in ECEF....118
- 3.14.26 UBX-NAV-VELNED (0x01 0x12)....118
- 3.14.26.1 Velocity solution in NED frame....118
- 15 UBX-RXM (0x02)....119

UBX-23002975 - R02 C1-Public

Contents

Page 7 of 181

<!-- page 8 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

- 3.15.1 UBX-RXM-MEASX (0x02 0x14)....119
- 3.15.1.1 Satellite measurements for RRLP....119
- 3.15.2 UBX-RXM-PMREQ (0x02 0x41)....120
- 3.15.2.1 Power management request....120
- 3.15.3 UBX-RXM-RLM (0x02 0x59)....121
- 3.15.3.1 Galileo SAR short-RLM report....121
- 3.15.3.2 Galileo SAR long-RLM report....122
- 3.15.4 UBX-RXM-SFRBX (0x02 0x13)....122
- 3.15.4.1 Broadcast navigation data subframe....122
- 3.16 UBX-SEC (0x27)....123
- 3.16.1 UBX-SEC-SIG (0x27 0x09)....123
- 3.16.1.1 Signal security information....123
- 3.16.2 UBX-SEC-SIGLOG (0x27 0x10)....123
- 3.16.2.1 Signal security log....124
- 3.16.3 UBX-SEC-UNIQID (0x27 0x03)....124
- 3.16.3.1 Unique chip ID....124
- 3.17 UBX-TIM (0x0d)....125
- 3.17.1 UBX-TIM-TM2 (0x0d 0x03)....125
- 3.17.1.1 Time mark data....125
- 3.17.2 UBX-TIM-TP (0x0d 0x01)....126
- 3.17.2.1 Time pulse time data....126
- 3.17.3 UBX-TIM-VRFY (0x0d 0x06)....127
- 3.17.3.1 Sourced time verification....127
- 3.18 UBX-UPD (0x09)....128
- 3.18.1 UBX-UPD-SOS (0x09 0x14)....128
- 3.18.1.1 Poll backup restore status....128
- 3.18.1.2 Create backup in flash....128
- 3.18.1.3 Clear backup in flash....128
- 3.18.1.4 Backup creation acknowledge....129
- 3.18.1.5 System restored from backup....129
- Configuration interface....130
- 4.1 Configuration database....130
- 4.2 Configuration items....130
- 4.3 Configuration layers....131
- 4.4 Configuration interface access....132
- 4.4.1 UBX protocol interface....132
- 4.5 Configuration data....132
- 4.6 Configuration transactions....133
- 4.7 Configuration reset behavior....134
- 4.8 Configuration overview....134
- 4.9 Configuration reference....135
- 4.9.1 CFG-ANA: AssistNow Autonomous and Offline configuration....135
- 4.9.2 CFG-BDS: BeiDou system configuration....135
- 4.9.3 CFG-HW: Hardware configuration....135
- 4.9.4 CFG-I2C: Configuration of the I2C interface....136
- 4.9.5 CFG-I2CINPROT: Input protocol configuration of the I2C interface....137
- 4.9.6 CFG-I2COUTPROT: Output protocol configuration of the I2C interface....137
- 4.9.7 CFG-INFMSG: Information message configuration....137
- 4.9.8 CFG-ITFM: Jamming and interference monitor configuration....138
- 4.9.9 CFG-MOT: Motion detector configuration....138

UBX-23002975 - R02 C1-Public

Contents

Page 8 of 181

<!-- page 9 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

- 4.9.10 CFG-MSGOUT: Message output configuration....138
- 4.9.11 CFG-NAVMASK: Satellite Mask Configuration....145
- 4.9.12 CFG-NAVSPG: Standard precision navigation configuration....148
- 4.9.13 CFG-NMEA: NMEA protocol configuration....152
- 4.9.14 CFG-ODO: Odometer and low-speed course over ground filter configuration....154
- 4.9.15 CFG-QZSS: QZSS system configuration....154
- 4.9.16 CFG-RATE: Navigation and measurement rate configuration....155
- 4.9.17 CFG-RINV: Remote inventory....155
- 4.9.18 CFG-SBAS: SBAS configuration....156
- 4.9.19 CFG-SEC: Security configuration....158
- 4.9.20 CFG-SIGNAL: Satellite systems (GNSS) signal configuration....158
- 4.9.21 CFG-SPI: Configuration of the SPI interface....159
- 4.9.22 CFG-SPIINPROT: Input protocol configuration of the SPI interface....159
- 4.9.23 CFG-SPIOUTPROT: Output protocol configuration of the SPI interface....159
- 4.9.24 CFG-TP: Time pulse configuration....159
- 4.9.25 CFG-TXREADY: TX ready configuration....161
- 4.9.26 CFG-UART1: Configuration of the UART1 interface....162
- 4.9.27 CFG-UART1INPROT: Input protocol configuration of the UART1 interface....163
- 4.9.28 CFG-UART1OUTPROT: Output protocol configuration of the UART1 interface....163
- 4.10 Legacy UBX message fields reference....163
- Configuration defaults....168
- Related documents....179
- Revision history....180

UBX-23002975 - R02 C1-Public

Contents

Page 9 of 181

<!-- page 10 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## 1 General information

## 1.1 Document overview

This document describes the interface of the Standard precision GNSS firmware. The interface consists of the following parts:

• NMEA protocol

• UBX protocol

Configuration interface

Some of the features described here may not be available in the receiver, and some may require specific configurations to be enabled. See the applicable data sheet for availability of the features and the integration manual for instructions for enabling them.

Previous versions of u-blox receiver documentation combined general receiver description and interface specification. In the current documentation the receiver description is included in the integration manual.

See also Related documents.

## 1.2 Firmware and protocol versions

u-blox receivers execute firmware from internal ROM or load an external image and execute it from internal code-RAM.

• If the product does not have internal code-RAM, the firmware runs from the ROM.

If the product has internal code-RAM but an external image is not available, the firmware runs from the ROM. Some products have only limited ROM and enter boot mode with no GNSS function if an external image is not available.

• If the external firmware image is stored in a flash memory, it is loaded into the code-RAM before execution.

• In some products, the firmware image can be stored in the host system and loaded into the code-RAM from there.

The location and the version of the currently running firmware can be found in the boot screen and in the UBX-MON-VER message. If the firmware has been loaded from the flash memory or from the host processor, it is indicated by text "EXT". Running from the internal ROM is indicated by text "ROM". When the receiver is started, the boot screen is output automatically in UBX-INF-NOTICE or NMEA-Standard-TXT messages if configured using CFG-INFMSG. The UBX-MON-VER message can be polled using the UBX polling mechanism.

The following u-center screenshots show an example of boot information:

UBX-23002975 - R02 C1-Public

1 General information

Page 10 of 181

<!-- page 11 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Time (PC) | Message |
| --- | --- |
| 09:32:45 | $GNTXT,01,01,02,u-blox AG - www.u-blox.com*4E |
| 09:32:45 | $GNTXT,01,01,02,HW UBX 10 000S0000*55 |
| 09:32:45 | $GNTXT,01,01,02,ROM SPG 5.10 (c00d69)*22 |
| 09:32:45 | $GNTXT,01,01,02,FWVER=SPG 5.10*44 |
| 09:32:45 | $GNTXT,01,01,02,PROTVER=34.10*11 |
| 09:32:45 | $GNTXT,01,01,02,CHIPID=000000D0D69D0F7A55*BB |
| 09:32:45 | $GNTXT,01,01,02,MOD=EVK-M101*20 |
| 09:32:45 | $GNTXT,01,01,02,GPS;GLO;GAL;BDS*77 |
| 09:32:45 | $GNTXT,01,01,02,SBAS;QZSS*60 |
| 09:32:45 | $GNTXT,01,01,02,ANTSUPERV=*22 |
| 09:32:45 | $GNTXT,01,01,02,ANTSTATUS=DONTKNOW*2D |
| 09:32:45 | $GNTXT,01,01,02,PF=FFFFF*3E |

| UBX - MON (Monitor) - VER (Version) |
| --- |
| Software Version |
| ROM SPG 5.10 (c00d69) |
| Hardware Version |
| 000S0000 |
| Extension(s) |
| FWVER=SPG 5.10PROTVER=34.10MOD=EVK-M101GPS:GLO;GAL;BDSSBAS:QZSS |

The following information is available (✓) from the boot screen (B) and the UBX-MON-VER message (M):

<table><tr><td colspan="2">B M Example</td><td>Information</td></tr><tr><td>✓</td><td>u-blox AG - www.u-blox.com</td><td>Start of the boot screen.</td></tr><tr><td>✓</td><td>HW UBX 10 00000000✓ 00000000</td><td>Hardware version of the u-blox receiver.</td></tr><tr><td>✓</td><td>✓ ROM SPG 5.10 (000000)</td><td>Firmware version and revision identifier.</td></tr><tr><td>✓</td><td>✓ ROM BASE 0x118B2060</td><td>Revision of the underlying boot loader firmware in ROM.</td></tr><tr><td>✓</td><td>✓ FWVER=SPG 5.10</td><td>Product firmware version, where:SPG = Standard precision GNSS productHPG = High precision GNSS productADR = Automotive dead reckoning productTIM = Time sync productLAP = Lane accurate positioning productHPS = High precision sensor fusion productDBS = Dual band standard precisionMDR = Multi-mode dead reckoning productPMP = L-Band Inmarsat point-to-multipoint receiverQZS = QZSS L6 centimeter level augmentation service (CLAS) message receiverDBD = Dual band dead reckoning productLDR = ROM bootloader, no GNSS functionality</td></tr><tr><td>✓</td><td>✓ PROTVER=34.00</td><td>Supported protocol version.</td></tr><tr><td>✓</td><td>CHIPID=000000D0D69D0F7A54</td><td>Unique chip identification number.</td></tr><tr><td>✓</td><td>✓ MOD=EVK-M101</td><td>Module name.</td></tr><tr><td>✓</td><td>✓ GPS;GLO;GAL;BDS</td><td>List of supported major GNSS (see GNSS identifiers).</td></tr><tr><td>✓</td><td>✓ SBAS;QZSS</td><td>List of supported augmentation systems (see GNSS identifiers).</td></tr><tr><td>✓</td><td>✓ NAVIC</td><td>Extended list of supported GNSS (see GNSS identifiers).</td></tr><tr><td>✓</td><td>ANTSUPERV=AC SD PDoS SR</td><td>Configuration of the antenna supervisor, where:AC = Active antenna control enabledSD = Short circuit detection enabledOD = Open circuit detection enabledPDoS = Short circuit power down logic enabledSR = Automatic recovery from short state enabled</td></tr></table>

UBX-23002975 - R02 C1-Public

1 General information

Page 11 of 181

<!-- page 12 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td colspan="2">B M Example</td><td>Information</td></tr><tr><td>✓</td><td>PF=FFF79</td><td>Product configuration.</td></tr><tr><td>✓</td><td>BD=E01C</td><td>GNSS band configuration.</td></tr></table>

The "FWVER" product firmware version indicates which firmware is currently running. This is referred to as "firmware version" in this and other documents.

心 The version and revision numbers should only be used to identify a known firmware version. They are not necessarily numeric nor are they guaranteed to increase with later firmware versions.

All u-blox receivers output the start text, hardware version, and firmware version and revision. Some of the other entries in the boot screen example may be omitted.

The product firmware version and revision relate to the protocol version:

| Firmware version | Version and revision identifier | Protocol version |
| --- | --- | --- |
| SPG 6.00 | EXT SPGL1L5 6.00 (041e8a) | 40.00 |
| SPG 6.00 | ROM SPGL1L5 6.00 (041e8a) | 40.00 |

## 1.3 Receiver configuration

u-blox positioning receivers are fully configurable with UBX protocol messages. The configuration used by the receiver during normal operation is called the "current configuration". The current configuration can be changed during normal operation by sending UBX-CFG-VALSET messages over any I/O port. The receiver changes its current configuration immediately after receiving a configuration message. The receiver always uses the current configuration only.

The current configuration is loaded from permanent configuration hard-coded in the receiver firmware (the defaults) and from non-volatile memory (user configuration) on startup of the receiver. Changes made to the current configuration at run-time will be lost when there is a power cycle, a hardware reset or a (complete) controlled software reset (see Configuration reset behavior).

See Configuration interface for a detailed description of the receiver configuration system, the explanation of the configuration concept and its principles and interfaces.

The configuration interface has changed from earlier u-blox positioning receivers. There is some backwards compatibility provided in UBX-CFG configuration messages. Users are strongly advised to only use the Configuration interface. See also Legacy UBX message fields reference.

See the integration manual for a basic receiver configuration most commonly used.

## 1.4 Message naming

Message names are written in full with the parts of the name separated by hyphens ("-"). The full message name consists of the protocol name (e.g. UBX), the class name (e.g. NAV) and the message name (e.g. PVT). For example, the receiver software version information message is referred to as UBX-MON-VER. Similarly, the NMEA-Standard-GGA is the NMEA standard message (sentence) with the global positioning fix data.

References to fields of the message add the field name separated by a dot ("."), e.g. UBX-MON-VER.swVersion.

Some messages use a fourth level of naming, called the message version. One example is the UBX-MGA-GPS message for GPS assistance data, which exists in versions for ephemerides (UBX-MGA-GPS-EPH) and almanacs (UBX-MGA-GPS-ALM).

UBX-23002975 - R02 C1-Public

1 General information

Page 12 of 181

<!-- page 13 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

Names of configuration items are of the form CFG-GROUP-ITEM. For example, CFG-NAVSPG-DYNMODEL refers to the navigation dynamic platform model the receiver uses. Constants add a fourth level to the item name, such as CFG-NAVSPG-DYNMODEL-AUTOMOT for the automotive platform model. In the context of describing an item's value, only the last part of the constant name can be used (e.g. "set CFG-NAVSPG-DYNMODEL to PORT for portable applications").

## 1.5 GNSS, satellite, and signal identifiers

## 1.5.1 Overview

Many UBX protocol messages contain infomation about specific satellites. Any single satellite can be identified by a gnssId field indicating the GNSS the satellite is part of and an svId (SV for space vehicle) field indicating the number of the satellite in that system. Usually, the svId is the native number associated with the satellite in the specific GNSS. For example, the Galileo SV4 is identified as gnssId 2, svId 4, while the GPS SV4 is gnssId 0, svId 4.

Some legacy UBX protocol messages combine both the satellite number and the GNSS identification into a one-byte (type U1) field. See the single svid mapping in Satellite identifiers to identify the corresponding GNSS and satellite.

GLONASS satellites can be tracked before they have been identified. In UBX messages, the unknown satellites are reported with svId 255. In NMEA messages, the unknown satellites are null (empty) fields. Product-related documentation and u-center use R? to label unidentified GLONASS satellites.

Signal identifiers are used when different signals from the same GNSS satellite need to be distinguished (e.g. in the UBX-NAV-SIG message). A separate sigId field identifies the signal. These signal identifiers are only valid when combined with a GNSS identifier (gnssId field).

The NMEA protocol (version 4.10 and later) identifies GNSS satellites with a one-digit system ID and a two-digit satellite number. u-blox receivers support this method in their NMEA output when "strict" SV numbering is selected. In most cases this is the default setting, but it can be checked or changed using the Configuration interface (see also NMEA GNSS, satellite, and signal numbering).

In order to support some GNSS (e.g. BeiDou, Galileo, QZSS), which are not supported by some or all NMEA protocol versions, an "extended" SV numbering scheme can be enabled. This uses the NMEA-defined numbers where possible but adds other number ranges to support other GNSS. Note however that these non-standard extensions require 3-digit numbers, which may not be supported by some NMEA parsing software. For example, QZSS satellites use numbers in the range 193 to 202.

The NMEA standard defines signal identifiers to distinguish different signals sent by a single GNSS satellite (e.g. L2 CL and CM). u-blox positioning receivers use those identifiers for signal identification, as far as the corresponding standard is supported in a particular product.

![Image block](doc:a53f2ec/tier:standard/page:13/block:13)

Note that the following sections are a generic overview for different u-blox positioning receivers. A particular product may not support all of the described GNSS identifiers, satellite numbers, signal identifiers or combinations thereof.

## 1.5.2 GNSS identifiers

Table 1 lists each GNSS along with the GNSS identifier (UBX protocol), the NMEA system identifiers (NMEA protocol), and abbreviations used in this document:

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">1 While not defined by NMEA 4.10, in this mode, u-blox receivers use system ID 4 for BeiDou and, if extended satellite numbering is enabled, system ID 1 for QZSS.</span></small>

UBX-23002975 - R02 C1-Public

1 General information

Page 13 of 181

<!-- page 14 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| GNSS | Abbrevi | ations | UBX gnssId | 2.3 - 4.0 | NMEA system ID4.10 | 4.11 |
| --- | --- | --- | --- | --- | --- | --- |
| GPS | GPS | G | 0 | 1 | 1 | 1 |
| SBAS | SBAS | S | 1 | 1 | 1 | 1 |
| Galileo | GAL | E | 2 | n/a | 3 | 3 |
| BeiDou | BDS | B | 3 | n/a | (4)<sup>1</sup> | 4 |
| QZSS | QZSS | Q | 5 | n/a | (1)<sup>1</sup> | 5 |
| GLONASS | GLO | R | 6 | 2 | 2 | 2 |
| NavIC | NavIC | N | 7 | n/a | n/a | 6 |

Table 1: GNSS identifiers

See also NMEA Talker ID.

## 1.5.3 Satellite identifiers

The satellite numbering scheme for the UBX protocol is provided in Table 2. The satellite numbering scheme for the NMEA protocol is provided in Table 3.

<table><tr><td>GNSS</td><td>SV Range</td><td>gnssld:svld</td><td>single svid</td></tr><tr><td>GPS</td><td>G1-G32</td><td>0:1-32</td><td>1-32</td></tr><tr><td>SBAS</td><td>S120-S158</td><td>1:120-158</td><td>120-158</td></tr><tr><td>Galileo</td><td>E1-E36</td><td>2:1-36</td><td>211-246</td></tr><tr><td rowspan="3">BeiDou</td><td>B1-B5</td><td>3:1-5</td><td>159-163</td></tr><tr><td>B6-B37</td><td>3:6-37</td><td>33-64</td></tr><tr><td>B38-B63</td><td>3:38-63</td><td>n/a</td></tr><tr><td>QZSS</td><td>Q1-Q10</td><td>5:1-10</td><td>193-202</td></tr><tr><td rowspan="2">GLONASS</td><td>R1-R32</td><td>6:1-32</td><td>65-96</td></tr><tr><td>R?</td><td>6:255</td><td>255</td></tr><tr><td>NavIC</td><td>N1-N7</td><td>7:1-7</td><td>247-253</td></tr><tr><td></td><td>N8-N14</td><td>7:8-14</td><td>n/a</td></tr></table>

**Table 2: UBX protocol satellite numbering scheme**

<table><tr><td rowspan="2">GNSS</td><td rowspan="2">SV Range</td><td colspan="2">NMEA 2.3 - 4.0</td><td colspan="2">NMEA 4.10</td><td colspan="2">NMEA 4.11</td></tr><tr><td>strict</td><td>extended</td><td>strict</td><td>extended</td><td>strict</td><td>extended</td></tr><tr><td>GPS</td><td>G1-G32</td><td>1-32</td><td>1-32</td><td>1-32</td><td>1-32</td><td>1-32</td><td>1-32</td></tr><tr><td>SBAS</td><td>S120-S158</td><td>33-64</td><td>33-64, 152-158</td><td>33-64</td><td>33-64, 152-158</td><td>33-64</td><td>33-64, 152-158</td></tr><tr><td>Galileo</td><td>E1-E36</td><td>n/a</td><td>301-336</td><td>1-36</td><td>1-36</td><td>1-36</td><td>1-36</td></tr><tr><td rowspan="3">BeiDou</td><td>B1-B5</td><td>n/a</td><td>401-405</td><td>1-5</td><td>1-5</td><td>1-5</td><td>1-5</td></tr><tr><td>B6-B37</td><td>n/a</td><td>406-437</td><td>6-37</td><td>6-37</td><td>6-37</td><td>6-37</td></tr><tr><td>B38-B63</td><td>n/a</td><td>438-463</td><td>38-63</td><td>38-63</td><td>38-63</td><td>38-63</td></tr><tr><td>QZSS</td><td>Q1-Q10</td><td>n/a</td><td>193-202</td><td>n/a</td><td>193-202</td><td>1-10</td><td>1-10</td></tr><tr><td rowspan="2">GLONASS</td><td>R1-R32</td><td>65-96</td><td>65-96</td><td>65-96</td><td>65-96</td><td>65-96</td><td>65-96</td></tr><tr><td>R?</td><td>null</td><td>null</td><td>null</td><td>null</td><td>null</td><td>null</td></tr><tr><td>NavIC</td><td>N1-N7</td><td>n/a</td><td>n/a</td><td>n/a</td><td>n/a</td><td>1-7</td><td>1-7</td></tr></table>

UBX-23002975 - R02 C1-Public

1 General information

Page 14 of 181

<!-- page 15 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tbody><tr><td colspan="4">NMEA 2.3 - 4.0</td><td colspan="2">NMEA 4.10</td><td colspan="2">NMEA 4.11</td></tr><tr><td>GNSS</td><td>SVRange</td><td>strict</td><td>extended</td><td>strict</td><td>extended</td><td>strict</td><td>extended</td></tr><tr><td></td><td>N8-N14</td><td>n/a</td><td>n/a</td><td>n/a</td><td>n/a</td><td>8-14</td><td>8-14</td></tr></tbody></table>

Table 3: NMEA protocol satellite numbering scheme

## 1.5.4 Signal identifiers

A summary of all the signal identification schemes used in the NMEA protocol and the UBX protocol is provided in the following table. (Only a subset of the signals is supported by each product.) In the NMEA protocol, system and signal identifiers are in hexadecimal format. An unknown signal identifier is presented as 0 in the NMEA protocol.

<table><tbody><tr><td colspan="3">UBX Protocol</td><td colspan="2">NMEA Protocol 4.10</td><td colspan="2">NMEA Protocol 4.11</td></tr><tr><td>Signal</td><td>gnssId</td><td>sigId</td><td>System ID</td><td>Signal ID</td><td>System ID</td><td>Signal ID</td></tr><tr><td>GPS L1C/A<sup>2</sup></td><td>0</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td></tr><tr><td>GPS L2 CL</td><td>0</td><td>3</td><td>1</td><td>6</td><td>1</td><td>6</td></tr><tr><td>GPS L2 CM</td><td>0</td><td>4</td><td>1</td><td>5</td><td>1</td><td>5</td></tr><tr><td>GPS L5 I</td><td>0</td><td>6</td><td>1</td><td>7</td><td>1</td><td>7</td></tr><tr><td>GPS L5 Q</td><td>0</td><td>7</td><td>1</td><td>8</td><td>1</td><td>8</td></tr><tr><td>SBAS L1C/A<sup>2</sup></td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td></tr><tr><td>Galileo E1 C<sup>2</sup></td><td>2</td><td>0</td><td>3</td><td>7</td><td>3</td><td>7</td></tr><tr><td>Galileo E1 B<sup>2</sup></td><td>2</td><td>1</td><td>3</td><td>7</td><td>3</td><td>7</td></tr><tr><td>Galileo E5 aI</td><td>2</td><td>3</td><td>3</td><td>1</td><td>3</td><td>1</td></tr><tr><td>Galileo E5 aQ</td><td>2</td><td>4</td><td>3</td><td>1</td><td>3</td><td>1</td></tr><tr><td>Galileo E5 bI</td><td>2</td><td>5</td><td>3</td><td>2</td><td>3</td><td>2</td></tr><tr><td>Galileo E5 bQ</td><td>2</td><td>6</td><td>3</td><td>2</td><td>3</td><td>2</td></tr><tr><td>BeiDou B1I D1<sup>2</sup></td><td>3</td><td>0</td><td>(4)<sup>3</sup></td><td>(1)<sup>4</sup></td><td>4</td><td>1</td></tr><tr><td>BeiDou B1I D2<sup>2</sup></td><td>3</td><td>1</td><td>(4)<sup>3</sup></td><td>(1)<sup>4</sup></td><td>4</td><td>1</td></tr><tr><td>BeiDou B2I D1</td><td>3</td><td>2</td><td>(4)<sup>3</sup></td><td>(3)<sup>4</sup></td><td>4</td><td>B</td></tr><tr><td>BeiDou B2I D2</td><td>3</td><td>3</td><td>(4)<sup>3</sup></td><td>(3)<sup>4</sup></td><td>4</td><td>B</td></tr><tr><td>BeiDou B1 Cp (pilot)</td><td>3</td><td>5</td><td>(4)<sup>3</sup></td><td>N/A</td><td>4</td><td>3</td></tr><tr><td>BeiDou B1 Cd (data)</td><td>3</td><td>6</td><td>(4)<sup>3</sup></td><td>N/A</td><td>4</td><td>3</td></tr><tr><td>BeiDou B2 ap (pilot)</td><td>3</td><td>7</td><td>(4)<sup>3</sup></td><td>N/A</td><td>4</td><td>5</td></tr><tr><td>BeiDou B2 ad (data)</td><td>3</td><td>8</td><td>(4)<sup>3</sup></td><td>N/A</td><td>4</td><td>5</td></tr><tr><td>QZSS L1C/A<sup>2</sup></td><td>5</td><td>0</td><td>(1)<sup>3</sup></td><td>(1)<sup>4</sup></td><td>5</td><td>1</td></tr><tr><td>QZSS L1S</td><td>5</td><td>1</td><td>(1)<sup>3</sup></td><td>(4)<sup>4</sup></td><td>5</td><td>4</td></tr><tr><td>QZSS L2 CM</td><td>5</td><td>4</td><td>(1)<sup>3</sup></td><td>(5)<sup>4</sup></td><td>5</td><td>5</td></tr><tr><td>QZSS L2 CL</td><td>5</td><td>5</td><td>(1)<sup>3</sup></td><td>(6)<sup>4</sup></td><td>5</td><td>6</td></tr><tr><td>QZSS L5 I</td><td>5</td><td>8</td><td>(1)<sup>3</sup></td><td>N/A</td><td>5</td><td>7</td></tr><tr><td>QZSS L5 Q</td><td>5</td><td>9</td><td>(1)<sup>3</sup></td><td>N/A</td><td>5</td><td>8</td></tr></tbody></table>

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">2 UBX messages that do not have an explicit sigId field contain information about the subset of signals marked.</span></small>

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">3 While not defined by NMEA 4.10, in this mode, u-blox receivers use system ID 4 for BeiDou and, if extended satellite numbering is enabled, system ID 1 for QZSS.</span></small>

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">4 BeiDou and QZSS signal ID are not defined in the NMEA protocol version 4.10. Values shown in the table are only valid for u-blox products and, for QZSS signal ID, if extended satellite numbering is enabled.</span></small>

UBX-23002975 - R02 C1-Public

1 General information

Page 15 of 181

<!-- page 16 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tbody><tr><td colspan="3">UBXProtocol</td><td colspan="2">NMEAProtocol4.10</td><td colspan="2">NMEAProtocol4.11</td></tr><tr><td>Signal</td><td>gnssId</td><td>sigId</td><td>SystemID</td><td>Signal ID</td><td>SystemID</td><td>Signal ID</td></tr><tr><td>GLONASSL1OF<sup>2</sup></td><td>6</td><td>0</td><td>2</td><td>1</td><td>2</td><td>1</td></tr><tr><td>GLONASSL2OF</td><td>6</td><td>2</td><td>2</td><td>3</td><td>2</td><td>3</td></tr><tr><td>NavICL5A<sup>2</sup></td><td>7</td><td>0</td><td>N/A</td><td>N/A</td><td>6</td><td>1</td></tr></tbody></table>

Table 4: Signal identifiers

## 1.6 Message types

The following message types are defined:

| Message type | Description |
| --- | --- |
| Input | Messages that are input to the receiver and never output. E.g. UBX-MGA-GPS-EPH. |
| Output | Messages that are output by the receiver in no particular interval and never input. E.g. UBX-ACK-ACK. |
| Input/output | Messages that can be output by or input to the receiver. E.g. UBX-MGA-DBD-DATA0. |
| Periodic | Messages that are output in regular intervals but cannot be polled. E.g. UBX-NAV-EOE. |
| Periodic/polled | Messages that are output in regular intervals and can be polled. E.g. UBX-NAV-PVT. |
| Command | Messages that are a command to the receiver. Similar to type Inputthese are input-only. E.g. UBX-CFG-RST. |
| Get | Output-only configuration or command messages. E.g. UBX-CFG-DAT. |
| Set | Input-only configuration or command messages. E.g. UBX-CFG-VALDEL. |
| Get/set | Input/output configuration or command messages. E.g. UBX-CFG-NAVX5. |
| Polled | Non-periodic messages that can only be polled. E.g. UBX-MON-VER. |
| Poll request | Poll request. E.g. UBX-MGA-DBD-POLL. |

UBX-23002975 - R02 C1-Public

1 General information

Page 16 of 181

<!-- page 17 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## 2 NMEA protocol

The following sections give an overview of the NMEA messages used by u-blox positioning receivers.

By default, the NMEA messages sent by u-blox positioning receivers are based on the NMEA 0183 version 4.11 standard. For further information on the NMEA standard, refer to the NMEA 0183 Standard for Interfacing Marine Electronic Devices, Version 4.11, November 2018, which is available on [http://www.nmea.org/](http://www.nmea.org/).

## 2.1 NMEA frame structure

The following figure shows the structure of a NMEA protocol message (called "sentences" in the standard).

![Image block](doc:a53f2ec/tier:standard/page:17/block:8)

| $ | GP | ZDA | ,141644.00,22,03,2002,00,00 | *67 | \\r\\n |
| --- | --- | --- | --- | --- | --- |

## 2.2 NMEA protocol configuration

The NMEA protocol on u-blox receivers can be configured for customer applications by using the Configuration interface (CFG-NMEA-\* items).

Several NMEA standard versions are supported. Version 4.11 (not in all products), 4.10, 4.00, 2.3, or 2.1 can be configured. See Configuration defaults for the default version. See CFG-NMEA-PROTVER to configure the version. See NMEA multi-GNSS operation and NMEA data fields for details on how this affects the output.

The following filtering flags can be used to configure the output of some NMEA message fields:

| Filter | Configuration Item | Description |
| --- | --- | --- |
| Position filtering | CFG-NMEA-OUT_INVFIX | Enable to permit positions from failed or invalid fixes to be reported (with the "V" status flag to indicate that the data is not valid). |
| Valid position filtering | CFG-NMEA-OUT_MSKFIX | Enable to permit positions from invalid fixes to be reported (with the "V" status flag to indicate that the data is not valid). |
| Time filtering | CFG-NMEA-OUT_INVTIME | Enable to permit the receiver's best knowledge of time to be output, even though it might be wrong. |

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 17 of 181

<!-- page 18 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Filter | Configuration Item | Description |
| --- | --- | --- |
| Date filtering | CFG-NMEA-OUT_INVDATE | Enable to permit the receiver's best knowledge of date to be output, even though it might be wrong. |
| GPS-only filtering | CFG-NMEA-OUT_ONLYGPS | Enable to restrict output to only report GPS satellites. |
| Track filtering | CFG-NMEA-OUT_FROZENCOG | Enable to permit course over ground (COG) to be reported even when it would otherwise be frozen. |

The following filtering flags can be used to configure the output of some NMEA message flags:

| Mode | Configuration Item | Description |
| --- | --- | --- |
| Compatibility mode | CFG-NMEA-COMPAT | Some older NMEA applications expect the NMEA output to be formatted in a specific way, for example, they will only work if the latitude and longitude have exactly four digits behind the decimal point. u-blox receivers offer a compatibility mode to support these legacy applications. |
| Consideration mode | CFG-NMEA-CONSIDER | u-blox receivers use a sophisticated signal quality detection scheme, in order to produce the best possible position output. This algorithm considers all SV measurements, and may eventually decide to only use a subset thereof, if it improves the overall position accuracy. If consideration mode is enabled, all satellites, which were considered for navigation, are communicated as being used for the position determination. If consideration mode is disabled, only those satellites which after the consideration step remained in the position output are marked as being used. |
| Limit length mode | CFG-NMEA-LIMIT82 | Enabling this mode will limit the NMEA sentence length to a maximum of 82 characters. |
| High precision mode | CFG-NMEA-HIGHPREC | Enabling this mode increases precision of the position output. Latitude and longitude then have seven digits after the decimal point, and altitude has three digits after the decimal point. Note: The high precision mode cannot be set in conjunction with either compatibility mode or Limit82 mode. |

The following extended configuration options are available:

| Option | Configuration Item(s) | Description |
| --- | --- | --- |
| GNSS to filter | CFG-NMEA-FILT_GPS etc. | Filters satellites based on the GNSS they belong to. |
| Satellite numbering | CFG-NMEA-SVNUMBERING | This field configures the display of satellites that do not have an NMEA-defined value. Note: this does not apply to satellites with an unknown ID. See also Satellite identifiers. |
| Main Talker ID | CFG-NMEA-MAINTALKERID | By default the main Talker ID (i.e. the Talker ID used for all messages other than GSV) is determined by the GNSS assignment of the receiver's channels (see configuration items CFG-SIGNAL-*). This field enables the main Talker ID to be overridden. See also NMEA Talker ID. |
| GSV Talker ID | CFG-NMEA-GSVTALKERID | By default the Talker ID for GSV messages is GNSS-specific (as defined by NMEA). This field enables the GSV Talker ID to be overridden. |
| BDS Talker ID | CFG-NMEA-BDSTALKERID | By default the Talker ID for BeiDou is "GB". This field enables the BeiDou Talker ID to be overridden. |

## 2.3 NMEA-proprietary messages

The NMEA standard allows for proprietary, manufacturer-specific messages to be added. These shall be marked with a manufacturer mnemonic. The mnemonic assigned to u-blox is UBX and is used for all non-standard messages. These proprietary NMEA messages therefore have the address field set to PUBX. The first data field in a PUBX message identifies the message number with two digits.

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 18 of 181

<!-- page 19 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## 2.4 NMEA multi-GNSS operation

Many applications that process NMEA messages assume that only a single GNSS is active. However, when multiple GNSS are configured, the NMEA specification requires the output to change in the following ways:

**Main Talker ID** The main NMEA Talker ID is "GN" (e.g. instead of "GP" for a GPS-only receiver).

**GSV Talker and Signal IDs** The GSV message reports the signal strength of the visible satellites. In multi-GNSS operation, other messages use the main Talker ID "GN" but the Talker ID in the GSV message is specific to the GNSS it is reporting information for.

The GSV messages are grouped by the Talker and Signal IDs. Separate sets of GSV messages are sent for each GNSS and signal. The Signal ID of a satellite may be unknown. Such satellites are presented in their own set with Signal ID 0. Grouping the GSV messages by the Signal ID is supported in firmware versions 27.12 and later.

**Multiple GSA and GRS messages** Multiple GSA and GRS messages are output for each fix, one for each GNSS. This may confuse applications that assume they are output only once per position fix (as is the case for a single GNSS receiver).

**GGA Talker IDs** The NMEA specification indicates that the GGA message is GPS-specific. However, u-blox receivers support the output of a GGA message for each of the Talker IDs.

**BeiDou and Galileo** Only NMEA version 4.10 and later have support for these systems.

**QZSS** Only NMEA version 4.11 and later have support for this system.

**Extended satellite numbering** In order to support some GNSS (e.g. BeiDou, Galileo, QZSS) that are not supported by some or all NMEA protocol versions, an "extended" SV numbering scheme can be enabled. This uses the NMEA-defined numbers where possible, but adds other number ranges to support other GNSS. Note however that these non-standard extensions require 3-digit numbers, which may not be supported by some NMEA parsing software. For example, QZSS satellites use numbers in the range 193 to 202. See NMEA protocol configuration and Satellite identifiers.

## 2.5 NMEA data fields

Various data fields in NMEA messages depend on NMEA protocol configuration or require a definition for their interpretation.

## 2.5.1 NMEA Talker ID

One of the ways the NMEA standard differs depending on the GNSS is by using a two-letter message identifier, the "Talker ID". The specific Talker ID used by a u-blox receiver will depend on the product and its configuration. The table below shows the Talker ID that will be used for various GNSS configurations by default.

| GNSS | Talker ID | Comments |
| --- | --- | --- |
| GPS, SBAS | GP | NMEA 2.3+ |
| GLONASS | GL | NMEA 2.3+ |
| Galileo | GA | NMEA 4.10+ |
| BeiDou | GB | NMEA 4.10+ (official NMEA only since 4.11) |
| NavIC | GI | NMEA 4.11+ |
| QZSS | GQ | NMEA 4.11+ (GP for NMEA 2.3 - 4.10) |

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 19 of 181

<!-- page 20 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| GNSS | Talker ID | Comments |
| --- | --- | --- |
| Any combination of GNSS | GN |  |

## 2.5.2 NMEA extra fields

The following extra fields are available in NMEA 4.10 and later.

| Message | Extra fields |
| --- | --- |
| NMEA-Standard-GBS | systemId and signalId |
| NMEA-Standard-GNS | navStatus |
| NMEA-Standard-GRS | systemId and signalId |
| NMEA-Standard-GSA | systemId |
| NMEA-Standard-GSV | signalId |
| NMEA-Standard-RMC | navStatus |

## 2.5.3 NMEA latitude and longitude format

According to the NMEA standard, latitude and longitude are output in the format degrees, minutes and (decimal) fractions of minutes. To convert to degrees and fractions of degrees, or degrees, minutes, seconds and fractions of seconds, the minutes and fractional minutes parts need to be converted. For example:

<table><tr><td>Format</td><td>Latitude</td><td>Longitude</td></tr><tr><td>Receiver output</td><td colspan="2">$GNRMC, 014230.00, A, 4722.80340, N, 00831.68218, E, 0.000,, 120477,,, A, V*14</td></tr><tr><td>(d)ddmm.mmmmm</td><td>4722.80340 North</td><td>00831.68218 East</td></tr><tr><td>Degrees and minutes</td><td>47 degrees, 22.80340 minutes</td><td>8 degrees, 31.68218 minutes</td></tr><tr><td>Degrees</td><td>47.38005667 degrees</td><td>8.52803633 degrees</td></tr><tr><td>Degrees, minutes and seconds</td><td>47 degrees, 22 minutes, 48.2040 seconds</td><td>8 degrees, 31 minutes, 40.9308 seconds</td></tr></table>

## 2.5.4 NMEA GNSS, satellite, and signal numbering

See GNSS, satellite, and signal identifiers for details on how GNSS, satellites and signals are numbered in the NMEA protocol.

NMEA defines satellite numbering systems for some, but not all GNSS. The exact behavior depends on the configured NMEA protocol version and ("extended" or "strict") mode. See NMEA protocol configuration for details.

## 2.5.5 NMEA position fix flags

This section shows how u-blox positioning receivers implement the NMEA protocol and the conditions determining how flags are set.

The following flags are used in NMEA 4.10 and later.

| NMEA Message | GLL, RMC | GGA | GLL, VTG | RMC, GNS |
| --- | --- | --- | --- | --- |
| Field | $status^5$ | $quality^6$ | $posMode^7$ | $posMode^7$ |
| No position fix (at power-up, after losing satellite lock) | V | 0 | N | N |

5 Possible status values: V = data invalid, A = data valid

6 Possible values for quality: 0 = No fix, 1 = autonomous GNSS fix, 2 = differential GNSS fix, 4 = RTK fixed, 5 = RTK float, 6 = estimated/dead reckoning fix

7 Possible values for posMode: N = No fix, E = estimated/dead reckoning fix, A = autonomous GNSS fix, D = differential GNSS fix, F = RTK float, R = RTK fixed. In NMEA GNS, u-blox uses a non-standard implementation where same single status is reported for all enabled and not filtered out constellations.

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 20 of 181

<!-- page 21 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| NMEA Message | GLL, RMC | GGA | GLL, VTG | RMC, GNS |
| --- | --- | --- | --- | --- |
| Field | status<sup>5</sup> | quality<sup>6</sup> | posMode<sup>7</sup> | posMode<sup>7</sup> |
| GNSS fix, but user limits exceeded | V | 0 | N | N |
| Dead reckoning fix, but user limits exceeded | V | 6 | E | E |
| Dead reckoning fix | A | 6 | E | E |
| RTK float | A | 5 | D | F |
| RTK fixed | A | 4 | D | R |
| 2D GNSS fix | A | 1 / 2 | A / D | A / D |
| 3D GNSS fix | A | 1 / 2 | A / D | A / D |
| Combined GNSS/dead reckoning fix | A | 1 / 2 | A / D | A / D |

In high precision GNSS (HPG) products it is recommended to select NMEA version 4.10 or above. Earlier versions do not support the float RTK (F) and real time kinematic (R) mode indicator flags in all messages.

The following flags are used in NMEA 2.3 - 4.0.

| NMEA Message | GLL, RMC | GGA | GSA | GLL, VTG, RMC, GNS |
| --- | --- | --- | --- | --- |
| Field | status<sup>8</sup> | quality<sup>9</sup> | navMode<sup>10</sup> | posMode<sup>11</sup> |
| No position fix (at power-up, after losing satellite lock) | V | 0 | 1 | N |
| GNSS fix, but user limits exceeded | V | 0 | 1 | N |
| Dead reckoning fix, but user limits exceeded | V | 6 | 2 | E |
| Dead reckoning fix | A | 6 | 2 | E |
| 2D GNSS fix | A | 1 / 2 | 2 | A / D |
| 3D GNSS fix | A | 1 / 2 | 3 | A / D |
| Combined GNSS/dead reckoning fix | A | 1 / 2 | 3 | A / D |

• The posMode field is not output for GLL, RMC and VTG messages (each message has one field less).

• The GGA quality field is set to 1 (instead of 6) for both types of dead reckoning fix.

## 2.5.6 NMEA output of invalid or unknown data

By default the receiver will not output invalid data. In such cases, it will output empty fields. See NMEA protocol configuration for options to adjust this behavior.

A valid position fix is reported as follows:

\$GPGLL,4717.11634,N,00833.91297,E,124923.00,A,A\*6E

An invalid position fix (but valid time) is reported as follows:

\$GPGLL,,,,,124924.00,V,N\*42

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">8 Possible values for status: V = data invalid, A = data valid</span></small>

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">9 Possible values for quality: 0 = no fix, 1 = autonomous GNSS fix, 2 = differential GNSS fix, 4 = RTK fixed, 5 = RTK float, 6 = estimated/dead reckoning fix</span></small>

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">10 Possible values for navMode: 1 = No fix, 2 = 2D fix, 3 = 3D fix</span></small>

<small><span class="docvortex-page-footnote" data-block-type="page_footnote" style="color:#6b7280">11 Possible values for posMode: N = No fix, E = estimated/dead reckoning fix, A = autonomous GNSS fix, D = differential GNSS fix. In NMEA GNS, u-blox uses a non-standard implementation where same single status is reported for all enabled and not filtered out constellations.</span></small>

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 21 of 181

<!-- page 22 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

If the time is unknown (e.g. during a cold start):

\$GPGLL,,,,,,V,N\*64

![Image block](doc:a53f2ec/tier:standard/page:22/block:5)

Unlike the NMEA standard behavior to invalid data, dead reckoning products always report a position. It is marked as invalid (V) when the user limits are exceeded or valid (A) if the user limits are met.

2.6 NMEA messages overview

<table><tr><td>Message</td><td>Class/ID</td><td>Description (Type)</td></tr><tr><td colspan="3">NMEA-Standard - Standard NMEA messages</td></tr><tr><td>NMEA-Standard-DTM</td><td>0xf0 0x0a</td><td>• Datum reference (Output)</td></tr><tr><td>NMEA-Standard-GAQ</td><td>0xf0 0x45</td><td>• Poll a standard message (Talker ID GA) (Poll request)</td></tr><tr><td>NMEA-Standard-GBQ</td><td>0xf0 0x44</td><td>• Poll a standard message (Talker ID GB) (Poll request)</td></tr><tr><td>NMEA-Standard-GBS</td><td>0xf0 0x09</td><td>• GNSS satellite fault detection (Output)</td></tr><tr><td>NMEA-Standard-GGA</td><td>0xf0 0x00</td><td>• Global positioning system fix data (Output)</td></tr><tr><td>NMEA-Standard-GLL</td><td>0xf0 0x01</td><td>• Latitude and longitude, with time of position fix and status (Output)</td></tr><tr><td>NMEA-Standard-GLQ</td><td>0xf0 0x43</td><td>• Poll a standard message (Talker ID GL) (Poll request)</td></tr><tr><td>NMEA-Standard-GNQ</td><td>0xf0 0x42</td><td>• Poll a standard message (Talker ID GN) (Poll request)</td></tr><tr><td>NMEA-Standard-GNS</td><td>0xf0 0x0d</td><td>• GNSS fix data (Output)</td></tr><tr><td>NMEA-Standard-GPQ</td><td>0xf0 0x40</td><td>• Poll a standard message (Talker ID GP) (Poll request)</td></tr><tr><td>NMEA-Standard-GQQ</td><td>0xf0 0x47</td><td>• Poll a standard message (Talker ID GQ) (Poll request)</td></tr><tr><td>NMEA-Standard-GRS</td><td>0xf0 0x06</td><td>• GNSS range residuals (Output)</td></tr><tr><td>NMEA-Standard-GSA</td><td>0xf0 0x02</td><td>• GNSS DOP and active satellites (Output)</td></tr><tr><td>NMEA-Standard-GST</td><td>0xf0 0x07</td><td>• GNSS pseudorange error statistics (Output)</td></tr><tr><td>NMEA-Standard-GSV</td><td>0xf0 0x03</td><td>• GNSS satellites in view (Output)</td></tr><tr><td>NMEA-Standard-RLM</td><td>0xf0 0x0b</td><td>• Return link message (RLM) (Output)</td></tr><tr><td>NMEA-Standard-RMC</td><td>0xf0 0x04</td><td>• Recommended minimum data (Output)</td></tr><tr><td>NMEA-Standard-TXT</td><td>0xf0 0x41</td><td>• Text transmission (Output)</td></tr><tr><td>NMEA-Standard-VLW</td><td>0xf0 0x0f</td><td>• Dual ground/water distance (Output)</td></tr><tr><td>NMEA-Standard-VTG</td><td>0xf0 0x05</td><td>• Course over ground and ground speed (Output)</td></tr><tr><td>NMEA-Standard-ZDA</td><td>0xf0 0x08</td><td>• Time and date (Output)</td></tr><tr><td colspan="3">NMEA-PUBX - u-blox proprietary NMEA messages</td></tr><tr><td>NMEA-PUBX-CONFIG</td><td>0xf1 0x41</td><td>• Set protocols and baud rate (Set)</td></tr><tr><td>NMEA-PUBX-POSITION</td><td>0xf1 0x00</td><td>• Poll a PUBX,00 message (Poll request)• Lat/Long position data (Output)</td></tr><tr><td>NMEA-PUBX-RATE</td><td>0xf1 0x40</td><td>• Set NMEA message output rate (Set)</td></tr><tr><td>NMEA-PUBX-SVSTATUS</td><td>0xf1 0x03</td><td>• Poll a PUBX,03 message (Poll request)• Satellite status (Output)</td></tr><tr><td>NMEA-PUBX-TIME</td><td>0xf1 0x04</td><td>• Poll a PUBX,04 message (Poll request)• Time of day and clock information (Output)</td></tr></table>

## 2.7 Standard messages

Standard NMEA messages as defined by the NMEA 0183 standard. See NMEA protocol for details.

## 2.7.1 DTM

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 22 of 181

<!-- page 23 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

2.7.1.1 Datum reference

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-DTM Datum reference</td></tr><tr><td colspan="2">Type</td><td colspan="4">Output</td></tr><tr><td colspan="2">Comment</td><td colspan="4">This message gives the difference between the current datum and the reference datum.The current datum is set to WGS84 by default.The reference datum cannot be changed and is always set to WGS84.</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x0a</td><td colspan="3">Number of fields: 11</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxDTM, datum, subDatum, lat, NS, lon, EW, alt, refDatum*cs\r\n</td></tr><tr><td colspan="2">Examples</td><td colspan="4">$GPDTM, W84,, 0.0,N, 0.0,E, 0.0,W84*6F\r\n$GPDTM, 999,, 0.08,N, 0.07,E,-47.7,W84*1C\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxDTM</td><td>string</td><td>-</td><td>$GPDTM</td><td>DTM Message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>datum</td><td>string</td><td>-</td><td>W84</td><td>Local datum code: W84 = WGS84, P90 = PZ90, 999 = user-defined</td></tr><tr><td>2</td><td>subDatum</td><td>string</td><td>-</td><td>-</td><td>A null field (or a string describing the currently selected datum for protocol versions less than 14.00)</td></tr><tr><td>3</td><td>lat</td><td>numeric</td><td>min</td><td>0.08</td><td>Offset in Latitude</td></tr><tr><td>4</td><td>NS</td><td>character</td><td>-</td><td>S</td><td>North/South indicator</td></tr><tr><td>5</td><td>lon</td><td>numeric</td><td>min</td><td>0.07</td><td>Offset in Longitude</td></tr><tr><td>6</td><td>EW</td><td>character</td><td>-</td><td>E</td><td>East/West indicator</td></tr><tr><td>7</td><td>alt</td><td>numeric</td><td>m</td><td>-2.8</td><td>Offset in altitude</td></tr><tr><td>8</td><td>refDatum</td><td>string</td><td>-</td><td>W84</td><td>Reference datum code: W84 (WGS 84, fixed field)</td></tr><tr><td>9</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*67</td><td>Checksum</td></tr><tr><td>10</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.2 GAQ

2.7.2.1 Poll a standard message (Talker ID GA)

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-GAQ Poll a standard message (Talker ID GA)</td></tr><tr><td colspan="2">Type</td><td colspan="4">Poll request</td></tr><tr><td colspan="2">Comment</td><td colspan="4">Polls a standard NMEA message if the current Talker ID is GA.</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x45</td><td colspan="3">Number of fields: 4</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxGAQ, msgId*cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$EIGAQ, RMC*2B\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGAQ</td><td>string</td><td>-</td><td>$EIGAQ</td><td>GAQ Message ID (xx = Talker ID of the device requesting the poll)</td></tr><tr><td>1</td><td>msgId</td><td>string</td><td>-</td><td>RMC</td><td>Message ID of the message to be polled</td></tr><tr><td>2</td><td>cs</td><td colspan="2">hexadecimal -</td><td>*2B</td><td>Checksum</td></tr></table>

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 23 of 181

<!-- page 24 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 3 | CRLF | character | - | - | Carriage return and line feed |
| --- | --- | --- | --- | --- | --- |

## 2.7.3 GBQ

2.7.3.1 Poll a standard message (Talker ID GB)

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-GBQ Poll a standard message (Talker ID GB)</td></tr><tr><td colspan="2">Type</td><td colspan="4">Poll request</td></tr><tr><td colspan="2">Comment</td><td colspan="4">Polls a standard NMEA message if the current Talker ID is GB</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x44</td><td colspan="3">Number of fields: 4</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxGBQ, msgId*cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$EIGBQ, RMC*28\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGBQ</td><td>string</td><td>-</td><td>$EIGBQ</td><td>GBQ Message ID (xx = Talker ID of the device requesting the poll)</td></tr><tr><td>1</td><td>msgId</td><td>string</td><td>-</td><td>RMC</td><td>Message ID of the message to be polled</td></tr><tr><td>2</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*28</td><td>Checksum</td></tr><tr><td>3</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.4 GBS

2.7.4.1 GNSS satellite fault detection

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-GBSGNSS satellite fault detection</td></tr><tr><td colspan="2">Type</td><td colspan="4">Output</td></tr><tr><td colspan="2">Comment</td><td colspan="4">This message outputs the results of the Receiver Autonomous Integrity Monitoring Algorithm (RAIM).The fields errLat, errLon and errAlt output the standard deviation of the position calculation, using all satellites that pass the RAIM test successfully.The fields errLat, errLon and errAlt are only output if the RAIM process passed successfully (i.e. no or successful edits happened). These fields are never output if 4 or fewer satellites are used for the navigation calculation (because, in such cases, integrity cannot be determined by the receiver autonomously).The fields prob, bias and stdev are only output if at least one satellite failed in the RAIM test.If more than one satellites fail the RAIM test, only the information for the worst satellite is output in this message.</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x09</td><td colspan="3">Number of fields: 13</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxGBS,time,errLat,errLon,errAlt,svid,prob,bias,stddev,systemId,signalId*cs\r\n</td></tr><tr><td colspan="2">Examples</td><td colspan="4">$GPGBS,235503.00,1.6,1.4,3.2,,,,,*40\r\n$GPGBS,235458.00,1.4,1.3,3.1,03,-21.4,3.8,1,0*5B\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGBS</td><td>string</td><td>-</td><td>$GPGBS</td><td>GBS Message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>time</td><td>hhmmss.ss</td><td>-</td><td>235503.00</td><td>UTC time to which this RAIM sentence belongs. See section UTC representation in the integration manual for details.</td></tr><tr><td>2</td><td>errLat</td><td>numeric</td><td>m</td><td>1.6</td><td>Expected error in latitude</td></tr><tr><td>3</td><td>errLon</td><td>numeric</td><td>m</td><td>1.4</td><td>Expected error in longitude</td></tr></table>

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 24 of 181

<!-- page 25 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 4 | errAlt | numeric | m | 3.2 | Expected error in altitude |
| --- | --- | --- | --- | --- | --- |
| 5 | svid | numeric | - | 03 | Satellite ID of most likely failed satellite |
| 6 | prob | numeric | - | - | Probability of missed detection: null (not supported, fixed field) |
| 7 | bias | numeric | m | -21.4 | Estimated bias of most likely failed satellite (a priori residual) |
| 8 | stddev | numeric | m | 3.8 | Standard deviation of estimated bias |
| 9 | systemId | hexadecimal | - | 1 | NMEA-defined GNSS system ID, see Signal Identifiers table (only available in NMEA 4.10 and later) |
| 10 | signalId | hexadecimal | - | - | NMEA-defined GNSS signal ID, see Signal Identifiers table (only available in NMEA 4.10 and later) |
| 11 | cs | hexadecimal | - | *5B | Checksum |
| 12 | CRLF | character | - | - | Carriage return and line feed |

## 2.7.5 GGA

2.7.5.1 Global positioning system fix data

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-GGAGlobal positioning system fix data</td></tr><tr><td colspan="2">Type</td><td colspan="4">Output</td></tr><tr><td colspan="2">Comment</td><td colspan="4">Time and position, together with GPS fixing-related data (number of satellites in use, and the resulting HDOP, age of differential data if in use, etc.).The output of this message is dependent on the currently selected datum (default: WGS84). The NMEA specification indicates that the GGA message is GPS-specific. However, when the receiver is configured for multi-GNSS, the GGA message contents will be generated from the multi-GNSS solution. For multi-GNSS use, it is recommended that the NMEA-GNS message is used instead.</td></tr><tr><td colspan="2">Information</td><td colspan="4">Class/ID: 0xf0 0x00 Number of fields: 17</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxGGA, time, lat, NS, lon, EW, quality, numSV, HDOP, alt, altUnit, sep, sepUnit, diffAge, diffSta tion*cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$GPGGA, 092725.00, 4717.11399, N, 00833.91590, E, 1, 08, 1.01, 499.6, M, 48.0, M, , *5B\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGGA</td><td>string</td><td>-</td><td>$GPGGA</td><td>GGA Message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>time</td><td>hhmmss.ss</td><td>-</td><td>092725.00</td><td>UTC time. See section UTC representation in the integration manual for details.</td></tr><tr><td>2</td><td>lat</td><td>ddmm.mmmm</td><td>-</td><td>4717.11399</td><td>Latitude (degrees and minutes), see format description</td></tr><tr><td>3</td><td>NS</td><td>character</td><td>-</td><td>N</td><td>North/South indicator</td></tr><tr><td>4</td><td>lon</td><td>dddmm.mmmm</td><td>-</td><td>00833.91590</td><td>Longitude (degrees and minutes), see format description</td></tr><tr><td>5</td><td>EW</td><td>character</td><td>-</td><td>E</td><td>East/West indicator</td></tr><tr><td>6</td><td>quality</td><td>digit</td><td>-</td><td>1</td><td>Quality indicator for position fix, see position fix flags description</td></tr><tr><td>7</td><td>numSV</td><td>numeric</td><td>-</td><td>08</td><td>Number of satellites used (range: 0-12)</td></tr><tr><td>8</td><td>HDOP</td><td>numeric</td><td>-</td><td>1.01</td><td>Horizontal Dilution of Precision</td></tr><tr><td>9</td><td>alt</td><td>numeric</td><td>m</td><td>499.6</td><td>Altitude above mean sea level</td></tr><tr><td>10</td><td>altUnit</td><td>character</td><td>-</td><td>M</td><td>Altitude units: M (meters, fixed field)</td></tr></table>

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 25 of 181

<!-- page 26 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 11 | sep | numeric | m | 48.0 | Geoid separation: difference between ellipsoid and mean sea level |
| --- | --- | --- | --- | --- | --- |
| 12 | sepUnit | character | - | M | Geoid separation units: M (meters, fixed field) |
| 13 | diffAge | numeric | s | - | Age of differential corrections (null when DGPS is not used) |
| 14 | diffStation | numeric | - | - | ID of station providing differential corrections (null when DGPS is not used) |
| 15 | cs | hexadecimal | - | *5B | Checksum |
| 16 | CRLF | character | - | - | Carriage return and line feed |

## 2.7.6 GLL

2.7.6.1 Latitude and longitude, with time of position fix and status

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-GLLLatitude and longitude, with time of position fix and status</td></tr><tr><td colspan="2">Type</td><td colspan="4">Output</td></tr><tr><td colspan="2">Comment</td><td colspan="4">The output of this message is dependent on the currently selected datum (default: WGS84)</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x01</td><td colspan="3">Number of fields: 10</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxGLL, lat, NS, lon, EW, time, status, posMode*cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$GPGLL, 4717.11364, N, 00833.91565, E, 092321.00, A, A*60\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGLL</td><td>string</td><td>-</td><td>$GPGLL</td><td>GLL Message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>lat</td><td>ddmm.mmmm</td><td>-</td><td>4717.11364</td><td>Latitude (degrees and minutes), see format description</td></tr><tr><td>2</td><td>NS</td><td>character</td><td>-</td><td>N</td><td>North/South indicator</td></tr><tr><td>3</td><td>lon</td><td>dddmm.mmmm</td><td>-</td><td>00833.91565</td><td>Longitude (degrees and minutes), see format description</td></tr><tr><td>4</td><td>EW</td><td>character</td><td>-</td><td>E</td><td>East/West indicator</td></tr><tr><td>5</td><td>time</td><td>hhmmss.ss</td><td>-</td><td>092321.00</td><td>UTC time. See section UTC representation in the integration manual for details.</td></tr><tr><td>6</td><td>status</td><td>character</td><td>-</td><td>A</td><td>Data validity status, see position fix flags description</td></tr><tr><td>7</td><td>posMode</td><td>character</td><td>-</td><td>A</td><td>Positioning mode, see position fix flags description (only available in NMEA 2.3 and later)</td></tr><tr><td>8</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*60</td><td>Checksum</td></tr><tr><td>9</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.7 GLQ

2.7.7.1 Poll a standard message (Talker ID GL)

| Message | NMEA-Standard-GLQ Pollastandardmessage(TalkerIDGL) |
| --- | --- |
| Type | Pollrequest |
| Comment | PollsastandardNMEAmessageifthecurrentTalkerIDisGL |
| Information | Class/ID: 0xf0 0x43 Number of fields: 4 |
| Structure | $xxGLQ,msgId*cs\\r\\n |

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 26 of 181

<!-- page 27 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td colspan="6">Example $EIGLQ, RMC*3A\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGLQ</td><td>string</td><td>-</td><td>$EIGLQ</td><td>GLQ Message ID (xx = Talker ID of the device requesting the poll)</td></tr><tr><td>1</td><td>msgId</td><td>string</td><td>-</td><td>RMC</td><td>Message ID of the message to be polled</td></tr><tr><td>2</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*3A</td><td>Checksum</td></tr><tr><td>3</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.8 GNQ

2.7.8.1 Poll a standard message (Talker ID GN)

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-GNQ Poll a standard message (Talker ID GN)</td></tr><tr><td colspan="2">Type</td><td colspan="4">Poll request</td></tr><tr><td colspan="2">Comment</td><td colspan="4">Polls a standard NMEA message if the current Talker ID is GN</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x42</td><td colspan="3">Number of fields: 4</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxGNQ, msgId*cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$EIGNQ, RMC*3A\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGNQ</td><td>string</td><td>-</td><td>$EIGNQ</td><td>GNQ Message ID (xx = Talker ID of the device requesting the poll)</td></tr><tr><td>1</td><td>msgId</td><td>string</td><td>-</td><td>RMC</td><td>Message ID of the message to be polled</td></tr><tr><td>2</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*3A</td><td>Checksum</td></tr><tr><td>3</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.9 GNS

## 2.7.9.1 GNSS fix data

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-GNSGNSS fix data</td></tr><tr><td colspan="2">Type</td><td colspan="4">Output</td></tr><tr><td colspan="2">Comment</td><td colspan="4">Time and position, together with GNSS fixing-related data (number of satellites in use, and the resulting HDOP, age of differential data if in use, etc.).The output of this message is dependent on the currently selected datum (default: WGS84)</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x0d</td><td colspan="3">Number of fields: 16</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxGNS, time, lat, NS, lon, EW, posMode, numSV, HDOP, alt, sep, diffAge, diffStation, navStatus*c s\r\n</td></tr><tr><td colspan="2">Examples</td><td colspan="4">$GNGNS, 103600.01, 5114.51176, N, 00012.29380, W, ANNN, 07, 1.18, 111.5, 45.6,,, V*00\r\n$GNGNS, 122310.2, 3722.425671, N, 12258.856215, W, DAAA, 14, 0.9, 1005.543, 6.5,,, V*0E\r\n$GPGNS, 122310.2,,,,, 07,,,,, 5.2, 23, V*02\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGNS</td><td>string</td><td>-</td><td>$GPGNS</td><td>GNS Message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>time</td><td>hhmmss.ss</td><td>-</td><td>091547.00</td><td>UTC time. See section UTC representation in the integration manual for details.</td></tr></table>

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 27 of 181

<!-- page 28 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 2 | lat | ddmm.mmmm | - | 5114.50897 | Latitude (degrees and minutes), see format description |
| --- | --- | --- | --- | --- | --- |
| 3 | NS | character | - | N | North/South indicator |
| 4 | lon | dddmm.mmmm | - | 00012.28663 | Longitude (degrees and minutes), see format description |
| 5 | EW | character | - | E | East/West indicator |
| 6 | posMode | character | - | AAAA | Positioning mode, see position fix flags description. The first four characters indicate the status for GPS, GLONASS, Galileo and BeiDou. Note that the NMEA GNS message only reports a single status. It indicates the status for all enabled constellations that have not been filtered out. To obtain a more detailed status report, refer to the status provided in the UBX messages. |
| 7 | numSV | numeric | - | 10 | Number of satellites used (range: 0-99) |
| 8 | HDOP | numeric | - | 0.83 | Horizontal Dilution of Precision |
| 9 | alt | numeric | m | 111.1 | Altitude above mean sea level |
| 10 | sep | numeric | m | 45.6 | Geoid separation: difference between ellipsoid and mean sea level |
| 11 | diffAge | numeric | s | - | Age of differential corrections (null when DGPS is not used) |
| 12 | diffStation | numeric | - | - | ID of station providing differential corrections (null when DGPS is not used) |
| 13 | navStatus | character | - | V | Navigational status indicator: V (Equipment is not providing navigational status information, fixed field, only available in NMEA 4.10 and later) |
| 14 | cs | hexadecimal | - | *71 | Checksum |
| 15 | CRLF | character | - | - | Carriage return and line feed |

## 2.7.10 GPQ

2.7.10.1 Poll a standard message (Talker ID GP)

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-GPQ Poll a standard message (Talker ID GP)</td></tr><tr><td colspan="2">Type</td><td colspan="4">Poll request</td></tr><tr><td colspan="2">Comment</td><td colspan="4">Polls a standard NMEA message if the current Talker ID is GP</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x40</td><td colspan="3">Number of fields: 4</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxGPQ, msgId*cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$EIGPQ, RMC*3A\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGPQ</td><td>string</td><td>-</td><td>$EIGPQ</td><td>GPQ Message ID (xx = Talker ID of the device requesting the poll)</td></tr><tr><td>1</td><td>msgId</td><td>string</td><td>-</td><td>RMC</td><td>Message ID of the message to be polled</td></tr><tr><td>2</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*3A</td><td>Checksum</td></tr><tr><td>3</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.11 GQQ

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 28 of 181

<!-- page 29 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

2.7.11.1 Poll a standard message (Talker ID GQ)

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-GQQ Poll a standard message (Talker ID GQ)</td></tr><tr><td colspan="2">Type</td><td colspan="4">Poll request</td></tr><tr><td colspan="2">Comment</td><td colspan="4">Polls a standard NMEA message if the current Talker ID is GQ</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x47</td><td colspan="3">Number of fields: 4</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxGQQ, msgId*cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$EIGQQ, RMC*3A\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGQQ</td><td>string</td><td>-</td><td>$EIGQQ</td><td>GQQ Message ID (xx = Talker ID of the device requesting the poll)</td></tr><tr><td>1</td><td>msgId</td><td>string</td><td>-</td><td>RMC</td><td>Message ID of the message to be polled</td></tr><tr><td>2</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*3A</td><td>Checksum</td></tr><tr><td>3</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.12 GRS

## 2.7.12.1 GNSS range residuals

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-GRSGNSS range residuals</td></tr><tr><td colspan="2">Type</td><td colspan="4">Output</td></tr><tr><td colspan="2">Comment</td><td colspan="4">If less than 12 SVs are available, the remaining fields are output empty. If more than 12 SVs are used, only the residuals of the first 12 SVs are output, in order to remain consistent with the NMEA standard.In a multi-GNSS system this message will be output multiple times, once for each GNSS.This message relates to associated GGA and GSA messages.</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x06</td><td colspan="3">Number of fields: 19</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxGRS,time,mode{,residual},systemId,signalId*cs\r\n</td></tr><tr><td colspan="2">Examples</td><td colspan="4">$GNGRS,104148.00,1,2.6,2.2,-1.6,-1.1,-1.7,-1.5,5.8,1.7,,,,,1,1*52\r\n$GNGRS,104148.00,1,,0.0,2.5,0.0,,2.8,,,,,,1,5*52\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGRS</td><td>string</td><td>-</td><td>$GPGRS</td><td>GRS Message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>time</td><td>hhmmss.ss</td><td>-</td><td>082632.00</td><td>UTC time of associated position fix. See section UTC representation in the integration manual for details.</td></tr><tr><td>2</td><td>mode</td><td>digit</td><td>-</td><td>1</td><td>Computation method used:• 1 = Residuals were recomputed after the GGA position was computed (fixed)</td></tr><tr><td colspan="6">Start of repeated group (12 times)</td></tr><tr><td>3 + n</td><td>residual</td><td>numeric</td><td>m</td><td>0.54</td><td>Range residuals for SVs used in navigation. The SV order matches the order from the GSA sentence</td></tr><tr><td colspan="6">End of repeated group (12 times)</td></tr><tr><td>15</td><td>systemId</td><td colspan="2">hexadecimal -</td><td>1</td><td>NMEA-defined GNSS system ID, see Signal Identifiers table (only available in NMEA 4.10 and later)</td></tr><tr><td>16</td><td>signalId</td><td colspan="2">hexadecimal -</td><td>-</td><td>NMEA-defined GNSS signal ID, see Signal Identifiers table (only available in NMEA 4.10 and later)</td></tr><tr><td>17</td><td>cs</td><td colspan="2">hexadecimal -</td><td>*70</td><td>Checksum</td></tr></table>

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 29 of 181

<!-- page 30 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 18 | CRLF | character | - | - | Carriage return and line feed |
| --- | --- | --- | --- | --- | --- |

## 2.7.13 GSA

2.7.13.1 GNSS DOP and active satellites

| Message | NMEA-Standard-GSA GNSS DOP and active satellites |
| --- | --- |
| Type Comment | Output The GNSS receiver operating mode, satellites used for navigation, and DOP values. |
|  | • If less than 12 SVs are used for navigation, the remaining fields are left empty. If more than 12 SVs are used for navigation, only the IDs of the first 12 are output. • The SV numbers (fields 'svid') are in the range of 1 to 32 for GPS satellites, and 33 to 64 for SBAS satellites (33 = SBAS PRN 120, 34 = SBAS PRN 121, and so on) |

In a multi-GNSS system this message will be output multiple times, once for each GNSS.

<table><tr><td>Information</td><td colspan="2">Class/ID: 0xf0 0x02</td><td colspan="3">Number of fields: 21</td></tr><tr><td>Structure</td><td colspan="5">$xxGSA, opMode, navMode{, svid}, PDOP, HDOP, VDOP, systemId*cs\r\n</td></tr><tr><td>Example</td><td colspan="5">$GPGSA, A, 3, 23, 29, 07, 08, 09, 18, 26, 28, , , , 1.94, 1.18, 1.54, 1*0D\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGSA</td><td>string</td><td>-</td><td>$GPGSA</td><td>GSA Message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>opMode</td><td>character</td><td>-</td><td>A</td><td>Operation mode:M = Manually set to operate in 2D or 3D modeA = Automatically switching between 2D or 3D mode</td></tr><tr><td>2</td><td>navMode</td><td>digit</td><td>-</td><td>3</td><td>Navigation mode, see position fix flags description</td></tr><tr><td colspan="6">Start of repeated group (12 times)</td></tr><tr><td>3 + n</td><td>svid</td><td>numeric</td><td>-</td><td>29</td><td>Satellite number</td></tr><tr><td colspan="6">End of repeated group (12 times)</td></tr><tr><td>15</td><td>PDOP</td><td>numeric</td><td>-</td><td>1.94</td><td>Position dilution of precision</td></tr><tr><td>16</td><td>HDOP</td><td>numeric</td><td>-</td><td>1.18</td><td>Horizontal dilution of precision</td></tr><tr><td>17</td><td>VDOP</td><td>numeric</td><td>-</td><td>1.54</td><td>Vertical dilution of precision</td></tr><tr><td>18</td><td>systemId</td><td>hexadecimal</td><td>-</td><td>1</td><td>NMEA-defined GNSS system ID, see Signal Identifiers table (only available in NMEA 4.10 and later)</td></tr><tr><td>19</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*0D</td><td>Checksum</td></tr><tr><td>20</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.14 GST

2.7.14.1 GNSS pseudorange error statistics

| Message | NMEA-Standard-GST GNSS pseudorange error statistics |
| --- | --- |
| Type | Output |
| Comment | This message reports statistical information on the quality of the position solution. |
| Information | Class/ID: 0xf0 0x07 Number of fields: 11 |
| Structure | $xxGST,time,rangeRms,stdMajor,stdMinor,orient,stdLat,stdLong,stdAlt*cs\\r\\n |
| Example | $GPGST,082356.00,1.8,,,,1.7,1.3,2.2*7E\\r\\n |

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 30 of 181

<!-- page 31 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGST</td><td>string</td><td>-</td><td>$GPGST</td><td>GST Message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>time</td><td>hhmmss.ss</td><td>-</td><td>082356.00</td><td>UTC time of associated position fix. See section UTC representation in the integration manual for details.</td></tr><tr><td>2</td><td>rangeRms</td><td>numeric</td><td>m</td><td>1.8</td><td>RMS value of the standard deviation of the ranges</td></tr><tr><td>3</td><td>stdMajor</td><td>numeric</td><td>m</td><td>-</td><td>Standard deviation of semi-major axis</td></tr><tr><td>4</td><td>stdMinor</td><td>numeric</td><td>m</td><td>-</td><td>Standard deviation of semi-minor axis</td></tr><tr><td>5</td><td>orient</td><td>numeric</td><td>deg</td><td>-</td><td>Orientation of semi-major axis</td></tr><tr><td>6</td><td>stdLat</td><td>numeric</td><td>m</td><td>1.7</td><td>Standard deviation of latitude error</td></tr><tr><td>7</td><td>stdLong</td><td>numeric</td><td>m</td><td>1.3</td><td>Standard deviation of longitude error</td></tr><tr><td>8</td><td>stdAlt</td><td>numeric</td><td>m</td><td>2.2</td><td>Standard deviation of altitude error</td></tr><tr><td>9</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*7E</td><td>Checksum</td></tr><tr><td>10</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.15 GSV

2.7.15.1 GNSS satellites in view

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-GSVGNSS satellites in view</td></tr><tr><td colspan="2">Type</td><td colspan="4">Output</td></tr><tr><td colspan="2">Comment</td><td colspan="4">The number of satellites in view, together with each SV ID, elevation azimuth, and signal strength (C/No) value. Only four satellite details are transmitted in one message.In a multi-GNSS system, sets of GSV messages will be output multiple times, one set for each GNSS.The messages are grouped by the signal ID and separate messages are output for each signal ID. (supported for protocol versions 27.12 and later)If a satellite is visible but not tracked, the signal ID is unknown and is presented as 0.</td></tr><tr><td colspan="2">Information</td><td colspan="4">Class/ID: 0xf0 0x03 Number of fields: 7 + [1..4]·4</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxGSV, numMsg, msgNum, numSV{, svid, elv, az, cno}, signalId*cs\r\n</td></tr><tr><td colspan="2">Examples</td><td colspan="4">$GPGSV, 3, 1, 09, 09, , , 17, 10, , , 40, 12, , , 49, 13, , , 35, 1*6F\r\n$GPGSV, 3, 2, 09, 15, , , 44, 17, , , 45, 19, , , 44, 24, , , 50, 1*64\r\n$GPGSV, 3, 3, 09, 25, , , 40, 1*6E\r\n$GPGSV, 1, 1, 03, 12, , , 42, 24, , , 47, 32, , , 37, 5*66\r\n$GPGSV, 1, 1, 01, 03, 05, 218, , 0*59\r\n$GAGSV, 1, 1, 00, 2*76\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxGSV</td><td>string</td><td>-</td><td>$GPGSV</td><td>GSV Message ID (xx = GSV Talker ID, see NMEA Talker IDs table). Talker ID GN shall not be used.</td></tr><tr><td>1</td><td>numMsg</td><td>digit</td><td>-</td><td>3</td><td>Number of messages, total number of GSV messages being output (range: 1-9)</td></tr><tr><td>2</td><td>msgNum</td><td>digit</td><td>-</td><td>1</td><td>Number of this message (range: 1-numMsg)</td></tr><tr><td>3</td><td>numSV</td><td>numeric</td><td>-</td><td>10</td><td>Number of known satellites in view regarding both the talker ID and the signalId</td></tr><tr><td colspan="6">Start of repeated group (1...4 times)</td></tr><tr><td>4 + n·4</td><td>svid</td><td>numeric</td><td>-</td><td>23</td><td>Satellite ID</td></tr><tr><td>5 + n·4</td><td>elv</td><td>numeric</td><td>deg</td><td>38</td><td>Elevation (&lt;= 90)</td></tr></table>

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 31 of 181

<!-- page 32 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>6 + n·4 az</td><td>numeric</td><td>deg</td><td>230</td><td>Azimuth (range: 0-359)</td></tr><tr><td>7 + n·4 cno</td><td>numeric</td><td>dBHz</td><td>44</td><td>Signal strength (C/N0, range: 0-99), null when not tracking</td></tr><tr><td colspan="5">End of repeated group (1...4 times)</td></tr><tr><td>4 + N·4 signalId</td><td colspan="2">hexadecimal -</td><td>-</td><td>NMEA-defined GNSS signal ID, see Signal Identifiers table (only available in NMEA 4.10 and later)</td></tr><tr><td>5 + N·4 cs</td><td colspan="2">hexadecimal -</td><td>*7F</td><td>Checksum</td></tr><tr><td>6 + N·4 CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.16 RLM

## 2.7.16.1 Return link message (RLM)

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-RLM Return link message (RLM)</td></tr><tr><td colspan="2">Type</td><td colspan="4">Output</td></tr><tr><td colspan="2">Comment</td><td colspan="4">The RLM sentence is used to transfer a Return link message from a Cospas-Sarsat recognized Return link service provider (RLSP). The RLM sentence supports communications to an emitting beacon once a distress alert has been detected, located and confirmed. The communications may include acknowledgement of the alert to the emitting beacon as well as optional text messages, and may also include remote beacon configuration and testing.</td></tr><tr><td colspan="2">Information</td><td colspan="4">Class/ID: 0xf0 0x0b Number of fields: 7</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxRLM, beacon, time, code, body*cs\r\n</td></tr><tr><td colspan="2">Examples</td><td colspan="4">$GARLM, 00000078A9FBAD5, 083559.00, 3, C45B*57\r\n $GARLM, F7129D41BC6A78C, 034433.02, 3, B63CA732AFD419D2*57\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxRLM</td><td>string</td><td>-</td><td>$GARLM</td><td>RLM message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>beacon</td><td colspan="2">hexadecimal -</td><td>00000078A 9FBAD5</td><td>Beacon ID, identifies beacon intended to receive this message (fixed length 15 hexadecimal character field)</td></tr><tr><td>2</td><td>time</td><td colspan="2">hhmmss.ss -</td><td>083559.00</td><td>Time of reception field to indicate RLM timestamp in UTC. See section UTC representation in the integration manual for details.</td></tr><tr><td>3</td><td>code</td><td>character</td><td>-</td><td>3</td><td>Message code field to identify type of RLM Message Service: 0 = Reserved for future RLM services 1 = Acknowledgement service RLM 2 = Command service RLM 3 = Message service RLM 4-E = Reserved for future RLM services F = Test service RLM (currently used only by the Galileo program)</td></tr><tr><td>4</td><td>body</td><td colspan="2">hexadecimal -</td><td>C45B</td><td>Message body encapsulates the data parameters provided by the RLSP into hexadecimal format.</td></tr><tr><td>5</td><td>cs</td><td colspan="2">hexadecimal -</td><td>*57</td><td>Checksum</td></tr><tr><td>6</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.17 RMC

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 32 of 181

<!-- page 33 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

2.7.17.1 Recommended minimum data

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-RMC Recommended minimum data</td></tr><tr><td colspan="2">Type</td><td colspan="4">Output</td></tr><tr><td colspan="2">Comment</td><td colspan="4">The recommended minimum sentence defined by NMEA for GNSS system data. The output of this message is dependent on the currently selected datum (default: WGS84)</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x04</td><td colspan="3">Number of fields: 16</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxRMC, time, status, lat, NS, lon, EW, spd, cog, date, mv, mvEW, posMode, navStatus*cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$GPRMC, 083559.00, A, 4717.11437, N, 00833.91522, E, 0.004, 77.52, 091202, , ,A, V*57\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxRMC</td><td>string</td><td>-</td><td>$GPRMC</td><td>RMC Message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>time</td><td>hhmmss.ss</td><td>-</td><td>083559.00</td><td>UTC time. See section UTC representation in the integration manual for details.</td></tr><tr><td>2</td><td>status</td><td>character</td><td>-</td><td>A</td><td>Data validity status, see position fix flags description</td></tr><tr><td>3</td><td>lat</td><td>ddmm.mmmm</td><td>-</td><td>4717.11437</td><td>Latitude (degrees and minutes), see format description</td></tr><tr><td>4</td><td>NS</td><td>character</td><td>-</td><td>N</td><td>North/South indicator</td></tr><tr><td>5</td><td>lon</td><td>dddmm.mmmm</td><td>-</td><td>00833.91522</td><td>Longitude (degrees and minutes), see format description</td></tr><tr><td>6</td><td>EW</td><td>character</td><td>-</td><td>E</td><td>East/West indicator</td></tr><tr><td>7</td><td>spd</td><td>numeric</td><td>knots</td><td>0.004</td><td>Speed over ground</td></tr><tr><td>8</td><td>cog</td><td>numeric</td><td>deg</td><td>77.52</td><td>Course over ground</td></tr><tr><td>9</td><td>date</td><td>ddmmyy</td><td>-</td><td>091202</td><td>Date in day, month, year format. See section UTC representation in the integration manual for details.</td></tr><tr><td>10</td><td>mv</td><td>numeric</td><td>deg</td><td>-</td><td>Magnetic variation value</td></tr><tr><td>11</td><td>mvEW</td><td>character</td><td>-</td><td>-</td><td>Magnetic variation E/W indicator</td></tr><tr><td>12</td><td>posMode</td><td>character</td><td>-</td><td>A</td><td>Mode Indicator, see position fix flags description (only available in NMEA 2.3 and later)</td></tr><tr><td>13</td><td>navStatus</td><td>character</td><td>-</td><td>V</td><td>Navigational status indicator: V (Equipment is not providing navigational status information, fixed field, only available in NMEA 4.10 and later)</td></tr><tr><td>14</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*57</td><td>Checksum</td></tr><tr><td>15</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.18 TXT

## 2.7.18.1 Text transmission

| Message | NMEA-Standard-TXT Text transmission |
| --- | --- |
| Type | Output |
| Comment | This message outputs various information on the receiver, such as power-up screen, software version etc. This message can be configured using the CFG-INFMSG configuration group. |
| Information | Class/ID: 0xf0 0x41 Number of fields: 7 |
| Structure | $xxTXT,numMsg,msgNum,msgType,text*cs\\r\\n |

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 33 of 181

<!-- page 34 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

Examples \$GPTXT,01,01,02,u-blox ag - www.u-blox.com\*50\r\n \$GPTXT,01,01,02,ANTARIS ATR0620 HW 00000040\*67\r\n

<table><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxTXT</td><td>string</td><td>-</td><td>$GPTXT</td><td>TXT Message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>numMsg</td><td>numeric</td><td>-</td><td>01</td><td>Total number of messages in this transmission (range: 1-99)</td></tr><tr><td>2</td><td>msgNum</td><td>numeric</td><td>-</td><td>01</td><td>Message number in this transmission (range: 1-numMsg)</td></tr><tr><td>3</td><td>msgType</td><td>numeric</td><td>-</td><td>02</td><td>Text identifier (u-blox receivers specify the type of the message with this number):00 = Error01 = Warning02 = Notice07 = User</td></tr><tr><td>4</td><td>text</td><td>string</td><td>-</td><td>www.u-blox.com</td><td>Any ASCII text</td></tr><tr><td>5</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*67</td><td>Checksum</td></tr><tr><td>6</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.19 VLW

## 2.7.19.1 Dual ground/water distance

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-VLWDual ground/water distance</td></tr><tr><td colspan="2">Type</td><td colspan="4">Output</td></tr><tr><td colspan="2">Comment</td><td colspan="4">The distance traveled, relative to the water and over the ground. This message relates to the odometer feature detailed in the integration manual.</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x0f</td><td colspan="3">Number of fields: 11</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxVLW,twd,twdUnit,wd,wdUnit,tgd,tgdUnit,gd,gdUnit*cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$GPVLW,,N,,N,15.8,N,1.2,N*06\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxVLW</td><td>string</td><td>-</td><td>$GPVLW</td><td>VLW Message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>twd</td><td>numeric</td><td>nmi</td><td>-</td><td>Total cumulative water distance: null (fixed field)</td></tr><tr><td>2</td><td>twdUnit</td><td>character</td><td>-</td><td>N</td><td>Total cumulative water distance units: N (nautical miles, fixed field)</td></tr><tr><td>3</td><td>wd</td><td>numeric</td><td>nmi</td><td>-</td><td>Water distance since reset: null (fixed field)</td></tr><tr><td>4</td><td>wdUnit</td><td>character</td><td>-</td><td>N</td><td>Water distance since reset units: N (nautical miles, fixed field)</td></tr><tr><td>5</td><td>tgd</td><td>numeric</td><td>nmi</td><td>15.8</td><td>Total cumulative ground distance (only available in NMEA 4.00 and later)</td></tr><tr><td>6</td><td>tgdUnit</td><td>character</td><td>-</td><td>N</td><td>Total cumulative ground distance units: N (nautical miles, fixed field, only available in NMEA 4.00 and later)</td></tr><tr><td>7</td><td>gd</td><td>numeric</td><td>nmi</td><td>1.2</td><td>Ground distance since reset (only available in NMEA 4.00 and later)</td></tr></table>

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 34 of 181

<!-- page 35 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 8 | gdUnit | character - | N | Ground distance since reset units: N (nautical miles, fixed field, only available in NMEA 4.00 and later) |
| --- | --- | --- | --- | --- |
| 9 | cs | hexadecimal - | *06 | Checksum |
| 10 | CRLF | character - | - | Carriage return and line feed |

## 2.7.20 VTG

2.7.20.1 Course over ground and ground speed

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-Standard-VTG Course over ground and ground speed</td></tr><tr><td colspan="2">Type</td><td colspan="4">Output</td></tr><tr><td colspan="2">Comment</td><td colspan="4">Velocity is given as course over ground (COG) and speed over ground (SOG).</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf0 0x05</td><td colspan="3">Number of fields: 12</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$xxVTG, cogt, cogtUnit, cogm, cogmUnit, sogn, sognUnit, sogk, sogkUnit, posMode*cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$GPVTG, 77.52, T,,M, 0.004, N, 0.008, K, A*06\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>xxVTG</td><td>string</td><td>-</td><td>$GPVTG</td><td>VTG Message ID (xx = current Talker ID, see NMEA Talker IDs table)</td></tr><tr><td>1</td><td>cogt</td><td>numeric</td><td>degrees</td><td>77.52</td><td>Course over ground (true)</td></tr><tr><td>2</td><td>cogtUnit</td><td>character</td><td>-</td><td>T</td><td>Course over ground units: T (degrees true, fixed field)</td></tr><tr><td>3</td><td>cogm</td><td>numeric</td><td>degrees</td><td>-</td><td>Course over ground (magnetic)</td></tr><tr><td>4</td><td>cogmUnit</td><td>character</td><td>-</td><td>M</td><td>Course over ground units: M (degrees magnetic, fixed field)</td></tr><tr><td>5</td><td>sogn</td><td>numeric</td><td>knots</td><td>0.004</td><td>Speed over ground</td></tr><tr><td>6</td><td>sognUnit</td><td>character</td><td>-</td><td>N</td><td>Speed over ground units: N (knots, fixed field)</td></tr><tr><td>7</td><td>sogk</td><td>numeric</td><td>km/h</td><td>0.008</td><td>Speed over ground</td></tr><tr><td>8</td><td>sogkUnit</td><td>character</td><td>-</td><td>K</td><td>Speed over ground units: K (kilometers per hour, fixed field)</td></tr><tr><td>9</td><td>posMode</td><td>character</td><td>-</td><td>A</td><td>Mode indicator, see position fix flags description (only available in NMEA 2.3 and later)</td></tr><tr><td>10</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*06</td><td>Checksum</td></tr><tr><td>11</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.7.21 ZDA

## 2.7.21.1 Time and date

| Message | NMEA-Standard-ZDA Timeanddate |
| --- | --- |
| Type | Output |
| Comment | UTC,day,month,yearandlocaltimezone. |
| Information | Class/ID: 0xf0 0x08 Number of fields: 9 |
| Structure | $xxZDA,time,day,month,year,ltzh,ltzn*cs\\r\\n |
| Example | $GPZDA,082710.00,16,09,2002,00,00*64\\r\\n |
| Payload: |  |
| Field Name | Format Unit Example Description |

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 35 of 181

<!-- page 36 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 0 | xxZDA | string | - | $GPZDA | ZDA Message ID (xx = current Talker ID, see NMEA Talker IDs table) |
| --- | --- | --- | --- | --- | --- |
| 1 | time | hhmmss.ss | - | 082710.00 | UTC Time. See section UTC representation in the integration manual for details. |
| 2 | day | dd | day | 16 | UTC day (range: 1-31) |
| 3 | month | mm | month | 09 | UTC month (range: 1-12) |
| 4 | year | yyyy | year | 2002 | UTC year |
| 5 | ltzh | xx | - | 00 | Local time zone hours (fixed field, always 00) |
| 6 | ltzn | zz | - | 00 | Local time zone minutes (fixed field, always 00) |
| 7 | cs | hexadecimal | - | *64 | Checksum |
| 8 | CRLF | character | - | - | Carriage return and line feed |

## 2.8 PUBX messages

Proprietary NMEA messages for u-blox positioning receivers. See also NMEA-proprietary messages.

## 2.8.1 CONFIG (PUBX,41)

2.8.1.1 Set protocols and baud rate

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-PUBX-CONFIGSet protocols and baud rate</td></tr><tr><td colspan="2">Type</td><td colspan="4">Set</td></tr><tr><td colspan="6">Comment</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf1 0x41</td><td colspan="3">Number of fields: 9</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$PUBX, 41, portId, inProto, outProto, baudrate, autobauding*cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$PUBX, 41, 1,0007,0003, 19200, 0*25\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>PUBX</td><td>string</td><td>-</td><td>$PUBX</td><td>Message ID, UBX protocol header, proprietary sentence</td></tr><tr><td>1</td><td>msgId</td><td>numeric</td><td>-</td><td>41</td><td>Proprietary message identifier</td></tr><tr><td>2</td><td>portId</td><td>numeric</td><td>-</td><td>1</td><td>ID of communication port. See section Communication ports in the integration manual for details.</td></tr><tr><td>3</td><td>inProto</td><td colspan="2">hexadecimal -</td><td>0007</td><td>Input protocol mask. Bitmask, specifying which protocols(s) are allowed for input. See section Communication ports in the integration manual for details.</td></tr><tr><td>4</td><td>outProto</td><td colspan="2">hexadecimal -</td><td>0003</td><td>Output protocol mask. Bitmask, specifying which protocols(s) are allowed for input. See section Communication ports in the integration manual for details.</td></tr><tr><td>5</td><td>baudrate</td><td>numeric</td><td>bits/s</td><td>19200</td><td>Baud rate</td></tr><tr><td>6</td><td>autobauding</td><td>numeric</td><td>-</td><td>-</td><td>Autobauding: 1=enable, 0=disable (not supported on u-blox 5, set to 0)</td></tr><tr><td>7</td><td>cs</td><td colspan="2">hexadecimal -</td><td>*25</td><td>Checksum</td></tr><tr><td>8</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.8.2 POSITION (PUBX,00)

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 36 of 181

<!-- page 37 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

2.8.2.1 Poll a PUBX,00 message

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-PUBX-POSITION Poll a PUBX,00 message</td></tr><tr><td colspan="2">Type</td><td colspan="4">Poll request</td></tr><tr><td colspan="2">Comment</td><td colspan="4">A PUBX,00 message is polled by sending the PUBX,00 message without any data fields.</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf1 0x00</td><td colspan="3">Number of fields: 4</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$PUBX, 00*33\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$PUBX, 00*33\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>PUBX</td><td>string</td><td>-</td><td>$PUBX</td><td>Message ID, UBX protocol header, proprietary sentence</td></tr><tr><td>1</td><td>msgId</td><td>numeric</td><td>-</td><td>00</td><td>Set to 00 to poll a PUBX,00 message</td></tr><tr><td>2</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*33</td><td>Checksum</td></tr><tr><td>3</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

2.8.2.2 Lat/Long position data

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-PUBX-POSITION Lat/Long position data</td></tr><tr><td colspan="2">Type</td><td colspan="4">Output</td></tr><tr><td colspan="2">Comment</td><td colspan="4">This message contains position solution data. The datum selection may be changed using the message UBX-CFG-DAT. The output of this message is dependent on the currently selected datum (default: WGS84).</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf1 0x00</td><td colspan="3">Number of fields: 23</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$PUBX, 00, time, lat, NS, long, EW, altRef, navStat, hAcc, vAcc, SOG, COG, vVel, diffAge, HDOP, VDOP, TDOP, numSvs, reserved, DR, *cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$PUBX, 00, 081350.00, 4717.113210, N, 00833.915187, E, 546.589, G3, 2.1, 2.0, 0.007, 77.52, 0.007, , 0.92, 1.19, 0.77, 9, 0, 0*5F\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>PUBX</td><td>string</td><td>-</td><td>$PUBX</td><td>Message ID, UBX protocol header, proprietary sentence</td></tr><tr><td>1</td><td>msgId</td><td>numeric</td><td>-</td><td>00</td><td>Proprietary message identifier: 00</td></tr><tr><td>2</td><td>time</td><td>hhmmss.ss</td><td>-</td><td>081350.00</td><td>UTC time. See section UTC representation in the integration manual for details.</td></tr><tr><td>3</td><td>lat</td><td>ddmm.mmmm</td><td>-</td><td>4717.113210</td><td>Latitude (degrees and minutes), see format description</td></tr><tr><td>4</td><td>NS</td><td>character</td><td>-</td><td>N</td><td>North/South Indicator</td></tr><tr><td>5</td><td>long</td><td>dddmm.mmmm</td><td>-</td><td>00833.915187</td><td>Longitude (degrees and minutes), see format description</td></tr><tr><td>6</td><td>EW</td><td>character</td><td>-</td><td>E</td><td>East/West indicator</td></tr><tr><td>7</td><td>altRef</td><td>numeric</td><td>m</td><td>546.589</td><td>Altitude above user datum ellipsoid</td></tr></table>

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 37 of 181

<!-- page 38 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 8 | navStat | string | - | G3 | Navigation Status:NF = No FixDR = Dead reckoning only solutionG2 = Stand alone 2D solutionG3 = Stand alone 3D solutionD2 = Differential 2D solutionD3 = Differential 3D solutionRK = Combined GPS + dead reckoning solutionTT = Time only solution |
| --- | --- | --- | --- | --- | --- |

| 9 | hAcc | numeric | m | 2.1 | Horizontal accuracy estimate |
| --- | --- | --- | --- | --- | --- |
| 10 | vAcc | numeric | m | 2.0 | Vertical accuracy estimate |
| 11 | SOG | numeric | km/h | 0.007 | Speed over ground |
| 12 | COG | numeric | deg | 77.52 | Course over ground |
| 13 | vVel | numeric | m/s | 0.007 | Vertical velocity (positive downwards) |
| 14 | diffAge | numeric | s | - | Age of differential corrections (blank when DGPS is not used) |
| 15 | HDOP | numeric | - | 0.92 | HDOP, Horizontal Dilution of Precision |
| 16 | VDOP | numeric | - | 1.19 | VDOP, Vertical Dilution of Precision |
| 17 | TDOP | numeric | - | 0.77 | TDOP, Time Dilution of Precision |
| 18 | numSvs | numeric | - | 9 | Number of satellites used in the navigation solution |
| 19 | reserved | numeric | - | - | Reserved, always set to 0 |
| 20 | DR | numeric | - | - | DR used |
| 21 | cs | hexadecimal | - | *5B | Checksum |
| 22 | CRLF | character | - | - | Carriage return and line feed |

## 2.8.3 RATE (PUBX,40)

## 2.8.3.1 Set NMEA message output rate

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-PUBX-RATESet NMEA message output rate</td></tr><tr><td colspan="2">Type</td><td colspan="4">Set</td></tr><tr><td colspan="2">Comment</td><td colspan="4">Set/Get message rate configuration (s) to/from the receiver.Send rate is relative to the event a message is registered on. For example, if the rate of a navigation message is set to 2, the message is sent every second navigation solution.</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf1 0x40</td><td colspan="3">Number of fields: 11</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$PUBX, 40, msgId, rddc, rus1, rus2, rusb, rspi, reserved*cs\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$PUBX, 40, GLL, 1, 0, 0, 0, 0*5D\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>PUBX</td><td>string</td><td>-</td><td>$PUBX</td><td>Message ID, UBX protocol header, proprietary sentence</td></tr><tr><td>1</td><td>ID</td><td>numeric</td><td>-</td><td>40</td><td>Proprietary message identifier</td></tr><tr><td>2</td><td>msgId</td><td>string</td><td>-</td><td>GLL</td><td>NMEA message identifier</td></tr><tr><td>3</td><td>rddc</td><td>numeric</td><td>cycles</td><td>1</td><td>output rate on DDC0 disables that message from being output on this port1 means that this message is output every epoch</td></tr></table>

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 38 of 181

<!-- page 39 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 4 | rus1 | numeric | cycles | 1 | output rate on USART 10 disables that message from being output on this port1 means that this message is output every epoch |
| --- | --- | --- | --- | --- | --- |
| 5 | rus2 | numeric | cycles | 1 | output rate on USART 20 disables that message from being output on this port1 means that this message is output every epoch |
| 6 | rusb | numeric | cycles | 1 | output rate on USB0 disables that message from being output on this port1 means that this message is output every epoch |
| 7 | rspi | numeric | cycles | 1 | output rate on SPI0 disables that message from being output on this port1 means that this message is output every epoch |
| 8 | reserved | numeric | - | - | Reserved: always fill with 0 |
| 9 | cs | hexadecimal | - | *5D | Checksum |
| 10 | CRLF | character | - | - | Carriage return and line feed |

## 2.8.4 SVSTATUS (PUBX,03)

2.8.4.1 Poll a PUBX,03 message

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-PUBX-SVSTATUS Poll a PUBX,03 message</td></tr><tr><td colspan="2">Type</td><td colspan="4">Poll request</td></tr><tr><td colspan="2">Comment</td><td colspan="4">A PUBX,03 message is polled by sending the PUBX,03 message without any data fields.</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf1 0x03</td><td colspan="3">Number of fields: 4</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$PUBX, 03*30\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$PUBX, 03*30\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>PUBX</td><td>string</td><td>-</td><td>$PUBX</td><td>Message ID, UBX protocol header, proprietary sentence</td></tr><tr><td>1</td><td>msgId</td><td>numeric</td><td>-</td><td>03</td><td>Set to 03 to poll a PUBX,03 message</td></tr><tr><td>2</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*30</td><td>Checksum</td></tr><tr><td>3</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

2.8.4.2 Satellite status

| Message | NMEA-PUBX-SVSTATUS Satellite status |
| --- | --- |
| Type | Output |
| Comment | The PUBX,03 message contains satellite status information. |
| Information | Class/ID: 0xf1 0x03 Number of fields: 5 + n·6 |
| Structure | $PUBX,03,GT{,sv,s,az,el,cno,lck},*cs\\r\\n |
| Example | $PUBX,03,11,23,-,,,45,010,29,-,,,46,013,07,-,,,42,015,08,U,067,31,42,025,10,U,195,33 ↲,46,026,18,U,326,08,39,026,17,-,,,32,015,26,U,306,66,48,025,27,U,073,10,36,026,28,U, ↲ 089,61,46,024,15,-,,,39,014*0D\\r\\n |
| Payload: |  |
| Field Name | Format Unit Example Description |

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 39 of 181

<!-- page 40 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>0</td><td>PUBX</td><td>string</td><td>-</td><td>$PUBX</td><td>Message ID, UBX protocol header, proprietary sentence</td></tr><tr><td>1</td><td>msgId</td><td>numeric</td><td>-</td><td>03</td><td>Proprietary message identifier: 03</td></tr><tr><td>2</td><td>n</td><td>numeric</td><td>-</td><td>11</td><td>Number of GNSS satellites tracked</td></tr><tr><td colspan="6">Start of repeated group (n times)</td></tr><tr><td>3 + n·6</td><td>sv</td><td>numeric</td><td>-</td><td>23</td><td>Satellite ID according to UBX svld mapping (see Satellite Numbering)</td></tr><tr><td>4 + n·6</td><td>s</td><td>character</td><td>-</td><td>-</td><td>Satellite status:• - = Not used• U = Used in solution• e = Ephemeris available, but not used for navigation</td></tr><tr><td>5 + n·6</td><td>az</td><td>numeric</td><td>deg</td><td>-</td><td>Satellite azimuth (range: 0-359)</td></tr><tr><td>6 + n·6</td><td>el</td><td>numeric</td><td>deg</td><td>-</td><td>Satellite elevation (&lt;= 90)</td></tr><tr><td>7 + n·6</td><td>cno</td><td>numeric</td><td>dBHz</td><td>45</td><td>Signal strength (C/N0, range 0-99), blank when not tracking</td></tr><tr><td>8 + n·6</td><td>lck</td><td>numeric</td><td>s</td><td>010</td><td>Satellite carrier lock time (range: 0-64)• 0 = code lock only• 64 = lock for 64 seconds or more</td></tr><tr><td colspan="6">End of repeated group (n times)</td></tr><tr><td>3 + n·6</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*0D</td><td>Checksum</td></tr><tr><td>4 + n·6</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

## 2.8.5 TIME (PUBX,04)

## 2.8.5.1 Poll a PUBX,04 message

<table><tr><td colspan="2">Message</td><td colspan="4">NMEA-PUBX-TIME Poll a PUBX,04 message</td></tr><tr><td colspan="2">Type</td><td colspan="4">Poll request</td></tr><tr><td colspan="2">Comment</td><td colspan="4">A PUBX,04 message is polled by sending the PUBX,04 message without any data fields.</td></tr><tr><td colspan="2">Information</td><td>Class/ID: 0xf1 0x04</td><td colspan="3">Number of fields: 4</td></tr><tr><td colspan="2">Structure</td><td colspan="4">$PUBX, 04*37\r\n</td></tr><tr><td colspan="2">Example</td><td colspan="4">$PUBX, 04*37\r\n</td></tr><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>PUBX</td><td>string</td><td>-</td><td>$PUBX</td><td>Message ID, UBX protocol header, proprietary sentence</td></tr><tr><td>1</td><td>msgId</td><td>numeric</td><td>-</td><td>04</td><td>Set to 04 to poll a PUBX,04 message</td></tr><tr><td>2</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*37</td><td>Checksum</td></tr><tr><td>3</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

2.8.5.2 Time of day and clock information

| Message | NMEA-PUBX-TIME Timeofdayandclockinformation |
| --- | --- |
| Type | Output |
| Comment |  |
| Information | Class/ID: 0xf1 0x04 Number of fields: 12 |
| Structure | $PUBX,04,time,date,utcTow,utcWk,leapSec,clkBias,clkDrift,tpGran,*cs\\r\\n |

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 40 of 181

<!-- page 41 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

Example \$PUBX,04,073731.00,091202,113851.00,1196,15D,1930035,-2660.664,43,\*3C\r\n

<table><tr><td colspan="6">Payload:</td></tr><tr><td>Field</td><td>Name</td><td>Format</td><td>Unit</td><td>Example</td><td>Description</td></tr><tr><td>0</td><td>PUBX</td><td>string</td><td>-</td><td>$PUBX</td><td>Message ID, UBX protocol header, proprietary sentence</td></tr><tr><td>1</td><td>msgId</td><td>numeric</td><td>-</td><td>04</td><td>Proprietary message identifier: 04</td></tr><tr><td>2</td><td>time</td><td>hhmmss.ss</td><td>-</td><td>073731.00</td><td>UTC time. See section UTC representation in the integration manual for details.</td></tr><tr><td>3</td><td>date</td><td>ddmmyy</td><td>-</td><td>091202</td><td>UTC date, day, month, year. See section UTC representation in the integration manual for details.</td></tr><tr><td>4</td><td>utcTow</td><td>numeric</td><td>s</td><td>113851.00</td><td>UTC time of week</td></tr><tr><td>5</td><td>utcWk</td><td>numeric</td><td>-</td><td>1196</td><td>UTC week number, continues beyond 1023</td></tr><tr><td>6</td><td>leapSec</td><td>numeric/text</td><td>s</td><td>15D</td><td>Leap seconds (not supported for protocol versions less than 13.01)The number is marked with a D if the value is the firmware default value. If the value is not marked it has been received from a satellite.</td></tr><tr><td>7</td><td>clkBias</td><td>numeric</td><td>ns</td><td>1930035</td><td>Receiver clock bias</td></tr><tr><td>8</td><td>clkDrift</td><td>numeric</td><td>ns/s</td><td>-2660.664</td><td>Receiver clock drift</td></tr><tr><td>9</td><td>tpGran</td><td>numeric</td><td>ns</td><td>43</td><td>Time pulse granularity, the quantization error of the TIMEPULSE pin</td></tr><tr><td>10</td><td>cs</td><td>hexadecimal</td><td>-</td><td>*3C</td><td>Checksum</td></tr><tr><td>11</td><td>CRLF</td><td>character</td><td>-</td><td>-</td><td>Carriage return and line feed</td></tr></table>

UBX-23002975 - R02 C1-Public

2 NMEA protocol

Page 41 of 181

<!-- page 42 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## 3 UBX protocol

## 3.1 UBX protocol key features

u-blox receivers support a u-blox-proprietary protocol to communicate with a host computer. This protocol has the following key features:

• Compact – uses 8-bit binary data

• Checksum protected – uses a low-overhead checksum algorithm

• Modular – uses a two-stage message identifier (Class and Message ID)

## 3.2 UBX frame structure

The structure of a basic UBX frame is shown in the following diagram.

Preamble sync character 1 (0xb5 in hex, 181 in dec, ISO 8859-1 forμ)

Preamble sync character 2 (0x62 in hex, 98 in dec, ISO 8859-1 for b)

<table><tr><td rowspan="638"></td><td rowspan="637"></td><td colspan="4">Message class</td><td rowspan="637" colspan="4">Length (N) of the payload only (in bytes)</td></tr><tr><td rowspan="636" colspan="4">Message ID</td></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr></table>

Range over which the UBX checksum is to be calculated

• Every frame starts with a 2-byte preamble consisting of two synchronization characters: 0xb5 and 0x62.

• A 1-byte message class field follows. A class is a group of messages that are related to each other.

• A 1-byte message ID field defines the message that is to follow.

A 2-byte length field follows. The length is defined as being that of the payload only. It does not include the preamble, message class, message ID, length, or UBX checksum fields. The number format of the length field is an unsigned little-endian 16-bit integer (a "U2" in UBX data types).

• The payload field contains a variable number (= length) of bytes.

• The two 1-byte CK\_A and CK\_B fields hold a 16-bit checksum whose calculation is defined in UBX checksum section. This concludes the frame.

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 42 of 181

<!-- page 43 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## 3.3 UBX payload definition rules

This section contains the rules and guidelines for UBX message payloads. See also UBX message example.

## 3.3.1 UBX structure packing

Values are placed in such an order that structure packing is not a problem. This means that twobyte values shall start on offsets that are a multiple of two; four-byte values shall start at a multiple of four; and so on.

## 3.3.2 UBX reserved elements

Some messages contain reserved fields or bits to allow for future expansion. The contents of these elements should be ignored in output messages and must be set to zero in input messages. Where a message is output and subsequently returned to the receiver as an input message, reserved elements can either be explicitly set to zero or left with whatever value they were output with.

For fields in a bitfield the same rules apply. Note that bits not described are automatically reserved and are not explicitly stated (see UBX message example).

## 3.3.3 UBX undefined values

The description of some fields provide specific meanings for specific values. For example, the field gnssId appears in many UBX messages and uses 0 to indicate GPS, 1 for SBAS and so on (see GNSS identifiers for details); however it is usually stored in a byte with far more possible values than the handful currently defined. All such undefined values are reserved for future expansion and therefore should not be used.

## 3.3.4 UBX conditional values

Some UBX messages use validity flag fields to indicate whether the values of some value fields are valid. For example, the UBX-NAV-PVT message has the validDate and validTime fields that indicate whether the date (year, month and day fields), and, respectively, the time (hour, min and sec fields) are valid. This means that these value fields will only contain meaningful data if the corresponding flag field is set (has the value 1).

## 3.3.5 UBX data types

The following data types (number formats) are defined.

| Name | Type | Size (Bytes) | Range | Resolution |
| --- | --- | --- | --- | --- |
| U1 | unsigned 8-bit integer | 1 | 0…2<sup>8</sup>-1 | 1 |
| I1 | signed 8-bit integer, two's complement | 1 | -2<sup>7</sup>…2<sup>7</sup>-1 | 1 |
| X1 | 8-bit bitfield | 1 | n/a | n/a |
| U2 | unsigned little-endian 16-bit integer | 2 | 0…2<sup>16</sup>-1 | 1 |
| I2 | signed little-endian 16-bit integer, two's complement | 2 | -2<sup>15</sup>…2<sup>15</sup>-1 | 1 |
| X2 | 16-bit little-endian bitfield | 2 | n/a | n/a |
| U4 | unsigned little-endian 32-bit integer | 4 | 0…2<sup>32</sup>-1 | 1 |
| I4 | signed little-endian 32-bit integer, two's complement | 4 | -2<sup>31</sup>…2<sup>31</sup>-1 | 1 |
| X4 | 32-bit little-endian bitfield | 4 | n/a | n/a |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 43 of 181

<!-- page 44 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Name | Type | Size (Bytes) | Range | Resolution |
| --- | --- | --- | --- | --- |
| R4 | IEEE 754 single (32-bit) precision | 4 | -2<sup>127</sup>…2<sup>127</sup> | ~ value·2-24 |
| R8 | IEEE 754 double (64-bit) precision | 8 | -2<sup>1023</sup>…2<sup>1023</sup> | ~ value·2-53 |
| CH | ASCII/ISO 8859-1 char (8-bit) | 1 | n/a | n/a |
| U:n | unsigned bitfield value of n bits width | var. | variable | variable |
| I:n | signed (two's complement) bitfield value of n bits width | var. | variable | variable |
| S:n | signed bitfield value of n bits width, in sign (most significant bit) and magnitude (remaining bits) notation | var. | variable | variable |

## 3.3.6 UBX fields scale and unit

Fields in UBX messages can have a unit defined. Whenever possible, SI units and symbols are used (e.g. "m" for meters, "s" for seconds). For civil (UTC) time representation units of years (y), months (month), days (d), hours (h), minutes (min) and seconds (s) are used.

Fields in UBX messages can have a scale factor defined. Unity (factor 1) is assumed if no scale is specified. For integer type fields this is often combined with a unit. When a scale is combined with a unit, the scale represents the smallest storage unit. For example, if meters (m) are expressed (stored) in centimeters the scale would be 0.01 (or 1e-2). This is equivalent of specifying a unit of centimeters (cm) and no scale.

The description of some integer values (e.g. U2, I4 or I8) indicates a fixed-point format (e.g. [UU.FF], [IIIII.FFF] or [IIIIIII.FFFFFFFF]). The fixed-point value can be retrieved from the integer value by first casting it to appropriate type (e.g. as a floating-point number) and then scaling it with the indicated scaling factor.

## 3.3.7 UBX repeated fields

There are two types of repetitions in UBX messages. The first type specifies that a single field is repeated a constant number of times. This repetition is defined in the type of the field. For example, the UBX message example can specify a field data of type U1[5]. In this case the data field should be interpreted as an array of five U1 values.

The second type of repetition in messages is referred to as repeated groups, which groups one or more fields into a block of payload data. There are several types of repetition:

The number of repetitions of variable-by-field group is indicated by another, earlier field in the same message. The number of repetitions can be zero or more, depending on the value of the referenced field.

• A constant group has a constant number of repetitions.

An optional group is repeated zero or one times, depending on the available payload data. That is, the fields are present in the message only if the payload of the message is large enough to cover the whole group of fields.

The number of repetitions of a variable-by-size group is given by the available payload size. The group will repeat until there is not enough payload data left to cover the whole group of fields another time.

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 44 of 181

<!-- page 45 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

Note that only some combinations of repeated groups of fields are possible in a single message. See also UBX payload decoding.

## 3.3.8 UBX payload decoding

UBX message payloads are designed so that the data (fields) can be extracted by a single pass through the payload from start to end. Fixed-size messages are the trivial case where the offset of all fields is unambiguously defined. Variable-size messages have variable number of repetitions of one or multiple groups of fields. For groups where the number of repetitions is given by the value of another field, that field can always be found at a fixed offset in the message payload before the respective group of fields. Groups whose number of repetitions depend on the payload size can only be the last group of fields in a message and only one such group may exist in a message. See also UBX repeated fields.

## 3.4 UBX checksum

The checksum is calculated over the message, starting and including the class field up until, but excluding, the checksum fields (see the figure UBX frame structure).

The checksum algorithm used is the 8-bit Fletcher algorithm, which is used in the TCP standard [RFC 1145](http://www.ietf.org/rfc/rfc1145.txt)). This algorithm works as follows:

Buffer[N] is an array of bytes that contains the data over which the checksum is to be calculated.

The two CK\_A and CK\_A values are 8-bit unsigned integers, only! If implementing with largersized integer values, make sure to mask both CK\_A and CK\_B with the value 0xff after both operations in the loop.

• After the loop, the two U1 values contain the checksum, transmitted after the message payload, which concludes the frame.

```c
CK_A = 0, CK_B = 0
For (I = 0; I < N; I++)
{
    CK_A = CK_A + Buffer[I]
    CK_B = CK_B + CK_A
}
```

## 3.5 UBX message flow

There are certain features associated with the messages being sent back and forth:

## 3.5.1 UBX acknowledgement

When messages from the class CFG are sent to the receiver, the receiver will send an "acknowledge" (UBX-ACK-ACK) or a "not acknowledge" (UBX-ACK-NAK) message back to the sender, depending on whether or not the message was processed correctly.

Some messages from other classes also use the same acknowledgement mechanism.

## 3.5.2 UBX polling mechanism

The UBX protocol is designed so that messages can be polled by sending the message required to the receiver but without a payload (or with just a single parameter that identifies the poll request). The receiver then responds with the same message with the payload populated.

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 45 of 181

<!-- page 46 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## 3.6 GNSS, satellite, and signal numbering

See GNSS, satellite, and signal identifiers for details on how GNSS, satellites and signals are numbered in the UBX protocol.

## 3.7 UBX message example

This is an example of the definition of UBX messages as shown in the following sections.

<table><tr><td>Message1</td><td colspan="6">UBX-DEMO-EXAMPLEExample demo message</td></tr><tr><td>Type 2</td><td colspan="6">Periodic/polled</td></tr><tr><td>Comment3</td><td colspan="6">This is a comment that describes the use of the demo example message.There can be references to other sections in the documentation (such as:UBX protocol).Note that there can be important remarks here.</td></tr><tr><td>Message4</td><td>Header</td><td>Class ID</td><td colspan="2">Length (bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>Structure</td><td>0xb5</td><td>0x62 0x01</td><td>0x07</td><td>16 + numRepeat*4</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="7">Payload description: 5</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U4</td><td>aField</td><td>-</td><td>-</td><td colspan="2">a field that contains an unsigned integer with no particular scale or unit</td></tr><tr><td>4</td><td>I4</td><td>anotherField</td><td>1e-2</td><td>m</td><td colspan="2">a field that contains a length in meters (m) with a scale of 1e-2 (= 0.01), i.e. a length in centimeters</td></tr><tr><td>8</td><td>X2</td><td>bitfield 6</td><td>-</td><td>-</td><td colspan="2">this field contains flags or values smaller than one byte, whose definition follows below (bits not described are reserved)</td></tr><tr><td>bit 0</td><td>U:1</td><td>aFieldValid</td><td>-</td><td>-</td><td colspan="2">the first bit in bitfield indicates whether the aField is valid or not (seeUBX conditional values)</td></tr><tr><td>bit 1</td><td>U:1</td><td>someFlag</td><td>-</td><td>-</td><td colspan="2">the second bit is a flag (1 = true, 0 = false)</td></tr><tr><td>bits 5...2</td><td>U:4</td><td>aBitFieldValue</td><td>-</td><td>-</td><td colspan="2">a 4-bits value (range: 0...15)</td></tr><tr><td>10</td><td>U1[5] 7</td><td>reserved0</td><td>-</td><td>-</td><td colspan="2">a reserved field, whose value shall be ignored (in output messages) or set to 0 (in input messages)</td></tr><tr><td>15</td><td>U1</td><td>numRepeat</td><td>-</td><td>-</td><td colspan="2">number of repetitions in the group of fields below</td></tr><tr><td colspan="7">Start of repeated group (numRepeat times) 8</td></tr><tr><td>16 + n*4</td><td>I2</td><td>someValue</td><td>-</td><td>-</td><td colspan="2">a signed value in a repeated group of fields</td></tr><tr><td>18 + n*4</td><td>U2</td><td>anotherValue</td><td>-</td><td>-</td><td colspan="2">another value in a repeated group of fields</td></tr><tr><td colspan="7">End of repeated group (numRepeat times)</td></tr></table>

❶ The first line shows the message name (see Message naming). The second line shows a short description of the message.

❷ The message type (see Message types).

❸ This section contains comments that describe the message. Often links to other related sections in the documentation or other related messages are found here.

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 46 of 181

<!-- page 47 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

❹ The message structure gives the parameters for the UBX frame structure, notably the message class and message ID values and the payload length. For many messages the payload length is a fixed number (of bytes). Messages that contain repeated blocks of information (fields) have a variable payload (see UBX repeated fields).

❺ The message payload definition is given as a list of fields and their parameters. Each field starts at a specified offset (in bytes) in the payload (see also UBX structure packing), is of a specific type (see UBX data types), has a unique name (within the message), and a description. Optionally, fields can have a scale and/or a unit (see UBX fields scale and unit).

❻ Bitfields ("X" types) are broken down into smaller parts. Each part can be one or more bits wide. Values that are two or more bits wide can be unsigned or one of two signed value representation (see UBX data types). Note that the ten unused bits 15…6 are not explicitly stated as UBX reserved elements.

❼ Fields can be arrays of values of the same type (see UBX repeated fields).

❽ Groups of fields can be repeated in the payload. The number of repetitions can be given by another field in the message (this example), a constant number, zero or one times (known as "optional group"), or derived from the remaining payload size (labeled as "repeated N times"). See also UBX repeated fields and UBX payload decoding.

## 3.8 UBX messages overview

<table><tr><td>Message</td><td>Class/ID</td><td>Description (Type)</td></tr><tr><td colspan="3">UBX-ACK – Acknowledgement and negative acknowledgement messages</td></tr><tr><td>UBX-ACK-ACK</td><td>0x05 0x01</td><td>• Message acknowledged (Output)</td></tr><tr><td>UBX-ACK-NAK</td><td>0x05 0x00</td><td>• Message not acknowledged (Output)</td></tr><tr><td colspan="3">UBX-CFG – Configuration and command messages</td></tr><tr><td>UBX-CFG-CFG</td><td>0x06 0x09</td><td>• Clear, save and load configurations (Command)</td></tr><tr><td>UBX-CFG-RST</td><td>0x06 0x04</td><td>• Reset receiver / Clear backup data structures (Command)</td></tr><tr><td>UBX-CFG-VALDEL</td><td>0x06 0x8c</td><td>• Delete configuration item values (Set)• Delete configuration item values (with transaction) (Set)</td></tr><tr><td>UBX-CFG-VALGET</td><td>0x06 0x8b</td><td>• Get configuration items (Poll request)• Configuration items (Polled)</td></tr><tr><td>UBX-CFG-VALSET</td><td>0x06 0x8a</td><td>• Set configuration item values (Set)• Set configuration item values (with transaction) (Set)</td></tr><tr><td colspan="3">UBX-INF – Information messages</td></tr><tr><td>UBX-INF-DEBUG</td><td>0x04 0x04</td><td>• ASCII output with debug contents (Output)</td></tr><tr><td>UBX-INF-ERROR</td><td>0x04 0x00</td><td>• ASCII output with error contents (Output)</td></tr><tr><td>UBX-INF-NOTICE</td><td>0x04 0x02</td><td>• ASCII output with informational contents (Output)</td></tr><tr><td>UBX-INF-TEST</td><td>0x04 0x03</td><td>• ASCII output with test contents (Output)</td></tr><tr><td>UBX-INF-WARNING</td><td>0x04 0x01</td><td>• ASCII output with warning contents (Output)</td></tr><tr><td colspan="3">UBX-MGA – GNSS assistance (A-GNSS) messages</td></tr><tr><td>UBX-MGA-ACK</td><td>0x13 0x60</td><td>• Multiple GNSS acknowledge message (Output)</td></tr><tr><td>UBX-MGA-ANO</td><td>0x13 0x20</td><td>• Multiple GNSS AssistNow Offline assistance (Input)</td></tr><tr><td>UBX-MGA-BDS</td><td>0x13 0x03</td><td>• BeiDou ephemeris assistance for satellites svld 1..37 (Input)• BeiDou almanac assistance (Input)• BeiDou health assistance (Input)• BeiDou UTC assistance (Input)• BeiDou ionosphere assistance (Input)</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 47 of 181

<!-- page 48 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Message</td><td>Class/ID</td><td>Description (Type)</td></tr><tr><td>UBX-MGA-DBD</td><td>0x13 0x80</td><td>Poll the navigation database (Poll request)Navigation database dump entry (Input/output)</td></tr><tr><td>UBX-MGA-FLASH</td><td>0x13 0x21</td><td>Transfer MGA-ANO data block to flash (Input)Finish flashing MGA-ANO data (Input)Acknowledge last FLASH-DATA or -STOP (Output)</td></tr><tr><td>UBX-MGA-GAL</td><td>0x13 0x02</td><td>Galileo ephemeris assistance (Input)Galileo almanac assistance (Input)Galileo GPS time offset assistance (Input)Galileo UTC assistance (Input)</td></tr><tr><td>UBX-MGA-GPS</td><td>0x13 0x00</td><td>GPS ephemeris assistance (Input)GPS almanac assistance (Input)GPS health assistance (Input)GPS UTC assistance (Input)GPS ionosphere assistance (Input)</td></tr><tr><td>UBX-MGA-INI</td><td>0x13 0x40</td><td>Initial position assistance (Input)Initial time assistance (Input)Initial clock drift assistance (Input)Initial frequency assistance (Input)Earth orientation parameters assistance (Input)</td></tr><tr><td>UBX-MGA-QZSS</td><td>0x13 0x05</td><td>QZSS ephemeris assistance (Input)QZSS almanac assistance (Input)QZSS health assistance (Input)</td></tr><tr><td colspan="3">UBX-MON - Monitoring messages</td></tr><tr><td>UBX-MON-COMMS</td><td>0x0a 0x36</td><td>Communication port information (Periodic/polled)</td></tr><tr><td>UBX-MON-GNSS</td><td>0x0a 0x28</td><td>Information message major GNSS selection (Polled)</td></tr><tr><td>UBX-MON-HW3</td><td>0x0a 0x37</td><td>I/O pin status (Periodic/polled)</td></tr><tr><td>UBX-MON-PATCH</td><td>0x0a 0x27</td><td>Installed patches (Polled)</td></tr><tr><td>UBX-MON-RCVRSTAT</td><td>0x0a 0x40</td><td>Status of the configurable receiver features (Polled)</td></tr><tr><td>UBX-MON-RF</td><td>0x0a 0x38</td><td>RF information (Periodic/polled)</td></tr><tr><td>UBX-MON-RXR</td><td>0x0a 0x21</td><td>Receiver status information (Output)</td></tr><tr><td>UBX-MON-SPAN</td><td>0x0a 0x31</td><td>Signal characteristics (Periodic/polled)</td></tr><tr><td>UBX-MON-VER</td><td>0x0a 0x04</td><td>Poll receiver and software version (Poll request)Receiver and software version (Polled)</td></tr><tr><td colspan="3">UBX-NAV - Navigation solution messages</td></tr><tr><td>UBX-NAV-AOPSTATUS</td><td>0x01 0x60</td><td>AssistNow Autonomous status (Periodic/polled)</td></tr><tr><td>UBX-NAV-CLOCK</td><td>0x01 0x22</td><td>Clock solution (Periodic/polled)</td></tr><tr><td>UBX-NAV-COV</td><td>0x01 0x36</td><td>Covariance matrices (Periodic/polled)</td></tr><tr><td>UBX-NAV-DOP</td><td>0x01 0x04</td><td>Dilution of precision (Periodic/polled)</td></tr><tr><td>UBX-NAV-EOE</td><td>0x01 0x61</td><td>End of epoch (Periodic)</td></tr><tr><td>UBX-NAV-ODO</td><td>0x01 0x09</td><td>Odometer solution (Periodic/polled)</td></tr><tr><td>UBX-NAV-ORB</td><td>0x01 0x34</td><td>GNSS orbit database info (Periodic/polled)</td></tr><tr><td>UBX-NAV-PL</td><td>0x01 0x62</td><td>Protection level information (Periodic)</td></tr><tr><td>UBX-NAV-POSECEF</td><td>0x01 0x01</td><td>Position solution in ECEF (Periodic/polled)</td></tr><tr><td>UBX-NAV-POSLLH</td><td>0x01 0x02</td><td>Geodetic position solution (Periodic/polled)</td></tr><tr><td>UBX-NAV-PVT</td><td>0x01 0x07</td><td>Navigation position velocity time solution (Periodic/polled)</td></tr><tr><td>UBX-NAV-RESETODO</td><td>0x01 0x10</td><td>Reset odometer (Command)</td></tr><tr><td>UBX-NAV-SAT</td><td>0x01 0x35</td><td>Satellite information (Periodic/polled)</td></tr><tr><td>UBX-NAV-SBAS</td><td>0x01 0x32</td><td>SBAS status data (Periodic/polled)</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 48 of 181

<!-- page 49 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Message</td><td>Class/ID</td><td>Description (Type)</td></tr><tr><td>UBX-NAV-SIG</td><td>0x01 0x43</td><td>Signal information (Periodic/polled)</td></tr><tr><td>UBX-NAV-SLAS</td><td>0x01 0x42</td><td>QZSS L1S SLAS status data (Periodic/polled)</td></tr><tr><td>UBX-NAV-STATUS</td><td>0x01 0x03</td><td>Receiver navigation status (Periodic/polled)</td></tr><tr><td>UBX-NAV-TIMEBDS</td><td>0x01 0x24</td><td>BeiDou time solution (Periodic/polled)</td></tr><tr><td>UBX-NAV-TIMEGAL</td><td>0x01 0x25</td><td>Galileo time solution (Periodic/polled)</td></tr><tr><td>UBX-NAV-TIMEGPS</td><td>0x01 0x20</td><td>GPS time solution (Periodic/polled)</td></tr><tr><td>UBX-NAV-TIMELS</td><td>0x01 0x26</td><td>Leap second event information (Periodic/polled)</td></tr><tr><td>UBX-NAV-TIMENAVIC</td><td>0x01 0x63</td><td>NavIC time solution (Periodic/polled)</td></tr><tr><td>UBX-NAV-TIMEQZSS</td><td>0x01 0x27</td><td>QZSS time solution (Periodic/polled)</td></tr><tr><td>UBX-NAV-TIMEUTC</td><td>0x01 0x21</td><td>UTC time solution (Periodic/polled)</td></tr><tr><td>UBX-NAV-VELECEF</td><td>0x01 0x11</td><td>Velocity solution in ECEF (Periodic/polled)</td></tr><tr><td>UBX-NAV-VELNED</td><td>0x01 0x12</td><td>Velocity solution in NED frame (Periodic/polled)</td></tr><tr><td colspan="3">UBX-RXM - Receiver manager messages</td></tr><tr><td>UBX-RXM-MEASX</td><td>0x02 0x14</td><td>Satellite measurements for RRLP (Periodic/polled)</td></tr><tr><td>UBX-RXM-PMREQ</td><td>0x02 0x41</td><td>Power management request (Command)</td></tr><tr><td>UBX-RXM-RLM</td><td>0x02 0x59</td><td>Galileo SAR short-RLM report (Output)Galileo SAR long-RLM report (Output)</td></tr><tr><td>UBX-RXM-SFRBX</td><td>0x02 0x13</td><td>Broadcast navigation data subframe (Output)</td></tr><tr><td colspan="3">UBX-SEC - Security messages</td></tr><tr><td>UBX-SEC-SIG</td><td>0x27 0x09</td><td>Signal security information (Periodic/polled)</td></tr><tr><td>UBX-SEC-SIGLOG</td><td>0x27 0x10</td><td>Signal security log (Periodic/polled)</td></tr><tr><td>UBX-SEC-UNIQID</td><td>0x27 0x03</td><td>Unique chip ID (Output)</td></tr><tr><td colspan="3">UBX-TIM - Timing messages</td></tr><tr><td>UBX-TIM-TM2</td><td>0x0d 0x03</td><td>Time mark data (Periodic/polled)</td></tr><tr><td>UBX-TIM-TP</td><td>0x0d 0x01</td><td>Time pulse time data (Periodic/polled)</td></tr><tr><td>UBX-TIM-VRFY</td><td>0x0d 0x06</td><td>Sourced time verification (Periodic/polled)</td></tr><tr><td colspan="3">UBX-UPD - Firmware update messages</td></tr><tr><td>UBX-UPD-SOS</td><td>0x09 0x14</td><td>Poll backup restore status (Poll request)Create backup in flash (Command)Clear backup in flash (Command)Backup creation acknowledge (Output)System restored from backup (Output)</td></tr></table>

## 3.9 UBX-ACK (0x05)

The messages in the UBX-ACK class are used to indicate acknowledgement or rejection (i.e. negative acknowledgement) of input messages, such as UBX-CFG messages.

## 3.9.1 UBX-ACK-ACK (0x05 0x01)

## 3.9.1.1 Message acknowledged

| Message | UBX-ACK-ACK Message acknowledged |
| --- | --- |
| Type | Output |
| Comment | Outputuponprocessingofaninputmessage.AUBX-ACK-ACKissentassoonaspossiblebutatleastwithin one second. |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 49 of 181

<!-- page 50 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x05</td><td>0x01</td><td>2</td><td></td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="9">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td colspan="3">Description</td></tr><tr><td>0</td><td>U1</td><td>clsID</td><td></td><td>-</td><td>-</td><td colspan="3">Class ID of the Acknowledged Message</td></tr><tr><td>1</td><td>U1</td><td>msgID</td><td></td><td>-</td><td>-</td><td colspan="3">Message ID of the Acknowledged Message</td></tr></table>

## 3.9.2 UBX-ACK-NAK (0x05 0x00)

3.9.2.1 Message not acknowledged

<table><tr><td>Message</td><td colspan="7">UBX-ACK-NAKMessage not acknowledged</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">Output upon processing of an input message. A UBX-ACK-NAK is sent as soon as possible but at least within one second.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x05</td><td>0x00</td><td>2</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>clsID</td><td></td><td>-</td><td>-</td><td colspan="2">Class ID of the Not-Acknowledged Message</td></tr><tr><td>1</td><td>U1</td><td>msgID</td><td></td><td>-</td><td>-</td><td colspan="2">Message ID of the Not-Acknowledged Message</td></tr></table>

## 3.10 UBX-CFG (0x06)

The messages in the UBX-CFG class are used to configure the receiver and poll current configuration values as well as for sending commands to the receiver. Unless stated otherwise, any message in this class sent to the receiver is either acknowledged (by a UBX-ACK-ACK message) if processed successfully or rejected (with a UBX-ACK-NAK message) if processed unsuccessfully.

## 3.10.1 UBX-CFG-CFG (0x06 0x09)

## 3.10.1.1 Clear, save and load configurations

| Message | UBX-CFG-CFGClear, save and load configurations |
| --- | --- |
| Type | Command |
| Comment | SeeReceiver configurationfor a detailed description on how receiver configuration should be used. The behavior of this message has changed for protocol versions greater than 23.01. Use UBX-CFG-VALSET and UBX-CFG-VALDEL with the appropriate layers instead. These new messages support selective saving and clearing to retain the behavior removed from this message. The three masks which were used to clear, save and load a subsection of configuration have lost their meaning. It is no longer possible to save or clear a subsection of the configuration using this message. The behavior of the masks is now:if any bit is set in the clearMask: all configuration in the selected non-volatile memory is deletedif any bit is set in the saveMask: all current configuration is stored (copied) to the selected layersif any bit is set in the loadMask: The current configuration is discarded and rebuilt from all the lower layersNote that commands can be combined. The sequence of execution is clear, save, then load. The receiver replies with a single UBX-ACK-ACK or UBX-ACK-NAK. A UBX-ACK-ACK indicates that all operations were successful. A UBX-ACK-NAK indicates that at least one of the configured operations was unsuccessful. It is recommended to send individual commands for a more comprehensive monitoring of the success or not of the individual operations. |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 50 of 181

<!-- page 51 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

☞ Old functionality of this message is not available in protocol versions greater than 23.01. Use UBX-CFG-VALSET, UBX-CFG-VALGET, UBX-CFG-VALDEL instead.

<table><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x06</td><td>0x09</td><td colspan="2">12 + [0,1]</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>X4</td><td colspan="2">clearMask</td><td>-</td><td>-</td><td colspan="2">Mask for configuration to clear</td></tr><tr><td>bits 31...0</td><td>U:32</td><td colspan="2">clearAll</td><td>-</td><td>-</td><td colspan="2">Clear all saved configuration from the selected non-volatile memory if any bit is set</td></tr><tr><td>4</td><td>X4</td><td colspan="2">saveMask</td><td>-</td><td>-</td><td colspan="2">Mask for configuration to save</td></tr><tr><td>bits 31...0</td><td>U:32</td><td colspan="2">saveAll</td><td>-</td><td>-</td><td colspan="2">Save all current configuration to the selected non-volatile memory if any bit is set</td></tr><tr><td>8</td><td>X4</td><td colspan="2">loadMask</td><td>-</td><td>-</td><td colspan="2">Mask for configuration to load</td></tr><tr><td>bits 31...0</td><td>U:32</td><td colspan="2">loadAll</td><td>-</td><td>-</td><td colspan="2">Discard current configuration and rebuilt it from lower non-volatile memory layers if any bit is set</td></tr><tr><td colspan="8">Start of optional group</td></tr><tr><td>12</td><td>X1</td><td colspan="2">deviceMask</td><td>-</td><td>-</td><td colspan="2">Mask which selects the memory devices for saving and/or clearing operationNote that if a deviceMask is not provided, the receiver defaults the operation requested to battery-backed RAM (BBR) and Flash (if available)</td></tr><tr><td>bit 0</td><td>U:1</td><td colspan="2">devBBR</td><td>-</td><td>-</td><td colspan="2">Battery-backed RAM</td></tr><tr><td>bit 1</td><td>U:1</td><td colspan="2">devFlash</td><td>-</td><td>-</td><td colspan="2">Flash</td></tr><tr><td>bit 2</td><td>U:1</td><td colspan="2">devEEPROM</td><td>-</td><td>-</td><td colspan="2">EEPROM (only supported for protocol versions less than 14.00)</td></tr><tr><td>bit 4</td><td>U:1</td><td colspan="2">devSpiFlash</td><td>-</td><td>-</td><td colspan="2">SPI Flash (only supported for protocol versions less than 14.00)</td></tr></table>

End of optional group

## 3.10.2 UBX-CFG-RST (0x06 0x04)

## 3.10.2.1 Reset receiver / Clear backup data structures

<table><tr><td>Message</td><td colspan="7">UBX-CFG-RSTReset receiver / Clear backup data structures</td></tr><tr><td>Type</td><td colspan="7">Command</td></tr><tr><td>Comment</td><td colspan="7">Do not expect this message to be acknowledged by the receiver.Newer FW version will not acknowledge this message at all.Older FW version will acknowledge this message but the acknowledge may not be sent completely before the receiver is reset.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x06</td><td>0x04</td><td>4</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>X2</td><td>navBbrMask</td><td></td><td>-</td><td>-</td><td colspan="2">BBR sections to clear. The following special sets apply:0x0000 Hot start0x0001 Warm start0xFFFF Cold start</td></tr><tr><td>bit 0</td><td>U:1</td><td>eph</td><td></td><td>-</td><td>-</td><td colspan="2">Ephemeris</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 51 of 181

<!-- page 52 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| bit 1 | U:1 | alm | - | - | Almanac |
| --- | --- | --- | --- | --- | --- |
| bit 2 | U:1 | health | - | - | Health |
| bit 3 | U:1 | klob | - | - | Klobuchar parameters |
| bit 4 | U:1 | pos | - | - | Position |
| bit 5 | U:1 | clkd | - | - | Clock drift |
| bit 6 | U:1 | osc | - | - | Oscillator parameter |
| bit 7 | U:1 | utc | - | - | UTC correction + GPS leap seconds parameters |
| bit 8 | U:1 | rtc | - | - | RTC |
| bit 11 | U:1 | sfdr | - | - | SFDR Parameters (only available on the ADR/UDR/HPS product variant) and weak signal compensation estimates |
| bit 12 | U:1 | vmon | - | - | SFDR Vehicle Monitoring Parameter (only available on the ADR/UDR/HPS product variant) |
| bit 13 | U:1 | tct | - | - | TCT Parameters (only available on the ADR/UDR/HPS product variant) |
| bit 15 | U:1 | aop | - | - | Autonomous orbit parameters |
| 2 | U1 | resetMode | - | - | Reset Type0x00 = Hardware reset (watchdog) immediately0x01 = Controlled software reset0x02 = Controlled software reset (GNSS only)0x04 = Hardware reset (watchdog) after shutdown0x08 = Controlled GNSS stop0x09 = Controlled GNSS start |
| 3 | U1 | reserved0 | - | - | Reserved |

## 3.10.3 UBX-CFG-VALDEL (0x06 0x8c)

## 3.10.3.1 Delete configuration item values

| Message | UBX-CFG-VALDELDelete configuration item values |
| --- | --- |
| Type | Set |
| Comment | Overview:This message can be used to delete saved configuration to effectively revert the item values to defaults.This message can delete saved configuration from the flash configuration layer and the BBR configuration layer. The changes will not be effective until these layers are loaded into the RAM layer.This message is limited to containing a maximum of 64 keys up for deletion; i.e. N is a maximum of 64.This message can be used multiple times and every time the result will be applied immediately. To send this message multiple times with the result being applied at the end, see version 1 ofUBX-CFG-VALDELthat supports transactions.This message does not check if the resulting configuration is valid.SeeReceiver configurationfor details.This message returns a UBX-ACK-NAK and no configuration is applied:if any key is unknown to the receiver FWif the layer's bitfield does not specify a layer to delete a value from.Notes:If a key is sent multiple times within the same message, the value is effectively deleted only once.Attempting to delete items that have not been set before, or that have already been deleted, is considered a valid request. |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 52 of 181

<!-- page 53 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

The provided keys can be complete key values (group and item specifiers) or wild-card specifications. A complete key value constitutes a deletion request for one key-value pair. A key value with a valid group specifier and 0xffff in the item part of the key value (bits 0-15) constitutes a deletion request for all items in the specified group. A key with a value of 0xfff in the group part of the key value (bits 16-27) is a deletion request for all items known to the receiver in all groups.

<table><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x06</td><td>0x8c</td><td>4 + [0..n]·4</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>1</td><td>X1</td><td>layers</td><td></td><td>-</td><td>-</td><td>The layers where the configuration should be deleted from</td><td></td></tr><tr><td>bit 1</td><td>U:1</td><td>bbr</td><td></td><td>-</td><td>-</td><td>Delete configuration from the BBR layer</td><td></td></tr><tr><td>bit 2</td><td>U:1</td><td>flash</td><td></td><td>-</td><td>-</td><td>Delete configuration from the Flash layer</td><td></td></tr><tr><td>2</td><td>U1[2]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td colspan="8">Start of repeated group (N times)</td></tr><tr><td>4 + n·4</td><td>U4</td><td>keys</td><td></td><td>-</td><td>-</td><td>Configuration key IDs of the configuration items to be deleted</td><td></td></tr></table>

End of repeated group (N times)

## 3.10.3.2 Delete configuration item values (with transaction)

| Message | UBX-CFG-VALDELDelete configuration item values (with transaction) |
| --- | --- |
| Type | Set |
| Comment | Overview:This message can be used to delete saved configuration to effectively revert them to defaults.This message can delete saved configuration from the flash configuration layer and the BBR configuration layer. The changes will not be effective until these layers are loaded into the RAM layer.This message is limited to containing a maximum of 64 keys up for deletion; i.e. N is a maximum of 64.This message can be used multiple times with the result being managed within a transaction.This message does not check if the resulting configuration is valid.SeeReceiver configurationfor details.See version 0 ofUBX-CFG-VALDELfor simplified version of this message.This message returns a UBX-ACK-NAK, cancels any started transaction, and no configuration is applied:if any key within a transaction is unknown to the receiver FWif an invalid transaction state transition is requestedif the layer's bitfield changes within a transactionif the layer's bitfield does not specify a layer to delete a value from.Notes:Any request for another UBX-CFG- message type (including UBX-CFG-VALSET and UBX-CFG-VALGET) will cancel any started transaction, and no configuration is applied.This message can be sent with no keys to delete for the purposes of managing the transaction state transition.If a key is sent multiple times within the same message or within the same transaction, the value is effectively deleted only once.Attempting to delete items that have not been set before, or that have already been deleted, is considered a valid request.The provided keys can be complete key values (group and item specifiers) or wild-card specifications. A complete key value constitutes a deletion request for one key-value pair. A key value with a valid group specifier and 0xffff in the item part of the key value (bits 0-15) constitutes a deletion request for all items in the specified group. A key with a value of 0xffff in the group part of the key value (bits 16-27) is a deletion request for all items known to the receiver in all groups. |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 53 of 181

<!-- page 54 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x06</td><td>0x8c</td><td colspan="2">4 + [0..n]·4</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x01 for this version)</td></tr><tr><td>1</td><td>X1</td><td colspan="2">layers</td><td>-</td><td>-</td><td colspan="2">The layers where the configuration should be deleted from</td></tr><tr><td>bit 1</td><td>U:1</td><td colspan="2">bbr</td><td>-</td><td>-</td><td colspan="2">Delete configuration from the BBR layer</td></tr><tr><td>bit 2</td><td>U:1</td><td colspan="2">flash</td><td>-</td><td>-</td><td colspan="2">Delete configuration from the Flash layer</td></tr><tr><td>2</td><td>X1</td><td colspan="2">transaction</td><td>-</td><td>-</td><td colspan="2">Transaction action to be applied:</td></tr><tr><td>bits 1...0</td><td>U:2</td><td colspan="2">action</td><td>-</td><td>-</td><td colspan="2">Transaction action to be applied:0 = Transactionless UBX-CFG-VALDEL: In the next UBX-CFG-VALDEL, it can be either 0 or 1. If a transaction has not yet been started, the incoming configuration is applied. If a transaction has already been started, cancels any started transaction and the incoming configuration is applied.1 = (Re)Start deletion transaction: In the next UBX-CFG-VALDEL, it can be either 0, 1, 2 or 3. If a transaction has not yet been started, a transaction will be started. If a transaction has already been started, restarts the transaction, effectively removing all previous non-applied UBX-CFG-VALDEL messages.2 = Deletion transaction ongoing: In the next UBX-CFG-VALDEL, it can be either 0, 1, 2 or 3.3 = Apply and end a deletion transaction: In the next UBX-CFG-VALDEL, it can be either 0 or 1.</td></tr></table>

<table><tr><td>3</td><td>U1</td><td>reserved0</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td colspan="6">Start of repeated group (N times)</td></tr><tr><td>4 + n·4</td><td>U4</td><td>keys</td><td>-</td><td>-</td><td>Configuration key IDs of the configuration items to be deleted</td></tr><tr><td colspan="6">End of repeated group (N times)</td></tr></table>

## 3.10.4 UBX-CFG-VALGET (0x06 0x8b)

## 3.10.4.1 Get configuration items

| Message | UBX-CFG-VALGETGet configuration items |
| --- | --- |
| Type | Poll request |
| Comment | Overview:This message is used to get configuration values by providing a list of configuration key IDs, which identify the configuration items to retrieve.This message can specify the configuration layer where the values of the specified configuration items are retrieved from.This message is limited to containing a maximum of 64 key IDs.SeeReceiver configurationfor details. |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 54 of 181

<!-- page 55 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

This message returns a UBX-ACK-NAK:

• if any key is unknown to the receiver FW

• if the layer field specifies an invalid layer to get the value from

• if the keys array specifies more than 64 key IDs.

Notes:

• If a value is requested multiple times within the same poll request, then the reply will contain it multiple times.

The provided keys can be complete key values (group and item specifiers) or wild-card specifications. A complete key value will constitute a request for one key-value pair. A key value that has a valid group specifier and 0xffff in the item part of the key value (bits 0-15) constitutes a request for all items in the specified group. A key with a value of 0xfff in the group part of the key value (bits 16-27) is a request for all items known to the receiver in all groups.

The response message is limited to containing a maximum of 64 key-value pairs. If there are wild-card specifications then there may be more than 64 possible responses. In order to handle this, the 'position' field can specify that the response message should skip this number of key-value pairs before it starts constructing the message. This allows a large set of values to be retrieved 64 at a time. If the response contains less than 64 key-value pairs then all values have been reported, otherwise there may be more to read.

It is not possible to retrieve configuration values for the same configuration item from multiple configuration layers. Separate poll requests must be made for each desired layer.

<table><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x06</td><td>0x8b</td><td>4 + [0..n]·4</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>1</td><td>U1</td><td>layer</td><td></td><td>-</td><td>-</td><td>The layer from which the configuration items should be retrieved:0 - RAM layer1 - BBR layer2 - Flash layer7 - Default layer</td><td></td></tr><tr><td>2</td><td>U2</td><td>position</td><td></td><td>-</td><td>-</td><td>Skip this many key values before constructing output message</td><td></td></tr><tr><td colspan="8">Start of repeated group (N times)</td></tr><tr><td>4 + n·4</td><td>U4</td><td>keys</td><td></td><td>-</td><td>-</td><td>Configuration key IDs of the configuration items to be retrieved</td><td></td></tr><tr><td colspan="8">End of repeated group (N times)</td></tr></table>

## 3.10.4.2 Configuration items

<table><tr><td>Message</td><td colspan="7">UBX-CFG-VALGETConfiguration items</td></tr><tr><td>Type</td><td colspan="7">Polled</td></tr><tr><td>Comment</td><td colspan="7">This message is output by the receiver to return requested configuration data (key and value pairs).See Receiver configuration for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x06</td><td>0x8b</td><td>4 + [0..n]</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x01 for this version)</td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 55 of 181

<!-- page 56 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>1</td><td>U1</td><td>layer</td><td>-</td><td>-</td><td>The layer from which the configuration item was retrieved:0 - RAM layer1 - BBR2 - Flash7 - Default</td></tr><tr><td>2</td><td>U2</td><td>position</td><td>-</td><td>-</td><td>Number of configuration items skipped in the result set before constructing this message (mirrors the equivalent field in the request message)</td></tr><tr><td colspan="6">Start of repeated group (N times)</td></tr><tr><td>4 + n</td><td>U1</td><td>cfgData</td><td>-</td><td>-</td><td>Configuration data (key and value pairs)</td></tr><tr><td colspan="6">End of repeated group (N times)</td></tr></table>

## 3.10.5 UBX-CFG-VALSET (0x06 0x8a)

## 3.10.5.1 Set configuration item values

| Message | UBX-CFG-VALSETSet configuration item values |
| --- | --- |
| Type | Set |
| Comment | Overview:This message is used to set a configuration by providing configuration data (a list of key and value pairs), which identify the configuration items to change, and their new values.This message is limited to containing a maximum of 64 key-value pairs.This message can be used multiple times and every time the result will be applied immediately. To send this message multiple times with the result being applied at the end, see version 1 ofUBX-CFG-VALSETthat supports transactions.SeeReceiver configurationfor details.This message returns a UBX-ACK-NAK and no configuration is applied:if any key is unknown to the receiver FWif the layer's bitfield does not specify a layer to save a value toif the requested configuration is not valid. The validity of a configuration is checked only if the message requests to apply the configuration to the RAM configuration layer.Notes: |

• If a key is sent multiple times within the same message, then the value eventually being applied is the last sent.

<table><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x06</td><td>0x8a</td><td colspan="2">4 + [0..n]</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td rowspan="4">1</td><td>X1</td><td colspan="2">layers</td><td>-</td><td>-</td><td colspan="2">The layers where the configuration should be applied</td></tr><tr><td>bit 0</td><td>U:1</td><td>ram</td><td>-</td><td>-</td><td colspan="2">Update configuration in the RAM layer</td></tr><tr><td>bit 1</td><td>U:1</td><td>bbr</td><td>-</td><td>-</td><td colspan="2">Update configuration in the BBR layer</td></tr><tr><td>bit 2</td><td>U:1</td><td>flash</td><td>-</td><td>-</td><td colspan="2">Update configuration in the Flash layer</td></tr><tr><td>2</td><td>U1[2]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td colspan="8">Start of repeated group (N times)</td></tr><tr><td>4 + n</td><td>U1</td><td colspan="2">cfgData</td><td>-</td><td>-</td><td colspan="2">Configuration data (key and value pairs)</td></tr><tr><td colspan="8">End of repeated group (N times)</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 56 of 181

<!-- page 57 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.10.5.2 Set configuration item values (with transaction)

| Message | UBX-CFG-VALSETSet configuration item values (with transaction) |
| --- | --- |
| Type | Set |
| Comment | Overview:This message is used to set a configuration by providing configuration data (a list of key and value pairs), which identify the configuration items to change, and their new values.This message is limited to containing a maximum of 64 key-value pairs.This message can be used multiple times with the result being managed within a transaction. Within a transaction there is no limit on the number key-value pairs; a transaction is effectively limited to the number of known keys.SeeReceiver configurationfor details.See version 0 ofUBX-CFG-VALSETfor simplified version of this message.This message returns a UBX-ACK-NAK, cancels any started transaction, and no configuration is applied:if any key within a transaction is unknown to the receiver FWif an invalid transaction state transition is requestedif the layer's bitfield changes within a transactionif the layer's bitfield does not specify a layer to save a value toThis message returns a UBX-ACK-NAK, and no configuration is applied:if the requested configuration is not valid. While in a transaction context, only the last message that requests to apply the transaction returns a UBX-ACK-NAK. The validity of a configuration is checked only if the message requests to apply the configuration to the RAM configuration layer. This also applies to a transactionless request.Notes:Any request for another UBX-CFG-message type (including UBX-CFG-VALDEL and UBX-CFG-VALGET) will cancel any started transaction, and no configuration is applied.This message can be sent with no key/values to set for the purposes of managing the transaction state transition.If a key is sent multiple times within the same message or within the same transaction, then the value eventually being applied is the last sent. |

<table><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x06</td><td>0x8a</td><td colspan="2">4 + [0..n]</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x01 for this version)</td></tr><tr><td>1</td><td>X1</td><td colspan="2">layers</td><td>-</td><td>-</td><td colspan="2">The layers where the configuration should be applied</td></tr><tr><td>bit 0</td><td>U:1</td><td colspan="2">ram</td><td>-</td><td>-</td><td colspan="2">Update configuration in the RAM layer</td></tr><tr><td>bit 1</td><td>U:1</td><td colspan="2">bbr</td><td>-</td><td>-</td><td colspan="2">Update configuration in the BBR layer</td></tr><tr><td>bit 2</td><td>U:1</td><td colspan="2">flash</td><td>-</td><td>-</td><td colspan="2">Update configuration in the Flash layer</td></tr><tr><td>2</td><td>U1</td><td colspan="2">transaction</td><td>-</td><td>-</td><td colspan="2">Transaction action to be applied</td></tr><tr><td>bits 1...0</td><td>U:2</td><td colspan="2">action</td><td>-</td><td>-</td><td colspan="2">Transaction action to be applied:0 = Transactionless UBX-CFG-VALSET: In the next UBX-CFG-VALSET, it can be either 0 or 1. If a transaction has not yet been started, the incoming configuration is applied (if valid). If a transaction has already been started, cancels any started transaction and the incoming configuration is applied (if valid).1 = (Re)Start set transaction: In the next UBX-CFG-VALSET, it can be either 0, 1, 2 or 3. If a transaction has not yet been started, a</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 57 of 181

<!-- page 58 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

transaction will be started. If a transaction has already been started, restarts the transaction, effectively removing all previous non-applied UBX-CFG-VALSET messages.

2 = Set transaction ongoing: In the next UBX-CFG-VALSET, it can be either 0, 1, 2 or 3.

3 = Apply and end a set transaction: In the next UBX-CFG-VALSET, it can be either 0 or 1.

<table><tr><td>3</td><td>U1</td><td>reserved0</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td colspan="6">Start of repeated group (N times)</td></tr><tr><td>4 + n</td><td>U1</td><td>cfgData</td><td>-</td><td>-</td><td>Configuration data (key and value pairs)</td></tr><tr><td colspan="6">End of repeated group (N times)</td></tr></table>

## 3.11 UBX-INF (0x04)

Messages in the UBX-INF class are used to output strings from the firmware or application code. All messages have an associated type to indicate the nature or priority of the message.

## 3.11.1 UBX-INF-DEBUG (0x04 0x04)

## 3.11.1.1 ASCII output with debug contents

<table><tr><td>Message</td><td colspan="7">UBX-INF-DEBUGASCII output with debug contents</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">This message has a variable length payload, representing an ASCII string.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x04</td><td>0x04</td><td>[0..n]</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td colspan="8">Start of repeated group (N times)</td></tr><tr><td>0 + n</td><td>CH</td><td>str</td><td></td><td>-</td><td>-</td><td>ASCII Character</td><td></td></tr><tr><td colspan="8">End of repeated group (N times)</td></tr></table>

## 3.11.2 UBX-INF-ERROR (0x04 0x00)

## 3.11.2.1 ASCII output with error contents

<table><tr><td>Message</td><td colspan="7">UBX-INF-ERRORASCII output with error contents</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">This message has a variable length payload, representing an ASCII string.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x04</td><td>0x00</td><td>[0..n]</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td colspan="8">Start of repeated group (N times)</td></tr><tr><td>0 + n</td><td>CH</td><td>str</td><td></td><td>-</td><td>-</td><td>ASCII Character</td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 58 of 181

<!-- page 59 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

End of repeated group (N times)

## 3.11.3 UBX-INF-NOTICE (0x04 0x02)

3.11.3.1 ASCII output with informational contents

<table><tr><td>Message</td><td colspan="7">UBX-INF-NOTICEASCII output with informational contents</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">This message has a variable length payload, representing an ASCII string.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x04</td><td>0x02</td><td>[0..n]</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td colspan="8">Start of repeated group (N times)</td></tr><tr><td>0 + n</td><td>CH</td><td>str</td><td></td><td>-</td><td>-</td><td>ASCII Character</td><td></td></tr><tr><td colspan="8">End of repeated group (N times)</td></tr></table>

## 3.11.4 UBX-INF-TEST (0x04 0x03)

## 3.11.4.1 ASCII output with test contents

<table><tr><td>Message</td><td colspan="7">UBX-INF-TESTASCII output with test contents</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">This message has a variable length payload, representing an ASCII string.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x04</td><td>0x03</td><td>[0..n]</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td colspan="8">Start of repeated group (N times)</td></tr><tr><td>0 + n</td><td>CH</td><td>str</td><td></td><td>-</td><td>-</td><td>ASCII Character</td><td></td></tr><tr><td colspan="8">End of repeated group (N times)</td></tr></table>

## 3.11.5 UBX-INF-WARNING (0x04 0x01)

## 3.11.5.1 ASCII output with warning contents

<table><tr><td>Message</td><td colspan="7">UBX-INF-WARNINGASCII output with warning contents</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">This message has a variable length payload, representing an ASCII string.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x04</td><td>0x01</td><td>[0..n]</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td colspan="8">Start of repeated group (N times)</td></tr><tr><td>0 + n</td><td>CH</td><td>str</td><td></td><td>-</td><td>-</td><td>ASCII Character</td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 59 of 181

<!-- page 60 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

End of repeated group (N times)

## 3.12 UBX-MGA (0x13)

The messages in the UBX-MGA class are used for sending GNSS assistance (A-GNSS, aiding) information to the receiver as well as backing up the navigation database from the receiver to a host.

## 3.12.1 UBX-MGA-ACK (0x13 0x60)

3.12.1.1 Multiple GNSS acknowledge message

<table><tr><td>Message</td><td colspan="7">UBX-MGA-ACK-DATA0Multiple GNSS acknowledge message</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">This message is sent by a u-blox receiver to acknowledge the receipt of an assistance message.Acknowledgments are enabled by setting the CFG-NAVSPG-ACKAIDING item.See section Flow control in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x60</td><td>8</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>type</td><td></td><td>-</td><td>-</td><td colspan="2">Type of acknowledgment:0 = The message was not used by the receiver (see infoCode field for an indication of why)1 = The message was accepted for use by the receiver (the infoCode field will be 0)</td></tr><tr><td>1</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1</td><td>infoCode</td><td></td><td>-</td><td>-</td><td colspan="2">Provides greater information on what the receiver chose to do with the message contents:0 = The receiver accepted the data1 = The receiver does not know the time so it cannot use the data (To resolve this a UBX-MGA-INI-TIME_UTC message should be supplied first)2 = The message version is not supported by the receiver3 = The message size does not match the message version4 = The message data could not be stored to the database5 = The receiver is not ready to use the message data6 = The message type is unknown</td></tr><tr><td>3</td><td>U1</td><td>msgId</td><td></td><td>-</td><td>-</td><td colspan="2">UBX message ID of the acknowledged message</td></tr><tr><td>4</td><td>U1[4]</td><td>msgPayload Start</td><td></td><td>-</td><td>-</td><td colspan="2">The first 4 bytes of the acknowledged message&#x27;s payload</td></tr></table>

## 3.12.2 UBX-MGA-ANO (0x13 0x20)

3.12.2.1 Multiple GNSS AssistNow Offline assistance

| Message | UBX-MGA-ANO MultipleGNSSAssistNowOfflineassistance |
| --- | --- |
| Type | Input |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 60 of 181

<!-- page 61 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Comment</td><td colspan="7">This message is created by the AssistNow Offline service to deliver AssistNow Offline assistance to the receiver.See AssistNow Offline section in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x20</td><td>76</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x00 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1</td><td colspan="2">svId</td><td>-</td><td>-</td><td colspan="2">Satellite identifier (see Satellite Numbering)</td></tr><tr><td>3</td><td>U1</td><td colspan="2">gnssId</td><td>-</td><td>-</td><td colspan="2">GNSS identifier (see Satellite Numbering)</td></tr><tr><td>4</td><td>U1</td><td colspan="2">year</td><td>-</td><td>-</td><td colspan="2">years since the year 2000</td></tr><tr><td>5</td><td>U1</td><td colspan="2">month</td><td>-</td><td>-</td><td colspan="2">month (1..12)</td></tr><tr><td>6</td><td>U1</td><td colspan="2">day</td><td>-</td><td>-</td><td colspan="2">day (1..31)</td></tr><tr><td>7</td><td>U1</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>8</td><td>U1[64]</td><td colspan="2">data</td><td>-</td><td>-</td><td colspan="2">assistance data</td></tr><tr><td>72</td><td>U1[4]</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

## 3.12.3 UBX-MGA-BDS (0x13 0x03)

3.12.3.1 BeiDou ephemeris assistance for satellites svId 1..37

<table><tr><td>Message</td><td colspan="7">UBX-MGA-BDS-EPHBeiDou ephemeris assistance for satellites svld 1..37</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of BeiDou D1/D2 ephemeris assistance to a receiver.See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x03</td><td>88</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x01 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1</td><td colspan="2">svId</td><td>-</td><td>-</td><td colspan="2">BeiDou satellite identifier (seeSatellite Numbering)</td></tr><tr><td>3</td><td>U1</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>U1</td><td colspan="2">SatH1</td><td>-</td><td>-</td><td colspan="2">Autonomous satellite Health flag</td></tr><tr><td>5</td><td>U1</td><td colspan="2">IODC</td><td>-</td><td>-</td><td colspan="2">Issue of Data, Clock</td></tr><tr><td>6</td><td>I2</td><td colspan="2">a2</td><td>2^-66</td><td>s/s^2</td><td colspan="2">Time polynomial coefficient 2</td></tr><tr><td>8</td><td>I4</td><td colspan="2">a1</td><td>2^-50</td><td>s/s</td><td colspan="2">Time polynomial coefficient 1</td></tr><tr><td>12</td><td>I4</td><td colspan="2">a0</td><td>2^-33</td><td>s</td><td colspan="2">Time polynomial coefficient 0</td></tr><tr><td>16</td><td>U4</td><td colspan="2">toc</td><td>2^3</td><td>s</td><td colspan="2">Clock data reference time</td></tr><tr><td>20</td><td>I2</td><td colspan="2">TGD1</td><td>0.1</td><td>ns</td><td colspan="2">Equipment Group Delay Differential</td></tr><tr><td>22</td><td>U1</td><td colspan="2">URAI</td><td>-</td><td>-</td><td colspan="2">User Range Accuracy Index</td></tr><tr><td>23</td><td>U1</td><td colspan="2">IODE</td><td>-</td><td>-</td><td colspan="2">Issue of Data, Ephemeris</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 61 of 181

<!-- page 62 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 24 | U4 | toe | 2^3 | s | Ephemeris reference time |
| --- | --- | --- | --- | --- | --- |
| 28 | U4 | sqrtA | 2^-19 | m^0.5 | Square root of semi-major axis |
| 32 | U4 | e | 2^-33 | - | Eccentricity |
| 36 | I4 | omega | 2^-31 | semi-circles | Argument of perigee |
| 40 | I2 | Deltan | 2^-43 | semi-circles/s | Mean motion difference from computed value |
| 42 | I2 | IDOT | 2^-43 | semi-circles/s | Rate of inclination angle |
| 44 | I4 | M0 | 2^-31 | semi-circles | Mean anomaly at reference time |
| 48 | I4 | Omega0 | 2^-31 | semi-circles | Longitude of ascending node of orbital of plane computed according to reference time |
| 52 | I4 | OmegaDot | 2^-43 | semi-circles/s | Rate of right ascension |
| 56 | I4 | i0 | 2^-31 | semi-circles | Inclination angle at reference time |
| 60 | I4 | Cuc | 2^-31 | radians | Amplitude of cosine harmonic correction term to the argument of latitude |
| 64 | I4 | Cus | 2^-31 | radians | Amplitude of sine harmonic correction term to the argument of latitude |
| 68 | I4 | Crc | 2^-6 | m | Amplitude of cosine harmonic correction term to the orbit radius |
| 72 | I4 | Crs | 2^-6 | m | Amplitude of sine harmonic correction term to the orbit radius |
| 76 | I4 | Cic | 2^-31 | radians | Amplitude of cosine harmonic correction term to the angle of inclination |
| 80 | I4 | Cis | 2^-31 | radians | Amplitude of sine harmonic correction term to the angle of inclination |
| 84 | U1[4] | reserved1 | - | - | Reserved |

3.12.3.2 BeiDou almanac assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-BDS-ALMBeiDou almanac assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of BeiDou almanac assistance to a receiver.See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x03</td><td>40</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>type</td><td></td><td>-</td><td>-</td><td>Message type (0x02 for this version)</td><td></td></tr><tr><td>1</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>2</td><td>U1</td><td>svId</td><td></td><td>-</td><td>-</td><td>BeiDou satellite identifier (seeSatellite Numbering)</td><td></td></tr><tr><td>3</td><td>U1</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td>4</td><td>U1</td><td>Wna</td><td></td><td>-</td><td>week</td><td>Almanac Week Number</td><td></td></tr><tr><td>5</td><td>U1</td><td>toa</td><td></td><td>2^12</td><td>s</td><td>Almanac reference time</td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 62 of 181

<!-- page 63 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 6 | I2 | deltaI | 2^-19 | semi-circles | Almanac correction of orbit reference inclination at reference time |
| --- | --- | --- | --- | --- | --- |
| 8 | U4 | sqrtA | 2^-11 | m^0.5 | Almanac square root of semi-major axis |
| 12 | U4 | e | 2^-21 | - | Almanac eccentricity |
| 16 | I4 | omega | 2^-23 | semi-circles | Almanac argument of perigee |
| 20 | I4 | M0 | 2^-23 | semi-circles | Almanac mean anomaly at reference time |
| 24 | I4 | Omega0 | 2^-23 | semi-circles | Almanac longitude of ascending node of orbit plane at computed according to reference time |
| 28 | I4 | omegaDot | 2^-38 | semi-circles/s | Almanac rate of right ascension |
| 32 | I2 | a0 | 2^-20 | s | Almanac satellite clock bias |
| 34 | I2 | a1 | 2^-38 | s/s | Almanac satellite clock rate |
| 36 | U1[4] | reserved1 | - | - | Reserved |

3.12.3.3 BeiDou health assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-BDS-HEALTHBeiDou health assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of BeiDou health assistance from D1/D2 ephemeris to a receiver.See section AssistNow online in the integration manual for details.This message allows the delivery of health assistance data for all satellites with svld 1 to 30.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x03</td><td>68</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x04 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1[2]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>U2[30]</td><td colspan="2">healthCode</td><td>-</td><td>-</td><td colspan="2">Each two-byte value represents a BeiDou SV (1-30).The 9 LSBs of each byte contain the 9 bit health code from subframe 5 pages 7,8 of the D1 message, and from subframe 5 pages 35,36 of the D2 message.</td></tr><tr><td>64</td><td>U1[4]</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

3.12.3.4 BeiDou UTC assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-BDS-UTCBeiDou UTC assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of BeiDou UTC assistance to a receiver.See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x03</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>type</td><td></td><td>-</td><td>-</td><td>Message type (0x05 for this type)</td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 63 of 181

<!-- page 64 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 1 | U1 | version | - | - | Message version (0x00 for this version) |
| --- | --- | --- | --- | --- | --- |
| 2 | U1[2] | reserved0 | - | - | Reserved |
| 4 | I4 | a0UTC | 2^-30 | s | BDT clock bias relative to UTC |
| 8 | I4 | a1UTC | 2^-50 | s/s | BDT clock rate relative to UTC |
| 12 | I1 | dtLS | - | s | Delta time due to leap seconds before the new leap second effective |
| 13 | U1 | reserved1 | - | - | Reserved |
| 14 | U1 | wnRec | - | week | BeiDou week number of reception of this UTC parameter set (8-bit truncated) |
| 15 | U1 | wnLSF | - | week | Week number of the new leap second |
| 16 | U1 | dN | - | day | Day number of the new leap second |
| 17 | I1 | dtLSF | - | s | Delta time due to leap seconds after the new leap second effective |
| 18 | U1[2] | reserved2 | - | - | Reserved |

3.12.3.5 BeiDou ionosphere assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-BDS-IONOBeiDou ionosphere assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of BeiDou ionospheric assistance to a receiver.See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x03</td><td>16</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x06 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1[2]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>I1</td><td colspan="2">alpha0</td><td>2^-30</td><td>s</td><td colspan="2">Ionospheric parameter alpha0</td></tr><tr><td>5</td><td>I1</td><td colspan="2">alpha1</td><td>2^-27</td><td>s/pi</td><td colspan="2">Ionospheric parameter alpha1</td></tr><tr><td>6</td><td>I1</td><td colspan="2">alpha2</td><td>2^-24</td><td>s/pi^2</td><td colspan="2">Ionospheric parameter alpha2</td></tr><tr><td>7</td><td>I1</td><td colspan="2">alpha3</td><td>2^-24</td><td>s/pi^3</td><td colspan="2">Ionospheric parameter alpha3</td></tr><tr><td>8</td><td>I1</td><td colspan="2">beta0</td><td>2^11</td><td>s</td><td colspan="2">Ionospheric parameter beta0</td></tr><tr><td>9</td><td>I1</td><td colspan="2">beta1</td><td>2^14</td><td>s/pi</td><td colspan="2">Ionospheric parameter beta1</td></tr><tr><td>10</td><td>I1</td><td colspan="2">beta2</td><td>2^16</td><td>s/pi^2</td><td colspan="2">Ionospheric parameter beta2</td></tr><tr><td>11</td><td>I1</td><td colspan="2">beta3</td><td>2^16</td><td>s/pi^3</td><td colspan="2">Ionospheric parameter beta3</td></tr><tr><td>12</td><td>U1[4]</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

## 3.12.4 UBX-MGA-DBD (0x13 0x80)

3.12.4.1 Poll the navigation database

| Message | UBX-MGA-DBD Pollthenavigationdatabase |
| --- | --- |
| Type | Pollrequest |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 64 of 181

<!-- page 65 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Comment</td><td colspan="6">Poll the whole navigation data base. The receiver will send all available data from its internal database. The receiver will indicate the finish of the transmission with a UBX-MGA-ACK. The msgPayloadStart field of the UBX-MGA-ACK message will contain a U4 representing the number of UBX-MGA-DBD-DATA* messages sent.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x80</td><td>0</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td>Payload</td><td colspan="6">This message has no payload.</td></tr></table>

## 3.12.4.2 Navigation database dump entry

<table><tr><td>Message</td><td colspan="7">UBX-MGA-DBDNavigation database dump entry</td></tr><tr><td>Type</td><td colspan="7">Input/output</td></tr><tr><td>Comment</td><td colspan="7">Navigation database entry. The data fields are firmware-specific. Transmission of this type of message will be acknowledged by UBX-MGA-ACK messages, if acknowledgment has been enabled.See section AssistNow online in the integration manual for details.The maximum payload size for firmware 2.01 onwards is 164 bytes (which makes the maximum message size 172 bytes).UBX-MGA-DBD messages are only intended to be sent back to the same receiver that generated them.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x80</td><td>12 + [0..n]</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1[12]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td colspan="8">Start of repeated group (N times)</td></tr><tr><td>12 + n</td><td>U1</td><td>data</td><td></td><td>-</td><td>-</td><td>firmware-specific data</td><td></td></tr><tr><td colspan="8">End of repeated group (N times)</td></tr></table>

## 3.12.5 UBX-MGA-FLASH (0x13 0x21)

## 3.12.5.1 Transfer MGA-ANO data block to flash

<table><tr><td>Message</td><td colspan="7">UBX-MGA-FLASH-DATATransfer MGA-ANO data block to flash</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message is used to transfer a block of MGA-ANO data from host to the receiver. Upon reception of this message, the receiver will write the payload data to its internal non-volatile memory (flash). Also, on reception of the first MGA-FLASH-DATA message, the receiver will erase the flash allocated to storing any existing MGA-ANO data. The payload can be up to 512 bytes. Payloads larger than this would exceed the receiver&#x27;s internal buffering capabilities. The receiver will ACK/NACK this message using the message alternatives given below. The host shall wait for an acknowledge message before sending the next data block. See Flash-based AssistNow Offline for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x21</td><td>6 + size</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>type</td><td></td><td>-</td><td>-</td><td>Message type (0x01 for this type)</td><td></td></tr><tr><td>1</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>2</td><td>U2</td><td>sequence</td><td></td><td>-</td><td>-</td><td colspan="2">Message sequence number, starting at 0 and incrementing by 1 for each MGA-FLASH-DATA message sent.</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 65 of 181

<!-- page 66 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>4</td><td>U2</td><td>size</td><td>-</td><td>-</td><td>Payload size in bytes.</td></tr><tr><td colspan="6">Start of repeated group (size times)</td></tr><tr><td>6 + n</td><td>U1</td><td>data</td><td>-</td><td>-</td><td>Payload data.</td></tr><tr><td colspan="6">End of repeated group (size times)</td></tr></table>

## 3.12.5.2 Finish flashing MGA-ANO data

<table><tr><td>Message</td><td colspan="7">UBX-MGA-FLASH-STOPFinish flashing MGA-ANO data</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message is used to tell the receiver that there are no more MGA-FLASH type 1 messages coming, and that it can do any final internal operations needed to commit the data to flash as a background activity. A UBX-MGA-ACK message will be sent at the end of this process. Note that there may be a delay of several seconds before the UBX-MGA-ACK for this message is sent because of the time taken for this processing. See Flash-based AssistNow Offline for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x21</td><td>2</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>type</td><td></td><td>-</td><td>-</td><td>Message type (0x02 for this type)</td><td></td></tr><tr><td>1</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr></table>

## 3.12.5.3 Acknowledge last FLASH-DATA or -STOP

<table><tr><td>Message</td><td colspan="7">UBX-MGA-FLASH-ACK Acknowledge last FLASH-DATA or -STOP</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">This message reports an ACK/NACK to the host for the last MGA-FLASH type 1 or type 2 message message received. See Flash-based AssistNow Offline for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x21</td><td>6</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>type</td><td></td><td>-</td><td>-</td><td>Message type (0x03 for this type)</td><td></td></tr><tr><td>1</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>2</td><td>U1</td><td>ack</td><td></td><td>-</td><td>-</td><td>Acknowledgment type. 0 - ACK: Message received and written to flash. 1 - NACK: Problem with last message, re-transmission required (this only happens while acknowledging a UBX-MGA_FLASH_DATA message). 2 - NACK: problem with last message, give up.</td><td></td></tr><tr><td>3</td><td>U1</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td>4</td><td>U2</td><td>sequence</td><td></td><td>-</td><td>-</td><td>If acknowledging a UBX-MGA-FLASH-DATA message this is the Message sequence number being ack&#x27;ed. If acknowledging a UBX-MGA-FLASH-STOP message it will be set to 0xffff.</td><td></td></tr></table>

## 3.12.6 UBX-MGA-GAL (0x13 0x02)

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 66 of 181

<!-- page 67 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.12.6.1 Galileo ephemeris assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-GAL-EPH Galileo ephemeris assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of Galileo ephemeris assistance to a receiver. See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x02</td><td>76</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x01 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1</td><td colspan="2">svId</td><td>-</td><td>-</td><td colspan="2">Galileo Satellite identifier (seeSatellite Numbering)</td></tr><tr><td>3</td><td>U1</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>U2</td><td colspan="2">iodNav</td><td>-</td><td>-</td><td colspan="2">Ephemeris and clock correction Issue of Data</td></tr><tr><td>6</td><td>I2</td><td colspan="2">deltaN</td><td>2^-43</td><td>semi-circles/s</td><td colspan="2">Mean motion difference from computed value</td></tr><tr><td>8</td><td>I4</td><td colspan="2">m0</td><td>2^-31</td><td>semi-circles</td><td colspan="2">Mean anomaly at reference time</td></tr><tr><td>12</td><td>U4</td><td colspan="2">e</td><td>2^-33</td><td>-</td><td colspan="2">Eccentricity</td></tr><tr><td>16</td><td>U4</td><td colspan="2">sqrtA</td><td>2^-19</td><td>m^0.5</td><td colspan="2">Square root of the semi-major axis</td></tr><tr><td>20</td><td>I4</td><td colspan="2">omega0</td><td>2^-31</td><td>semi-circles</td><td colspan="2">Longitude of ascending node of orbital plane at weekly epoch</td></tr><tr><td>24</td><td>I4</td><td colspan="2">i0</td><td>2^-31</td><td>semi-circles</td><td colspan="2">Inclination angle at reference time</td></tr><tr><td>28</td><td>I4</td><td colspan="2">omega</td><td>2^-31</td><td>semi-circles</td><td colspan="2">Argument of perigee</td></tr><tr><td>32</td><td>I4</td><td colspan="2">omegaDot</td><td>2^-43</td><td>semi-circles/s</td><td colspan="2">Rate of change of right ascension</td></tr><tr><td>36</td><td>I2</td><td colspan="2">iDot</td><td>2^-43</td><td>semi-circles/s</td><td colspan="2">Rate of change of inclination angle</td></tr><tr><td>38</td><td>I2</td><td colspan="2">cuc</td><td>2^-29</td><td>radians</td><td colspan="2">Amplitude of the cosine harmonic correction term to the argument of latitude</td></tr><tr><td>40</td><td>I2</td><td colspan="2">cus</td><td>2^-29</td><td>radians</td><td colspan="2">Amplitude of the sine harmonic correction term to the argument of latitude</td></tr><tr><td>42</td><td>I2</td><td colspan="2">crc</td><td>2^-5</td><td>radians</td><td colspan="2">Amplitude of the cosine harmonic correction term to the orbit radius</td></tr><tr><td>44</td><td>I2</td><td colspan="2">crs</td><td>2^-5</td><td>radians</td><td colspan="2">Amplitude of the sine harmonic correction term to the orbit radius</td></tr><tr><td>46</td><td>I2</td><td colspan="2">cic</td><td>2^-29</td><td>radians</td><td colspan="2">Amplitude of the cosine harmonic correction term to the angle of inclination</td></tr><tr><td>48</td><td>I2</td><td colspan="2">cis</td><td>2^-29</td><td>radians</td><td colspan="2">Amplitude of the sine harmonic correction term to the angle of inclination</td></tr><tr><td>50</td><td>U2</td><td colspan="2">toe</td><td>60</td><td>s</td><td colspan="2">Ephemeris reference time</td></tr><tr><td>52</td><td>I4</td><td colspan="2">af0</td><td>2^-34</td><td>s</td><td colspan="2">SV clock bias correction coefficient</td></tr><tr><td>56</td><td>I4</td><td colspan="2">af1</td><td>2^-46</td><td>s/s</td><td colspan="2">SV clock drift correction coefficient</td></tr><tr><td>60</td><td>I1</td><td colspan="2">af2</td><td>2^-59</td><td>s/s squared</td><td colspan="2">SV clock drift rate correction coefficient</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 67 of 181

<!-- page 68 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 61 | U1 | sisaIndexE1E5b | - | - | Signal-In-SpaceAccuracyindexfordualfrequencyE1-E5b |
| --- | --- | --- | --- | --- | --- |
| 62 | U2 | toc | 60 | s | Clock correction data reference Time of Week |
| 64 | I2 | bgdE1E5b | 2^-32 | s | E1-E5b Broadcast Group Delay |
| 66 | U1[2] | reserved1 | - | - | Reserved |
| 68 | U1 | healthE1B | - | - | E1-B Signal Health Status |
| 69 | U1 | dataValidityE1B | - | - | E1-B Data Validity Status |
| 70 | U1 | healthE5b | - | - | E5b Signal Health Status |
| 71 | U1 | dataValidityE5b | - | - | E5b Data Validity Status |
| 72 | U1[4] | reserved2 | - | - | Reserved |

3.12.6.2 Galileo almanac assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-GAL-ALM Galileo almanac assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of Galileo almanac assistance to a receiver. See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x02</td><td>32</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x02 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1</td><td colspan="2">svId</td><td>-</td><td>-</td><td colspan="2">Galileo Satellite identifier (seeSatellite Numbering)</td></tr><tr><td>3</td><td>U1</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>U1</td><td colspan="2">ioda</td><td>-</td><td>-</td><td colspan="2">Almanac Issue of Data</td></tr><tr><td>5</td><td>U1</td><td colspan="2">almWNa</td><td>-</td><td>week</td><td colspan="2">Almanac reference week number</td></tr><tr><td>6</td><td>U2</td><td colspan="2">toa</td><td>600</td><td>s</td><td colspan="2">Almanac reference time</td></tr><tr><td>8</td><td>I2</td><td colspan="2">deltaSqrtA</td><td>2^-9</td><td>m^0.5</td><td colspan="2">Difference with respect to the square root of the nominal semi-major axis (29 600 km)</td></tr><tr><td>10</td><td>U2</td><td colspan="2">e</td><td>2^-16</td><td>-</td><td colspan="2">Eccentricity</td></tr><tr><td>12</td><td>I2</td><td colspan="2">deltaI</td><td>2^-14</td><td>semi-circles</td><td colspan="2">Inclination at reference time relative to i0 = 56 degree</td></tr><tr><td>14</td><td>I2</td><td colspan="2">omega0</td><td>2^-15</td><td>semi-circles</td><td colspan="2">Longitude of ascending node of orbital plane at weekly epoch</td></tr><tr><td>16</td><td>I2</td><td colspan="2">omegaDot</td><td>2^-33</td><td>semi-circles/s</td><td colspan="2">Rate of change of right ascension</td></tr><tr><td>18</td><td>I2</td><td colspan="2">omega</td><td>2^-15</td><td>semi-circles</td><td colspan="2">Argument of perigee</td></tr><tr><td>20</td><td>I2</td><td colspan="2">m0</td><td>2^-15</td><td>semi-circles</td><td colspan="2">Satellite mean anomaly at reference time</td></tr><tr><td>22</td><td>I2</td><td colspan="2">af0</td><td>2^-19</td><td>s</td><td colspan="2">Satellite clock correction bias &#x27;truncated&#x27;</td></tr><tr><td>24</td><td>I2</td><td colspan="2">af1</td><td>2^-38</td><td>s/s</td><td colspan="2">Satellite clock correction linear &#x27;truncated&#x27;</td></tr><tr><td>26</td><td>U1</td><td colspan="2">healthE1B</td><td>-</td><td>-</td><td colspan="2">Satellite E1-B signal health status</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 68 of 181

<!-- page 69 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 27 | U1 | healthE5b | - | - | Satellite E5b signal health status |
| --- | --- | --- | --- | --- | --- |
| 28 | U1[4] | reserved1 | - | - | Reserved |

3.12.6.3 Galileo GPS time offset assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-GAL-TIMEOFFSETGalileo GPS time offset assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of Galileo time to GPS time offset.See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x02</td><td>12</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>type</td><td></td><td>-</td><td>-</td><td>Message type (0x03 for this type)</td><td></td></tr><tr><td>1</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>2</td><td>U1[2]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td>4</td><td>I2</td><td>a0G</td><td></td><td> $2^-35$ </td><td>s</td><td>Constant term of the polynomial describing the offset</td><td></td></tr><tr><td>6</td><td>I2</td><td>a1G</td><td></td><td> $2^-51$ </td><td>s/s</td><td>Rate of change of the offset</td><td></td></tr><tr><td>8</td><td>U1</td><td>t0G</td><td></td><td>3600</td><td>s</td><td>Reference time for GGTO data</td><td></td></tr><tr><td>9</td><td>U1</td><td>wn0G</td><td></td><td>-</td><td>weeks</td><td>Week Number of GGTO reference</td><td></td></tr><tr><td>10</td><td>U1[2]</td><td>reserved1</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr></table>

## 3.12.6.4 Galileo UTC assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-GAL-UTC Galileo UTC assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of Galileo UTC assistance to a receiver. See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x02</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x05 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1[2]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>I4</td><td colspan="2">a0</td><td>2^-30</td><td>s</td><td colspan="2">First parameter of UTC polynomial</td></tr><tr><td>8</td><td>I4</td><td colspan="2">a1</td><td>2^-50</td><td>s/s</td><td colspan="2">Second parameter of UTC polynomial</td></tr><tr><td>12</td><td>I1</td><td colspan="2">dtLS</td><td>-</td><td>s</td><td colspan="2">Delta time due to current leap seconds</td></tr><tr><td>13</td><td>U1</td><td colspan="2">tot</td><td>3600</td><td>s</td><td colspan="2">UTC parameters reference time of week (Galileo time)</td></tr><tr><td>14</td><td>U1</td><td colspan="2">wnt</td><td>-</td><td>weeks</td><td colspan="2">UTC parameters reference week number (the 8-bit WNT field)</td></tr><tr><td>15</td><td>U1</td><td colspan="2">wnLSF</td><td>-</td><td>weeks</td><td colspan="2">Week number at the end of which the future leap second becomes effective (the 8-bit WNLSF field)</td></tr><tr><td>16</td><td>U1</td><td colspan="2">dN</td><td>-</td><td>days</td><td colspan="2">Day number at the end of which the future leap second becomes effective</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 69 of 181

<!-- page 70 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 17 | I1 | dTLSF | - | s | Delta time due to future leap seconds |
| --- | --- | --- | --- | --- | --- |
| 18 | U1[2] | reserved1 | - | - | Reserved |

## 3.12.7 UBX-MGA-GPS (0x13 0x00)

## 3.12.7.1 GPS ephemeris assistance

<table><tr><td rowspan="2">Message</td><td colspan="7">UBX-MGA-GPS-EPH</td></tr><tr><td colspan="7">GPS ephemeris assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of GPS ephemeris assistance to a receiver.See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x00</td><td>68</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x01 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1</td><td colspan="2">svId</td><td>-</td><td>-</td><td colspan="2">GPS Satellite identifier (seeSatellite Numbering)</td></tr><tr><td>3</td><td>U1</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>U1</td><td colspan="2">fitInterval</td><td>-</td><td>-</td><td colspan="2">Fit interval flag</td></tr><tr><td>5</td><td>U1</td><td colspan="2">uraIndex</td><td>-</td><td>-</td><td colspan="2">URA index</td></tr><tr><td>6</td><td>U1</td><td colspan="2">svHealth</td><td>-</td><td>-</td><td colspan="2">SV health</td></tr><tr><td>7</td><td>I1</td><td colspan="2">tgd</td><td>2^-31</td><td>s</td><td colspan="2">Group delay differential</td></tr><tr><td>8</td><td>U2</td><td colspan="2">iodc</td><td>-</td><td>-</td><td colspan="2">IODC</td></tr><tr><td>10</td><td>U2</td><td colspan="2">toc</td><td>2^4</td><td>s</td><td colspan="2">Clock data reference time</td></tr><tr><td>12</td><td>U1</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>13</td><td>I1</td><td colspan="2">af2</td><td>2^-55</td><td>s/s squared</td><td colspan="2">Time polynomial coefficient 2</td></tr><tr><td>14</td><td>I2</td><td colspan="2">af1</td><td>2^-43</td><td>s/s</td><td colspan="2">Time polynomial coefficient 1</td></tr><tr><td>16</td><td>I4</td><td colspan="2">af0</td><td>2^-31</td><td>s</td><td colspan="2">Time polynomial coefficient 0</td></tr><tr><td>20</td><td>I2</td><td colspan="2">crs</td><td>2^-5</td><td>m</td><td colspan="2">Crs</td></tr><tr><td>22</td><td>I2</td><td colspan="2">deltaN</td><td>2^-43</td><td>semi-circles/s</td><td colspan="2">Mean motion difference from computed value</td></tr><tr><td>24</td><td>I4</td><td colspan="2">m0</td><td>2^-31</td><td>semi-circles</td><td colspan="2">Mean anomaly at reference time</td></tr><tr><td>28</td><td>I2</td><td colspan="2">cuc</td><td>2^-29</td><td>radians</td><td colspan="2">Amplitude of cosine harmonic correction term to argument of latitude</td></tr><tr><td>30</td><td>I2</td><td colspan="2">cus</td><td>2^-29</td><td>radians</td><td colspan="2">Amplitude of sine harmonic correction term to argument of latitude</td></tr><tr><td>32</td><td>U4</td><td colspan="2">e</td><td>2^-33</td><td>-</td><td colspan="2">Eccentricity</td></tr><tr><td>36</td><td>U4</td><td colspan="2">sqrtA</td><td>2^-19</td><td>m^0.5</td><td colspan="2">Square root of the semi-major axis</td></tr><tr><td>40</td><td>U2</td><td colspan="2">toe</td><td>2^4</td><td>s</td><td colspan="2">Reference time of ephemeris</td></tr><tr><td>42</td><td>I2</td><td colspan="2">cic</td><td>2^-29</td><td>radians</td><td colspan="2">Amplitude of cos harmonic correction term to angle of inclination</td></tr><tr><td>44</td><td>I4</td><td colspan="2">omega0</td><td>2^-31</td><td>semi-circles</td><td colspan="2">Longitude of ascending node of orbit plane at weekly epoch</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 70 of 181

<!-- page 71 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 48 | I2 | cis | 2^-29 | radians | Amplitude of sine harmonic correction term to angle of inclination |
| --- | --- | --- | --- | --- | --- |
| 50 | I2 | crc | 2^-5 | m | Amplitudeofcosineharmoniccorrectiontermtoorbit radius |
| 52 | I4 | i0 | 2^-31 | semi-circles | Inclination angle at reference time |
| 56 | I4 | omega | 2^-31 | semi-circles | Argument of perigee |
| 60 | I4 | omegaDot | 2^-43 | semi-circles/s | Rate of right ascension |
| 64 | I2 | idot | 2^-43 | semi-circles/s | Rate of inclination angle |
| 66 | U1[2] | reserved2 | - | - | Reserved |

## 3.12.7.2 GPS almanac assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-GPS-ALM GPS almanac assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of GPS almanac assistance to a receiver. See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x00</td><td>36</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x02 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1</td><td colspan="2">svId</td><td>-</td><td>-</td><td colspan="2">GPS Satellite identifier (seeSatellite Numbering)</td></tr><tr><td>3</td><td>U1</td><td colspan="2">svHealth</td><td>-</td><td>-</td><td colspan="2">SV health information</td></tr><tr><td>4</td><td>U2</td><td colspan="2">e</td><td>2^-21</td><td>-</td><td colspan="2">Eccentricity</td></tr><tr><td>6</td><td>U1</td><td colspan="2">almWNa</td><td>-</td><td>week</td><td colspan="2">Reference week number of almanac (the 8-bit WNa field)</td></tr><tr><td>7</td><td>U1</td><td colspan="2">toa</td><td>2^12</td><td>s</td><td colspan="2">Reference time of almanac</td></tr><tr><td>8</td><td>I2</td><td colspan="2">deltaI</td><td>2^-19</td><td>semi-circles</td><td colspan="2">Delta inclination angle at reference time</td></tr><tr><td>10</td><td>I2</td><td colspan="2">omegaDot</td><td>2^-38</td><td>semi-circles/s</td><td colspan="2">Rate of right ascension</td></tr><tr><td>12</td><td>U4</td><td colspan="2">sqrtA</td><td>2^-11</td><td>m^0.5</td><td colspan="2">Square root of the semi-major axis</td></tr><tr><td>16</td><td>I4</td><td colspan="2">omega0</td><td>2^-23</td><td>semi-circles</td><td colspan="2">Longitude of ascending node of orbit plane</td></tr><tr><td>20</td><td>I4</td><td colspan="2">omega</td><td>2^-23</td><td>semi-circles</td><td colspan="2">Argument of perigee</td></tr><tr><td>24</td><td>I4</td><td colspan="2">m0</td><td>2^-23</td><td>semi-circles</td><td colspan="2">Mean anomaly at reference time</td></tr><tr><td>28</td><td>I2</td><td colspan="2">af0</td><td>2^-20</td><td>s</td><td colspan="2">Time polynomial coefficient 0 (8 MSBs)</td></tr><tr><td>30</td><td>I2</td><td colspan="2">af1</td><td>2^-38</td><td>s/s</td><td colspan="2">Time polynomial coefficient 1</td></tr><tr><td>32</td><td>U1[4]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 71 of 181

<!-- page 72 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.12.7.3 GPS health assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-GPS-HEALTHGPS health assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of GPS health assistance to a receiver.See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x00</td><td>40</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x04 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1[2]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>U1[32]</td><td colspan="2">healthCode</td><td>-</td><td>-</td><td colspan="2">Each byte represents a GPS SV (1-32). The 6 LSBs of each byte contains the 6 bit health code from subframes 4/5 page 25.</td></tr><tr><td>36</td><td>U1[4]</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

## 3.12.7.4 GPS UTC assistance

<table><tr><td rowspan="2">Message</td><td colspan="7">UBX-MGA-GPS-UTC</td></tr><tr><td colspan="7">GPS UTC assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of GPS UTC assistance to a receiver.See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x00</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x05 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1[2]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>I4</td><td colspan="2">utcA0</td><td>2^-30</td><td>s</td><td colspan="2">First parameter of UTC polynomial</td></tr><tr><td>8</td><td>I4</td><td colspan="2">utcA1</td><td>2^-50</td><td>s/s</td><td colspan="2">Second parameter of UTC polynomial</td></tr><tr><td>12</td><td>I1</td><td colspan="2">utcDtLS</td><td>-</td><td>s</td><td colspan="2">Delta time due to current leap seconds</td></tr><tr><td>13</td><td>U1</td><td colspan="2">utcTot</td><td>2^12</td><td>s</td><td colspan="2">UTC parameters reference time of week (GPS time)</td></tr><tr><td>14</td><td>U1</td><td colspan="2">utcWNt</td><td>-</td><td>weeks</td><td colspan="2">UTC parameters reference week number (the 8-bit WNt field)</td></tr><tr><td>15</td><td>U1</td><td colspan="2">utcWNlsf</td><td>-</td><td>weeks</td><td colspan="2">Week number at the end of which the future leap second becomes effective (the 8-bit WNLSF field)</td></tr><tr><td>16</td><td>U1</td><td colspan="2">utcDn</td><td>-</td><td>days</td><td colspan="2">Day number at the end of which the future leap second becomes effective</td></tr><tr><td>17</td><td>I1</td><td colspan="2">utcDtLSF</td><td>-</td><td>s</td><td colspan="2">Delta time due to future leap seconds</td></tr><tr><td>18</td><td>U1[2]</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 72 of 181

<!-- page 73 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.12.7.5 GPS ionosphere assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-GPS-IONOGPS ionosphere assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of GPS ionospheric assistance to a receiver.See section AssistNow online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x00</td><td>16</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x06 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1[2]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>I1</td><td colspan="2">ionoAlpha0</td><td>2^-30</td><td>s</td><td colspan="2">Ionospheric parameter alpha0 [s]</td></tr><tr><td>5</td><td>I1</td><td colspan="2">ionoAlpha1</td><td>2^-27</td><td>s/semi-circle</td><td colspan="2">Ionospheric parameter alpha1 [s/semi-circle]</td></tr><tr><td>6</td><td>I1</td><td colspan="2">ionoAlpha2</td><td>2^-24</td><td>s/(semi-circle^2)</td><td colspan="2">Ionospheric parameter alpha2 [s/semi-circle^2]</td></tr><tr><td>7</td><td>I1</td><td colspan="2">ionoAlpha3</td><td>2^-24</td><td>s/(semi-circle^3)</td><td colspan="2">Ionospheric parameter alpha3 [s/semi-circle^3]</td></tr><tr><td>8</td><td>I1</td><td colspan="2">ionoBeta0</td><td>2^11</td><td>s</td><td colspan="2">Ionospheric parameter beta0 [s]</td></tr><tr><td>9</td><td>I1</td><td colspan="2">ionoBeta1</td><td>2^14</td><td>s/semi-circle</td><td colspan="2">Ionospheric parameter beta1 [s/semi-circle]</td></tr><tr><td>10</td><td>I1</td><td colspan="2">ionoBeta2</td><td>2^16</td><td>s/(semi-circle^2)</td><td colspan="2">Ionospheric parameter beta2 [s/semi-circle^2]</td></tr><tr><td>11</td><td>I1</td><td colspan="2">ionoBeta3</td><td>2^16</td><td>s/(semi-circle^3)</td><td colspan="2">Ionospheric parameter beta3 [s/semi-circle^3]</td></tr><tr><td>12</td><td>U1[4]</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

## 3.12.8 UBX-MGA-INI (0x13 0x40)

## 3.12.8.1 Initial position assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-INI-POS_XYZInitial position assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of initial position assistance to a receiver in cartesian ECEF coordinates.This message is equivalent to theUBX-MGA-INI-POS_LLHmessage, except for the coordinate system.See section AssistNow Online in the integration manual for details.Supplying position assistance that is inaccurate by more than the specified position accuracy, may lead to substantially degraded receiver performance.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x40</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>type</td><td></td><td>-</td><td>-</td><td>Message type (0x00 for this type)</td><td></td></tr><tr><td>1</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>2</td><td>U1[2]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 73 of 181

<!-- page 74 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 4 | I4 | ecefX | - | cm | WGS84 ECEF X coordinate |
| --- | --- | --- | --- | --- | --- |
| 8 | I4 | ecefY | - | cm | WGS84 ECEF Y coordinate |
| 12 | I4 | ecefZ | - | cm | WGS84 ECEF Z coordinate |
| 16 | U4 | posAcc | - | cm | Position accuracy (stddev) |

3.12.8.2 Initial position assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-INI-POS_LLHInitial position assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of initial position assistance to a receiver in WGS84 lat/long/alt coordinates.This message is equivalent to theUBX-MGA-INI-POS_XYZmessage, except for the coordinate system.See section AssistNow online in the integration manual for details.→ Supplying position assistance that is inaccurate by more than the specified position accuracy, may lead to substantially degraded receiver performance.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x40</td><td colspan="2">20</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td>Message type (0x01 for this type)</td><td></td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>2</td><td>U1[2]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td>4</td><td>I4</td><td colspan="2">lat</td><td>1e-7</td><td>deg</td><td>WGS84 Latitude</td><td></td></tr><tr><td>8</td><td>I4</td><td colspan="2">lon</td><td>1e-7</td><td>deg</td><td>WGS84 Longitude</td><td></td></tr><tr><td>12</td><td>I4</td><td colspan="2">alt</td><td>-</td><td>cm</td><td>WGS84 Altitude</td><td></td></tr><tr><td>16</td><td>U4</td><td colspan="2">posAcc</td><td>-</td><td>cm</td><td>Position accuracy (stddev)</td><td></td></tr></table>

## 3.12.8.3 Initial time assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-INI-TIME_UTCInitial time assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of UTC time assistance to a receiver. This message is equivalent to the UBX-MGA-INI-TIME_GNSS message, except for the time base.See section AssistNow online in the integration manual for details.Supplying time assistance that is inaccurate by more than the specified time accuracy, may lead to substantially degraded receiver performance.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x40</td><td colspan="2">24</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x10 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>X1</td><td colspan="2">ref</td><td>-</td><td>-</td><td colspan="2">Reference to be used to set time</td></tr><tr><td>bits 3...0</td><td>U:4</td><td colspan="2">source</td><td>-</td><td>-</td><td colspan="2">0 = none, i.e. on receipt of message (will be inaccurate!)1 = relative to pulse sent to EXTINT02 = relative to pulse sent to EXTINT1</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 74 of 181

<!-- page 75 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td rowspan="2"></td><td>bit 4</td><td>U:1</td><td>fall</td><td>-</td><td>-</td><td>use falling edge of EXTINT pulse (default rising) - only if source is EXTINT</td></tr><tr><td>bit 5</td><td>U:1</td><td>last</td><td>-</td><td>-</td><td>use last EXTINT pulse (default next pulse) - only if source is EXTINT</td></tr><tr><td>3</td><td></td><td>I1</td><td>leapSecs</td><td>-</td><td>s</td><td>Number of leap seconds since 1980 (or 0x80 = -128 if unknown)</td></tr><tr><td>4</td><td></td><td>U2</td><td>year</td><td>-</td><td>-</td><td>Year</td></tr><tr><td>6</td><td></td><td>U1</td><td>month</td><td>-</td><td>-</td><td>Month, starting at 1</td></tr><tr><td>7</td><td></td><td>U1</td><td>day</td><td>-</td><td>-</td><td>Day, starting at 1</td></tr><tr><td>8</td><td></td><td>U1</td><td>hour</td><td>-</td><td>-</td><td>Hour, from 0 to 23</td></tr><tr><td>9</td><td></td><td>U1</td><td>minute</td><td>-</td><td>-</td><td>Minute, from 0 to 59</td></tr><tr><td>10</td><td></td><td>U1</td><td>second</td><td>-</td><td>s</td><td>Seconds, from 0 to 59</td></tr><tr><td>11</td><td></td><td>X1</td><td>bitfield0</td><td>-</td><td>-</td><td>bitfield:</td></tr><tr><td></td><td>bit 0</td><td>U:1</td><td>trustedSource</td><td>-</td><td>-</td><td>Time is provided from a trusted source. Potentially usable for replay attack detection• 0: Unknown• 1: Time source can be trusted for spoofing detection</td></tr><tr><td>12</td><td></td><td>U4</td><td>ns</td><td>-</td><td>ns</td><td>Nanoseconds, from 0 to 999,999,999</td></tr><tr><td>16</td><td></td><td>U2</td><td>tAccS</td><td>-</td><td>s</td><td>Seconds part of time accuracy</td></tr><tr><td>18</td><td></td><td>U1[2]</td><td>reserved0</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td>20</td><td></td><td>U4</td><td>tAccNs</td><td>-</td><td>ns</td><td>Nanoseconds part of time accuracy, from 0 to 999,999,999</td></tr></table>

## 3.12.8.4 Initial time assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-INI-TIME_GNSSInitial time assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of time assistance to a receiver in a chosen GNSS timebase. This message is equivalent to theUBX-MGA-INI-TIME_UTCmessage, except for the time base.See section AssistNow online in the integration manual for details.Supplying time assistance that is inaccurate by more than the specified time accuracy, may lead to substantially degraded receiver performance.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x40</td><td>24</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x11 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>X1</td><td colspan="2">ref</td><td>-</td><td>-</td><td colspan="2">Reference to be used to set time</td></tr><tr><td>bits 3...0</td><td>U:4</td><td colspan="2">source</td><td>-</td><td>-</td><td colspan="2">0 = none, i.e. on receipt of message (will be inaccurate!)1 = relative to pulse sent to EXTINT02 = relative to pulse sent to EXTINT13-15 = reserved</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 75 of 181

<!-- page 76 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td rowspan="2"></td><td>bit 4</td><td>U:1</td><td>fall</td><td>-</td><td>-</td><td>use falling edge of EXTINT pulse (default rising) - only if source is EXTINT</td></tr><tr><td>bit 5</td><td>U:1</td><td>last</td><td>-</td><td>-</td><td>use last EXTINT pulse (default next pulse) - only if source is EXTINT</td></tr><tr><td>3</td><td></td><td>U1</td><td>gnssId</td><td>-</td><td>-</td><td>Source of time information. Currently supported:0 = GPS time2 = Galileo time3 = BeiDou time6 = GLONASS time7 = NavIC time</td></tr><tr><td rowspan="2">4</td><td></td><td>X1</td><td>bitfield0</td><td>-</td><td>-</td><td>bitfield:</td></tr><tr><td>bit 0</td><td>U:1</td><td>trustedSource</td><td>-</td><td>-</td><td>Time is provided from a trusted source. Potentially usable for replay attack detection0: Unknown1: Time source can be trusted for spoofing detection</td></tr><tr><td>5</td><td></td><td>U1</td><td>reserved0</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td>6</td><td></td><td>U2</td><td>week</td><td>-</td><td>-</td><td>GNSS week number</td></tr><tr><td>8</td><td></td><td>U4</td><td>tow</td><td>-</td><td>s</td><td>GNSS time of week</td></tr><tr><td>12</td><td></td><td>U4</td><td>ns</td><td>-</td><td>ns</td><td>GNSS time of week, nanosecond part from 0 to 999,999,999</td></tr><tr><td>16</td><td></td><td>U2</td><td>tAccS</td><td>-</td><td>s</td><td>Seconds part of time accuracy</td></tr><tr><td>18</td><td></td><td>U1[2]</td><td>reserved1</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td>20</td><td></td><td>U4</td><td>tAccNs</td><td>-</td><td>ns</td><td>Nanoseconds part of time accuracy, from 0 to 999,999,999</td></tr></table>

## 3.12.8.5 Initial clock drift assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-INI-CLKDInitial clock drift assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of clock drift assistance to a receiver.See section AssistNow online in the integration manual for details.→ Supplying clock drift assistance that is inaccurate by more than the specified accuracy, may lead to substantially degraded receiver performance.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x40</td><td>12</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>type</td><td></td><td>-</td><td>-</td><td>Message type (0x20 for this type)</td><td></td></tr><tr><td>1</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>2</td><td>U1[2]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td>4</td><td>I4</td><td>clkD</td><td></td><td>-</td><td>ns/s</td><td>Clock drift</td><td></td></tr><tr><td>8</td><td>U4</td><td>clkDAcc</td><td></td><td>-</td><td>ns/s</td><td>Clock drift accuracy</td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 76 of 181

<!-- page 77 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.12.8.6 Initial frequency assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-INI-FREQInitial frequency assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td rowspan="2">Comment</td><td colspan="7">This message allows the delivery of external frequency assistance to a receiver.See section AssistNow online in the integration manual for details.</td></tr><tr><td colspan="7">Supplying external frequency assistance that is inaccurate by more than the specified accuracy, may lead to substantially degraded receiver performance.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x40</td><td colspan="2">12</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td>Message type (0x21 for this type)</td><td></td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>2</td><td>U1</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td>3</td><td>X1</td><td colspan="2">flags</td><td>-</td><td>-</td><td>Frequency reference</td><td></td></tr><tr><td rowspan="2">bits 3...0</td><td>U:4</td><td colspan="2">source</td><td>-</td><td>-</td><td>0 = frequency available on EXTINT01 = frequency available on EXTINT12-15 = reserved</td><td></td></tr><tr><td>bit 4</td><td>U:1</td><td>fall</td><td>-</td><td>-</td><td>use falling edge of EXTINT pulse (default rising)</td><td></td></tr><tr><td>4</td><td>I4</td><td colspan="2">freq</td><td>1e-2</td><td>Hz</td><td>Frequency</td><td></td></tr><tr><td>8</td><td>U4</td><td colspan="2">freqAcc</td><td>-</td><td>ppb</td><td>Frequency accuracy</td><td></td></tr></table>

3.12.8.7 Earth orientation parameters assistance

<table><tr><td rowspan="2">Message</td><td colspan="7">UBX-MGA-INI-EOP</td></tr><tr><td colspan="7">Earth orientation parameters assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of new earth orientation parameters (EOP) to a receiver to improve AssistNow Autonomous operation.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x40</td><td>72</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>type</td><td></td><td>-</td><td>-</td><td>Message type (0x30 for this type)</td><td></td></tr><tr><td>1</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>2</td><td>U1[2]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td>4</td><td>U2</td><td>d2kRef</td><td></td><td>-</td><td>d</td><td>reference time (days since 1.1.2000 12.00h UTC)</td><td></td></tr><tr><td>6</td><td>U2</td><td>d2kMax</td><td></td><td>-</td><td>d</td><td>expiration time (days since 1.1.2000 12.00h UTC)</td><td></td></tr><tr><td>8</td><td>I4</td><td>xpP0</td><td></td><td>2^-30</td><td>arcsec</td><td>x_p t^0 polynomial term (offset)</td><td></td></tr><tr><td>12</td><td>I4</td><td>xpP1</td><td></td><td>2^-30</td><td>arcsec/d</td><td>x_p t^1 polynomial term (drift)</td><td></td></tr><tr><td>16</td><td>I4</td><td>ypP0</td><td></td><td>2^-30</td><td>arcsec</td><td>y_p t^0 polynomial term (offset)</td><td></td></tr><tr><td>20</td><td>I4</td><td>ypP1</td><td></td><td>2^-30</td><td>arcsec/d</td><td>y_p t^1 polynomial term (drift)</td><td></td></tr><tr><td>24</td><td>I4</td><td>dUT1</td><td></td><td>2^-25</td><td>s</td><td>dUT1 t^0 polynomial term (offset)</td><td></td></tr><tr><td>28</td><td>I4</td><td>ddUT1</td><td></td><td>2^-30</td><td>s/d</td><td>dUT1 t^1 polynomial term (drift)</td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 77 of 181

<!-- page 78 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

32

U1[40] reserved1

Reserved

## 3.12.9 UBX-MGA-QZSS (0x13 0x05)

## 3.12.9.1 QZSS ephemeris assistance

<table><tr><td rowspan="2">Message</td><td colspan="7">UBX-MGA-QZSS-EPH</td></tr><tr><td colspan="7">QZSS ephemeris assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of QZSS ephemeris assistance to a receiver.See section AssistNow Online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x05</td><td>68</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x01 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1</td><td colspan="2">svId</td><td>-</td><td>-</td><td colspan="2">QZSS Satellite identifier (seeSatellite Numbering), Range 1-5</td></tr><tr><td>3</td><td>U1</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>U1</td><td colspan="2">fitInterval</td><td>-</td><td>-</td><td colspan="2">Fit interval flag</td></tr><tr><td>5</td><td>U1</td><td colspan="2">uraIndex</td><td>-</td><td>-</td><td colspan="2">URA index</td></tr><tr><td>6</td><td>U1</td><td colspan="2">svHealth</td><td>-</td><td>-</td><td colspan="2">SV health</td></tr><tr><td>7</td><td>I1</td><td colspan="2">tgd</td><td>2^-31</td><td>s</td><td colspan="2">Group delay differential</td></tr><tr><td>8</td><td>U2</td><td colspan="2">iodc</td><td>-</td><td>-</td><td colspan="2">IODC</td></tr><tr><td>10</td><td>U2</td><td colspan="2">toc</td><td>2^4</td><td>s</td><td colspan="2">Clock data reference time</td></tr><tr><td>12</td><td>U1</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>13</td><td>I1</td><td colspan="2">af2</td><td>2^-55</td><td>s/s squared</td><td colspan="2">Time polynomial coefficient 2</td></tr><tr><td>14</td><td>I2</td><td colspan="2">af1</td><td>2^-43</td><td>s/s</td><td colspan="2">Time polynomial coefficient 1</td></tr><tr><td>16</td><td>I4</td><td colspan="2">af0</td><td>2^-31</td><td>s</td><td colspan="2">Time polynomial coefficient 0</td></tr><tr><td>20</td><td>I2</td><td colspan="2">crs</td><td>2^-5</td><td>m</td><td colspan="2">Crs</td></tr><tr><td>22</td><td>I2</td><td colspan="2">deltaN</td><td>2^-43</td><td>semi-circles/s</td><td colspan="2">Mean motion difference from computed value</td></tr><tr><td>24</td><td>I4</td><td colspan="2">m0</td><td>2^-31</td><td>semi-circles</td><td colspan="2">Mean anomaly at reference time</td></tr><tr><td>28</td><td>I2</td><td colspan="2">cuc</td><td>2^-29</td><td>radians</td><td colspan="2">Amp of cosine harmonic corr term to arg of lat</td></tr><tr><td>30</td><td>I2</td><td colspan="2">cus</td><td>2^-29</td><td>radians</td><td colspan="2">Amp of sine harmonic corr term to arg of lat</td></tr><tr><td>32</td><td>U4</td><td colspan="2">e</td><td>2^-33</td><td>-</td><td colspan="2">eccentricity</td></tr><tr><td>36</td><td>U4</td><td colspan="2">sqrtA</td><td>2^-19</td><td>m^0.5</td><td colspan="2">Square root of the semi-major axis A</td></tr><tr><td>40</td><td>U2</td><td colspan="2">toe</td><td>2^4</td><td>s</td><td colspan="2">Reference time of ephemeris</td></tr><tr><td>42</td><td>I2</td><td colspan="2">cic</td><td>2^-29</td><td>radians</td><td colspan="2">Amp of cos harmonic corr term to angle of inclination</td></tr><tr><td>44</td><td>I4</td><td colspan="2">omega0</td><td>2^-31</td><td>semi-circles</td><td colspan="2">Long of asc node of orbit plane at weekly epoch</td></tr><tr><td>48</td><td>I2</td><td colspan="2">cis</td><td>2^-29</td><td>radians</td><td colspan="2">Amp of sine harmonic corr term to angle of inclination</td></tr><tr><td>50</td><td>I2</td><td colspan="2">crc</td><td>2^-5</td><td>m</td><td colspan="2">Amp of cosine harmonic corr term to orbit radius</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 78 of 181

<!-- page 79 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 52 | I4 | i0 | 2^-31 | semi-circles | Inclination angle at reference time |
| --- | --- | --- | --- | --- | --- |
| 56 | I4 | omega | 2^-31 | semi-circles | Argument of perigee |
| 60 | I4 | omegaDot | 2^-43 | semi-circles/s | Rate of right ascension |
| 64 | I2 | idot | 2^-43 | semi-circles/s | Rate of inclination angle |
| 66 | U1[2] | reserved2 | - | - | Reserved |

## 3.12.9.2 QZSS almanac assistance

<table><tr><td>Message</td><td colspan="7">UBX-MGA-QZSS-ALMQZSS almanac assistance</td></tr><tr><td>Type</td><td colspan="7">Input</td></tr><tr><td>Comment</td><td colspan="7">This message allows the delivery of QZSS almanac assistance to a receiver.See section AssistNow Online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x05</td><td>36</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x02 for this type)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>2</td><td>U1</td><td colspan="2">svId</td><td>-</td><td>-</td><td colspan="2">QZSS Satellite identifier (seeSatellite Numbering), Range 1-5</td></tr><tr><td>3</td><td>U1</td><td colspan="2">svHealth</td><td>-</td><td>-</td><td colspan="2">Almanac SV health information</td></tr><tr><td>4</td><td>U2</td><td colspan="2">e</td><td>2^-21</td><td>-</td><td colspan="2">Almanac eccentricity</td></tr><tr><td>6</td><td>U1</td><td colspan="2">almWNa</td><td>-</td><td>week</td><td colspan="2">Reference week number of almanac (the 8-bit WNa field)</td></tr><tr><td>7</td><td>U1</td><td colspan="2">toa</td><td>2^12</td><td>s</td><td colspan="2">Reference time of almanac</td></tr><tr><td>8</td><td>I2</td><td colspan="2">deltaI</td><td>2^-19</td><td>semi-circles</td><td colspan="2">Delta inclination angle at reference time</td></tr><tr><td>10</td><td>I2</td><td colspan="2">omegaDot</td><td>2^-38</td><td>semi-circles/s</td><td colspan="2">Almanac rate of right ascension</td></tr><tr><td>12</td><td>U4</td><td colspan="2">sqrtA</td><td>2^-11</td><td>m^0.5</td><td colspan="2">Almanac square root of the semi-major axis A</td></tr><tr><td>16</td><td>I4</td><td colspan="2">omega0</td><td>2^-23</td><td>semi-circles</td><td colspan="2">Almanac long of asc node of orbit plane at weekly</td></tr><tr><td>20</td><td>I4</td><td colspan="2">omega</td><td>2^-23</td><td>semi-circles</td><td colspan="2">Almanac argument of perigee</td></tr><tr><td>24</td><td>I4</td><td colspan="2">m0</td><td>2^-23</td><td>semi-circles</td><td colspan="2">Almanac mean anomaly at reference time</td></tr><tr><td>28</td><td>I2</td><td colspan="2">af0</td><td>2^-20</td><td>s</td><td colspan="2">Almanac time polynomial coefficient 0 (8 MSBs)</td></tr><tr><td>30</td><td>I2</td><td colspan="2">af1</td><td>2^-38</td><td>s/s</td><td colspan="2">Almanac time polynomial coefficient 1</td></tr><tr><td>32</td><td>U1[4]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

## 3.12.9.3 QZSS health assistance

| Message | UBX-MGA-QZSS-HEALTH QZSShealthassistance |
| --- | --- |
| Type | Input |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 79 of 181

<!-- page 80 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Comment</td><td colspan="7">This message allows the delivery of QZSS health assistance to a receiver.See section AssistNow Online in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x13</td><td>0x05</td><td>12</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>type</td><td></td><td>-</td><td>-</td><td>Message type (0x04 for this type)</td><td></td></tr><tr><td>1</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>2</td><td>U1[2]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td>4</td><td>U1[5]</td><td>healthCode</td><td></td><td>-</td><td>-</td><td>Each byte represents a QZSS SV (1-5). The 6 LSBs of each byte contains the 6 bit health code from subframes 4/5, data ID = 3, SV ID = 51</td><td></td></tr><tr><td>9</td><td>U1[3]</td><td>reserved1</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr></table>

## 3.13 UBX-MON (0x0a)

The messages in the UBX-MON class are used to report the receiver status, such as hardware status or I/O subsystem statistics.

## 3.13.1 UBX-MON-COMMS (0x0a 0x36)

## 3.13.1.1 Communication port information

<table><tr><td>Message</td><td colspan="7">UBX-MON-COMMSCommunication port information</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">Consolidated communications information for all ports. The size of the message is determined by the number of ports that are in use on the receiver. A port is only included if communication, either send or receive, has been initiated on that port.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x0a</td><td>0x36</td><td>8 + nPorts·40</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">nPorts</td><td>-</td><td>-</td><td colspan="2">Number of ports included</td></tr><tr><td>2</td><td>X1</td><td colspan="2">txErrors</td><td>-</td><td>-</td><td colspan="2">TX error bitmask</td></tr><tr><td>bit 0</td><td>U:1</td><td colspan="2">mem</td><td>-</td><td>-</td><td colspan="2">Memory Allocation error</td></tr><tr><td>bit 1</td><td>U:1</td><td colspan="2">alloc</td><td>-</td><td>-</td><td colspan="2">Allocation error (TX buffer full)</td></tr><tr><td>bits 4...2</td><td>U:3</td><td colspan="2">outputPort</td><td>-</td><td>-</td><td colspan="2">Output port: Reports the port from which this message was output from.0 = N/A1 = I2C2 = UART13 = UART24 = USB5 = SPI</td></tr><tr><td>3</td><td>U1</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 80 of 181

<!-- page 81 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>4</td><td>U1[4]</td><td>protIds</td><td>-</td><td></td><td>The identifiers of the protocols reported in the msgs array. 0: UBX, 1: NMEA, 2: RTCM2, 5: RTCM3, 6: SPARTN, 0xFF: No protocol reported.</td></tr><tr><td colspan="6">Start of repeated group (nPorts times)</td></tr><tr><td>8 + n·40</td><td>U2</td><td>portId</td><td>-</td><td>-</td><td>Unique identifier for the port. See section Communications ports in the integration manual for details.</td></tr><tr><td>10 + n·40</td><td>U2</td><td>txPending</td><td>-</td><td>bytes</td><td>Number of bytes pending in transmitter buffer</td></tr><tr><td>12 + n·40</td><td>U4</td><td>txBytes</td><td>-</td><td>bytes</td><td>Number of bytes ever sent</td></tr><tr><td>16 + n·40</td><td>U1</td><td>txUsage</td><td>-</td><td>%</td><td>Maximum usage transmitter buffer during the last sysmon period</td></tr><tr><td>17 + n·40</td><td>U1</td><td>txPeakUsage</td><td>-</td><td>%</td><td>Maximum usage transmitter buffer</td></tr><tr><td>18 + n·40</td><td>U2</td><td>rxPending</td><td>-</td><td>bytes</td><td>Number of bytes in receiver buffer</td></tr><tr><td>20 + n·40</td><td>U4</td><td>rxBytes</td><td>-</td><td>bytes</td><td>Number of bytes ever received</td></tr><tr><td>24 + n·40</td><td>U1</td><td>rxUsage</td><td>-</td><td>%</td><td>Maximum usage receiver buffer during the last sysmon period</td></tr><tr><td>25 + n·40</td><td>U1</td><td>rxPeakUsage</td><td>-</td><td>%</td><td>Maximum usage receiver buffer</td></tr><tr><td>26 + n·40</td><td>U2</td><td>overrunErrs</td><td>-</td><td>-</td><td>Number of 100 ms timeslots with overrun errors</td></tr><tr><td>28 + n·40</td><td>U2[4]</td><td>msgs</td><td>-</td><td>msg</td><td>Number of successfully parsed messages for each protocol. The reported protocols are identified through the protIds field.</td></tr><tr><td>36 + n·40</td><td>U1[8]</td><td>reserved1</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td>44 + n·40</td><td>U4</td><td>skipped</td><td>-</td><td>bytes</td><td>Number of skipped bytes</td></tr><tr><td colspan="6">End of repeated group (nPorts times)</td></tr></table>

## 3.13.2 UBX-MON-GNSS (0x0a 0x28)

## 3.13.2.1 Information message major GNSS selection

<table><tr><td>Message</td><td colspan="7">UBX-MON-GNSSInformation message major GNSS selection</td></tr><tr><td>Type</td><td colspan="7">Polled</td></tr><tr><td>Comment</td><td colspan="7">This message reports major GNSS selection. It does this by means of bit masks in U1 fields. Each bit in a bit mask corresponds to one major GNSS. Augmentation systems are not reported.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x0a</td><td>0x28</td><td>8</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>1</td><td>X1</td><td colspan="2">supported</td><td>-</td><td>-</td><td colspan="2">A bit mask showing the major GNSS that can be supported by this receiver</td></tr><tr><td>bit 0</td><td>U:1</td><td colspan="2">GPSSup</td><td>-</td><td>-</td><td colspan="2">GPS is supported</td></tr><tr><td>bit 1</td><td>U:1</td><td colspan="2">GlonassSup</td><td>-</td><td>-</td><td colspan="2">GLONASS is supported</td></tr><tr><td>bit 2</td><td>U:1</td><td colspan="2">BeidouSup</td><td>-</td><td>-</td><td colspan="2">BeiDou is supported</td></tr><tr><td>bit 3</td><td>U:1</td><td colspan="2">GalileoSup</td><td>-</td><td>-</td><td colspan="2">Galileo is supported</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 81 of 181

<!-- page 82 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 2 | X1 | defaultGnss | - | - | AbitmaskshowingthedefaultmajorGNSSselection. If the default major GNSS selection is currently configured in the OTP memory for this receiver, it takes precedence over the default major GNSS selection configured in the executing firmware of this |
| --- | --- | --- | --- | --- | --- |
|  | bit 0 <sup>U</sup>:1 | GPSDef | - | - | receiver. GPS is default-enabled |
|  | bit 1 <sup>U</sup>:1 | GlonassDef | - | - | GLONASS is default-enabled |
|  | bit 2 <sup>U</sup>:1 | BeidouDef | - | - | BeiDou is default-enabled |
|  | bit 3 <sup>U</sup>:1 | GalileoDef | - | - | Galileo is default-enabled |
| 3 | X1 | enabled | - | - | AbitmaskshowingthecurrentmajorGNSSselection enabled for this receiver |
|  | bit 0 <sup>U</sup>:1 | GPSEna | - | - | GPS is enabled |
|  | bit 1 <sup>U</sup>:1 | GlonassEna | - | - | GLONASS is enabled |
|  | bit 2 <sup>U</sup>:1 | BeidouEna | - | - | BeiDou is enabled |
|  | bit 3 <sup>U</sup>:1 | GalileoEna | - | - | Galileo is enabled |
| 4 | U1 | simultaneous | - | - | MaximumnumberofconcurrentmajorGNSSthatcan be supported by this receiver |
| 5 | U1[3] | reserved0 | - | - | Reserved |

## 3.13.3 UBX-MON-HW3 (0x0a 0x37)

## 3.13.3.1 I/O pin status

<table><tr><td>Message</td><td colspan="7">UBX-MON-HW3I/O pin status</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message contains information specific to each HW I/O pin, for example whether the pin is set as Input or Output.For the antenna supervisor status and other RF status information, see theUBX-MON-RFmessage.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x0a</td><td>0x37</td><td>22 + nPins·6</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">nPins</td><td>-</td><td>-</td><td colspan="2">The number of I/O pins included</td></tr><tr><td>2</td><td>X1</td><td colspan="2">flags</td><td>-</td><td>-</td><td colspan="2">Flags</td></tr><tr><td>bit 0</td><td>U:1</td><td colspan="2">rtcCalib</td><td>-</td><td>-</td><td colspan="2">RTC is calibrated</td></tr><tr><td>bit 1</td><td>U:1</td><td colspan="2">safeBoot</td><td>-</td><td>-</td><td colspan="2">Safeboot mode (0 = inactive, 1 = active)</td></tr><tr><td>bit 2</td><td>U:1</td><td colspan="2">xtalAbsent</td><td>-</td><td>-</td><td colspan="2">RTC xtal has been determined to be absent</td></tr><tr><td>3</td><td>CH[10]</td><td colspan="2">hwVersion</td><td>-</td><td>-</td><td colspan="2">Zero-terminated hardware version string (same as that returned in theUBX-MON-VERmessage)</td></tr><tr><td>13</td><td>U1[9]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

Start of repeated group (nPins times)

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 82 of 181

<!-- page 83 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>22 + n·6</td><td>U1</td><td>reserved1</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td>23 + n·6</td><td>U1</td><td>pinId</td><td>-</td><td>-</td><td>Identifier for the pin, including both external and internal pins</td></tr><tr><td>24 + n·6</td><td>X2</td><td>pinMask</td><td>-</td><td>-</td><td>Pin mask</td></tr><tr><td>bit 0</td><td>U:1</td><td>periphPIO</td><td>-</td><td>-</td><td>Pin is set to peripheral or PIO? 0=Peripheral 1=PIO</td></tr><tr><td>bits 3...1</td><td>U:3</td><td>pinBank</td><td>-</td><td>-</td><td>Bank the pin belongs to, where 0=A 1=B 2=C 3=D 4=E 5=F 6=G 7=H</td></tr><tr><td>bit 4</td><td>U:1</td><td>direction</td><td>-</td><td>-</td><td>Pin direction? 0=Input 1=Output</td></tr><tr><td>bit 5</td><td>U:1</td><td>value</td><td>-</td><td>-</td><td>Pin value? 0=Low 1=High</td></tr><tr><td>bit 6</td><td>U:1</td><td>vpManager</td><td>-</td><td>-</td><td>Used by virtual pin manager? 0=No 1=Yes</td></tr><tr><td>bit 7</td><td>U:1</td><td>pioIrq</td><td>-</td><td>-</td><td>Interrupt enabled? 0=No 1=Yes</td></tr><tr><td>bit 8</td><td>U:1</td><td>pioPullHigh</td><td>-</td><td>-</td><td>Using pull high resistor? 0=No 1=Yes</td></tr><tr><td>bit 9</td><td>U:1</td><td>pioPullLow</td><td>-</td><td>-</td><td>Using pull low resistor 0=No 1=Yes</td></tr><tr><td>26 + n·6</td><td>U1</td><td>VP</td><td>-</td><td>-</td><td>Virtual pin mapping</td></tr><tr><td>27 + n·6</td><td>U1</td><td>reserved2</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td colspan="6">End of repeated group (nPins times)</td></tr></table>

## 3.13.4 UBX-MON-PATCH (0x0a 0x27)

## 3.13.4.1 Installed patches

<table><tr><td>Message</td><td colspan="7">UBX-MON-PATCHInstalled patches</td></tr><tr><td>Type</td><td colspan="7">Polled</td></tr><tr><td>Comment</td><td colspan="7">This message reports information about patches installed and currently enabled on the receiver. It does not report on patches installed and then disabled. An enabled patch is considered active when the receiver executes from the code space where the patch resides on. For example, a ROM patch is reported active only when the system runs from ROM.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x0a</td><td>0x27</td><td colspan="2">4 + nEntries·16</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U2</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x0001 for this version)</td></tr><tr><td>2</td><td>U2</td><td colspan="2">nEntries</td><td>-</td><td>-</td><td colspan="2">Total number of reported patches</td></tr><tr><td colspan="8">Start of repeated group (nEntries times)</td></tr><tr><td>4 + n·16</td><td>X4</td><td colspan="2">patchInfo</td><td>-</td><td>-</td><td colspan="2">Status information about the reported patch</td></tr><tr><td>bit 0</td><td>U:1</td><td colspan="2">activated</td><td>-</td><td>-</td><td colspan="2">1: the patch is active, 0: otherwise</td></tr><tr><td>bits 2...1</td><td>U:2</td><td colspan="2">location</td><td>-</td><td>-</td><td colspan="2">Indicates where the patch is stored. 0: OTP, 1: ROM, 2: BBR, 3: file system</td></tr><tr><td>8 + n·16</td><td>U4</td><td colspan="2">comparatorNumber</td><td>-</td><td>-</td><td colspan="2">The number of the comparator</td></tr><tr><td>12 + n·16</td><td>U4</td><td colspan="2">patchAddress</td><td>-</td><td>-</td><td colspan="2">The address that is targeted by the patch</td></tr><tr><td>16 + n·16</td><td>U4</td><td colspan="2">patchData</td><td>-</td><td>-</td><td colspan="2">The data that is inserted at the patchAddress</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 83 of 181

<!-- page 84 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

End of repeated group (nEntries times)

## 3.13.5 UBX-MON-RCVRSTAT (0x0a 0x40)

3.13.5.1 Status of the configurable receiver features

| Message | UBX-MON-RCVRSTATStatus of the configurable receiver features |
| --- | --- |
| Type | Polled |
| Comment | Status of the configurable receiver features, e.g. enabled signals.The current configuration of the receiver is visible on the RAM layer. This message shows the current configuration values and the source of the data. The source is one of the following:0 = RAM (run-time settings)1 = BBR (settings restored from the BBR or set at run-time)2 = FLASH (settings restored from the Flash or set at run-time)3-6 = Default (firmware default setting)7 = Setting is not supported in this firmware versionThe bits indicating enabling of a feature mean:0 = Disabled1 = Enabled |

<table><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x0a</td><td>0x40</td><td>84</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x01 for this version)</td></tr><tr><td>1</td><td>X1</td><td colspan="2">sbas</td><td>-</td><td>-</td><td colspan="2">SBAS signals enabled</td></tr><tr><td>bit 0</td><td>U:1</td><td colspan="2">sig_sbas_en_val</td><td>-</td><td>-</td><td colspan="2">SBAS enabled</td></tr><tr><td>bits 3...1</td><td>U:3</td><td colspan="2">sig_sbas_en_src</td><td>-</td><td>-</td><td colspan="2">Data source</td></tr><tr><td>bit 4</td><td>U:1</td><td colspan="2">sig_sbasL1ca_en_val</td><td>-</td><td>-</td><td colspan="2">SBAS L1C/A enabled</td></tr><tr><td>bits 7...5</td><td>U:3</td><td colspan="2">sig_sbasL1ca_en_src</td><td>-</td><td>-</td><td colspan="2">Data source</td></tr><tr><td>2</td><td>X1</td><td colspan="2">navic</td><td>-</td><td>-</td><td colspan="2">NAVIC signals enabled</td></tr><tr><td>bit 0</td><td>U:1</td><td colspan="2">sig navic_en_val</td><td>-</td><td>-</td><td colspan="2">NAVIC enabled</td></tr><tr><td>bits 3...1</td><td>U:3</td><td colspan="2">sig navic_en_src</td><td>-</td><td>-</td><td colspan="2">Data source</td></tr><tr><td>bit 4</td><td>U:1</td><td colspan="2">sig navicL5_en_val</td><td>-</td><td>-</td><td colspan="2">NAVIC L5 enabled</td></tr><tr><td>bits 7...5</td><td>U:3</td><td colspan="2">sig navicL5_en_src</td><td>-</td><td>-</td><td colspan="2">Data source</td></tr><tr><td>3</td><td>U1</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>X4</td><td colspan="2">gps</td><td>-</td><td>-</td><td colspan="2">GPS signals enabled</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 84 of 181

<!-- page 85 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| bit 0 <sup>U</sup>:1 | sig_gps_en_val | - | - | GPS enabled |
| --- | --- | --- | --- | --- |
| bits3…1 <sup>U</sup>:3 | sig_gps_en_src | - | - | Data source |
| bit 4 <sup>U</sup>:1 | sig_gpsL1ca_en_val | - | - | GPS L1C/A enabled |
| bits7…5 <sup>U</sup>:3 | sig_gpsL1ca_en_src | - | - | Data source |
| bit 8 <sup>U</sup>:1 | sig_gpsL1c_en_val | - | - | GPS L1C enabled |
| bits11…9 <sup>U</sup>:3 | sig_gpsL1c_en_src | - | - | Data source |
| bit 12 <sup>U</sup>:1 | sig_gpsL2c_en_val | - | - | GPS L2C enabled |
| bits15…13 <sup>U</sup>:3 | sig_gpsL2c_en_src | - | - | Data source |
| bit 16 <sup>U</sup>:1 | sig_gpsL5_en_val | - | - | GPS L5 enabled |
| bits19…17 <sup>U</sup>:3 | sig_gpsL5_en_src | - | - | Data source |
| X4 | gal | - | - | GAL signals enabled |
| bit 0 <sup>U</sup>:1 | sig_gal_en_val | - | - | GAL enabled |
| bits3…1 <sup>U</sup>:3 | sig_gal_en_src | - | - | Data source |
| bit 4 <sup>U</sup>:1 | sig_galE1_en_val | - | - | GAL E1 enabled |
| bits7…5 <sup>U</sup>:3 | sig_galE1_en_src | - | - | Data source |
| bit 8 <sup>U</sup>:1 | sig_galE5a_en_val | - | - | GAL E5A enabled |
| bits11…9 <sup>U</sup>:3 | sig_galE5a_en_src | - | - | Data source |
| bit 12 <sup>U</sup>:1 | sig_galE5b_en_val | - | - | GAL E5B enabled |
| bits15…13 <sup>U</sup>:3 | sig_galE5b_en_src | - | - | Data source |
| bit 16 <sup>U</sup>:1 | sig_galE6_en_val | - | - | GAL E6 enabled |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 85 of 181

<!-- page 86 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| bits 19...17 | U:3 | sig_galE6_en_src | - | - | Data source |
| --- | --- | --- | --- | --- | --- |
| 12 | X4 | qzss | - | - | QZSS signals enabled |
| bit 0 | U:1 | sig_qzss_en_val | - | - | QZSS enabled |
| bits 3...1 | U:3 | sig_qzss_en_src | - | - | Data source |
| bit 4 | U:1 | sig_qzssL1ca_en_val | - | - | QZSS L1C/A enabled |
| bits 7...5 | U:3 | sig_qzssL1ca_src | - | - | Data source |
| bit 8 | U:1 | sig_qzssL1c_en_val | - | - | QZSS L1C enabled |
| bits 11...9 | U:3 | sig_qzssL1c_src | - | - | Data source |
| bit 12 | U:1 | sig_qzssL1s_en_val | - | - | QZSS L1S enabled |
| bits 15...13 | U:3 | sig_qzssL1s_src | - | - | Data source |
| bit 16 | U:1 | sig_qzssL2c_en_val | - | - | QZSS L2C enabled |
| bits 19...17 | U:3 | sig_qzssL2c_src | - | - | Data source |
| bit 20 | U:1 | sig_qzssL5_en_val | - | - | QZSS L5 enabled |
| bits 23...21 | U:3 | sig_qzssL5_src | - | - | Data source |
| 16 | X4 | bds | - | - | BDS signals enabled |
| bit 0 | U:1 | sig_bds_en_val | - | - | BDS enabled |
| bits 3...1 | U:3 | sig_bds_en_src | - | - | Data source |
| bit 4 | U:1 | sig_bdsB1i_en_val | - | - | BDS B1I enabled |
| bits 7...5 | U:3 | sig_bdsB1i_src | - | - | Data source |
| bit 8 | U:1 | sig_bdsB1c_en_val | - | - | BDS B1C enabled |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 86 of 181

<!-- page 87 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| bits 11...9 | U:3 | sig_bdsB1c_src | - | - | Data source |
| --- | --- | --- | --- | --- | --- |
| bit 12 | U:1 | sig_bdsB2_en_val | - | - | BDS B2 enabled |
| bits 15...13 | U:3 | sig_bdsB2_src | - | - | Data source |
| bit 16 | U:1 | sig_bdsB2a_en_val | - | - | BDS B2A enabled |
| bits 19...17 | U:3 | sig_bdsB2a_src | - | - | Data source |
| 20 | X2 | glo | - | - | GLONASS signals enabled |
| bit 0 | U:1 | sig_glo_en_val | - | - | GLO enabled |
| bits 3...1 | U:3 | sig_glo_en_src | - | - | Data source |
| bit 4 | U:1 | sig_gloL1_en_val | - | - | GLO L1 enabled |
| bits 7...5 | U:3 | sig_gloL1_src | - | - | Data source |
| bit 8 | U:1 | sig_gloL2_en_val | - | - | GLO L2 enabled |
| bits 11...9 | U:3 | sig_gloL2_src | - | - | Data source |
| bit 12 | U:1 | sig_gloL3_en_val | - | - | GLO L3 enabled |
| bits 15...13 | U:3 | sig_gloL3_src | - | - | Data source |
| 22 | X2 | lnaMode | - | - | Mode for internal LNA |
| bits 3...0 | U:4 | lna_lnaMode_reg_val | - | - | Current internal LNA gain0 = RF normal gain1 = RF now gain2 = RF bypass LNA |
| bits 7...4 | U:4 | lna_lnaMode_cfg_val | - | - | Configured internal LNA gain0 = RF normal gain1 = RF low gain2 = RF bypass LNA |
| bits 15...13 | U:3 | lna_lnaMode_src | - | - | Data source |
| 24 | U1[4] | reserved1 | - | - | Reserved |
| 28 | U1[4] | reserved2 | - | - | Reserved |
| 32 | U1[4] | reserved3 | - | - | Reserved |
| 36 | X4 | uartBitfield | - | - | UART settings |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 87 of 181

<!-- page 88 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| bit 0 | U:1 | uart_enable_val | - | - | UART enabled |
| --- | --- | --- | --- | --- | --- |
| bits 3...1 | U:3 | uart_enable_src | - | - | Data source |
| bit 4 | U:1 | uart_remaped_val | - | - | UART remapping |
| bits 7...5 | U:3 | uart_remaped_src | - | - | Data source |
| bit 8 | U:1 | uart_dataBits_val | - | - | UART number of used databits0 = 8 bits1 = 7 bits |
| bits 11...9 | U:3 | uart_dataBits_src | - | - | Data source |
| bits 13...12 | U:2 | uart_stopBits_val | - | - | UART number of stopbits0 = 0.5 bits1 = 1 bit2 = 1.5 bits3 = 2 bits |
| bits 16...14 | U:3 | uart_stopBits_src | - | - | Data source |
| bits 18...17 | U:2 | uart_parityBits_val | - | - | UART parity mode0 = None1 = Odd2 = Even |
| bits 21...19 | U:3 | uart_parityBits_src | - | - | Data source |
| 40 | X4 | uartBaudrate | - | - | UART baud rate |
| bits 19...0 | U:20 | uart_baudrate_val | - | - | UART baud rate |
| bits 22...20 | U:3 | uart_baudrate_src | - | - | Data source |
| 44 | X4 | spiBitfield | - | - | SPI settings |
| bit 0 | U:1 | spi_enable_val | - | - | SPI enabled |
| bits 3...1 | U:3 | spi_enable_src | - | - | Data source |
| bit 4 | U:1 | spi_extendedTimeout_val | - | - | Disable timeout of the interface after 1.5 s0 = False1 = True |
| bits 7...5 | U:3 | spi_extendedTimeout_src | - | - | Data source |
| bit 8 | U:1 | spi_CPolarity_val | - | - | Clock polarity0 = Active hight clock, SCLK idles low |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 88 of 181

<!-- page 89 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

• 1 = Active low clock, SCLK idles high

| bits 11...9 | U:3 | spi_CPolarity_src | - | - | Data source |
| --- | --- | --- | --- | --- | --- |
| bit 12 | U:1 | spi_CPhase_val | - | - | Clock phase0 = Data captured on first edge of SCLK1 = Data captured on second edge of SCLK |
| bits 15...13 | U:3 | spi_CPhase_src | - | - | Data source |
| bits 23...16 | U:8 | spi_maxFf_val | - | - | Number of bytes containing 0xFF to receive before switching off reception. Range: 0 (mechanism off) - 63 |
| bits 26...24 | U:3 | spi_maxFf_src | - | - | Data source |
| 48 | X4 | i2cBitfield | - | - | I2C settings |
| bit 0 | U:1 | i2c_enable_val | - | - | I2C enabled |
| bits 3...1 | U:3 | i2c_enable_src | - | - | Data source |
| bit 4 | U:1 | i2c_extendedTimeout_val | - | - | Disable timeout of the interface after 1.5 s0 = False1 = True |
| bits 7...5 | U:3 | i2c_extendedTimeout_src | - | - | Data source |
| bit 8 | U:1 | i2c_remap_val | - | - | I2C remapped |
| bits 11...9 | U:3 | i2c_remap_src | - | - | Data source |
| bits 19...12 | U:8 | i2c_address_val | - | - | I2C address of the receiver (7 bits) |
| bits 22...20 | U:3 | i2c_address_src | - | - | Data source |
| 52 | U1 | reserved4 | - | - | Reserved |
| 53 | X1 | operatemode | - | - | General mode of operation |
| bits 1...0 | U:2 | psm_operateMode_val | - | - | 0 = PSM disabled1 = PSMOO2 = PSMCT |
| bits 4...2 | U:3 | psm_operateMode_src | - | - | Data source |
| bits 6...5 | U:2 | psm_operateMode_state | - | - | Power save mode state0 = ACQUISITION [or when PSM disabled]1 = TRACKING2 = POWER OPTIMIZED TRACKING3 = INACTIVE |
| 54 | X1 | status | - | - | Antenna supervisor status |
| bits 1...0 | U:2 | antSup_smStatus_val | - | - | Status of the antenna supervisor engine0 = INIT1 = DONTKNOW |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 89 of 181

<!-- page 90 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

|  |  |  |  |  | 2 = OK3 = SHORT4 = OPEN |
| --- | --- | --- | --- | --- | --- |
| bits 3...2 | U:2 | antSup_aPower_val | - | - | Current power status of antenna0 = OFF1 = ON2 = DONTKNOW |
| 55 | X1 | switchPin | - | - | Antenna switch PIO number |
| bits 4...0 | U:5 | antSup_switchPi_val | - | - | PIO number |
| bits 7...5 | U:3 | antSup_switchPin_src | - | - | Data source |
| 56 | X1 | shortPin | - | - | Antenna short detection PIO number |
| bits 4...0 | U:5 | antSup_shortPin_val | - | - | PIO number |
| bits 7...5 | U:3 | antSup_shortPin_src | - | - | Data source |
| 57 | X1 | openPin | - | - | Antenna open detection PIO number |
| bits 4...0 | U:5 | antSup_openPin_val | - | - | PIO number |
| bits 7...5 | U:3 | antSup_openPin_src | - | - | Data source |
| 58 | U1[4] | reserved5 | - | - | Reserved |
| 62 | X2 | recInt | - | - | Number of seconds after which the receiver should attempt to recover from a short state. |
| bits 7...0 | U:8 | antSup_recInt_val | - | - | Recovery interval after short detection [s] |
| bits 15...13 | U:3 | antSup_recInt_src | - | - | Data source |
| 64 | X4 | antSupBitfield | - | - | Control flags bitfield |
| bit 0 | U:1 | antSup_voltctrl_val | - | - | Enable active antenna voltage control flag |
| bits 3...1 | U:3 | antSup_voltctrl_src | - | - | Data source |
| bit 4 | U:1 | antSup_shortDet_val | - | - | Enable short antenna detection flag |
| bits 7...5 | U:3 | antSup_shortDet_src | - | - | Data source |
| bit 8 | U:1 | antSup_shortDet_pol_val | - | - | True if polarity of the antenna short detection is active low |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 90 of 181

<!-- page 91 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| bits 11...9 | U:3 | antSup_short Det_pol_src | - | - | Data source |
| --- | --- | --- | --- | --- | --- |
| bit 12 | U:1 | antSup_open Det_val | - | - | Enable open antenna detection flag |
| bits 15...13 | U:3 | antSup_open Det_src | - | - | Data source |
| bit 16 | U:1 | antSup_open Det_pol_val | - | - | True if polarity of the antenna open detection is active low |
| bits 19...17 | U:3 | antSup_open Det_pol_src | - | - | Data source |
| bit 20 | U:1 | antSup_pwr Down_val | - | - | Enable power down antenna logic in the event of an antenna short circuit |
| bits 23...21 | U:3 | antSup_pwr Down_src | - | - | Data source |
| bit 24 | U:1 | antSup_pwr Down_pol_val | - | - | True if polarity of the antenna power down logic is active high |
| bits 27...25 | U:3 | antSup_pwr Down_pol_src | - | - | Data source |
| bit 28 | U:1 | antSup_ recover_val | - | - | Enable automatic recovery from short state |
| bits 31...29 | U:3 | antSup_ recover_src | - | - | Data source |
| 68 | X4 | shortUs | - | - | Maximum timeout from switching on the antenna to the short detection |
| bits 15...0 | U:16 | antSup_short Us_val | - | - | ANT on to short detection timeout [us] |
| bits 31...29 | U:3 | antSup_short Us_src | - | - | Data source |
| 72 | U1[4] | reserved6 | - | - | Reserved |
| 76 | U1[2] | reserved7 | - | - | Reserved |
| 78 | U1[2] | reserved8 | - | - | Reserved |
| 80 | U1[4] | reserved9 | - | - | Reserved |

## 3.13.6 UBX-MON-RF (0x0a 0x38)

3.13.6.1 RF information

<table><tbody><tr><td rowspan="2">Message</td><td rowspan="2">UBX-MON-RF RF information</td></tr><tr></tr><tr><td>Type</td><td>Periodic/polled</td></tr><tr><td>Comment</td><td>Information for each RF block. There are as many RF blocks reported as bands supported by this receiver.</td></tr></tbody></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 91 of 181

<!-- page 92 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td rowspan="2">Message structure</td><td>Header</td><td colspan="2">Class ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x0a</td><td>0x38</td><td>4 + nBlocks·24</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">nBlocks</td><td>-</td><td>-</td><td colspan="2">The number of RF blocks included</td></tr><tr><td>2</td><td>U1[2]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td colspan="8">Start of repeated group (nBlocks times)</td></tr><tr><td>4 + n·24</td><td>U1</td><td colspan="2">blockId</td><td>-</td><td>-</td><td colspan="2">RF block ID (0 = L1 band, 1 = L2 or L5 band depending on product configuration)</td></tr><tr><td>5 + n·24</td><td>X1</td><td colspan="2">flags</td><td>-</td><td>-</td><td colspan="2">Flags</td></tr><tr><td>bits 1...0</td><td>U:2</td><td colspan="2">jammingState</td><td>-</td><td>-</td><td colspan="2">Output from jamming/interference monitor (0 = unknown or feature disabled or flag unavailable, 1 = ok - no significant jamming, 2 = warning - interference visible but fix OK, 3 = critical - interference visible and no fix). This flag is deprecated in protocol versions that support UBX-SEC-SIG (version 0x02) and always reported as 0; instead jammingState in UBX-SEC-SIG should be monitored.</td></tr><tr><td>6 + n·24</td><td>U1</td><td colspan="2">antStatus</td><td>-</td><td>-</td><td colspan="2">Status of the antenna supervisor state machine (0x00=INIT, 0x01=DONTKNOW, 0x02=OK, 0x03=SHORT, 0x04=OPEN)</td></tr><tr><td>7 + n·24</td><td>U1</td><td colspan="2">antPower</td><td>-</td><td>-</td><td colspan="2">Current power status of antenna (0x00=OFF, 0x01=ON, 0x02=DONTKNOW)</td></tr><tr><td>8 + n·24</td><td>U4</td><td colspan="2">postStatus</td><td>-</td><td>-</td><td colspan="2">POST status word</td></tr><tr><td>12 + n·24</td><td>U1[4]</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>16 + n·24</td><td>U2</td><td colspan="2">noisePerMS</td><td>-</td><td>-</td><td colspan="2">Noise level as measured by the GPS core</td></tr><tr><td>18 + n·24</td><td>U2</td><td colspan="2">agcCnt</td><td>-</td><td>-</td><td colspan="2">AGC Monitor, as percentage of maximum gain, range 0 to 8191 (100%)</td></tr><tr><td>20 + n·24</td><td>U1</td><td colspan="2">cwSuppression</td><td>-</td><td>-</td><td colspan="2">CW interference suppression level, scaled (0=no CW jamming, 255 = strong CW jamming)</td></tr><tr><td>21 + n·24</td><td>I1</td><td colspan="2">ofsI</td><td>-</td><td>-</td><td colspan="2">Imbalance of I-part of complex signal, scaled (-128 = max. negative imbalance, 127 = max. positive imbalance)</td></tr><tr><td>22 + n·24</td><td>U1</td><td colspan="2">magI</td><td>-</td><td>-</td><td colspan="2">Magnitude of I-part of complex signal, scaled (0 = no signal, 255 = max.magnitude)</td></tr><tr><td>23 + n·24</td><td>I1</td><td colspan="2">ofsQ</td><td>-</td><td>-</td><td colspan="2">Imbalance of Q-part of complex signal, scaled (-128 = max. negative imbalance, 127 = max. positive imbalance)</td></tr><tr><td>24 + n·24</td><td>U1</td><td colspan="2">magQ</td><td>-</td><td>-</td><td colspan="2">Magnitude of Q-part of complex signal, scaled (0 = no signal, 255 = max.magnitude)</td></tr><tr><td>25 + n·24</td><td>U1[3]</td><td colspan="2">reserved2</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td colspan="8">End of repeated group (nBlocks times)</td></tr></table>

## 3.13.7 UBX-MON-RXR (0x0a 0x21)

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 92 of 181

<!-- page 93 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.13.7.1 Receiver status information

<table><tr><td rowspan="2">Message</td><td colspan="7">UBX-MON-RXR</td></tr><tr><td colspan="7">Receiver status information</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">The receiver ready message is sent when the receiver changes from or to backup mode.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x0a</td><td>0x21</td><td>1</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>X1</td><td>flags</td><td></td><td>-</td><td>-</td><td>Receiver status flags</td><td></td></tr><tr><td>bit 0</td><td>U:1</td><td>awake</td><td></td><td>-</td><td>-</td><td>not in backup mode</td><td></td></tr></table>

## 3.13.8 UBX-MON-SPAN (0x0a 0x31)

## 3.13.8.1 Signal characteristics

| Message | UBX-MON-SPANSignal characteristics |
| --- | --- |
| Type | Periodic/polled |
| Comment | This message is to be used as a basic spectrum analyzer, where it displays one spectrum for each of the receiver's existing RF paths. The spectrum is conveyed with the following parameters: The frequency span in Hz, the frequency bin resolution in Hz, the center frequency in Hz, and 256 bins with amplitude data. Additionally, in order to give further insight on the signal captured by the receiver, the current gain of the internal programmable gain amplifier (PGA) is provided.This message gives information for comparative analysis rather than absolute and precise spectrum overview. Users should not expect highly accurate spectrum amplitude.Note that the PGA gain is not included in the spectrum data but is available as a separate field. Neither the spectrum, nor the PGA gain considers the internal fixed LNA gain or an external third-party LNA.The center frequency at each bin, assuming a zero-based bin count, can be computed as$f(i) = \text{center} + \text{span} * (i - 127) / 256$ |

<table><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x0a</td><td>0x31</td><td colspan="2">4 + numRfBlocks·272</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td><td></td></tr><tr><td>1</td><td>U1</td><td colspan="2">numRfBlocks</td><td>-</td><td>-</td><td>Number of RF blocks included</td><td></td></tr><tr><td>2</td><td>U1[2]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td colspan="8">Start of repeated group (numRfBlocks times)</td></tr><tr><td>4 + n·272</td><td>U1[256]</td><td colspan="2">spectrum</td><td>2^-2</td><td>dB</td><td>Spectrum data (number of points = span/res) [Uuu.ff dB]</td><td></td></tr><tr><td>260 + n·272</td><td>U4</td><td colspan="2">span</td><td>-</td><td>Hz</td><td>Spectrum span</td><td></td></tr><tr><td>264 + n·272</td><td>U4</td><td colspan="2">res</td><td>-</td><td>Hz</td><td>Resolution of the spectrum</td><td></td></tr><tr><td>268 + n·272</td><td>U4</td><td colspan="2">center</td><td>-</td><td>Hz</td><td>Center of spectrum span</td><td></td></tr><tr><td>272 + n·272</td><td>U1</td><td colspan="2">pga</td><td>-</td><td>dB</td><td>Programmable gain amplifier</td><td></td></tr><tr><td>273 + n·272</td><td>U1[3]</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td>Reserved</td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 93 of 181

<!-- page 94 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

End of repeated group (numRfBlocks times)

## 3.13.9 UBX-MON-VER (0x0a 0x04)

3.13.9.1 Poll receiver and software version

| Message | UBX-MON-VER Pollreceiverandsoftwareversion |
| --- | --- |
| Type | Pollrequest |
| Comment |  |
| Message | Header Class ID Length (Bytes) Payload Checksum |
| structure | 0xb5 0x62 0x0a 0x04 0 see below CK_ACK_B |
| Payload | Thismessagehasnopayload. |

3.13.9.2 Receiver and software version

<table><tr><td>Message</td><td colspan="7">UBX-MON-VERReceiver and software version</td></tr><tr><td>Type</td><td colspan="7">Polled</td></tr><tr><td>Comment</td><td colspan="7"></td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x0a</td><td>0x04</td><td>40 + [0..n]·30</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>CH[30]</td><td>swVersion</td><td></td><td>-</td><td>-</td><td>Nul-terminated software version string.</td><td></td></tr><tr><td>30</td><td>CH[10]</td><td>hwVersion</td><td></td><td>-</td><td>-</td><td>Nul-terminated hardware version string</td><td></td></tr><tr><td colspan="8">Start of repeated group (N times)</td></tr><tr><td>40 + n·30</td><td>CH[30]</td><td>extension</td><td></td><td>-</td><td>-</td><td colspan="2">Extended software information strings.A series of nul-terminated strings. Each extension field is 30 characters long and contains varying software information. Not all extension fields may appear.Examples of reported information: the software version string of the underlying ROM (when the receiver&#x27;s firmware is running from flash), the firmware version, the supported protocol version, the module identifier, the flash information structure (FIS) file information, the supported major GNSS, the supported augmentation systems.See Firmware and protocol versions for details.</td></tr></table>

End of repeated group (N times)

## 3.14 UBX-NAV (0x01)

The messages in the UBX-NAV class are used to output navigation results and data, such as position, altitude and velocity in a number of formats, and status flags and accuracy estimate figures, or satellite and signal information. The messages are generated with the configured navigation rate.

## 3.14.1 UBX-NAV-AOPSTATUS (0x01 0x60)

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 94 of 181

<!-- page 95 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.14.1.1 AssistNow Autonomous status

<table><tr><td>Message</td><td colspan="7">UBX-NAV-AOPSTATUS AssistNow Autonomous status</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message provides information on the status of the AssistNow Autonomous subsystem on the receiver. For example, a host application can determine the optimal time to shut down the receiver by monitoring the status field for a steady 0. See the integration manual for details on this feature.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x60</td><td>16</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See the description of iTOW for details.</td></tr><tr><td>4</td><td>U1</td><td>aopCfg</td><td></td><td>-</td><td>-</td><td colspan="2">AssistNow Autonomous configuration</td></tr><tr><td>bit 0</td><td>U:1</td><td>useAOP</td><td></td><td>-</td><td>-</td><td colspan="2">AOP enabled flag</td></tr><tr><td>5</td><td>U1</td><td>status</td><td></td><td>-</td><td>-</td><td colspan="2">AssistNow Autonomous subsystem is idle (0) or running (not 0)</td></tr><tr><td>6</td><td>U1[10]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

## 3.14.2 UBX-NAV-CLOCK (0x01 0x22)

## 3.14.2.1 Clock solution

<table><tr><td>Message</td><td colspan="7">UBX-NAV-CLOCKClock solution</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7"></td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x22</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch. See section Navigation epochs in the integration manual for details.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>I4</td><td>clkB</td><td></td><td>-</td><td>ns</td><td>Clock bias</td><td></td></tr><tr><td>8</td><td>I4</td><td>clkD</td><td></td><td>-</td><td>ns/s</td><td>Clock drift</td><td></td></tr><tr><td>12</td><td>U4</td><td>tAcc</td><td></td><td>-</td><td>ns</td><td>Time accuracy estimate</td><td></td></tr><tr><td>16</td><td>U4</td><td>fAcc</td><td></td><td>-</td><td>ps/s</td><td>Frequency accuracy estimate</td><td></td></tr></table>

## 3.14.3 UBX-NAV-COV (0x01 0x36)

<table><tbody><tr><td colspan="2">3.14.3.1 Covariance matrices</td></tr><tr><td>Message</td><td>UBX-NAV-COV Covariancematrices</td></tr><tr><td>Type</td><td>Periodic/polled</td></tr></tbody></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 95 of 181

<!-- page 96 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

Comment This message outputs the covariance matrices for the position and velocity solutions in the topocentric coordinate system defined as the local-level North (N), East (E), Down (D) frame. As the covariance matrices are symmetric, only the upper triangular part is output.

<table><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x36</td><td>64</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>5</td><td>U1</td><td>posCovValid</td><td></td><td>-</td><td>-</td><td colspan="2">Position covariance matrix validity flag</td></tr><tr><td>6</td><td>U1</td><td>velCovValid</td><td></td><td>-</td><td>-</td><td colspan="2">Velocity covariance matrix validity flag</td></tr><tr><td>7</td><td>U1[9]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>16</td><td>R4</td><td>posCovNN</td><td></td><td>-</td><td>m^2</td><td colspan="2">Position covariance matrix value p_NN</td></tr><tr><td>20</td><td>R4</td><td>posCovNE</td><td></td><td>-</td><td>m^2</td><td colspan="2">Position covariance matrix value p_NE</td></tr><tr><td>24</td><td>R4</td><td>posCovND</td><td></td><td>-</td><td>m^2</td><td colspan="2">Position covariance matrix value p_ND</td></tr><tr><td>28</td><td>R4</td><td>posCovEE</td><td></td><td>-</td><td>m^2</td><td colspan="2">Position covariance matrix value p_EE</td></tr><tr><td>32</td><td>R4</td><td>posCovED</td><td></td><td>-</td><td>m^2</td><td colspan="2">Position covariance matrix value p_ED</td></tr><tr><td>36</td><td>R4</td><td>posCovDD</td><td></td><td>-</td><td>m^2</td><td colspan="2">Position covariance matrix value p_DD</td></tr><tr><td>40</td><td>R4</td><td>velCovNN</td><td></td><td>-</td><td>m^2/s^2</td><td colspan="2">Velocity covariance matrix value v_NN</td></tr><tr><td>44</td><td>R4</td><td>velCovNE</td><td></td><td>-</td><td>m^2/s^2</td><td colspan="2">Velocity covariance matrix value v_NE</td></tr><tr><td>48</td><td>R4</td><td>velCovND</td><td></td><td>-</td><td>m^2/s^2</td><td colspan="2">Velocity covariance matrix value v_ND</td></tr><tr><td>52</td><td>R4</td><td>velCovEE</td><td></td><td>-</td><td>m^2/s^2</td><td colspan="2">Velocity covariance matrix value v_EE</td></tr><tr><td>56</td><td>R4</td><td>velCovED</td><td></td><td>-</td><td>m^2/s^2</td><td colspan="2">Velocity covariance matrix value v_ED</td></tr><tr><td>60</td><td>R4</td><td>velCovDD</td><td></td><td>-</td><td>m^2/s^2</td><td colspan="2">Velocity covariance matrix value v_DD</td></tr></table>

## 3.14.4 UBX-NAV-DOP (0x01 0x04)

## 3.14.4.1 Dilution of precision

<table><tr><td>Message</td><td colspan="7">UBX-NAV-DOPDilution of precision</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">• DOP values are dimensionless.• All DOP values are scaled by a factor of 100. If the unit transmits a value of e.g. 156, the DOP value is 1.56.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x04</td><td>18</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>U2</td><td>gDOP</td><td></td><td>0.01</td><td>-</td><td colspan="2">Geometric DOP</td></tr><tr><td>6</td><td>U2</td><td>pDOP</td><td></td><td>0.01</td><td>-</td><td colspan="2">Position DOP</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 96 of 181

<!-- page 97 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 8 | U2 | tDOP | 0.01 | - | Time DOP |
| --- | --- | --- | --- | --- | --- |
| 10 | U2 | vDOP | 0.01 | - | Vertical DOP |
| 12 | U2 | hDOP | 0.01 | - | Horizontal DOP |
| 14 | U2 | nDOP | 0.01 | - | Northing DOP |
| 16 | U2 | eDOP | 0.01 | - | Easting DOP |

## 3.14.5 UBX-NAV-EOE (0x01 0x61)

## 3.14.5.1 End of epoch

<table><tr><td>Message</td><td colspan="7">UBX-NAV-EOEEnd of epoch</td></tr><tr><td>Type</td><td colspan="7">Periodic</td></tr><tr><td>Comment</td><td colspan="7">This message is intended to be used as a marker to collect all navigation messages of an epoch. It is output after all enabled NAV class messages and after all enabled NMEA messages.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x61</td><td>4</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr></table>

## 3.14.6 UBX-NAV-ODO (0x01 0x09)

## 3.14.6.1 Odometer solution

<table><tr><td>Message</td><td colspan="7">UBX-NAV-ODO Odometer solution</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message outputs the traveled distance since last reset (see UBX-NAV-RESETODO) together with an associated estimated accuracy and the total cumulated ground distance (can only be reset by a cold start of the receiver).</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x09</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>1</td><td>U1[3]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>U4</td><td colspan="2">iTOW</td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>8</td><td>U4</td><td colspan="2">distance</td><td>-</td><td>m</td><td colspan="2">Ground distance since last reset</td></tr><tr><td>12</td><td>U4</td><td colspan="2">totalDistance</td><td>-</td><td>m</td><td colspan="2">Total cumulative ground distance</td></tr><tr><td>16</td><td>U4</td><td colspan="2">distanceStd</td><td>-</td><td>m</td><td colspan="2">Ground distance accuracy (1-sigma)</td></tr></table>

## 3.14.7 UBX-NAV-ORB (0x01 0x34)

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 97 of 181

<!-- page 98 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.14.7.1 GNSS orbit database info

<table><tr><td>Message</td><td colspan="7">UBX-NAV-ORBGNSS orbit database info</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">Status of the GNSS orbit database knowledge.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x34</td><td>8 + numSv·6</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td colspan="2">Message version (0x01 for this version)</td></tr><tr><td>5</td><td>U1</td><td>numSv</td><td></td><td>-</td><td>-</td><td colspan="2">Number of SVs in the database</td></tr><tr><td>6</td><td>U1[2]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td colspan="8">Start of repeated group (numSv times)</td></tr><tr><td>8 + n·6</td><td>U1</td><td>gnssId</td><td></td><td>-</td><td>-</td><td colspan="2">GNSS ID</td></tr><tr><td>9 + n·6</td><td>U1</td><td>svId</td><td></td><td>-</td><td>-</td><td colspan="2">Satellite ID</td></tr><tr><td>10 + n·6</td><td>X1</td><td>svFlag</td><td></td><td>-</td><td>-</td><td colspan="2">Information Flags</td></tr><tr><td>bits 1...0</td><td>U:2</td><td>health</td><td></td><td>-</td><td>-</td><td colspan="2">SV health:0 = unknown1 = healthy2 = not healthy</td></tr><tr><td>bits 3...2</td><td>U:2</td><td>visibility</td><td></td><td>-</td><td>-</td><td colspan="2">SV health:0 = unknown1 = below horizon2 = above horizon3 = above elevation mask</td></tr><tr><td>11 + n·6</td><td>X1</td><td>eph</td><td></td><td>-</td><td>-</td><td colspan="2">Ephemeris dataIn products supporting L5 signals, the receiver may store multiple ephemeris data sets per satellite. ephUsability and ephSource fields show information on one of the data sets. It is not possible to choose which data set&#x27;s status is shown.</td></tr><tr><td>bits 4...0</td><td>U:5</td><td>ephUsability</td><td></td><td>-</td><td>-</td><td colspan="2">How long the receiver will be able to use the stored ephemeris data from now on:31 = The usability period is unknown30 = The usability period is more than 450 minutes30 &gt; n &gt; 0 = The usability period is between (n-1)*15 and n*15 minutes0 = Ephemeris can no longer be used</td></tr><tr><td>bits 7...5</td><td>U:3</td><td>ephSource</td><td></td><td>-</td><td>-</td><td colspan="2">0 = not available1 = GNSS transmission2 = external aiding3-7 = other</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 98 of 181

<!-- page 99 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 12 + n·6 | X1 | alm | - | - | Almanac data |
| --- | --- | --- | --- | --- | --- |
| bits 4...0 | U:5 | almUsability | - | - | How long the receiver will be able to use the stored almanac data from now on:31 = The usability period is unknown30 = The usability period is more than 30 days30 > n > 0 = The usability period is between n-1 and n days0 = Almanac can no longer be used |
| bits 7...5 | U:3 | almSource | - | - | 0 = not available1 = GNSS transmission2 = external aiding3-7 = other |
| 13 + n·6 | X1 | otherOrb | - | - | Other orbit data available |
| bits 4...0 | U:5 | anoAopUsability | - | - | How long the receiver will be able to use the orbit data from now on:31 = The usability period is unknown30 = The usability period is more than 30 days30 > n > 0 = The usability period is between n-1 and n days0 = Data can no longer be used |
| bits 7...5 | U:3 | type | - | - | Type of orbit data:0 = No orbit data available1 = AssistNow Offline data2 = AssistNow Autonomous data3-7 = Other orbit data |

End of repeated group (numSv times)

## 3.14.8 UBX-NAV-PL (0x01 0x62)

## 3.14.8.1 Protection level information

<table><tr><td>Message</td><td colspan="7">UBX-NAV-PLProtection level information</td></tr><tr><td>Type</td><td colspan="7">Periodic</td></tr><tr><td>Comment</td><td colspan="7">This message provides protection level (PL) values per protection level state (e.g. position ECEF X/Y/Z) and w.r.t. the given target misleading information risk (TMIR) per coordinate axis.Target misleading information risk is expressed as X [%MI/epoch] (read: X% probability of having an MI per epoch). Misleading information (MI) occurs when the Protection Level value is smaller than the true position error.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x62</td><td>52</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">msgVersion</td><td>-</td><td>-</td><td colspan="2">Message version (0x01 for this version)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">tmirCoeff</td><td>-</td><td>-</td><td colspan="2">Target misleading information risk (TMIR) [%MI/ epoch], coefficient integer number of base 10 scientific notation (see e.g. pIPos field)</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 99 of 181

<!-- page 100 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 2 | I1 | tmirExp | - | - | Target misleading information risk (TMIR) [%MI/ epoch], exponent integer number of base 10 scientific notation (see e.g. plPos field) |
| --- | --- | --- | --- | --- | --- |
| 3 | U1 | plPosValid | - | - | Position protection level validity0: Invalid (Protection level should not be used)1: Protection level is valid |
| 4 | U1 | plPosFrame | - | - | Position protection level frame:0: Invalid (not possible to calculate frame conversion)1: North-East-Down2: Longitudinal-Lateral-Vertical3: HorizSemiMajorAxis-HorizSemiMinorAxis-Vertical |
| 5 | U1 | plVelValid | - | - | Velocity protection level validity0: Invalid (Protection level should not be used)1: Protection level is valid |
| 6 | U1 | plVelFrame | - | - | Velocity protection level frame:0: Invalid (not possible to calculate frame conversion)1: North-East-Down2: Longitudinal-Lateral-Vertical3: HorizSemiMajorAxis-HorizSemiMinorAxis-Vertical |
| 7 | U1 | plTimeValid | - | - | Time protection level validity0: Invalid (Protection level should not be used)1: Protection level is valid |
| 8 | U1 | plPosInvalidityReason | - | - | Position protection level invalidity reason0: Not available1- 29: Solution not trustworthy30-100: PL not verified for this receiver configuration |
| 9 | U1 | plVelInvalidityReason | - | - | Velocity protection level invalidity reason0: Not available1- 29: Solution not trustworthy30-100: PL not verified for this receiver configuration |
| 10 | U1 | plTimeInvalidityReason | - | - | Time protection level invalidity reason0: Not available1- 29: Solution not trustworthy30-100: PL not verified for this receiver configuration |
| 11 | U1 | reserved0 | - | - | Reserved |
| 12 | U4 | iTow | - | ms | GPS time of week |
| 16 | U4 | plPos1 | - | mm | First axis of position protection level value, given in coordinate frame of plPosFrame (see plPosFrame field for value order), w.r.t. the given target misleading information risk (TMIR) of [tmirCoeff * 10^(tmirExp)] |
| 20 | U4 | plPos2 | - | mm | Second axis of position protection level value, given in coordinate frame of plPosFrame (see plPosFrame field for value order), w.r.t. the given target misleading information risk (TMIR) of [tmirCoeff * 10^(tmirExp)] |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 100 of 181

<!-- page 101 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 24 | U4 | plPos3 | - | mm | Third axis of position protection level value, given in coordinate frame of plPosFrame (see plPosFrame field for value order), w.r.t. the given target misleading information risk (TMIR) of [tmirCoeff * 10^(tmirExp)] |
| --- | --- | --- | --- | --- | --- |
| 28 | U4 | plVel1 | - | mm/s | First axis of velocity protection level value, given in coordinate frame of plVelFrame (see plVelFrame field for value order), w.r.t. the given target misleading information risk (TMIR) of [tmirCoeff * 10^(tmirExp)] |
| 32 | U4 | plVel2 | - | mm/s | Second axis of velocity protection level value, given in coordinate frame of plVelFrame (see plVelFrame field for value order), w.r.t. the given target misleading information risk (TMIR) of [tmirCoeff * 10^(tmirExp)] |
| 36 | U4 | plVel3 | - | mm/s | Third axis of velocity protection level value, given in coordinate frame of plVelFrame (see plVelFrame field for value order), w.r.t. the given target misleading information risk (TMIR) of [tmirCoeff * 10^(tmirExp)] |
| 40 | U2 | plPosHoriz Orient | 1e-2 | deg | Orientation of HorizSemiMajorAxis (see plPosFrame) of horizontal ellipse position protection level (clockwise degrees from true North), if plPosFrame==3; zero otherwise. |
| 42 | U2 | plVelHoriz Orient | 1e-2 | deg | Orientation of HorizSemiMajorAxis (see plVelFrame) of horizontal ellipse velocity protection level (clockwise degrees from true North), if plVelFrame==3; zero otherwise. |
| 44 | U4 | plTime | - | ns | Time protection level value, w.r.t. the given target misleading information risk (TMIR) of [tmirCoeff * 10^(tmirExp)] |
| 48 | U1[4] | reserved1 | - | - | Reserved |

## 3.14.9 UBX-NAV-POSECEF (0x01 0x01)

## 3.14.9.1 Position solution in ECEF

<table><tr><td rowspan="2">Message</td><td colspan="7">UBX-NAV-POSECEF</td></tr><tr><td colspan="7">Position solution in ECEF</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">See important comments concerning validity of position given in section Navigation output filters in the integration manual.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x01</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>I4</td><td>ecefX</td><td></td><td>-</td><td>cm</td><td colspan="2">ECEF X coordinate</td></tr><tr><td>8</td><td>I4</td><td>ecefY</td><td></td><td>-</td><td>cm</td><td colspan="2">ECEF Y coordinate</td></tr><tr><td>12</td><td>I4</td><td>ecefZ</td><td></td><td>-</td><td>cm</td><td colspan="2">ECEF Z coordinate</td></tr><tr><td>16</td><td>U4</td><td>pAcc</td><td></td><td>-</td><td>cm</td><td colspan="2">Position Accuracy Estimate</td></tr></table>

## 3.14.10 UBX-NAV-POSLLH (0x01 0x02)

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 101 of 181

<!-- page 102 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.14.10.1 Geodetic position solution

<table><tr><td>Message</td><td colspan="7">UBX-NAV-POSLLHGeodetic position solution</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">See important comments concerning validity of position given in section Navigation output filters in the integration manual.This message outputs the Geodetic position in the currently selected ellipsoid. The default is the WGS84 Ellipsoid, but can be changed with the message CFG-NAVSPG-USE_USRDAT.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x02</td><td>28</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>I4</td><td>lon</td><td></td><td>1e-7</td><td>deg</td><td colspan="2">Longitude</td></tr><tr><td>8</td><td>I4</td><td>lat</td><td></td><td>1e-7</td><td>deg</td><td colspan="2">Latitude</td></tr><tr><td>12</td><td>I4</td><td>height</td><td></td><td>-</td><td>mm</td><td colspan="2">Height above ellipsoid</td></tr><tr><td>16</td><td>I4</td><td>hMSL</td><td></td><td>-</td><td>mm</td><td colspan="2">Height above mean sea level</td></tr><tr><td>20</td><td>U4</td><td>hAcc</td><td></td><td>-</td><td>mm</td><td colspan="2">Horizontal accuracy estimate</td></tr><tr><td>24</td><td>U4</td><td>vAcc</td><td></td><td>-</td><td>mm</td><td colspan="2">Vertical accuracy estimate</td></tr></table>

## 3.14.11 UBX-NAV-PVT (0x01 0x07)

3.14.11.1 Navigation position velocity time solution

<table><tr><td>Message</td><td colspan="7">UBX-NAV-PVTNavigation position velocity time solution</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message combines position, velocity and time solution, including accuracy figures.Note that during a leap second there may be more or less than 60 seconds in a minute.See description of leap seconds in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x07</td><td>92</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>U2</td><td>year</td><td></td><td>-</td><td>y</td><td colspan="2">Year (UTC)</td></tr><tr><td>6</td><td>U1</td><td>month</td><td></td><td>-</td><td>month</td><td colspan="2">Month, range 1..12 (UTC)</td></tr><tr><td>7</td><td>U1</td><td>day</td><td></td><td>-</td><td>d</td><td colspan="2">Day of month, range 1..31 (UTC)</td></tr><tr><td>8</td><td>U1</td><td>hour</td><td></td><td>-</td><td>h</td><td colspan="2">Hour of day, range 0..23 (UTC)</td></tr><tr><td>9</td><td>U1</td><td>min</td><td></td><td>-</td><td>min</td><td colspan="2">Minute of hour, range 0..59 (UTC)</td></tr><tr><td>10</td><td>U1</td><td>sec</td><td></td><td>-</td><td>s</td><td colspan="2">Seconds of minute, range 0..60 (UTC)</td></tr><tr><td>11</td><td>X1</td><td>valid</td><td></td><td>-</td><td>-</td><td colspan="2">Validity flags</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 102 of 181

<!-- page 103 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| bit 0 | U:1 | validDate | - | - | 1 = valid UTC Date (see section Time validity in the integration manual for details) |
| --- | --- | --- | --- | --- | --- |
| bit 1 | U:1 | validTime | - | - | 1 = valid UTC time of day (see section Time validity in the integration manual for details) |
| bit 2 | U:1 | fullyResolved | - | - | 1 = UTC time of day has been fully resolved (no seconds uncertainty). Cannot be used to check if time is completely solved. |
| bit 3 | U:1 | validMag | - | - | 1 = valid magnetic declination |
| 12 | U4 | tAcc | - | ns | Time accuracy estimate (UTC) |
| 16 | I4 | nano | - | ns | Fraction of second, range -1e9 .. 1e9 (UTC) |
| 20 | U1 | fixType | - | - | GNSSfix Type:0 = no fix1 = dead reckoning only2 = 2D-fix3 = 3D-fix4 = GNSS + dead reckoning combined5 = time only fix |
| 21 | X1 | flags | - | - | Fix status flags |
| bit 0 | U:1 | gnssFixOK | - | - | 1 = valid fix (i.e within DOP &amp; accuracy masks) |
| bit 1 | U:1 | diffSoln | - | - | 1 = differential corrections were applied |
| bits 4...2 | U:3 | psmState | - | - | Power save mode state (see Power management section in the integration manual for details.0 = PSM is not active1 = Enabled (an intermediate state before Acquisition state2 = Acquisition3 = Tracking4 = Power Optimized Tracking5 = Inactive |
| bit 5 | U:1 | headVehValid | - | - | 1 = heading of vehicle is valid, only set if the receiver is in sensor fusion mode |
| bits 7...6 | U:2 | carrSoln | - | - | Carrier phase range solution status:0 = no carrier phase range solution1 = carrier phase range solution with floating ambiguities2 = carrier phase range solution with fixed ambiguities(not supported for protocol versions less than 20.00) |
| 22 | X1 | flags2 | - | - | Additional flags |
| bit 5 | U:1 | confirmedAvai | - | - | 1 = information about UTC Date and Time of Day validity confirmation is available (see section Time validity in the integration manual for details)This flag is only supported in Protocol Versions 19.00, 19.10, 20.10, 20.20, 20.30, 22.00, 23.00, 23.01, 27 and 28. |
| bit 6 | U:1 | confirmedDate | - | - | 1 = UTC Date validity could be confirmed (see section Time validity in the integration manual for details) |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 103 of 181

<!-- page 104 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

|  | bit 7 <sup>U</sup>:1 | confirmedTime | - | - | 1 = UTC Time of Day could be confirmed (see section Time validity in the integration manual for details) |
| --- | --- | --- | --- | --- | --- |
| 23 | U1 | numSV | - | - | Number of satellites used in Nav Solution |
| 24 | I4 | lon | 1e-7 | deg | Longitude |
| 28 | I4 | lat | 1e-7 | deg | Latitude |
| 32 | I4 | height | - | mm | Height above ellipsoid |
| 36 | I4 | hMSL | - | mm | Height above mean sea level |
| 40 | U4 | hAcc | - | mm | Horizontal accuracy estimate |
| 44 | U4 | vAcc | - | mm | Vertical accuracy estimate |
| 48 | I4 | velN | - | mm/s | NED north velocity |
| 52 | I4 | velE | - | mm/s | NED east velocity |
| 56 | I4 | velD | - | mm/s | NED down velocity |
| 60 | I4 | gSpeed | - | mm/s | Ground Speed (2-D) |
| 64 | I4 | headMot | 1e-5 | deg | Heading of motion (2-D) |
| 68 | U4 | sAcc | - | mm/s | Speed accuracy estimate |
| 72 | U4 | headAcc | 1e-5 | deg | Heading accuracy estimate (both motion and vehicle) |
| 76 | U2 | pDOP | 0.01 | - | Position DOP |
| 78 | X2 | flags3 | - | - | Additional flags |
|  | bit 0 <sup>U</sup>:1 | invalidLlh | - | - | 1 = Invalid lon, lat, height and hMSL (applicable to heading products only) |
|  | bits4…1 <sup>U</sup>:4 | lastCorrection Age | - | - | Age of the most recently received differential correction: • 0 = Not available • 1 = Age between 0 and 1 second |
|  |  |  |  |  | • 2 = Age between 1 (inclusive) and 2 seconds • 3 = Age between 2 (inclusive) and 5 seconds • 4 = Age between 5 (inclusive) and 10 seconds • 5 = Age between 10 (inclusive) and 15 seconds • 6 = Age between 15 (inclusive) and 20 seconds • 7 = Age between 20 (inclusive) and 30 seconds • 8 = Age between 30 (inclusive) and 45 seconds • 9 = Age between 45 (inclusive) and 60 seconds • 10 = Age between 60 (inclusive) and 90 seconds • 11 = Age between 90 (inclusive) and 120 seconds • >=12 = Age greater or equal than 120 seconds |
|  | bit 13 <sup>U</sup>:1 | authTime | - | - | Flag that indicates if the output time has been validated against an external trusted time source • 0 = Time is not authenticated • 1 = Time is authenticated |
|  | bit 14 <sup>U</sup>:1 | nmaFixStatus | - | - | Flag assigned to a fix that has been computed mixing satellites with data authenticated through Navigation Message Authentication (NMA) methods and satellites using unauthenticated data. The fix is flagged as Verified when internal cross-checks validates the unauthenticated signals against the authenticated ones. Note that Not Verified status |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 104 of 181

<!-- page 105 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

does not imply directly spoofing attacks, to identify spoofing alerts refer to UBX-SEC-SIG.

0 = Not Verified: The mixed solution does not agree with the NMA authenticated data or the comparison could not be performed, e.g., not enough authenticated SVs to extrapolate the result or cryptographic data not decoded yet

1 = Verified: The mixed solution agrees with the NMA authenticated data

Currently, the only existing NMA method is Galileo Open Service Navigation Message Authentication (OSNMA) protocol.

| 80 | U1[4] | reserved0 | - | - | Reserved |
| --- | --- | --- | --- | --- | --- |
| 84 | I4 | headVeh | 1e-5 | deg | Heading of vehicle (2-D), this is only valid when headVehValid is set, otherwise the output is set to the heading of motion |
| 88 | I2 | magDec | 1e-2 | deg | Magnetic declination. Only supported in ADR 4.10 and later. |
| 90 | U2 | magAcc | 1e-2 | deg | Magnetic declination accuracy. Only supported in ADR 4.10 and later. |

## 3.14.12 UBX-NAV-RESETODO (0x01 0x10)

## 3.14.12.1 Reset odometer

| Message | UBX-NAV-RESETODO Reset odometer |
| --- | --- |
| Type | Command |
| Comment | This message resets the traveled distance computed by the odometer (see UBX-NAV-ODO). UBX-ACK-ACK or UBX-ACK-NAK are returned to indicate success or failure. |
| Message | Header Class ID Length (Bytes) Payload Checksum |
| structure | 0xb5 0x62 0x01 0x10 0 see below CK_A CK_B |
| Payload | This message has no payload. |

## 3.14.13 UBX-NAV-SAT (0x01 0x35)

## 3.14.13.1 Satellite information

<table><tr><td>Message</td><td colspan="7">UBX-NAV-SATSatellite information</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message displays information about SVs that are either known to be visible or currently tracked by the receiver. All signal related information corresponds to the subset of signals specified in Signal Identifiers.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x35</td><td>8 + numSvs·12</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 105 of 181

<!-- page 106 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>4</td><td>U1</td><td>version</td><td>-</td><td>-</td><td>Message version (0x01 for this version)</td></tr><tr><td>5</td><td>U1</td><td>numSvs</td><td>-</td><td>-</td><td>Number of satellites</td></tr><tr><td>6</td><td>U1[2]</td><td>reserved0</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td colspan="6">Start of repeated group (numSvs times)</td></tr><tr><td>8 + n·12</td><td>U1</td><td>gnssId</td><td>-</td><td>-</td><td>GNSS identifier (see Satellite Numbering) for assignment</td></tr><tr><td>9 + n·12</td><td>U1</td><td>svId</td><td>-</td><td>-</td><td>Satellite identifier (see Satellite Numbering) for assignment</td></tr><tr><td>10 + n·12</td><td>U1</td><td>cno</td><td>-</td><td>dBHz</td><td>Carrier to noise ratio (signal strength)</td></tr><tr><td>11 + n·12</td><td>I1</td><td>elev</td><td>-</td><td>deg</td><td>Elevation (range: +/-90), unknown if out of range</td></tr><tr><td>12 + n·12</td><td>I2</td><td>azim</td><td>-</td><td>deg</td><td>Azimuth (range 0-360), unknown if elevation is out of range</td></tr><tr><td>14 + n·12</td><td>I2</td><td>prRes</td><td>0.1</td><td>m</td><td>Pseudorange residual</td></tr><tr><td>16 + n·12</td><td>X4</td><td>flags</td><td>-</td><td>-</td><td>Bitmask</td></tr><tr><td>bits 2...0</td><td>U:3</td><td>qualityInd</td><td>-</td><td>-</td><td>Signal quality indicator:0 = no signal1 = searching signal2 = signal acquired3 = signal detected but unusable4 = code locked and time synchronized5, 6, 7 = code and carrier locked and time synchronized</td></tr><tr><td>bit 3</td><td>U:1</td><td>svUsed</td><td>-</td><td>-</td><td>1 = Signal in the subset specified in Signal Identifiers is currently being used for navigation</td></tr><tr><td>bits 5...4</td><td>U:2</td><td>health</td><td>-</td><td>-</td><td>Signal health flag:0 = unknown1 = healthy2 = unhealthy</td></tr><tr><td>bit 6</td><td>U:1</td><td>diffCorr</td><td>-</td><td>-</td><td>1 = differential correction data is available for this SV</td></tr><tr><td>bit 7</td><td>U:1</td><td>smoothed</td><td>-</td><td>-</td><td>1 = carrier smoothed pseudorange used</td></tr><tr><td>bits 10...8</td><td>U:3</td><td>orbitSource</td><td>-</td><td>-</td><td>Orbit source:0 = no orbit information is available for this SV1 = ephemeris is used2 = almanac is used3 = AssistNow Offline orbit is used4 = AssistNow Autonomous orbit is used5, 6, 7 = other orbit information is used</td></tr><tr><td>bit 11</td><td>U:1</td><td>ephAvail</td><td>-</td><td>-</td><td>1 = ephemeris is available for this SV</td></tr><tr><td>bit 12</td><td>U:1</td><td>almAvail</td><td>-</td><td>-</td><td>1 = almanac is available for this SV</td></tr><tr><td>bit 13</td><td>U:1</td><td>anoAvail</td><td>-</td><td>-</td><td>1 = AssistNow Offline data is available for this SV</td></tr><tr><td>bit 14</td><td>U:1</td><td>aopAvail</td><td>-</td><td>-</td><td>1 = AssistNow Autonomous data is available for this SV</td></tr><tr><td>bit 16</td><td>U:1</td><td>sbasCorrUsed</td><td>-</td><td>-</td><td>1 = SBAS corrections have been used for a signal in the subset specified in Signal Identifiers</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 106 of 181

<!-- page 107 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| bit 17 <sup>U</sup>:1 | rtcmCorrUsed | - | - | 1 = RTCM corrections have been used for a signal in the subset specified in Signal Identifiers |
| --- | --- | --- | --- | --- |
| bit 18 <sup>U</sup>:1 | slasCorrUsed | - | - | 1=QZSSSLAScorrectionshavebeenusedforasignal in the subset specified in Signal Identifiers |
| bit 19 <sup>U</sup>:1 | spartnCorrUsed | - | - | 1=SPARTNcorrectionshavebeenusedforasignalin the subset specified in Signal Identifiers |
| bit 20 <sup>U</sup>:1 | prCorrUsed | - | - | 1 = Pseudorange corrections have been used for a signal in the subset specified in Signal Identifiers |
| bit 21 <sup>U</sup>:1 | crCorrUsed | - | - | 1 = Carrier range corrections have been used for a signal in the subset specified in Signal Identifiers |
| bit 22 <sup>U</sup>:1 | doCorrUsed | - | - | 1 = Range rate (Doppler) corrections have been used forasignalinthesubsetspecifiedinSignalIdentifiers |
| bit 23 <sup>U</sup>:1 | clasCorrUsed | - | - | 1=CLAScorrectionshavebeenusedforasignalinthe subset specified in Signal Identifiers |

End of repeated group (numSvs times)

## 3.14.14 UBX-NAV-SBAS (0x01 0x32)

## 3.14.14.1 SBAS status data

<table><tr><td>Message</td><td colspan="7">UBX-NAV-SBAS SBAS status data</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message outputs the status of the SBAS sub system</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x32</td><td>12 + cnt·12</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See the description of iTOW for details.</td></tr><tr><td>4</td><td>U1</td><td>geo</td><td></td><td>-</td><td>-</td><td colspan="2">PRN Number of the GEO where correction and integrity data is used from</td></tr><tr><td>5</td><td>U1</td><td>mode</td><td></td><td>-</td><td>-</td><td colspan="2">SBAS Mode0 Disabled1 Enabled integrity3 Enabled test mode</td></tr><tr><td>6</td><td>I1</td><td>sys</td><td></td><td>-</td><td>-</td><td colspan="2">SBAS System (WAAS/EGNOS/...) -1 Unknown0 WAAS1 EGNOS2 MSAS3 GAGAN16 GPS</td></tr><tr><td rowspan="5">7</td><td>X1</td><td>service</td><td></td><td>-</td><td>-</td><td colspan="2">SBAS Services available</td></tr><tr><td>bit 0</td><td>U:1</td><td>Ranging</td><td>-</td><td>-</td><td colspan="2">GEO may be used as ranging source</td></tr><tr><td>bit 1</td><td>U:1</td><td>Corrections</td><td>-</td><td>-</td><td colspan="2">GEO is providing correction data</td></tr><tr><td>bit 2</td><td>U:1</td><td>Integrity</td><td>-</td><td>-</td><td colspan="2">GEO is providing integrity</td></tr><tr><td>bit 3</td><td>U:1</td><td>Testmode</td><td>-</td><td>-</td><td colspan="2">GEO is in test mode</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 107 of 181

<!-- page 108 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td></td><td>bit 4</td><td>U:1</td><td>Bad</td><td>-</td><td>-</td><td>Problem with signal or broadcast data indicated</td></tr><tr><td>8</td><td></td><td>U1</td><td>cnt</td><td>-</td><td>-</td><td>Number of SV data following</td></tr><tr><td>9</td><td></td><td>X1</td><td>statusFlags</td><td>-</td><td>-</td><td>SBAS status flags</td></tr><tr><td></td><td>bits 1...0</td><td>U:2</td><td>integrityUsed</td><td>-</td><td>-</td><td>SBAS integrity used0 = Unknown1 = Integrity information is not available or SBAS integrity is not enabled2 = Receiver uses only GPS satellites for which integrity information is available</td></tr><tr><td>10</td><td></td><td>U1[2]</td><td>reserved0</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td colspan="7">Start of repeated group (cnt times)</td></tr><tr><td>12 + n·12</td><td></td><td>U1</td><td>svid</td><td>-</td><td>-</td><td>SV ID</td></tr><tr><td>13 + n·12</td><td></td><td>U1</td><td>reserved1</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td>14 + n·12</td><td></td><td>U1</td><td>udre</td><td>-</td><td>-</td><td>Monitoring status</td></tr><tr><td>15 + n·12</td><td></td><td>U1</td><td>svSys</td><td>-</td><td>-</td><td>System (WAAS/EGNOS/...) same as SYS</td></tr><tr><td>16 + n·12</td><td></td><td>U1</td><td>svService</td><td>-</td><td>-</td><td>Services available same as SERVICE</td></tr><tr><td>17 + n·12</td><td></td><td>U1</td><td>reserved2</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td>18 + n·12</td><td></td><td>I2</td><td>prc</td><td>-</td><td>cm</td><td>Pseudo Range correction in [cm]</td></tr><tr><td>20 + n·12</td><td></td><td>U1[2]</td><td>reserved3</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td>22 + n·12</td><td></td><td>I2</td><td>ic</td><td>-</td><td>cm</td><td>Ionosphere correction in [cm]</td></tr><tr><td colspan="7">End of repeated group (cnt times)</td></tr></table>

## 3.14.15 UBX-NAV-SIG (0x01 0x43)

## 3.14.15.1 Signal information

<table><tr><td>Message</td><td colspan="7">UBX-NAV-SIG Signal information</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message displays information about signals currently tracked or searched by the receiver.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x43</td><td>8 + numSigs·16</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>5</td><td>U1</td><td>numSigs</td><td></td><td>-</td><td>-</td><td colspan="2">Number of signals</td></tr><tr><td>6</td><td>U1[2]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td colspan="8">Start of repeated group (numSigs times)</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 108 of 181

<!-- page 109 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 8 + n·16 | U1 | gnssId | - | - | GNSS identifier (seeSatellite Numbering) for assignment |
| --- | --- | --- | --- | --- | --- |
| 9 + n·16 | U1 | svId | - | - | Satellite identifier (seeSatellite Numbering) for assignment |
| 10 + n·16 | U1 | sigId | - | - | New style signal identifier (seeSignal Identifiers) |
| 11 + n·16 | U1 | freqId | - | - | Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13) |
| 12 + n·16 | I2 | prRes | 0.1 | m | Pseudorange residual |
| 14 + n·16 | U1 | cno | - | dBHz | Carrier-to-noise density ratio (signal strength) |
| 15 + n·16 | U1 | qualityInd | - | - | Signal quality indicator:0 = no signal1 = searching signal2 = signal acquired3 = signal detected but unusable4 = code locked and time synchronized5, 6, 7 = code and carrier locked and time synchronized |
| 16 + n·16 | U1 | corrSource | - | - | Correction source:0 = no corrections1 = SBAS corrections2 = BeiDou corrections3 = RTCM2 corrections4 = RTCM3 OSR corrections5 = RTCM3 SSR corrections6 = QZSS SLAS corrections7 = SPARTN corrections8 = CLAS corrections |
| 17 + n·16 | U1 | ionoModel | - | - | Ionospheric model used:0 = no model1 = Klobuchar model transmitted by GPS2 = SBAS model3 = Klobuchar model transmitted by BeiDou8 = Iono delay derived from dual frequency observations |
| 18 + n·16 | X2 | sigFlags | - | - | Signal related flags |
| bits 1...0 | U:2 | health | - | - | Signal health flag:0 = unknown1 = healthy2 = unhealthy |
| bit 2 | U:1 | prSmoothed | - | - | 1 = Pseudorange has been smoothed |
| bit 3 | U:1 | prUsed | - | - | 1 = Pseudorange has been used for this signal |
| bit 4 | U:1 | crUsed | - | - | 1 = Carrier range has been used for this signal |
| bit 5 | U:1 | doUsed | - | - | 1 = Range rate (Doppler) has been used for this signal |
| bit 6 | U:1 | prCorrUsed | - | - | 1 = Pseudorange corrections have been used for this signal |
| bit 7 | U:1 | crCorrUsed | - | - | 1 = Carrier range corrections have been used for this signal |
| bit 8 | U:1 | doCorrUsed | - | - | 1 = Range rate (Doppler) corrections have been used for this signal |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 109 of 181

<!-- page 110 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>bit9 U:1</td><td>authStatus</td><td>-</td><td>-</td><td>Authentication status of the navigation data used to compute the satellite&#x27;s position in current navigation epoch. If the authentication fails, the navigation data is not used so the authentication status in this message can take only two values:0 = Unknown1 = AuthenticatedNote that currently the only data authentication function is provided by Galileo Open Service Navigation Message Authentication (OSNMA) protocol for E1 I/NAV message.</td></tr><tr><td>20 + n·16</td><td>U1[4] reserved1</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td colspan="5">End of repeated group (numSigs times)</td></tr></table>

## 3.14.16 UBX-NAV-SLAS (0x01 0x42)

## 3.14.16.1 QZSS L1S SLAS status data

<table><tr><td>Message</td><td colspan="7">UBX-NAV-SLASQZSS L1S SLAS status data</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message outputs the status of the QZSS L1S SLAS sub system</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x42</td><td>20 + cnt·8</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See the description of iTOW for details.</td></tr><tr><td>4</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>5</td><td>U1[3]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>8</td><td>I4</td><td>gmsLon</td><td></td><td>1e-3</td><td>deg</td><td colspan="2">Longitude of the used ground monitoring station</td></tr><tr><td>12</td><td>I4</td><td>gmsLat</td><td></td><td>1e-3</td><td>deg</td><td colspan="2">Latitude of the used ground monitoring station</td></tr><tr><td>16</td><td>U1</td><td>gmsCode</td><td></td><td>-</td><td>-</td><td colspan="2">Code of the used ground monitoring station according to the QZSS SLAS Interface Specification, available from qzss.go.jp/en/</td></tr><tr><td>17</td><td>U1</td><td>qzssSvId</td><td></td><td>-</td><td>-</td><td colspan="2">Satellite identifier of the QZS/GEO whose correction data is used (see Satellite Numbering)</td></tr><tr><td rowspan="4">18</td><td>X1</td><td>serviceFlags</td><td></td><td>-</td><td>-</td><td colspan="2">Flags regarding SLAS service</td></tr><tr><td>bit 0</td><td>U:1</td><td>gmsAvailable</td><td>-</td><td>-</td><td colspan="2">1 = Ground monitoring station available</td></tr><tr><td>bit 1</td><td>U:1</td><td>qzssSvAvailable</td><td>-</td><td>-</td><td colspan="2">1 = Correction providing QZSS SV available</td></tr><tr><td>bit 2</td><td>U:1</td><td>testMode</td><td>-</td><td>-</td><td colspan="2">1 = Currently used QZSS SV in test mode</td></tr><tr><td>19</td><td>U1</td><td>cnt</td><td></td><td>-</td><td>-</td><td colspan="2">Number of pseudorange corrections following</td></tr><tr><td colspan="8">Start of repeated group (cnt times)</td></tr><tr><td>20 + n·8</td><td>U1</td><td>gnssId</td><td></td><td>-</td><td>-</td><td colspan="2">GNSS identifier (see Satellite Numbering)</td></tr><tr><td>21 + n·8</td><td>U1</td><td>svId</td><td></td><td>-</td><td>-</td><td colspan="2">Satellite identifier (see Satellite Numbering)</td></tr><tr><td>22 + n·8</td><td>U1</td><td>reserved1</td><td></td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 110 of 181

<!-- page 111 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>23 + n·8</td><td>U1[3]</td><td>reserved2</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td>26 + n·8</td><td>I2</td><td>prc</td><td>-</td><td>cm</td><td>Pseudorange correction</td></tr><tr><td colspan="6">End of repeated group (cnt times)</td></tr></table>

## 3.14.17 UBX-NAV-STATUS (0x01 0x03)

3.14.17.1 Receiver navigation status

<table><tr><td>Message</td><td colspan="7">UBX-NAV-STATUSReceiver navigation status</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">See important comments concerning the validity of the position given in section Navigation output filters in the Integration manual.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x03</td><td>16</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.For details, see section iTOW timestamps in the integration manual.</td></tr><tr><td>4</td><td>U1</td><td>gpsFix</td><td></td><td>-</td><td>-</td><td colspan="2">GPSfix Type, this value does not qualify a fix as valid and within the limits. See note on flag gpsFixOk below.0x00 = no fix0x01 = dead reckoning only0x02 = 2D-fix0x03 = 3D-fix0x04 = GPS + dead reckoning combined0x05 = Time only fix0x06..0xff = reserved</td></tr><tr><td rowspan="5">5</td><td>X1</td><td>flags</td><td></td><td>-</td><td>-</td><td colspan="2">Navigation Status Flags</td></tr><tr><td>bit 0</td><td>U:1</td><td>gpsFixOk</td><td>-</td><td>-</td><td colspan="2">1 = position and velocity valid and within DOP and ACC Masks.</td></tr><tr><td>bit 1</td><td>U:1</td><td>diffSoln</td><td>-</td><td>-</td><td colspan="2">1 = differential corrections were applied</td></tr><tr><td>bit 2</td><td>U:1</td><td>wknSet</td><td>-</td><td>-</td><td colspan="2">1 = Week Number valid (for details, see section Time validity in the Integration manual)</td></tr><tr><td>bit 3</td><td>U:1</td><td>towSet</td><td>-</td><td>-</td><td colspan="2">1 = Time of Week valid (for details, see section Time validity in the integration manual)</td></tr><tr><td rowspan="4">6</td><td>X1</td><td>fixStat</td><td></td><td>-</td><td>-</td><td colspan="2">Fix Status Information</td></tr><tr><td>bit 0</td><td>U:1</td><td>diffCorr</td><td>-</td><td>-</td><td colspan="2">1 = differential corrections available</td></tr><tr><td>bit 1</td><td>U:1</td><td>carrSolnValid</td><td>-</td><td>-</td><td colspan="2">1 = valid carrSoln</td></tr><tr><td>bits 7...6</td><td>U:2</td><td>mapMatching</td><td>-</td><td>-</td><td colspan="2">map matching status:00: none01: valid but not used, i.e. map matching data was received, but was too old10: valid and used, map matching data was applied11: valid and used, map matching data was applied. In case of sensor unavailability map</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 111 of 181

<!-- page 112 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td rowspan="4">7</td><td rowspan="2">bits 1...0</td><td>X1</td><td>flags2</td><td>-</td><td>-</td><td>further information about navigation output</td></tr><tr><td>U:2</td><td>psmState</td><td>-</td><td>-</td><td>power save mode state (not supported for protocol versions less than 13.01)0 = ACQUISITION [or when psm disabled]1 = TRACKING2 = POWER OPTIMIZED TRACKING3 = INACTIVE</td></tr><tr><td>bits 4...3</td><td>U:2</td><td>spoofDetState</td><td>-</td><td>-</td><td>Spoofing detection state (not supported for protocol versions less than 18.00)0: Unknown or deactivated1: No spoofing indicated2: Spoofing indicated3: Multiple spoofing indicationsNote that the spoofing state value only reflects the detector state for the current navigation epoch. As spoofing can be detected most easily at the transition from real signal to spoofing signal, this is also where the detector is triggered the most. i.e. a value of 1 - No spoofing indicated does not mean that the receiver is not spoofed, it simply states that the detector was not triggered in this epoch.</td></tr><tr><td>bits 7...6</td><td>U:2</td><td>carrSoln</td><td>-</td><td>-</td><td>Carrier phase range solution status:0 = no carrier phase range solution1 = carrier phase range solution with floating ambiguities2 = carrier phase range solution with fixed ambiguities</td></tr><tr><td>8</td><td></td><td>U4</td><td>ttff</td><td>-</td><td>ms</td><td>Time to first fix (millisecond time tag)</td></tr><tr><td>12</td><td></td><td>U4</td><td>msss</td><td>-</td><td>ms</td><td>Milliseconds since startup / reset</td></tr></table>

## 3.14.18 UBX-NAV-TIMEBDS (0x01 0x24)

## 3.14.18.1 BeiDou time solution

<table><tr><td>Message</td><td colspan="7">UBX-NAV-TIMEBDSBeiDou time solution</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message reports the precise BDS time of the most recent navigation solution including validity flags and an accuracy estimate.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x24</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 112 of 181

<!-- page 113 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 4 | U4 | SOW | - | s | BDS time of week (rounded to seconds) |
| --- | --- | --- | --- | --- | --- |
| 8 | I4 | fSOW | - | ns | Fractional part of SOW (range: +/-500000000).The precise BDS time of week in seconds is:SOW + fSOW * 1e-9 |
| 12 | I2 | week | - | - | BDS week number of the navigation epoch |
| 14 | I1 | leapS | - | s | BDS leap seconds (BDS-UTC) |
| 15 | X1 | valid | - | - | Validity Flags |
|  | bit 0 | U:1 | sowValid | - | 1 = Valid SOW and fSOW (see section Time validity in the integration manual for details) |
|  | bit 1 | U:1 | weekValid | - | 1 = Valid week (see section Time validity in the integration manual for details) |
|  | bit 2 | U:1 | leapSValid | - | 1 = Valid leap second |
| 16 | U4 | tAcc | - | ns | Time Accuracy Estimate |

## 3.14.19 UBX-NAV-TIMEGAL (0x01 0x25)

## 3.14.19.1 Galileo time solution

<table><tr><td>Message</td><td colspan="7">UBX-NAV-TIMEGALGalileo time solution</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message reports the precise Galileo time of the most recent navigation solution including validity flags and an accuracy estimate.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x25</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U4</td><td colspan="2">iTOW</td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>U4</td><td colspan="2">galTow</td><td>-</td><td>s</td><td colspan="2">Galileo time of week (rounded to seconds)</td></tr><tr><td>8</td><td>I4</td><td colspan="2">fGalTow</td><td>-</td><td>ns</td><td colspan="2">Fractional part of the Galileo time of week (range: +/-500000000).The precise Galileo time of week in seconds is:galTow + fGalTow * 1e-9</td></tr><tr><td>12</td><td>I2</td><td colspan="2">galWno</td><td>-</td><td>-</td><td colspan="2">Galileo week number</td></tr><tr><td>14</td><td>I1</td><td colspan="2">leapS</td><td>-</td><td>s</td><td colspan="2">Galileo leap seconds (Galileo-UTC)</td></tr><tr><td>15</td><td>X1</td><td colspan="2">valid</td><td>-</td><td>-</td><td colspan="2">Validity Flags</td></tr><tr><td>bit 0</td><td>U:1</td><td colspan="2">galTowValid</td><td>-</td><td>-</td><td colspan="2">1 = Valid galTow and fGalTow (see section Time validity in the integration manual for details)</td></tr><tr><td>bit 1</td><td>U:1</td><td colspan="2">galWnoValid</td><td>-</td><td>-</td><td colspan="2">1 = Valid galWno (see section Time validity in the integration manual for details)</td></tr><tr><td>bit 2</td><td>U:1</td><td colspan="2">leapSValid</td><td>-</td><td>-</td><td colspan="2">1 = Valid leapS</td></tr><tr><td>16</td><td>U4</td><td colspan="2">tAcc</td><td>-</td><td>ns</td><td colspan="2">Time Accuracy Estimate</td></tr></table>

## 3.14.20 UBX-NAV-TIMEGPS (0x01 0x20)

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 113 of 181

<!-- page 114 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.14.20.1 GPS time solution

<table><tr><td>Message</td><td colspan="7">UBX-NAV-TIMEGPSGPS time solution</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message reports the precise GPS time of the most recent navigation solution including validity flags and an accuracy estimate.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x20</td><td>16</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>I4</td><td>fTOW</td><td></td><td>-</td><td>ns</td><td colspan="2">Fractional part of iTOW (range: +/-500000).The precise GPS time of week in seconds is:(iTOW * 1e-3) + (fTOW * 1e-9)</td></tr><tr><td>8</td><td>I2</td><td>week</td><td></td><td>-</td><td>-</td><td colspan="2">GPS week number of the navigation epoch</td></tr><tr><td>10</td><td>I1</td><td>leapS</td><td></td><td>-</td><td>s</td><td colspan="2">GPS leap seconds (GPS-UTC)</td></tr><tr><td>11</td><td>X1</td><td>valid</td><td></td><td>-</td><td>-</td><td colspan="2">Validity Flags</td></tr><tr><td>bit 0</td><td>U:1</td><td>towValid</td><td></td><td>-</td><td>-</td><td colspan="2">1 = Valid GPS time of week (iTOW &amp; fTOW, (see section Time validity in the integration manual for details)</td></tr><tr><td>bit 1</td><td>U:1</td><td>weekValid</td><td></td><td>-</td><td>-</td><td colspan="2">1 = Valid GPS week number (see section Time validity in the integration manual for details)</td></tr><tr><td>bit 2</td><td>U:1</td><td>leapSValid</td><td></td><td>-</td><td>-</td><td colspan="2">1 = Valid GPS leap seconds</td></tr><tr><td>12</td><td>U4</td><td>tAcc</td><td></td><td>-</td><td>ns</td><td colspan="2">Time Accuracy Estimate</td></tr></table>

## 3.14.21 UBX-NAV-TIMELS (0x01 0x26)

## 3.14.21.1 Leap second event information

<table><tr><td>Message</td><td colspan="7">UBX-NAV-TIMELS Leap second event information</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">Information about the upcoming leap second event if one is scheduled.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x26</td><td>24</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>5</td><td>U1[3]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 114 of 181

<!-- page 115 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 8 | U1 | srcOfCurrLs | - | - | Information source for the current number of leap seconds.0 = Default (hardcoded in the firmware, can be outdated)1 = Derived from time difference between GPS and GLONASS time2 = GPS3 = SBAS4 = BeiDou5 = Galileo6 = Aided data7 = Configured8 = NavIC255 = Unknown |
| --- | --- | --- | --- | --- | --- |
| 9 | I1 | currLs | - | s | Current number of leap seconds since start of GPS time (Jan 6, 1980). It reflects how much GPS time is ahead of UTC time. Galileo number of leap seconds is the same as GPS. BeiDou number of leap seconds is 14 less than GPS. GLONASS follows UTC time, so no leap seconds. |
| 10 | U1 | srcOfLsChange | - | - | Information source for the future leap second event.0 = No source2 = GPS3 = SBAS4 = BeiDou5 = Galileo6 = GLONASS7 = NavIC |
| 11 | I1 | lsChange | - | s | Future leap second change if one is scheduled. +1 = positive leap second, -1 = negative leap second, 0 = no future leap second event scheduled or no information available. If the value is 0, then the amount of leap seconds did not change and the event should be ignored. |
| 12 | I4 | timeToLsEvent | - | s | Number of seconds until the next leap second event, or from the last leap second event if no future event scheduled. If > 0 event is in the future, = 0 event is now, &lt; 0 event is in the past. Valid only if validTimeToLsEvent = 1. |
| 16 | U2 | dateOfLsGpsWn | - | - | GPS week number (WN) of the next leap second event or the last one if no future event scheduled. Valid only if validTimeToLsEvent = 1. |
| 18 | U2 | dateOfLsGpsDn | - | - | GPS day of week number (DN) for the next leap second event or the last one if no future event scheduled. Valid only if validTimeToLsEvent = 1. (GPS and Galileo DN: from 1 = Sun to 7 = Sat. BeiDou DN: from 0 = Sun to 6 = Sat.) |
| 20 | U1[3] | reserved1 | - | - | Reserved |
| 23 | X1 | valid | - | - | Validity flags |
|  | bit 0 | validCurrLs | - | - | 1 = Valid current number of leap seconds value. |
|  | bit 1 | validTimeToLsEvent | - | - | 1 = Valid time to next leap second event or from the last leap second event if no future event scheduled. |

## 3.14.22 UBX-NAV-TIMENAVIC (0x01 0x63)

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 115 of 181

<!-- page 116 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.14.22.1 NavIC time solution

<table><tr><td>Message</td><td colspan="7">UBX-NAV-TIMENAVICNavIC time solution</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message reports the precise NavIC time of the most recent navigation solution including validity flags and an accuracy estimate.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x63</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U4</td><td colspan="2">iTOW</td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>U4</td><td colspan="2">NavICTow</td><td>-</td><td>s</td><td colspan="2">NavIC time of week (rounded to seconds)</td></tr><tr><td>8</td><td>I4</td><td colspan="2">fNavICTow</td><td>-</td><td>ns</td><td colspan="2">Fractional part of the NavIC time of week (range: +/-500000000).The precise NavIC time of week in seconds is:NavICTow + fNavICTow * 1e-9</td></tr><tr><td>12</td><td>I2</td><td colspan="2">NavICWno</td><td>-</td><td>-</td><td colspan="2">NavIC week number</td></tr><tr><td>14</td><td>I1</td><td colspan="2">leapS</td><td>-</td><td>s</td><td colspan="2">NavIC leap seconds (NavIC-UTC)</td></tr><tr><td rowspan="4">15</td><td>X1</td><td colspan="2">valid</td><td>-</td><td>-</td><td colspan="2">Validity Flags</td></tr><tr><td>bit 0</td><td>U:1</td><td>NavICTowValid</td><td>-</td><td>-</td><td colspan="2">1 = Valid NavICTow and fNavICTow (see section Time validity in the integration manual for details)</td></tr><tr><td>bit 1</td><td>U:1</td><td>NavICWnoValid</td><td>-</td><td>-</td><td colspan="2">1 = Valid NavICWno (see section Time validity in the integration manual for details)</td></tr><tr><td>bit 2</td><td>U:1</td><td>leapSValid</td><td>-</td><td>-</td><td colspan="2">1 = Valid leapS</td></tr><tr><td>16</td><td>U4</td><td colspan="2">tAcc</td><td>-</td><td>ns</td><td colspan="2">Time Accuracy Estimate</td></tr></table>

## 3.14.23 UBX-NAV-TIMEQZSS (0x01 0x27)

## 3.14.23.1 QZSS time solution

<table><tr><td rowspan="2">Message</td><td colspan="7">UBX-NAV-TIMEQZSS</td></tr><tr><td colspan="7">QZSS time solution</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message reports the precise QZSS time of the most recent navigation solution including validity flags and an accuracy estimate.See the Clocks and time section in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x27</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.</td></tr><tr><td>4</td><td>U4</td><td>qzssTow</td><td></td><td>-</td><td>s</td><td colspan="2">QZSS time of week (rounded to seconds)</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 116 of 181

<!-- page 117 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 8 | I4 | fQzssTow | - | ns | Fractional part of QZSS time of week (range:+/-500000000).The precise QZSS time of week in seconds is: qzssTow + (fQzssTow * 1e-9) |
| --- | --- | --- | --- | --- | --- |
| 12 | I2 | qzssWno | - | - | QZSS week number of the navigation epoch |
| 14 | I1 | leapS | - | s | QZSS leap seconds (QZSS-UTC) |
| 15 | X1 bit 0 <sup>U</sup>:1 bit 1 <sup>U</sup>:1 bit 2 <sup>U</sup>:1 | valid qzssTowValid qzssWnoValid leapSValid | ---- | ---- | Validity Flags 1 = Valid QZSS time of week (qzssTow and fQzssTow) 1 = Valid QZSS week number 1 = Valid QZSS leap seconds |
| 16 | U4 | tAcc | - | ns | Time Accuracy Estimate |

## 3.14.24 UBX-NAV-TIMEUTC (0x01 0x21)

## 3.14.24.1 UTC time solution

<table><tr><td rowspan="2">Message</td><td colspan="7">UBX-NAV-TIMEUTC</td></tr><tr><td colspan="7">UTC time solution</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">Note that during a leap second there may be more or less than 60 seconds in a minute.See the description of leap seconds in the integration manual for details.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x21</td><td colspan="2">20</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>U4</td><td>tAcc</td><td></td><td>-</td><td>ns</td><td colspan="2">Time accuracy estimate (UTC)</td></tr><tr><td>8</td><td>I4</td><td>nano</td><td></td><td>-</td><td>ns</td><td colspan="2">Fraction of second, range -1e9 .. 1e9 (UTC)</td></tr><tr><td>12</td><td>U2</td><td>year</td><td></td><td>-</td><td>y</td><td colspan="2">Year, range 1999..2099 (UTC)</td></tr><tr><td>14</td><td>U1</td><td>month</td><td></td><td>-</td><td>month</td><td colspan="2">Month, range 1..12 (UTC)</td></tr><tr><td>15</td><td>U1</td><td>day</td><td></td><td>-</td><td>d</td><td colspan="2">Day of month, range 1..31 (UTC)</td></tr><tr><td>16</td><td>U1</td><td>hour</td><td></td><td>-</td><td>h</td><td colspan="2">Hour of day, range 0..23 (UTC)</td></tr><tr><td>17</td><td>U1</td><td>min</td><td></td><td>-</td><td>min</td><td colspan="2">Minute of hour, range 0..59 (UTC)</td></tr><tr><td>18</td><td>U1</td><td>sec</td><td></td><td>-</td><td>s</td><td colspan="2">Seconds of minute, range 0..60 (UTC)</td></tr><tr><td>19</td><td>X1</td><td>valid</td><td></td><td>-</td><td>-</td><td colspan="2">Validity Flags</td></tr><tr><td>bit 0</td><td>U:1</td><td>validTOW</td><td></td><td>-</td><td>-</td><td colspan="2">1 = Valid Time of Week (see section Time validity in the integration manual for details)</td></tr><tr><td>bit 1</td><td>U:1</td><td>validWKN</td><td></td><td>-</td><td>-</td><td colspan="2">1 = Valid Week Number (see section Time validity in the integration manual for details)</td></tr><tr><td>bit 2</td><td>U:1</td><td>validUTC</td><td></td><td>-</td><td>-</td><td colspan="2">1 = Valid UTC Time</td></tr><tr><td>bit 3</td><td>U:1</td><td>authStatus</td><td></td><td>-</td><td>-</td><td colspan="2">Indicates if the parameters used to convert GNSS time into UTC time have been authenticated.</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 117 of 181

<!-- page 118 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

|  |  |  |  |  | Note that currently the only data authentication function is provided by Galileo Open Service Navigation Message Authentication (OSNMA) protocol for E1 I/NAV message which means that data can only be authenticated for EU UTC standard. |
| --- | --- | --- | --- | --- | --- |
| bits 7...4 | U:4 | utcStandard | - | - | UTC standard identifier. (Not supported for protocol versions less than 15.00)0 = Information not available1 = Communications Research Laboratory (CRL), Tokyo, Japan2 = National Institute of Standards and Technology (NIST)3 = U.S. Naval Observatory (USNO)4 = International Bureau of Weights and Measures (BIPM)5 = European laboratories6 = Former Soviet Union (SU)7 = National Time Service Center (NTSC), China8 = National Physics Laboratory India (NPLI)15 = Unknown |

## 3.14.25 UBX-NAV-VELECEF (0x01 0x11)

3.14.25.1 Velocity solution in ECEF

<table><tr><td>Message</td><td colspan="7">UBX-NAV-VELECEFVelocity solution in ECEF</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">See important comments concerning validity of position given in section Navigation output filters in the integration manual.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x11</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>I4</td><td>ecefVX</td><td></td><td>-</td><td>cm/s</td><td colspan="2">ECEF X velocity</td></tr><tr><td>8</td><td>I4</td><td>ecefVY</td><td></td><td>-</td><td>cm/s</td><td colspan="2">ECEF Y velocity</td></tr><tr><td>12</td><td>I4</td><td>ecefVZ</td><td></td><td>-</td><td>cm/s</td><td colspan="2">ECEF Z velocity</td></tr><tr><td>16</td><td>U4</td><td>sAcc</td><td></td><td>-</td><td>cm/s</td><td colspan="2">Speed accuracy estimate</td></tr></table>

## 3.14.26 UBX-NAV-VELNED (0x01 0x12)

3.14.26.1 Velocity solution in NED frame

| Message | UBX-NAV-VELNED VelocitysolutioninNEDframe |
| --- | --- |
| Type | Periodic/polled |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 118 of 181

<!-- page 119 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Comment</td><td colspan="7">See important comments concerning validity of position given in section Navigation output filters in the integration manual.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x01</td><td>0x12</td><td>36</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U4</td><td>iTOW</td><td></td><td>-</td><td>ms</td><td colspan="2">GPS time of week of the navigation epoch.See section iTOW timestamps in the integration manual for details.</td></tr><tr><td>4</td><td>I4</td><td>velN</td><td></td><td>-</td><td>cm/s</td><td colspan="2">North velocity component</td></tr><tr><td>8</td><td>I4</td><td>velE</td><td></td><td>-</td><td>cm/s</td><td colspan="2">East velocity component</td></tr><tr><td>12</td><td>I4</td><td>velD</td><td></td><td>-</td><td>cm/s</td><td colspan="2">Down velocity component</td></tr><tr><td>16</td><td>U4</td><td>speed</td><td></td><td>-</td><td>cm/s</td><td colspan="2">Speed (3-D)</td></tr><tr><td>20</td><td>U4</td><td>gSpeed</td><td></td><td>-</td><td>cm/s</td><td colspan="2">Ground speed (2-D)</td></tr><tr><td>24</td><td>I4</td><td>heading</td><td></td><td>1e-5</td><td>deg</td><td colspan="2">Heading of motion 2-D</td></tr><tr><td>28</td><td>U4</td><td>sAcc</td><td></td><td>-</td><td>cm/s</td><td colspan="2">Speed accuracy Estimate</td></tr><tr><td>32</td><td>U4</td><td>cAcc</td><td></td><td>1e-5</td><td>deg</td><td colspan="2">Course / Heading accuracy estimate</td></tr></table>

## 3.15 UBX-RXM (0x02)

The messages in the UBX-RXM class are used to output status and result data from the receiver manager as well as sending commands to the receiver manager.

## 3.15.1 UBX-RXM-MEASX (0x02 0x14)

## 3.15.1.1 Satellite measurements for RRLP

<table><tr><td>Message</td><td colspan="7">UBX-RXM-MEASXSatellite measurements for RRLP</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">The message payload data is, where possible and appropriate, according to the Radio Resource LCS (Location Services) Protocol (RRLP) [1]. One exception is the satellite and GNSS IDs, which here are given according to theSatellite Numberingscheme. The correct satellites have to be selected and their satellite ID translated accordingly [1, tab. A.10.14] for use in a RRLP Measure Position Response Component. Similarly, the measurement reference time of week has to be forwarded correctly (modulo 14400000 for the 24 LSB GPS measurements variant, modulo 3600000 for the 22 LSB Galileo and Additional Navigation Satellite Systems (GANSS) measurements variant) of the RRLP measure position response to the SMLC.Reference: [1] ETSI TS 144 031 V11.0.0 (2012-10), Digital cellular telecommunications system (Phase 2+), Location Services (LCS), Mobile Station (MS) - Serving Mobile Location Centre (SMLC), Radio Resource LCS Protocol (RRLP), (3GPP TS 44.031 version 11.0.0 Release 11).</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x02</td><td>0x14</td><td>44 + numSV·24</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version, currently 0x01</td><td></td></tr><tr><td>1</td><td>U1[3]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td>4</td><td>U4</td><td>gpsTOW</td><td></td><td>-</td><td>ms</td><td>GPS measurement reference time</td><td></td></tr><tr><td>8</td><td>U4</td><td>gloTOW</td><td></td><td>-</td><td>ms</td><td>GLONASS measurement reference time</td><td></td></tr><tr><td>12</td><td>U4</td><td>bdsTOW</td><td></td><td>-</td><td>ms</td><td>BeiDou measurement reference time</td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 119 of 181

<!-- page 120 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>16</td><td>U1[4]</td><td>reserved1</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td>20</td><td>U4</td><td>qzssTOW</td><td>-</td><td>ms</td><td>QZSS measurement reference time</td></tr><tr><td>24</td><td>U2</td><td>gpsTOWacc</td><td>2^-4</td><td>ms</td><td>GPS measurement reference time accuracy (0xffff = &gt; 4s)</td></tr><tr><td>26</td><td>U2</td><td>gloTOWacc</td><td>2^-4</td><td>ms</td><td>GLONASS measurement reference time accuracy (0xffff = &gt; 4s)</td></tr><tr><td>28</td><td>U2</td><td>bdsTOWacc</td><td>2^-4</td><td>ms</td><td>BeiDou measurement reference time accuracy (0xffff = &gt; 4s)</td></tr><tr><td>30</td><td>U1[2]</td><td>reserved2</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td>32</td><td>U2</td><td>qzssTOWacc</td><td>2^-4</td><td>ms</td><td>QZSS measurement reference time accuracy (0xffff = &gt; 4s)</td></tr><tr><td>34</td><td>U1</td><td>numSV</td><td>-</td><td>-</td><td>Number of satellites in repeated block</td></tr><tr><td>35</td><td>U1</td><td>flags</td><td>-</td><td>-</td><td>Flags</td></tr><tr><td>bits 1...0</td><td>U:2</td><td>towSet</td><td>-</td><td>-</td><td>TOW set (0 = no, 1 or 2 = yes)</td></tr><tr><td>36</td><td>U1[8]</td><td>reserved3</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td colspan="6">Start of repeated group (numSV times)</td></tr><tr><td>44 + n·24</td><td>U1</td><td>gnssId</td><td>-</td><td>-</td><td>GNSS ID (see Satellite Numbering)</td></tr><tr><td>45 + n·24</td><td>U1</td><td>svId</td><td>-</td><td>-</td><td>Satellite ID (see Satellite Numbering)</td></tr><tr><td>46 + n·24</td><td>U1</td><td>cNo</td><td>-</td><td>-</td><td>carrier noise ratio (0..63)</td></tr><tr><td>47 + n·24</td><td>U1</td><td>mpathIndic</td><td>-</td><td>-</td><td>multipath index (according to [1]) (0 = not measured, 1 = low, 2 = medium, 3 = high)</td></tr><tr><td>48 + n·24</td><td>I4</td><td>dopplerMS</td><td>0.04</td><td>m/s</td><td>Doppler measurement</td></tr><tr><td>52 + n·24</td><td>I4</td><td>dopplerHz</td><td>0.2</td><td>Hz</td><td>Doppler measurement</td></tr><tr><td>56 + n·24</td><td>U2</td><td>wholeChips</td><td>-</td><td>-</td><td>whole value of the code phase measurement (0..1022 for GPS)</td></tr><tr><td>58 + n·24</td><td>U2</td><td>fracChips</td><td>-</td><td>-</td><td>fractional value of the code phase measurement (0..1023)</td></tr><tr><td>60 + n·24</td><td>U4</td><td>codePhase</td><td>2^-21</td><td>ms</td><td>Code phase</td></tr><tr><td>64 + n·24</td><td>U1</td><td>intCodePhase</td><td>-</td><td>ms</td><td>Integer (part of the) code phase</td></tr><tr><td>65 + n·24</td><td>U1</td><td>pseuRangeRMS Err</td><td>-</td><td>-</td><td>pseudorange RMS error index (according to [1]) (0..63)</td></tr><tr><td>66 + n·24</td><td>U1[2]</td><td>reserved4</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td colspan="6">End of repeated group (numSV times)</td></tr></table>

## 3.15.2 UBX-RXM-PMREQ (0x02 0x41)

## 3.15.2.1 Power management request

<table><tbody><tr><td rowspan="2">Message</td><td rowspan="2">UBX-RXM-PMREQ Powermanagementrequest</td></tr><tr></tr><tr><td>Type</td><td>Command</td></tr><tr><td>Comment</td><td>Thismessagerequestsapowermanagementrelatedtaskofthereceiver.</td></tr><tr><td>Message</td><td>Header Class ID Length (Bytes) Payload Checksum</td></tr><tr><td>structure</td><td>0xb5 0x62 0x02 0x41 16 see below CK_ACK_B</td></tr></tbody></table>

Payload description:

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 120 of 181

<!-- page 121 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Byte offset</td><td>Type</td><td>Name</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>0</td><td>U1</td><td>version</td><td>-</td><td>-</td><td>Message version (0x00 for this version)</td></tr><tr><td>1</td><td>U1[3]</td><td>reserved0</td><td>-</td><td>-</td><td>Reserved</td></tr><tr><td>4</td><td>U4</td><td>duration</td><td>-</td><td>ms</td><td>Duration of the requested task. The maximum supported value is 12 days. Set to 0 to wait for a wakeup signal on a pin</td></tr><tr><td rowspan="3">8</td><td>X4</td><td>flags</td><td>-</td><td>-</td><td>task flags</td></tr><tr><td>bit 1</td><td>U:1</td><td>backup</td><td>-</td><td>-</td></tr><tr><td>bit 2</td><td>U:1</td><td>force</td><td>-</td><td>-</td></tr><tr><td rowspan="5">12</td><td>X4</td><td>wakeupSources</td><td>-</td><td>-</td><td>Configure pins to wake up the receiver. The receiver wakes up if there is either a falling or a rising edge on one of the configured pins.</td></tr><tr><td>bit 3</td><td>U:1</td><td>uartrx</td><td>-</td><td>-</td></tr><tr><td>bit 5</td><td>U:1</td><td>extint0</td><td>-</td><td>-</td></tr><tr><td>bit 6</td><td>U:1</td><td>extint1</td><td>-</td><td>-</td></tr><tr><td>bit 7</td><td>U:1</td><td>spics</td><td>-</td><td>-</td></tr></table>

## 3.15.3 UBX-RXM-RLM (0x02 0x59)

## 3.15.3.1 Galileo SAR short-RLM report

<table><tr><td>Message</td><td colspan="7">UBX-RXM-RLM Galileo SAR short-RLM report</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">This message contains the contents of any Galileo Search and Rescue (SAR) Short Return Link Message detected by the receiver.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x02</td><td>0x59</td><td>16</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x01 for Short-RLM)</td></tr><tr><td>2</td><td>U1</td><td colspan="2">svId</td><td>-</td><td>-</td><td colspan="2">Identifier of transmitting satellite (see Satellite Numbering)</td></tr><tr><td>3</td><td>U1</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>U1[8]</td><td colspan="2">beacon</td><td>-</td><td>-</td><td colspan="2">Beacon identifier (60 bits), with bytes ordered by earliest transmitted (most significant) first. Top four bits of first byte are zero.</td></tr><tr><td>12</td><td>U1</td><td colspan="2">message</td><td>-</td><td>-</td><td colspan="2">Message code (4 bits)</td></tr><tr><td>13</td><td>U1[2]</td><td colspan="2">params</td><td>-</td><td>-</td><td colspan="2">Parameters (16 bits), with bytes ordered by earliest transmitted (most significant) first.</td></tr><tr><td>15</td><td>U1</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 121 of 181

<!-- page 122 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.15.3.2 Galileo SAR long-RLM report

<table><tr><td>Message</td><td colspan="7">UBX-RXM-RLM Galileo SAR long-RLM report</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">This message contains the contents of any Galileo Search and Rescue (SAR) Long Return Link Message detected by the receiver.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x02</td><td>0x59</td><td>28</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">type</td><td>-</td><td>-</td><td colspan="2">Message type (0x02 for Long-RLM)</td></tr><tr><td>2</td><td>U1</td><td colspan="2">svId</td><td>-</td><td>-</td><td colspan="2">Identifier of transmitting satellite (see Satellite Numbering)</td></tr><tr><td>3</td><td>U1</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>U1[8]</td><td colspan="2">beacon</td><td>-</td><td>-</td><td colspan="2">Beacon identifier (60 bits), with bytes ordered by earliest transmitted (most significant) first. Top four bits of first byte are zero.</td></tr><tr><td>12</td><td>U1</td><td colspan="2">message</td><td>-</td><td>-</td><td colspan="2">Message code (4 bits)</td></tr><tr><td>13</td><td>U1[12]</td><td colspan="2">params</td><td>-</td><td>-</td><td colspan="2">Parameters (96 bits), with bytes ordered by earliest transmitted (most significant) first.</td></tr><tr><td>25</td><td>U1[3]</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

## 3.15.4 UBX-RXM-SFRBX (0x02 0x13)

3.15.4.1 Broadcast navigation data subframe

<table><tr><td>Message</td><td colspan="7">UBX-RXM-SFRBX Broadcast navigation data subframe</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">This message reports a complete subframe of broadcast navigation data decoded from a single signal. The number of data words reported in each message depends on the nature of the signal.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x02</td><td>0x13</td><td colspan="2">8 + numWords·4</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>gnssId</td><td></td><td>-</td><td>-</td><td>GNSS identifier (see Satellite Numbering)</td><td></td></tr><tr><td>1</td><td>U1</td><td>svId</td><td></td><td>-</td><td>-</td><td>Satellite identifier (see Satellite Numbering)</td><td></td></tr><tr><td>2</td><td>U1</td><td>sigId</td><td></td><td>-</td><td>-</td><td>Signal identifier (see Signal Identifiers)</td><td></td></tr><tr><td>3</td><td>U1</td><td>freqId</td><td></td><td>-</td><td>-</td><td>Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13)</td><td></td></tr><tr><td>4</td><td>U1</td><td>numWords</td><td></td><td>-</td><td>-</td><td>The number of data words contained in this message (up to 10, for currently supported signals)</td><td></td></tr><tr><td>5</td><td>U1</td><td>chn</td><td></td><td>-</td><td>-</td><td>The tracking channel number the message was received on</td><td></td></tr><tr><td>6</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version, (0x02 for this version)</td><td></td></tr><tr><td>7</td><td>U1</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr></table>

Start of repeated group (numWords times)

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 122 of 181

<!-- page 123 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

The data words

End of repeated group (numWords times)

## 3.16 UBX-SEC (0x27)

The messages in the UBX-SEC class are used for security features of the receiver.

## 3.16.1 UBX-SEC-SIG (0x27 0x09)

## 3.16.1.1 Signal security information

<table><tr><td>Message</td><td colspan="7">UBX-SEC-SIG Signal security information</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">Information related to the security, i.e. availability and integrity, of the signals.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x27</td><td>0x09</td><td>12</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x01 for this version)</td></tr><tr><td>1</td><td>U1[3]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>4</td><td>X1</td><td colspan="2">jamFlags</td><td>-</td><td>-</td><td colspan="2">Information related to jamming/interference</td></tr><tr><td>bit 0</td><td>U:1</td><td colspan="2">jamDetEnabled</td><td>-</td><td>-</td><td colspan="2">Flag indicates whether jamming/interference detection is enabled</td></tr><tr><td>bits 2...1</td><td>U:2</td><td colspan="2">jammingState</td><td>-</td><td>-</td><td colspan="2">Jamming/interference state0: Unknown1: No jamming indicated2: Warning; jamming indicated but fix OK3: Critical; jamming indicated and no fix</td></tr><tr><td>5</td><td>U1[3]</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td>8</td><td>X1</td><td colspan="2">spfFlags</td><td>-</td><td>-</td><td colspan="2">Information related to GNSS spoofing</td></tr><tr><td>bit 0</td><td>U:1</td><td colspan="2">spfDetEnabled</td><td>-</td><td>-</td><td colspan="2">Flag indicates whether spoofing detection is enabled</td></tr><tr><td>bits 3...1</td><td>U:3</td><td colspan="2">spoofingState</td><td>-</td><td>-</td><td colspan="2">Spoofing state0: Unknown1: No spoofing indicated2: Spoofing indicated3: Spoofing affirmedNote that the spoofing state value only reflects the detector state for the current navigation epoch. i.e. a value of 1: No spoofing indicated does not mean that the receiver is not spoofed, it simply states that the detector was not triggered in this epoch.</td></tr><tr><td>9</td><td>U1[3]</td><td colspan="2">reserved2</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

## 3.16.2 UBX-SEC-SIGLOG (0x27 0x10)

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 123 of 181

<!-- page 124 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

3.16.2.1 Signal security log

<table><tr><td>Message</td><td colspan="7">UBX-SEC-SIGLOGSignal security log</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message provides a log of past signal security related events, that is, events related to jamming and spoofing. Each event is a combination of a detection type and a event type, where the event type &#x27;indication started&#x27; and &#x27;indication stopped&#x27; and also the event type &#x27;indication triggered&#x27; and &#x27;indication timed-out&#x27; form a pair. A maximum of 16 events are logged; after the log is filled, recent events take precedence over past events in the log. Power cycles and restarts of the receiver reset the log, deleting its content.Note: It is advised not to restart the receiver while it&#x27;s indicating spoofing.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td colspan="2">Length (Bytes)</td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x27</td><td>0x10</td><td colspan="2">8 + numEvents·8</td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U1</td><td colspan="2">version</td><td>-</td><td>-</td><td colspan="2">Message version (0x00 for this version)</td></tr><tr><td>1</td><td>U1</td><td colspan="2">numEvents</td><td>-</td><td>-</td><td colspan="2">Number of events</td></tr><tr><td>2</td><td>U1[6]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr><tr><td colspan="8">Start of repeated group (numEvents times)</td></tr><tr><td>8 + n·8</td><td>U4</td><td colspan="2">timeElapsed</td><td>-</td><td>s</td><td colspan="2">Seconds elapsed since this eventSpecial value 0xFFFFFF: more than 45 days</td></tr><tr><td>12 + n·8</td><td>U1</td><td colspan="2">detectionType</td><td>-</td><td>-</td><td colspan="2">Type of the spoofing or jamming detection:0 = simulated signal1 = abnormal signal2 = INS/GNSS mismatch3 = abrupt changes in GNSS signal4 = broadband jamming/interference (deprecated)5 = narrowband jamming/interference (deprecated)</td></tr><tr><td>13 + n·8</td><td>U1</td><td colspan="2">eventType</td><td>-</td><td>-</td><td colspan="2">Type of the event:0 = indication started1 = indication stopped2 = indication triggered3 = indication timed-outNote: Single epoch events, caused by abrupt changes due to switching from the real to the spoofing signal or vice versa are handled as time-out events. This means that the time-out event is reported after a certain cool off period which is not related to any observations in the signal. The other detection types make use of &#x27;start&#x27; and &#x27;stop&#x27; event types.</td></tr><tr><td>14 + n·8</td><td>U1[2]</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td colspan="2">Reserved</td></tr></table>

End of repeated group (numEvents times)

## 3.16.3 UBX-SEC-UNIQID (0x27 0x03)

3.16.3.1 Unique chip ID

| Message | UBX-SEC-UNIQID Unique chip ID |
| --- | --- |
| Type | Output |

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 124 of 181

<!-- page 125 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Comment</td><td colspan="7">This message is used to retrieve a unique chip identifier (40 bits, 5 bytes).</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x27</td><td>0x03</td><td>9</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>version</td><td></td><td>-</td><td>-</td><td>Message version (0x01 for this version)</td><td></td></tr><tr><td>1</td><td>U1[3]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td>4</td><td>U1[5]</td><td>uniqueId</td><td></td><td>-</td><td>-</td><td>Unique chip ID</td><td></td></tr></table>

## 3.17 UBX-TIM (0x0d)

The messages in the UBX-TIM class are used to output timing information from the receiver, such as time pulse and time mark measurements.

## 3.17.1 UBX-TIM-TM2 (0x0d 0x03)

## 3.17.1.1 Time mark data

<table><tr><td>Message</td><td colspan="7">UBX-TIM-TM2Time mark data</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message contains information for high precision time stamping / pulse counting.The delay figures and timebase given in CFG-TP Configuration Items are also applied to the time results output in this message.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x0d</td><td>0x03</td><td>28</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>ch</td><td></td><td>-</td><td>-</td><td colspan="2">Channel (i.e. EXTINT) upon which the pulse was measured</td></tr><tr><td>1</td><td>X1</td><td>flags</td><td></td><td>-</td><td>-</td><td colspan="2">Bitmask</td></tr><tr><td>bit 0</td><td>U:1</td><td>mode</td><td></td><td>-</td><td>-</td><td colspan="2">0=single1=running</td></tr><tr><td>bit 1</td><td>U:1</td><td>run</td><td></td><td>-</td><td>-</td><td colspan="2">0=armed1=stopped</td></tr><tr><td>bit 2</td><td>U:1</td><td>newFallingEdge</td><td></td><td>-</td><td>-</td><td colspan="2">New falling edge detected</td></tr><tr><td>bits 4...3</td><td>U:2</td><td>timeBase</td><td></td><td>-</td><td>-</td><td colspan="2">0=Time base is Receiver time1=Time base is GNSS time (the system according to the configuration in CFG-TP Configuration Items for tpIdx=0)2=Time base is UTC (the variant according to the configuration in CFG-NAVSPG-* configuration items)</td></tr><tr><td>bit 5</td><td>U:1</td><td>utc</td><td></td><td>-</td><td>-</td><td colspan="2">0=UTC not available1=UTC available</td></tr><tr><td>bit 6</td><td>U:1</td><td>time</td><td></td><td>-</td><td>-</td><td colspan="2">0=Time is not valid</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 125 of 181

<!-- page 126 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

• 1=Time is valid (Valid GNSS fix)

|  | bit 7 <sup>U</sup>:1 | newRisingEdge | - | - | New rising edge detected |
| --- | --- | --- | --- | --- | --- |
| 2 | U2 | count | - | - | Rising edge counter |
| 4 | U2 | wnR | - | - | Week number of last rising edge |
| 6 | U2 | wnF | - | - | Week number of last falling edge |
| 8 | U4 | towMsR | - | ms | Tow of rising edge |
| 12 | U4 | towSubMsR | - | ns | Millisecond fraction of tow of rising edge in nanoseconds |
| 16 | U4 | towMsF | - | ms | Tow of falling edge |
| 20 | U4 | towSubMsF | - | ns | Millisecond fraction of tow of falling edge in nanoseconds |
| 24 | U4 | accEst | - | ns | Accuracy estimate |

## 3.17.2 UBX-TIM-TP (0x0d 0x01)

## 3.17.2.1 Time pulse time data

<table><tr><td rowspan="2">Message</td><td colspan="7">UBX-TIM-TP</td></tr><tr><td colspan="7">Time pulse time data</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message contains information on the timing of the next pulse at the TIMEPULSE0 output. The recommended configuration when using this message is to set both the measurement rate (CFG-RATE) and the timepulse frequency (CFG-TP) to 1 Hz.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x0d</td><td>0x01</td><td>16</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td colspan="2">Description</td></tr><tr><td>0</td><td>U4</td><td colspan="2">towMS</td><td>-</td><td>ms</td><td colspan="2">Time pulse time of week according to time base</td></tr><tr><td>4</td><td>U4</td><td colspan="2">towSubMS</td><td>2^-32</td><td>ms</td><td colspan="2">Submillisecond part of towMS</td></tr><tr><td>8</td><td>I4</td><td colspan="2">qErr</td><td>-</td><td>ps</td><td colspan="2">Quantization error of time pulse</td></tr><tr><td>12</td><td>U2</td><td colspan="2">week</td><td>-</td><td>weeks</td><td colspan="2">Time pulse week number according to time base</td></tr><tr><td>14</td><td>X1</td><td colspan="2">flags</td><td>-</td><td>-</td><td colspan="2">Flags</td></tr><tr><td>bit 0</td><td>U:1</td><td colspan="2">timeBase</td><td>-</td><td>-</td><td colspan="2">0 = Time base is GNSS1 = Time base is UTC</td></tr><tr><td>bit 1</td><td>U:1</td><td colspan="2">utc</td><td>-</td><td>-</td><td colspan="2">0 = UTC not available1 = UTC available</td></tr><tr><td>bits 3...2</td><td>U:2</td><td colspan="2">raim</td><td>-</td><td>-</td><td colspan="2">(T)RAIM information0 = Information not available1 = Not active2 = Active</td></tr><tr><td>bit 4</td><td>U:1</td><td colspan="2">qErrInvalid</td><td>-</td><td>-</td><td colspan="2">0 = Quantization error valid1 = Quantization error invalid</td></tr><tr><td>bit 5</td><td>U:1</td><td colspan="2">TpNotLocked</td><td>-</td><td>-</td><td colspan="2">0 = Next TP is locked to GNSS1 = Next TP is based on local time and not locked to GNSS - week/tow may be invalid</td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 126 of 181

<!-- page 127 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 15 | X1 | refInfo | - | - | Time reference information |
| --- | --- | --- | --- | --- | --- |
| bits 3...0 | U:4 | timeRefGnss | - | - | GNSS reference information. Only valid if time base is GNSS (timeBase=0).0 = GPS1 = GLONASS2 = BeiDou3 = Galileo4 = NavIC15 = Unknown |
| bits 7...4 | U:4 | utcStandard | - | - | UTC standard identifier. Only valid if time base is UTC (timeBase=1).0 = Information not available1 = Communications Research Laboratory (CRL), Tokyo, Japan2 = National Institute of Standards and Technology (NIST)3 = U.S. Naval Observatory (USNO)4 = International Bureau of Weights and Measures (BIPM)5 = European laboratories6 = Former Soviet Union (SU)7 = National Time Service Center (NTSC), China8 = National Physics Laboratory India (NPLI)15 = Unknown |

## 3.17.3 UBX-TIM-VRFY (0x0d 0x06)

## 3.17.3.1 Sourced time verification

<table><tr><td>Message</td><td colspan="7">UBX-TIM-VRFYSourced time verification</td></tr><tr><td>Type</td><td colspan="7">Periodic/polled</td></tr><tr><td>Comment</td><td colspan="7">This message contains verification information about previous time received via assistance data or from RTC.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x0d</td><td>0x06</td><td>20</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>I4</td><td>itow</td><td></td><td>-</td><td>ms</td><td>integer millisecond tow received by source</td><td></td></tr><tr><td>4</td><td>I4</td><td>frac</td><td></td><td>-</td><td>ns</td><td>sub-millisecond part of tow</td><td></td></tr><tr><td>8</td><td>I4</td><td>deltaMs</td><td></td><td>-</td><td>ms</td><td>integer milliseconds of delta time (current time minus sourced time)</td><td></td></tr><tr><td>12</td><td>I4</td><td>deltaNs</td><td></td><td>-</td><td>ns</td><td>Sub-millisecond part of delta time</td><td></td></tr><tr><td>16</td><td>U2</td><td>wno</td><td></td><td>-</td><td>week</td><td>Week number</td><td></td></tr><tr><td>18</td><td>X1</td><td>flags</td><td></td><td>-</td><td>-</td><td>Flags</td><td></td></tr><tr><td>bits 2...0</td><td>U:3</td><td>src</td><td></td><td>-</td><td>-</td><td>Aiding time source• 0 = no time aiding done</td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 127 of 181

<!-- page 128 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td colspan="5"></td><td>• 3 = source was assistance data</td></tr><tr><td>19</td><td>U1</td><td>reserved0</td><td>-</td><td>-</td><td>Reserved</td></tr></table>

## 3.18 UBX-UPD (0x09)

The messages in the UBX-UPD class are used to download a firmware to the receiver and to update the firmware on the flash.

## 3.18.1 UBX-UPD-SOS (0x09 0x14)

3.18.1.1 Poll backup restore status

| Message | UBX-UPD-SOS Poll backup restore status |
| --- | --- |
| Type | Poll request |
| Comment | Sendingthis(empty)messagetothereceiverresultsinthereceiverreturningaSystem restored from backup message as defined below. |
| Message | Header Class ID Length (Bytes) Payload Checksum |
| structure | 0xb5 0x62 0x09 0x14 0 see below CK_A CK_B |
| Payload | This message has no payload. |

3.18.1.2 Create backup in flash

<table><tr><td>Message</td><td colspan="7">UBX-UPD-SOSCreate backup in flash</td></tr><tr><td>Type</td><td colspan="7">Command</td></tr><tr><td>Comment</td><td colspan="7">The host can send this message in order to save part of the battery-backed memory (BBR) in a file in the flash file system. The feature is designed in order to emulate the presence of the backup battery even if it is not present; the host can issue the save on shutdown command before switching off the device supply. It is recommended to issue a GNSS stop command using UBX-CFG-RST before in order to keep the BBR memory content consistent.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x09</td><td>0x14</td><td>4</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>cmd</td><td></td><td>-</td><td>-</td><td>Command (must be 0)</td><td></td></tr><tr><td>1</td><td>U1[3]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr></table>

3.18.1.3 Clear backup in flash

<table><tr><td>Message</td><td colspan="8">UBX-UPD-SOSClear backup in flash</td></tr><tr><td>Type</td><td colspan="8">Command</td></tr><tr><td>Comment</td><td colspan="8">The host can send this message in order to erase the backup file present in flash. It is recommended that the clear operation is issued after the host has received the notification that the memory has been restored after a reset. Alternatively the host can parse the startup string Restored data saved on shutdown or poll the UBX-UPD-SOS message for obtaining the status.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x09</td><td>0x14</td><td>4</td><td></td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="9">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 128 of 181

<!-- page 129 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| 0 | U1 | cmd | - | - | Command (must be 1) |
| --- | --- | --- | --- | --- | --- |
| 1 | U1[3] | reserved0 | - | - | Reserved |

3.18.1.4 Backup creation acknowledge

<table><tr><td>Message</td><td colspan="7">UBX-UPD-SOS Backup creation acknowledge</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">The message is sent from the device as confirmation of creation of a backup file in flash. The host can safely shut down the device after having received this message.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x09</td><td>0x14</td><td>8</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td colspan="2">Name</td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td colspan="2">cmd</td><td>-</td><td>-</td><td>Command (must be 2)</td><td></td></tr><tr><td>1</td><td>U1[3]</td><td colspan="2">reserved0</td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td>4</td><td>U1</td><td colspan="2">response</td><td>-</td><td>-</td><td>0 = Not acknowledged1 = Acknowledged</td><td></td></tr><tr><td>5</td><td>U1[3]</td><td colspan="2">reserved1</td><td>-</td><td>-</td><td>Reserved</td><td></td></tr></table>

3.18.1.5 System restored from backup

<table><tr><td>Message</td><td colspan="7">UBX-UPD-SOSSystem restored from backup</td></tr><tr><td>Type</td><td colspan="7">Output</td></tr><tr><td>Comment</td><td colspan="7">The message is sent from the device to notify the host the BBR has been restored from a backup file in the flash file system. The host should clear the backup file after receiving this message. If the UBX-UPD-SOS message is polled, this message is resent.</td></tr><tr><td rowspan="2">Message structure</td><td>Header</td><td>Class</td><td>ID</td><td>Length (Bytes)</td><td></td><td>Payload</td><td>Checksum</td></tr><tr><td>0xb5 0x62</td><td>0x09</td><td>0x14</td><td>8</td><td></td><td>see below</td><td>CK_A CK_B</td></tr><tr><td colspan="8">Payload description:</td></tr><tr><td>Byte offset</td><td>Type</td><td>Name</td><td></td><td>Scale</td><td>Unit</td><td>Description</td><td></td></tr><tr><td>0</td><td>U1</td><td>cmd</td><td></td><td>-</td><td>-</td><td>Command (must be 3)</td><td></td></tr><tr><td>1</td><td>U1[3]</td><td>reserved0</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr><tr><td>4</td><td>U1</td><td>response</td><td></td><td>-</td><td>-</td><td>0 = Unknown1 = Failed restoring from backup2 = Restored from backup3 = Not restored (no backup)</td><td></td></tr><tr><td>5</td><td>U1[3]</td><td>reserved1</td><td></td><td>-</td><td>-</td><td>Reserved</td><td></td></tr></table>

UBX-23002975 - R02 C1-Public

3 UBX protocol

Page 129 of 181

<!-- page 130 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## 4 Configuration interface

This chapter describes the receiver configuration interface.

## 4.1 Configuration database

The configuration database in the receiver's RAM holds the current configuration, which is used by the receiver at run-time. It is constructed on startup of the receiver from several sources of configuration. These sources are called Configuration Layers. The current configuration is called the RAM Layer. Any configuration in any layer is organized as Configuration Items, where each Configuration Item is referenced to by a unique Configuration Key ID and holds a single Configuration Value.

## 4.2 Configuration items

The following figure shows the structure of a Configuration Item, which consists of a (Configuration) Key ID and its (Configuration) Value:

![Image block](doc:a53f2ec/tier:standard/page:130/block:9)

reserved (1 + 4 + 4 = 9 bits)

Configuration Value: 1 byte (1bit), 2 bytes, 4 bytes or 8 bytes

| size 0x01 | size 0x03 | size 0x04 | size 0x05 |
| --- | --- | --- | --- |
| 1 byte | 2 bytes | 4 bytes | 8 bytes |

A Configuration Key ID is a 32-bit integer value, which is split into the following parts:

• Bit 31: Currently unused. Reserved for future use.

Bits 30…28: Three bits that indicate the storage size of a Configuration Value (range 0x01-0x05, see below)

• Bits 27…24: Currently unused. Reserved for future use.

• Bits 23…16: Eight bits that define a unique group ID (range 0x01-0xfe)

• Bits 15…12: Currently unused. Reserved for future use.

• Bits 11…0: Twelve bits that define a unique item ID within a group (range 0x001-0xffe)

The entire 32-bit value is the unique Key ID, which uniquely identifies a particular item. The numeric representation of the Key ID uses the lower-case hexadecimal format, such as 0x20c400a1. An easier, more readable text representation uses the form CFG-GROUP-ITEM. This is also referred to as the (Configuration) Key Name.

Supported storage size identifiers (bits 30…28 of the Key ID) are:

• 0x01: one bit (the actual storage used is one byte, but only the least significant bit is used)

• 0x02: one byte

• 0x03: two bytes

• 0x04: four bytes

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 130 of 181

<!-- page 131 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## • 0x05: eight bytes

Each Configuration Item is of a certain type, which defines the interpretation of the raw binary data (see also UBX data types):

• U1, U2, U4, U8: unsigned little-endian integers of 8-, 16-, 32- and 64-bit widths

• I1, I2, I4, I8: signed little-endian, two's complement integers of 8-, 16-, 32- and 64-bit widths

• R4, R8: IEEE 754 single (32-bit) and double (64-bit) precision floats

• E1, E2, E4: unsigned little-endian enumeration of 8-, 16-, and 32-bit widths

• X1, X2, X4, X8: unsigned little-endian integers of 8-, 16-, 32- and 64-bit widths for bitfields and other binary data, such as strings

• L: single-bit boolean (true = 1, false = 0), stored as U1

## 4.3 Configuration layers

The receiver has several Configuration Layers. They are separate sources of Configuration Items. Some of the layers are read-only and others are modifiable. Layers are organized in terms of priority. Values in a high-priority layer replace values stored in a low-priority layer. At startup, the receiver reads all configuration layers and stacks up the items to create the Current Configuration, which is used by the receiver at run-time.

The following configuration layers are available (in order of priority, highest priority first):

RAM: This layer contains items stored in volatile RAM. This is the Current Configuration. The value of any item can be set by the user at run-time (see UBX protocol interface) and it is effective immediately.

**BBR**: This layer contains items stored in the battery-backed RAM. The contents in this layer are preserved as long as a battery backup supply is provided during off periods. The value of any item can be set by the user at run-time (see UBX protocol interface) and it becomes effective when the receiver is restarted.

**Flash**: This layer contains items stored permanently in the external flash memory. This layer is only available if there is a usable external flash memory. The value of any item can be set by the user at run-time (see UBX protocol interface) and it becomes effective when the receiver is restarted.

• **Default:** This layer contains all items known to the running receiver software and their hardcoded default values. Data in this layer is not writable.

The stacking of the configuration items from the different layers (sources) in order to construct the Current Configuration in the RAM Layer is depicted in the following figure. For each defined item, i.e. for each item in the Default Layer, the receiver software goes through the layers above and stacks all the found items on top. Some items may not be present in every layer. The result is the RAM Layer filled with all configuration items given Configuration Values coming from the highest priority layer the corresponding item was present. In the example figure below bold text indicates the source of the value in the Current Configuration (the RAM Layer). Empty boxes mean that the layer can hold the item but that it is not currently stored there. Boxes with text mean that an item is currently stored in the layer.

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 131 of 181

<!-- page 132 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

![Chart block](doc:a53f2ec/tier:standard/page:132/block:3)

Configuration Items

In the example figure above several items (e.g. the first item) are only set in the Default Layer and hence the default value ends up in Current Configuration in the RAM Layer. The third item is present in the Default, Flash and BBR Layers. The value from the BBR Layer has the highest priority and therefore it ends up in the RAM Layer. On the other hand, the default value of the sixth item is changed by the value in the Flash Layer. The value of the last item is changed in the RAM Layer only, i.e. upon startup the value in the RAM Layer was the value from the Default Layer, but the user has changed the value in the RAM Layer at run-time.

## 4.4 Configuration interface access

The following sections describe the existing interfaces to access the Configuration Database.

## 4.4.1 UBX protocol interface

The following UBX protocol messages are available to access the Configuration Database:

• UBX-CFG-VALGET to read configuration items from the database

• UBX-CFG-VALSET to set configuration items in the database

UBX-CFG-VALDEL to delete configuration items from the database

## 4.5 Configuration data

Configuration data is the binary representation of a list of Key ID and Value pairs. It is formed by concatenating keys (U4 values) and values (variable type) without any padding. This format is used in the UBX-CFG-VALSET and UBX-CFG-VALGET messages.

The figure below shows an example. The four Items (Key ID - Value pairs) on the left use the four fundamental storage sizes: one byte (L, U1, I1, E1 and X1 types), 2 bytes (U2, I2, E2 and X2 types), four byte (U4, I4, E4, X4 and R4 types) and eight bytes (U8, I8, X8 and R8 types). When concatenated (right) the Key IDs and Values are not aligned and there is no padding.

Key ID: 4 bytes Value: 1, 2, 4 or 8 bytes

<table><tr><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td><td>7</td><td>8</td><td>9</td><td>10</td><td>11</td><td>12</td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td colspan="4">Key ID</td><td>V.</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td colspan="4">Key ID</td><td>Value</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td colspan="4">Key ID</td><td colspan="4">Value</td><td></td><td></td><td></td><td></td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td colspan="4">Key ID</td><td colspan="8">Value</td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr></table>

$$
4 + 1 = 5 \text {bytes}
$$

$$
4 + 2 = 6 \text {bytes}
$$

$$
4 + 4 = 8 \text {bytes}
$$

<table><tr><td></td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td><td>7</td><td>8</td><td>9</td><td>10</td></tr><tr><td>0</td><td colspan="4">Key ID</td><td>V.</td><td colspan="4">Key ID</td><td>V.</td></tr><tr><td>10</td><td></td><td colspan="4">Key ID</td><td colspan="4">Value</td><td>K.</td></tr><tr><td>20</td><td colspan="3">ID</td><td colspan="7">Va-</td></tr><tr><td>30</td><td>Iue</td><td colspan="9"></td></tr></table>

Total 31 bytes

$$
4 + 8 = 1 2 \text {bytes}
$$

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 132 of 181

<!-- page 133 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

Note that this is an arbitrary example and any number of items of any value storage size can be concatenated the same way.

## 4.6 Configuration transactions

The configuration interface supports two mechanisms of configuration: the first is a transactionless mechanism where sent configuration changes are applied immediately to the configuration layer(s) requested. The second mechanism is a configuration transaction.

A transaction offers a way of queuing multiple configuration changes. It is particularly useful where different configuration keys depend on each other in such a way that sending one before the other can cause the configuration to be rejected. The queued configuration change requests are stored then checked collectively before being applied to the receiver.

A transaction can have the following states described in the figure below.

![Image block](doc:a53f2ec/tier:standard/page:133/block:8)

When starting a transaction, specify the layer(s) to apply the changes to. This list of configuration layer(s) must be observed throughout the transaction states. Modifying the configuration layer(s) mid-transaction causes the transaction to be aborted and consequently, no queued changes will be applied.

In the start transaction state, the receiver locks the configuration database so that changes from another entity or message cannot be applied. It is possible to send a configuration key-value pairs with the start transaction state. These are queued waiting to be applied.

In the ongoing state, a configuration key and value must be sent. The receiver aborts the transaction and does not apply any changes if this condition is violated. Key-value pairs sent in the ongoing state are queued waiting to be applied.

In the apply state, the receiver collectively checkes the queued changes and applied them to the requested configuration layer(s). Note that any additional key-value pairs sent within the apply state are ignored.

Note that a transaction can only come from a single source, a UBX-CFG-VALSET message or a UBX-CFG-VALDEL message. This means that in any given transaction it is not possible to mix a delete

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 133 of 181

<!-- page 134 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

and a save request. Starting a transaction from a different source aborts the current transaction and the queued changes are not applied.

Refer to UBX-CFG-VALSET and UBX-CFG-VALDEL messages for a detailed description of how to set up a configuration transaction, its limitations and conditions that would cause the transaction to be rejected.

## 4.7 Configuration reset behavior

The RAM layer is always rebuilt from the layers below when the chip's processor comes out from reset. When using UBX-CFG-RST the processor goes through a reset cycle with these reset types (resetMode field):

0x00 hardware reset (watchdog) immediately

0x01 controlled software reset

0x04 hardware reset (watchdog) after shutdown

See section Forcing a receiver reset in the integration manual.

4.8 Configuration overview

| Group | Description |
| --- | --- |
| CFG-ANA | AssistNow Autonomous and Offline configuration |
| CFG-BDS | BeiDou system configuration |
| CFG-HW | Hardware configuration |
| CFG-I2C | Configuration of the I2C interface |
| CFG-I2CINPROT | Input protocol configuration of the I2C interface |
| CFG-I2COUTPROT | Output protocol configuration of the I2C interface |
| CFG-INFMSG | Information message configuration |
| CFG-ITFM | Jamming and interference monitor configuration |
| CFG-MOT | Motion detector configuration |
| CFG-MSGOUT | Message output configuration |
| CFG-NAVMASK | Satellite Mask Configuration |
| CFG-NAVSPG | Standard precision navigation configuration |
| CFG-NMEA | NMEA protocol configuration |
| CFG-ODO | Odometer and low-speed course over ground filter configuration |
| CFG-QZSS | QZSS system configuration |
| CFG-RATE | Navigation and measurement rate configuration |
| CFG-RINV | Remote inventory |
| CFG-SBAS | SBAS configuration |
| CFG-SEC | Security configuration |
| CFG-SIGNAL | Satellite systems (GNSS) signal configuration |
| CFG-SPI | Configuration of the SPI interface |
| CFG-SPIINPROT | Input protocol configuration of the SPI interface |
| CFG-SPIOUTPROT | Output protocol configuration of the SPI interface |
| CFG-TP | Time pulse configuration |
| CFG-TXREADY | TX ready configuration |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 134 of 181

<!-- page 135 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Group | Description |
| --- | --- |
| CFG-UART1 | Configuration of the UART1 interface |
| CFG-UART1INPROT | Input protocol configuration of the UART1 interface |
| CFG-UART1OUTPROT | Output protocol configuration of the UART1 interface |

## 4.9 Configuration reference

## 4.9.1 CFG-ANA: AssistNow Autonomous and Offline configuration

Configuration for the AssistNow Autonomous feature. See section AssistNow Autonomous in the integration manual for feature details.

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-ANA-USE_ANA</td><td>0x10230001</td><td>L</td><td>-</td><td>-</td><td>Use AssistNow Autonomous</td></tr><tr><td>CFG-ANA-ORBMAXERR</td><td>0x30230002</td><td>U2</td><td>-</td><td>m</td><td>Maximum acceptable (modeled) orbit error</td></tr><tr><td colspan="6">Range is from 5 to 1000.</td></tr></table>

Table 5: CFG-ANA configuration items

## 4.9.2 CFG-BDS: BeiDou system configuration

Note that enabling and disabling of individual GNSS is done via the CFG-SIGNAL configuration group.

<table><tbody><tr><td>Configurationitem</td><td>KeyID</td><td colspan="2">Type Scale</td><td colspan="2">Unit Description</td></tr><tr><td>CFG-BDS-USE_GEO_PRN</td><td>0x10340014</td><td>L</td><td>-</td><td>-</td><td>UseBeiDougeostationarysatellites(PRN1-5 and 59-63)</td></tr></tbody></table>

Table 6: CFG-BDS configuration items

## 4.9.3 CFG-HW: Hardware configuration

Hardware configuration settings.

Note that not all settings are available for all products. See the applicable data sheet for supported features.

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-HW-ANT_CFG_VOLTCTRL</td><td>0x10a3002e</td><td>L</td><td>-</td><td>-</td><td>Active antenna voltage control flag</td></tr><tr><td colspan="6">Enable active antenna voltage control flag. Used by EXT and MADC engines.</td></tr><tr><td>CFG-HW-ANT_CFG_SHORTDET</td><td>0x10a3002f</td><td>L</td><td>-</td><td>-</td><td>Short antenna detection flag</td></tr><tr><td colspan="6">Enable short antenna detection flag. Used by EXT and MADC engines.</td></tr><tr><td>CFG-HW-ANT_CFG_SHORTDET_POL</td><td>0x10a30030</td><td>L</td><td>-</td><td>-</td><td>Short antenna detection polarity</td></tr><tr><td colspan="6">Set to true if polarity of the antenna short detection is active low. Used by EXT engine.</td></tr><tr><td>CFG-HW-ANT_CFG_OPENDET</td><td>0x10a30031</td><td>L</td><td>-</td><td>-</td><td>Open antenna detection flag</td></tr><tr><td colspan="6">Enable open antenna detection flag. Used by EXT and MADC engines.</td></tr><tr><td>CFG-HW-ANT_CFG_OPENDET_POL</td><td>0x10a30032</td><td>L</td><td>-</td><td>-</td><td>Open antenna detection polarity</td></tr><tr><td colspan="6">Set to true if polarity of the antenna open detection is active low. Used by EXT engine.</td></tr><tr><td>CFG-HW-ANT_CFG_PWRDOWN</td><td>0x10a30033</td><td>L</td><td>-</td><td>-</td><td>Power down antenna flag</td></tr><tr><td colspan="6">Enable power down antenna logic in the event of antenna short circuit. CFG-HW-ANT_CFG_SHORTDET must be enabled to use this feature. Used by EXT and MADC engines.</td></tr><tr><td>CFG-HW-ANT_CFG_PWRDOWN_POL</td><td>0x10a30034</td><td>L</td><td>-</td><td>-</td><td>Power down antenna logic polarity</td></tr><tr><td colspan="6">Set to true if polarity of the antenna power down logic is active high. Used by EXT and MADC engines.</td></tr></table>

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 135 of 181

<!-- page 136 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-HW-ANT_CFG_RECOVER</td><td>0x10a30035</td><td>L</td><td>-</td><td>-</td><td>Automatic recovery from short state flag</td></tr><tr><td colspan="6">Enable automatic recovery from short state. Used by EXT and MADC engines.</td></tr><tr><td>CFG-HW-ANT_SUP_SWITCH_PIN</td><td>0x20a30036</td><td>U1</td><td>-</td><td>-</td><td>Antenna switch PIO number</td></tr><tr><td colspan="6">Antenna switch PIO number. Used by EXT and MADC engines.</td></tr><tr><td>CFG-HW-ANT_SUP_SHORT_PIN</td><td>0x20a30037</td><td>U1</td><td>-</td><td>-</td><td>Antenna short detection PIO number</td></tr><tr><td colspan="6">Antenna short detection PIO number. Used by EXT engine.</td></tr><tr><td>CFG-HW-ANT_SUP_OPEN_PIN</td><td>0x20a30038</td><td>U1</td><td>-</td><td>-</td><td>Antenna open detection PIO number</td></tr><tr><td colspan="6">Antenna open detection PIO number. Used by EXT engine.</td></tr><tr><td>CFG-HW-ANT_ON_SHORT_US</td><td>0x30a3003c</td><td>U2</td><td>-</td><td>-</td><td>ANT on-&gt;short timeout[us]</td></tr><tr><td colspan="6">Delay in microseconds between turning the antenna power supply on and enabling the antenna short circuit detection.</td></tr><tr><td>CFG-HW-ANT_SUP_ENGINE</td><td>0x20a30054</td><td>E1</td><td>-</td><td>-</td><td>Antenna supervisor engine selection</td></tr><tr><td colspan="6">Select the engine used to evaluate antenna state.</td></tr><tr><td colspan="6">The EXT engine uses an external comparator for current measurement. The MADC engine uses built-in measurement ADC and requires only a shunt resistor for current measurement. The MADC engine is supported only in selected u-blox generation 9 receivers.</td></tr><tr><td colspan="6">See Table 8 below for a list of possible constants for this item.</td></tr><tr><td>CFG-HW-ANT_SUP_SHORT_THR</td><td>0x20a30055</td><td>U1</td><td>-</td><td>mV</td><td>Antenna supervisor MADC engine short detection threshold</td></tr><tr><td colspan="6">Threshold above which antenna short is detected. Used by MADC engine.</td></tr><tr><td>CFG-HW-ANT_SUP_OPEN_THR</td><td>0x20a30056</td><td>U1</td><td>-</td><td>mV</td><td>Antenna supervisor MADC engine open detection threshold</td></tr><tr><td colspan="6">Threshold below which antenna open/disconnected is detected. Used by MADC engine.</td></tr><tr><td>CFG-HW-RF_LNA_MODE</td><td>0x20a30057</td><td>E1</td><td>-</td><td>-</td><td>Mode for internal LNA</td></tr><tr><td colspan="6">Sets the operating mode for the RF LNA. Lowgain or bypass options can be used if there is already a external LNA in front of the chip with sufficient gain.</td></tr><tr><td colspan="6">See Table 9 below for a list of possible constants for this item.</td></tr></table>

Table 7: CFG-HW configuration items

| Constant | Value | Description |
| --- | --- | --- |
| EXT | 0 | Use the EXT engine. |
| MADC | 1 | Use the MADC engine. |

Table 8: Constants for CFG-HW-ANT\_SUP\_ENGINE

| Constant | Value | Description |
| --- | --- | --- |
| NORMAL | 0 | All RFs. Normal operation, internal LNA enabled at full gain |
| LOWGAIN | 1 | All RFs. LNA enabled in low gain mode |
| BYPASS | 2 | All RFs. Bypass LNA |

Table 9: Constants for CFG-HW-RF\_LNA\_MODE

## 4.9.4 CFG-I2C: Configuration of the I2C interface

Settings needed to configure the I2C communication interface.

| Configuration item | Key ID | Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- | --- |
| CFG-I2C-ADDRESS | 0x20510001 | U1 | - | - | I2C address of the receiver (7 bits) |
| CFG-I2C-EXTENDEDTIMEOUT | 0x10510002 | L | - | - | Flag to disable timeouting the interface after 1.5 s |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 136 of 181

<!-- page 137 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tbody><tr><td>Configurationitem</td><td>KeyID</td><td colspan="2">Type Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-I2C-ENABLED</td><td>0x10510003</td><td>L</td><td>-</td><td>-</td><td>FlagtoindicateiftheI2Cinterfaceshouldbeenabled</td></tr></tbody></table>

Table 10: CFG-I2C configuration items

## 4.9.5 CFG-I2CINPROT: Input protocol configuration of the I2C interface Input protocol enable flags of the I2C interface.

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td colspan="2">Unit Description</td></tr><tr><td>CFG-I2CINPROT-UBX</td><td>0x10710001</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if UBX should be an input protocol on I2C</td></tr><tr><td>CFG-I2CINPROT-NMEA</td><td>0x10710002</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if NMEA should be an input protocol on I2C</td></tr></tbody></table>

Table 11: CFG-I2CINPROT configuration items

## 4.9.6 CFG-I2COUTPROT: Output protocol configuration of the I2C interface Output protocol enable flags of the I2C interface.

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td colspan="2">Unit Description</td></tr><tr><td>CFG-I2COUTPROT-UBX</td><td>0x10720001</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if UBX should be an output protocol on I2C</td></tr><tr><td>CFG-I2COUTPROT-NMEA</td><td>0x10720002</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if NMEA should be an output protocol on I2C</td></tr></tbody></table>

Table 12: CFG-I2COUTPROT configuration items

## 4.9.7 CFG-INFMSG: Information message configuration

Information message configuration for the NMEA and UBX protocols.

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-INFMSG-UBX_I2C</td><td>0x20920001</td><td>X1</td><td>-</td><td>-</td><td>Information message enable flags for the UBX protocol on the I2C interface</td></tr><tr><td colspan="6">See Table 14 below for a list of possible constants for this item.</td></tr><tr><td>CFG-INFMSG-UBX_UART1</td><td>0x20920002</td><td>X1</td><td>-</td><td>-</td><td>Information message enable flags for the UBX protocol on the UART1 interface</td></tr><tr><td colspan="6">See Table 14 below for a list of possible constants for this item.</td></tr><tr><td>CFG-INFMSG-UBX_SPI</td><td>0x20920005</td><td>X1</td><td>-</td><td>-</td><td>Information message enable flags for the UBX protocol on the SPI interface</td></tr><tr><td colspan="6">See Table 14 below for a list of possible constants for this item.</td></tr><tr><td>CFG-INFMSG-NMEA_I2C</td><td>0x20920006</td><td>X1</td><td>-</td><td>-</td><td>Information message enable flags for the NMEA protocol on the I2C interface</td></tr><tr><td colspan="6">See Table 14 below for a list of possible constants for this item.</td></tr><tr><td>CFG-INFMSG-NMEA_UART1</td><td>0x20920007</td><td>X1</td><td>-</td><td>-</td><td>Information message enable flags for the NMEA protocol on the UART1 interface</td></tr><tr><td colspan="6">See Table 14 below for a list of possible constants for this item.</td></tr><tr><td>CFG-INFMSG-NMEA_SPI</td><td>0x2092000a</td><td>X1</td><td>-</td><td>-</td><td>Information message enable flags for the NMEA protocol on the SPI interface</td></tr></table>

See Table 14 below for a list of possible constants for this item.

Table 13: CFG-INFMSG configuration items

| Constant | Value | Description |
| --- | --- | --- |
| ERROR | 0x01 | Enable ERROR information messages |
| WARNING | 0x02 | Enable WARNING information messages |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 137 of 181

<!-- page 138 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Constant | Value | Description |
| --- | --- | --- |
| NOTICE | 0x04 | Enable NOTICE information messages |
| TEST | 0x08 | Enable TEST information messages |
| DEBUG | 0x10 | Enable DEBUG information messages |

Table 14: Constants for CFG-INFMSG-UBX\_I2C, CFG-INFMSG-UBX\_UART1, CFG-INFMSG-UBX\_SPI, CFG-INFMSG-NMEA\_I2C, CFG-INFMSG-NMEA\_UART1, CFG-INFMSG-NMEA\_SPI

## 4.9.8 CFG-ITFM: Jamming and interference monitor configuration

Configuration of jamming and interference monitor.

| Configuration item | Key ID Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- |
| CFG-ITFM-BBTHRESHOLD | 0x20410001 U1 | - | - | Broadbandjamming detection threshold |
| CFG-ITFM-CWTHRESHOLD | 0x20410002 U1 | - | - | CWjamming detection threshold |
| CFG-ITFM-ENABLE | 0x1041000d L | - | - | Enable interference detection |
| CFG-ITFM-ANTSETTING | 0x20410010 E1 | - | - | Antenna setting |

See Table 16 below for a list of possible constants for this item.

| CFG-ITFM-ENABLE_AUX | 0x10410013 | L | - | - | Scan auxiliary bands |
| --- | --- | --- | --- | --- | --- |

Set to true to scan auxiliary bands.

Supported on u-blox 8 / u-blox M8 only, otherwise ignored.

Table 15: CFG-ITFM configuration items

| Constant | Value | Description |
| --- | --- | --- |
| UNKNOWN | 0 | Unknown |
| PASSIVE | 1 | Passive |
| ACTIVE | 2 | Active |

Table 16: Constants for CFG-ITFM-ANTSETTING

## 4.9.9 CFG-MOT: Motion detector configuration

The items in this group specify the parameters used for the internal receiver motion detector. The platform motion is assessed by combining the detected motion of different detectors looking at specific data types (i.e. GNSS, gyroscopes, accelerometers, wheel ticks). The decision thresholds of the internal detectors can be specified using the configuration items in this group.

| Configuration item | Key ID | Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- | --- |
| CFG-MOT-GNSSSPEED_THRS | 0x20250038 | U1 | 0.01 | m/s | Static hold speed threshold, below which the receiver is considered to be stationary |

Set this parameter to 0 to enable the default firmware value or behavior.

| CFG-MOT-GNSSDIST_THRS | 0x3025003b | U2 | 1.0 | m | Static hold distance threshold, within which the receiver is considered to be stationary |
| --- | --- | --- | --- | --- | --- |

Set this parameter to 0 to enable the default firmware value or behavior.

**Table 17: CFG-MOT configuration items**

## 4.9.10 CFG-MSGOUT: Message output configuration

For each message and port a separate output rate (per second, per epoch) can be configured.

<table><tbody><tr><td>Configuration item Key ID Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-MSGOUT-NMEA_ID_DTM_I2C 0x209100a6 U1</td><td>-</td><td>-</td><td rowspan="2">Output rate of the NMEA-GX-DTM message on port I2C Output rate of the NMEA-GX-DTM message on port SPI</td></tr><tr><td>CFG-MSGOUT-NMEA_ID_DTM_SPI 0x209100aa U1</td><td>-</td><td>-</td></tr></tbody></table>

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 138 of 181

<!-- page 139 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID | Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- | --- |
| CFG-MSGOUT-NMEA_ID_DTM_UART1 | 0x209100a7 | U1 | - | - | Output rate of the NMEA-GX-DTM message on port UART1 |
| CFG-MSGOUT-NMEA_ID_GBS_I2C | 0x209100dd | U1 | - | - | Output rate of the NMEA-GX-GBS message on port I2C |
| CFG-MSGOUT-NMEA_ID_GBS_SPI | 0x209100e1 | U1 | - | - | Output rate of the NMEA-GX-GBS message on port SPI |
| CFG-MSGOUT-NMEA_ID_GBS_UART1 | 0x209100de | U1 | - | - | Output rate of the NMEA-GX-GBS message on port UART1 |
| CFG-MSGOUT-NMEA_ID_GGA_I2C | 0x209100ba | U1 | - | - | Output rate of the NMEA-GX-GGA message on port I2C |
| CFG-MSGOUT-NMEA_ID_GGA_SPI | 0x209100be | U1 | - | - | Output rate of the NMEA-GX-GGA message on port SPI |
| CFG-MSGOUT-NMEA_ID_GGA_UART1 | 0x209100bb | U1 | - | - | Output rate of the NMEA-GX-GGA message on port UART1 |
| CFG-MSGOUT-NMEA_ID_GLL_I2C | 0x209100c9 | U1 | - | - | Output rate of the NMEA-GX-GLL message on port I2C |
| CFG-MSGOUT-NMEA_ID_GLL_SPI | 0x209100cd | U1 | - | - | Output rate of the NMEA-GX-GLL message on port SPI |
| CFG-MSGOUT-NMEA_ID_GLL_UART1 | 0x209100ca | U1 | - | - | Output rate of the NMEA-GX-GLL message on port UART1 |
| CFG-MSGOUT-NMEA_ID_GNS_I2C | 0x209100b5 | U1 | - | - | Output rate of the NMEA-GX-GNS message on port I2C |
| CFG-MSGOUT-NMEA_ID_GNS_SPI | 0x209100b9 | U1 | - | - | Output rate of the NMEA-GX-GNS message on port SPI |
| CFG-MSGOUT-NMEA_ID_GNS_UART1 | 0x209100b6 | U1 | - | - | Output rate of the NMEA-GX-GNS message on port UART1 |
| CFG-MSGOUT-NMEA_ID_GRS_I2C | 0x209100ce | U1 | - | - | Output rate of the NMEA-GX-GRS message on port I2C |
| CFG-MSGOUT-NMEA_ID_GRS_SPI | 0x209100d2 | U1 | - | - | Output rate of the NMEA-GX-GRS message on port SPI |
| CFG-MSGOUT-NMEA_ID_GRS_UART1 | 0x209100cf | U1 | - | - | Output rate of the NMEA-GX-GRS message on port UART1 |
| CFG-MSGOUT-NMEA_ID_GSA_I2C | 0x209100bf | U1 | - | - | Output rate of the NMEA-GX-GSA message on port I2C |
| CFG-MSGOUT-NMEA_ID_GSA_SPI | 0x209100c3 | U1 | - | - | Output rate of the NMEA-GX-GSA message on port SPI |
| CFG-MSGOUT-NMEA_ID_GSA_UART1 | 0x209100c0 | U1 | - | - | Output rate of the NMEA-GX-GSA message on port UART1 |
| CFG-MSGOUT-NMEA_ID_GST_I2C | 0x209100d3 | U1 | - | - | Output rate of the NMEA-GX-GST message on port I2C |
| CFG-MSGOUT-NMEA_ID_GST_SPI | 0x209100d7 | U1 | - | - | Output rate of the NMEA-GX-GST message on port SPI |
| CFG-MSGOUT-NMEA_ID_GST_UART1 | 0x209100d4 | U1 | - | - | Output rate of the NMEA-GX-GST message on port UART1 |
| CFG-MSGOUT-NMEA_ID_GSV_I2C | 0x209100c4 | U1 | - | - | Output rate of the NMEA-GX-GSV message on port I2C |
| CFG-MSGOUT-NMEA_ID_GSV_SPI | 0x209100c8 | U1 | - | - | Output rate of the NMEA-GX-GSV message on port SPI |
| CFG-MSGOUT-NMEA_ID_GSV_UART1 | 0x209100c5 | U1 | - | - | Output rate of the NMEA-GX-GSV message on port UART1 |
| CFG-MSGOUT-NMEA_ID_RLM_I2C | 0x20910400 | U1 | - | - | Output rate of the NMEA-GX-RLM message on port I2C |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 139 of 181

<!-- page 140 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID | Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- | --- |
| CFG-MSGOUT-NMEA_ID_RLM_SPI | 0x20910404 | U1 | - | - | Output rate of the NMEA-GX-RLM message on port SPI |
| CFG-MSGOUT-NMEA_ID_RLM_UART1 | 0x20910401 | U1 | - | - | Output rate of the NMEA-GX-RLM message on port UART1 |
| CFG-MSGOUT-NMEA_ID_RMC_I2C | 0x209100ab | U1 | - | - | Output rate of the NMEA-GX-RMC message on port I2C |
| CFG-MSGOUT-NMEA_ID_RMC_SPI | 0x209100af | U1 | - | - | Output rate of the NMEA-GX-RMC message on port SPI |
| CFG-MSGOUT-NMEA_ID_RMC_UART1 | 0x209100ac | U1 | - | - | Output rate of the NMEA-GX-RMC message on port UART1 |
| CFG-MSGOUT-NMEA_ID_VLW_I2C | 0x209100e7 | U1 | - | - | Output rate of the NMEA-GX-VLW message on port I2C |
| CFG-MSGOUT-NMEA_ID_VLW_SPI | 0x209100eb | U1 | - | - | Output rate of the NMEA-GX-VLW message on port SPI |
| CFG-MSGOUT-NMEA_ID_VLW_UART1 | 0x209100e8 | U1 | - | - | Output rate of the NMEA-GX-VLW message on port UART1 |
| CFG-MSGOUT-NMEA_ID_VTG_I2C | 0x209100b0 | U1 | - | - | Output rate of the NMEA-GX-VTG message on port I2C |
| CFG-MSGOUT-NMEA_ID_VTG_SPI | 0x209100b4 | U1 | - | - | Output rate of the NMEA-GX-VTG message on port SPI |
| CFG-MSGOUT-NMEA_ID_VTG_UART1 | 0x209100b1 | U1 | - | - | Output rate of the NMEA-GX-VTG message on port UART1 |
| CFG-MSGOUT-NMEA_ID_ZDA_I2C | 0x209100d8 | U1 | - | - | Output rate of the NMEA-GX-ZDA message on port I2C |
| CFG-MSGOUT-NMEA_ID_ZDA_SPI | 0x209100dc | U1 | - | - | Output rate of the NMEA-GX-ZDA message on port SPI |
| CFG-MSGOUT-NMEA_ID_ZDA_UART1 | 0x209100d9 | U1 | - | - | Output rate of the NMEA-GX-ZDA message on port UART1 |
| CFG-MSGOUT-PUBX_ID_POLYP_I2C | 0x209100ec | U1 | - | - | Output rate of the NMEA-GX-PUBX00 message on port I2C |
| CFG-MSGOUT-PUBX_ID_POLYP_SPI | 0x209100f0 | U1 | - | - | Output rate of the NMEA-GX-PUBX00 message on port SPI |
| CFG-MSGOUT-PUBX_ID_POLYP_UART1 | 0x209100ed | U1 | - | - | Output rate of the NMEA-GX-PUBX00 message on port UART1 |
| CFG-MSGOUT-PUBX_ID_POLYS_I2C | 0x209100f1 | U1 | - | - | Output rate of the NMEA-GX-PUBX03 message on port I2C |
| CFG-MSGOUT-PUBX_ID_POLYS_SPI | 0x209100f5 | U1 | - | - | Output rate of the NMEA-GX-PUBX03 message on port SPI |
| CFG-MSGOUT-PUBX_ID_POLYS_UART1 | 0x209100f2 | U1 | - | - | Output rate of the NMEA-GX-PUBX03 message on port UART1 |
| CFG-MSGOUT-PUBX_ID_POLYT_I2C | 0x209100f6 | U1 | - | - | Output rate of the NMEA-GX-PUBX04 message on port I2C |
| CFG-MSGOUT-PUBX_ID_POLYT_SPI | 0x209100fa | U1 | - | - | Output rate of the NMEA-GX-PUBX04 message on port SPI |
| CFG-MSGOUT-PUBX_ID_POLYT_UART1 | 0x209100f7 | U1 | - | - | Output rate of the NMEA-GX-PUBX04 message on port UART1 |
| CFG-MSGOUT-UBX_MON_COMMS_I2C | 0x2091034f | U1 | - | - | Output rate of the UBX-MON-COMMS message on port I2C |
| CFG-MSGOUT-UBX_MON_COMMS_SPI | 0x20910353 | U1 | - | - | Output rate of the UBX-MON-COMMS message on port SPI |
| CFG-MSGOUT-UBX_MON_COMMS_UART1 | 0x20910350 | U1 | - | - | Output rate of the UBX-MON-COMMS message on port UART1 |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 140 of 181

<!-- page 141 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID | Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- | --- |
| CFG-MSGOUT-UBX_MON_HW2_I2C | 0x209101b9 | U1 | - | - | Output rate of the UBX-MON-HW2 message on port I2C |
| CFG-MSGOUT-UBX_MON_HW2_SPI | 0x209101bd | U1 | - | - | Output rate of the UBX-MON-HW2 message on port SPI |
| CFG-MSGOUT-UBX_MON_HW2_UART1 | 0x209101ba | U1 | - | - | Output rate of the UBX-MON-HW2 message on port UART1 |
| CFG-MSGOUT-UBX_MON_HW3_I2C | 0x20910354 | U1 | - | - | Output rate of the UBX-MON-HW3 message on port I2C |
| CFG-MSGOUT-UBX_MON_HW3_SPI | 0x20910358 | U1 | - | - | Output rate of the UBX-MON-HW3 message on port SPI |
| CFG-MSGOUT-UBX_MON_HW3_UART1 | 0x20910355 | U1 | - | - | Output rate of the UBX-MON-HW3 message on port UART1 |
| CFG-MSGOUT-UBX_MON_HW_I2C | 0x209101b4 | U1 | - | - | Output rate of the UBX-MON-HW message on port I2C |
| CFG-MSGOUT-UBX_MON_HW_SPI | 0x209101b8 | U1 | - | - | Output rate of the UBX-MON-HW message on port SPI |
| CFG-MSGOUT-UBX_MON_HW_UART1 | 0x209101b5 | U1 | - | - | Output rate of the UBX-MON-HW message on port UART1 |
| CFG-MSGOUT-UBX_MON_IO_I2C | 0x209101a5 | U1 | - | - | Output rate of the UBX-MON-IO message on port I2C |
| CFG-MSGOUT-UBX_MON_IO_SPI | 0x209101a9 | U1 | - | - | Output rate of the UBX-MON-IO message on port SPI |
| CFG-MSGOUT-UBX_MON_IO_UART1 | 0x209101a6 | U1 | - | - | Output rate of the UBX-MON-IO message on port UART1 |
| CFG-MSGOUT-UBX_MON_MSGPP_I2C | 0x20910196 | U1 | - | - | Output rate of the UBX-MON-MSGPP message on port I2C |
| CFG-MSGOUT-UBX_MON_MSGPP_SPI | 0x2091019a | U1 | - | - | Output rate of the UBX-MON-MSGPP message on port SPI |
| CFG-MSGOUT-UBX_MON_MSGPP_UART1 | 0x20910197 | U1 | - | - | Output rate of the UBX-MON-MSGPP message on port UART1 |
| CFG-MSGOUT-UBX_MON_RF_I2C | 0x20910359 | U1 | - | - | Output rate of the UBX-MON-RF message on port I2C |
| CFG-MSGOUT-UBX_MON_RF_SPI | 0x2091035d | U1 | - | - | Output rate of the UBX-MON-RF message on port SPI |
| CFG-MSGOUT-UBX_MON_RF_UART1 | 0x2091035a | U1 | - | - | Output rate of the UBX-MON-RF message on port UART1 |
| CFG-MSGOUT-UBX_MON_RXR_I2C | 0x20910187 | U1 | - | - | Output rate of the UBX-MON-RXR message on port I2C |
| CFG-MSGOUT-UBX_MON_RXR_SPI | 0x2091018b | U1 | - | - | Output rate of the UBX-MON-RXR message on port SPI |
| CFG-MSGOUT-UBX_MON_RXR_UART1 | 0x20910188 | U1 | - | - | Output rate of the UBX-MON-RXR message on port UART1 |
| CFG-MSGOUT-UBX_MON_SPAN_I2C | 0x2091038b | U1 | - | - | Output rate of the UBX-MON-SPAN message on port I2C |
| CFG-MSGOUT-UBX_MON_SPAN_SPI | 0x2091038f | U1 | - | - | Output rate of the UBX-MON-SPAN message on port SPI |
| CFG-MSGOUT-UBX_MON_SPAN_UART1 | 0x2091038c | U1 | - | - | Output rate of the UBX-MON-SPAN message on port UART1 |
| CFG-MSGOUT-UBX_MON_SYS_I2C | 0x2091069d | U1 | - | - | Output rate of the UBX-MON-SYS message on port I2C |
| CFG-MSGOUT-UBX_MON_SYS_SPI | 0x209106a1 | U1 | - | - | Output rate of the UBX-MON-SYS message on port SPI |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 141 of 181

<!-- page 142 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID | Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- | --- |
| CFG-MSGOUT-UBX_MON_SYS_UART1 | 0x2091069e | U1 | - | - | Output rate of the UBX-MON-SYS message on port UART1 |
| CFG-MSGOUT-UBX_NAV_AOPSTATUS_I2C | 0x20910079 | U1 | - | - | Output rate of the UBX-NAV-AOPSTATUS message on port I2C |
| CFG-MSGOUT-UBX_NAV_AOPSTATUS_SPI | 0x2091007d | U1 | - | - | Output rate of the UBX-NAV-AOPSTATUS message on port SPI |
| CFG-MSGOUT-UBX_NAV_AOPSTATUS_UART1 | 0x2091007a | U1 | - | - | Output rate of the UBX-NAV-AOPSTATUS message on port UART1 |
| CFG-MSGOUT-UBX_NAV_CLOCK_I2C | 0x20910065 | U1 | - | - | Output rate of the UBX-NAV-CLOCK message on port I2C |
| CFG-MSGOUT-UBX_NAV_CLOCK_SPI | 0x20910069 | U1 | - | - | Output rate of the UBX-NAV-CLOCK message on port SPI |
| CFG-MSGOUT-UBX_NAV_CLOCK_UART1 | 0x20910066 | U1 | - | - | Output rate of the UBX-NAV-CLOCK message on port UART1 |
| CFG-MSGOUT-UBX_NAV_COV_I2C | 0x20910083 | U1 | - | - | Output rate of the UBX-NAV-COV message on port I2C |
| CFG-MSGOUT-UBX_NAV_COV_SPI | 0x20910087 | U1 | - | - | Output rate of the UBX-NAV-COV message on port SPI |
| CFG-MSGOUT-UBX_NAV_COV_UART1 | 0x20910084 | U1 | - | - | Output rate of the UBX-NAV-COV message on port UART1 |
| CFG-MSGOUT-UBX_NAV_DOP_I2C | 0x20910038 | U1 | - | - | Output rate of the UBX-NAV-DOP message on port I2C |
| CFG-MSGOUT-UBX_NAV_DOP_SPI | 0x2091003c | U1 | - | - | Output rate of the UBX-NAV-DOP message on port SPI |
| CFG-MSGOUT-UBX_NAV_DOP_UART1 | 0x20910039 | U1 | - | - | Output rate of the UBX-NAV-DOP message on port UART1 |
| CFG-MSGOUT-UBX_NAV_EOE_I2C | 0x2091015f | U1 | - | - | Output rate of the UBX-NAV-EOE message on port I2C |
| CFG-MSGOUT-UBX_NAV_EOE_SPI | 0x20910163 | U1 | - | - | Output rate of the UBX-NAV-EOE message on port SPI |
| CFG-MSGOUT-UBX_NAV_EOE_UART1 | 0x20910160 | U1 | - | - | Output rate of the UBX-NAV-EOE message on port UART1 |
| CFG-MSGOUT-UBX_NAV_ODO_I2C | 0x2091007e | U1 | - | - | Output rate of the UBX-NAV-ODO message on port I2C |
| CFG-MSGOUT-UBX_NAV_ODO_SPI | 0x20910082 | U1 | - | - | Output rate of the UBX-NAV-ODO message on port SPI |
| CFG-MSGOUT-UBX_NAV_ODO_UART1 | 0x2091007f | U1 | - | - | Output rate of the UBX-NAV-ODO message on port UART1 |
| CFG-MSGOUT-UBX_NAV_ORB_I2C | 0x20910010 | U1 | - | - | Output rate of the UBX-NAV-ORB message on port I2C |
| CFG-MSGOUT-UBX_NAV_ORB_SPI | 0x20910014 | U1 | - | - | Output rate of the UBX-NAV-ORB message on port SPI |
| CFG-MSGOUT-UBX_NAV_ORB_UART1 | 0x20910011 | U1 | - | - | Output rate of the UBX-NAV-ORB message on port UART1 |
| CFG-MSGOUT-UBX_NAV_PL_I2C | 0x20910415 | U1 | - | - | Output rate of the UBX-NAV-PL message on port I2C |
| CFG-MSGOUT-UBX_NAV_PL_SPI | 0x20910419 | U1 | - | - | Output rate of the UBX-NAV-PL message on port SPI |
| CFG-MSGOUT-UBX_NAV_PL_UART1 | 0x20910416 | U1 | - | - | Output rate of the UBX-NAV-PL message on port UART1 |
| CFG-MSGOUT-UBX_NAV_POSECEF_I2C | 0x20910024 | U1 | - | - | Output rate of the UBX-NAV-POSECEF message on port I2C |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 142 of 181

<!-- page 143 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID | Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- | --- |
| CFG-MSGOUT-UBX_NAV_POSECEF_SPI | 0x20910028 | U1 | - | - | Output rate of the UBX-NAV-POSECEF message on port SPI |
| CFG-MSGOUT-UBX_NAV_POSECEF_UART1 | 0x20910025 | U1 | - | - | Output rate of the UBX-NAV-POSECEF message on port UART1 |
| CFG-MSGOUT-UBX_NAV_POSLLH_I2C | 0x20910029 | U1 | - | - | Output rate of the UBX-NAV-POSLLH message on port I2C |
| CFG-MSGOUT-UBX_NAV_POSLLH_SPI | 0x2091002d | U1 | - | - | Output rate of the UBX-NAV-POSLLH message on port SPI |
| CFG-MSGOUT-UBX_NAV_POSLLH_UART1 | 0x2091002a | U1 | - | - | Output rate of the UBX-NAV-POSLLH message on port UART1 |
| CFG-MSGOUT-UBX_NAV_PVT_I2C | 0x20910006 | U1 | - | - | Output rate of the UBX-NAV-PVT message on port I2C |
| CFG-MSGOUT-UBX_NAV_PVT_SPI | 0x2091000a | U1 | - | - | Output rate of the UBX-NAV-PVT message on port SPI |
| CFG-MSGOUT-UBX_NAV_PVT_UART1 | 0x20910007 | U1 | - | - | Output rate of the UBX-NAV-PVT message on port UART1 |
| CFG-MSGOUT-UBX_NAV_SAT_I2C | 0x20910015 | U1 | - | - | Output rate of the UBX-NAV-SAT message on port I2C |
| CFG-MSGOUT-UBX_NAV_SAT_SPI | 0x20910019 | U1 | - | - | Output rate of the UBX-NAV-SAT message on port SPI |
| CFG-MSGOUT-UBX_NAV_SAT_UART1 | 0x20910016 | U1 | - | - | Output rate of the UBX-NAV-SAT message on port UART1 |
| CFG-MSGOUT-UBX_NAV_SBAS_I2C | 0x2091006a | U1 | - | - | Output rate of the UBX-NAV-SBAS message on port I2C |
| CFG-MSGOUT-UBX_NAV_SBAS_SPI | 0x2091006e | U1 | - | - | Output rate of the UBX-NAV-SBAS message on port SPI |
| CFG-MSGOUT-UBX_NAV_SBAS_UART1 | 0x2091006b | U1 | - | - | Output rate of the UBX-NAV-SBAS message on port UART1 |
| CFG-MSGOUT-UBX_NAV_SIG_I2C | 0x20910345 | U1 | - | - | Output rate of the UBX-NAV-SIG message on port I2C |
| CFG-MSGOUT-UBX_NAV_SIG_SPI | 0x20910349 | U1 | - | - | Output rate of the UBX-NAV-SIG message on port SPI |
| CFG-MSGOUT-UBX_NAV_SIG_UART1 | 0x20910346 | U1 | - | - | Output rate of the UBX-NAV-SIG message on port UART1 |
| CFG-MSGOUT-UBX_NAV_SLAS_I2C | 0x20910336 | U1 | - | - | Output rate of the UBX-NAV-SLAS message on port I2C |
| CFG-MSGOUT-UBX_NAV_SLAS_SPI | 0x2091033a | U1 | - | - | Output rate of the UBX-NAV-SLAS message on port SPI |
| CFG-MSGOUT-UBX_NAV_SLAS_UART1 | 0x20910337 | U1 | - | - | Output rate of the UBX-NAV-SLAS message on port UART1 |
| CFG-MSGOUT-UBX_NAV_STATUS_I2C | 0x2091001a | U1 | - | - | Output rate of the UBX-NAV-STATUS message on port I2C |
| CFG-MSGOUT-UBX_NAV_STATUS_SPI | 0x2091001e | U1 | - | - | Output rate of the UBX-NAV-STATUS message on port SPI |
| CFG-MSGOUT-UBX_NAV_STATUS_UART1 | 0x2091001b | U1 | - | - | Output rate of the UBX-NAV-STATUS message on port UART1 |
| CFG-MSGOUT-UBX_NAV_TIMEBDS_I2C | 0x20910051 | U1 | - | - | Output rate of the UBX-NAV-TIMEBDS message on port I2C |
| CFG-MSGOUT-UBX_NAV_TIMEBDS_SPI | 0x20910055 | U1 | - | - | Output rate of the UBX-NAV-TIMEBDS message on port SPI |
| CFG-MSGOUT-UBX_NAV_TIMEBDS_UART1 | 0x20910052 | U1 | - | - | Output rate of the UBX-NAV-TIMEBDS message on port UART1 |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 143 of 181

<!-- page 144 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID | Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- | --- |
| CFG-MSGOUT-UBX_NAV_TIMEGAL_I2C | 0x20910056 | U1 | - | - | Output rate of the UBX-NAV-TIMEGAL message on port I2C |
| CFG-MSGOUT-UBX_NAV_TIMEGAL_SPI | 0x2091005a | U1 | - | - | Output rate of the UBX-NAV-TIMEGAL message on port SPI |
| CFG-MSGOUT-UBX_NAV_TIMEGAL_UART1 | 0x20910057 | U1 | - | - | Output rate of the UBX-NAV-TIMEGAL message on port UART1 |
| CFG-MSGOUT-UBX_NAV_TIMEGPS_I2C | 0x20910047 | U1 | - | - | Output rate of the UBX-NAV-TIMEGPS message on port I2C |
| CFG-MSGOUT-UBX_NAV_TIMEGPS_SPI | 0x2091004b | U1 | - | - | Output rate of the UBX-NAV-TIMEGPS message on port SPI |
| CFG-MSGOUT-UBX_NAV_TIMEGPS_UART1 | 0x20910048 | U1 | - | - | Output rate of the UBX-NAV-TIMEGPS message on port UART1 |
| CFG-MSGOUT-UBX_NAV_TIMELS_I2C | 0x20910060 | U1 | - | - | Output rate of the UBX-NAV-TIMELS message on port I2C |
| CFG-MSGOUT-UBX_NAV_TIMELS_SPI | 0x20910064 | U1 | - | - | Output rate of the UBX-NAV-TIMELS message on port SPI |
| CFG-MSGOUT-UBX_NAV_TIMELS_UART1 | 0x20910061 | U1 | - | - | Output rate of the UBX-NAV-TIMELS message on port UART1 |
| CFG-MSGOUT-UBX_NAV_TIMENAVIC_I2C | 0x209106a2 | U1 | - | - | Output rate of the UBX-NAV-TIMENAVIC message on port I2C |
| CFG-MSGOUT-UBX_NAV_TIMENAVIC_SPI | 0x209106a6 | U1 | - | - | Output rate of the UBX-NAV-TIMENAVIC message on port SPI |
| CFG-MSGOUT-UBX_NAV_TIMENAVIC_UART1 | 0x209106a3 | U1 | - | - | Output rate of the UBX-NAV-TIMENAVIC message on port UART1 |
| CFG-MSGOUT-UBX_NAV_TIMEQZSS_I2C | 0x20910386 | U1 | - | - | Output rate of the UBX-NAV-TIMEQZSS message on port I2C |
| CFG-MSGOUT-UBX_NAV_TIMEQZSS_SPI | 0x2091038a | U1 | - | - | Output rate of the UBX-NAV-TIMEQZSS message on port SPI |
| CFG-MSGOUT-UBX_NAV_TIMEQZSS_UART1 | 0x20910387 | U1 | - | - | Output rate of the UBX-NAV-TIMEQZSS message on port UART1 |
| CFG-MSGOUT-UBX_NAV_TIMEOUT_I2C | 0x2091005b | U1 | - | - | Output rate of the UBX-NAV-TIMEUTC message on port I2C |
| CFG-MSGOUT-UBX_NAV_TIMEOUT_SPI | 0x2091005f | U1 | - | - | Output rate of the UBX-NAV-TIMEUTC message on port SPI |
| CFG-MSGOUT-UBX_NAV_TIMEOUT_UART1 | 0x2091005c | U1 | - | - | Output rate of the UBX-NAV-TIMEUTC message on port UART1 |
| CFG-MSGOUT-UBX_NAV_VELECEF_I2C | 0x2091003d | U1 | - | - | Output rate of the UBX-NAV-VELECEF message on port I2C |
| CFG-MSGOUT-UBX_NAV_VELECEF_SPI | 0x20910041 | U1 | - | - | Output rate of the UBX-NAV-VELECEF message on port SPI |
| CFG-MSGOUT-UBX_NAV_VELECEF_UART1 | 0x2091003e | U1 | - | - | Output rate of the UBX-NAV-VELECEF message on port UART1 |
| CFG-MSGOUT-UBX_NAV_VELNED_I2C | 0x20910042 | U1 | - | - | Output rate of the UBX-NAV-VELNED message on port I2C |
| CFG-MSGOUT-UBX_NAV_VELNED_SPI | 0x20910046 | U1 | - | - | Output rate of the UBX-NAV-VELNED message on port SPI |
| CFG-MSGOUT-UBX_NAV_VELNED_UART1 | 0x20910043 | U1 | - | - | Output rate of the UBX-NAV-VELNED message on port UART1 |
| CFG-MSGOUT-UBX_RXM_MEASX_I2C | 0x20910204 | U1 | - | - | Output rate of the UBX-RXM-MEASX message on port I2C |
| CFG-MSGOUT-UBX_RXM_MEASX_SPI | 0x20910208 | U1 | - | - | Output rate of the UBX-RXM-MEASX message on port SPI |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 144 of 181

<!-- page 145 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID | Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- | --- |
| CFG-MSGOUT-UBX_RXM_MEASX_UART1 | 0x20910205 | U1 | - | - | Output rate of the UBX-RXM-MEASX message on port UART1 |
| CFG-MSGOUT-UBX_RXM_RLM_I2C | 0x2091025e | U1 | - | - | Output rate of the UBX-RXM-RLM message on port I2C |
| CFG-MSGOUT-UBX_RXM_RLM_SPI | 0x20910262 | U1 | - | - | Output rate of the UBX-RXM-RLM message on port SPI |
| CFG-MSGOUT-UBX_RXM_RLM_UART1 | 0x2091025f | U1 | - | - | Output rate of the UBX-RXM-RLM message on port UART1 |
| CFG-MSGOUT-UBX_RXM_SFRBX_I2C | 0x20910231 | U1 | - | - | Output rate of the UBX-RXM-SFRBX message on port I2C |
| CFG-MSGOUT-UBX_RXM_SFRBX_SPI | 0x20910235 | U1 | - | - | Output rate of the UBX-RXM-SFRBX message on port SPI |
| CFG-MSGOUT-UBX_RXM_SFRBX_UART1 | 0x20910232 | U1 | - | - | Output rate of the UBX-RXM-SFRBX message on port UART1 |
| CFG-MSGOUT-UBX_SEC_SIGLOG_I2C | 0x20910689 | U1 | - | - | Output rate of the UBX-SEC-SIGLOG message on port I2C |
| CFG-MSGOUT-UBX_SEC_SIGLOG_SPI | 0x2091068d | U1 | - | - | Output rate of the UBX-SEC-SIGLOG message on port SPI |
| CFG-MSGOUT-UBX_SEC_SIGLOG_UART1 | 0x2091068a | U1 | - | - | Output rate of the UBX-SEC-SIGLOG message on port UART1 |
| CFG-MSGOUT-UBX_SEC_SIG_I2C | 0x20910634 | U1 | - | - | Output rate of the UBX-SEC-SIG message on port I2C |
| CFG-MSGOUT-UBX_SEC_SIG_SPI | 0x20910638 | U1 | - | - | Output rate of the UBX-SEC-SIG message on port SPI |
| CFG-MSGOUT-UBX_SEC_SIG_UART1 | 0x20910635 | U1 | - | - | Output rate of the UBX-SEC-SIG message on port UART1 |
| CFG-MSGOUT-UBX_TIM_TM2_I2C | 0x20910178 | U1 | - | - | Output rate of the UBX-TIM-TM2 message on port I2C |
| CFG-MSGOUT-UBX_TIM_TM2_SPI | 0x2091017c | U1 | - | - | Output rate of the UBX-TIM-TM2 message on port SPI |
| CFG-MSGOUT-UBX_TIM_TM2_UART1 | 0x20910179 | U1 | - | - | Output rate of the UBX-TIM-TM2 message on port UART1 |
| CFG-MSGOUT-UBX_TIM_TP_I2C | 0x2091017d | U1 | - | - | Output rate of the UBX-TIM-TP message on port I2C |
| CFG-MSGOUT-UBX_TIM_TP_SPI | 0x20910181 | U1 | - | - | Output rate of the UBX-TIM-TP message on port SPI |
| CFG-MSGOUT-UBX_TIM_TP_UART1 | 0x2091017e | U1 | - | - | Output rate of the UBX-TIM-TP message on port UART1 |
| CFG-MSGOUT-UBX_TIM_VRFY_I2C | 0x20910092 | U1 | - | - | Output rate of the UBX-TIM-VRFY message on port I2C |
| CFG-MSGOUT-UBX_TIM_VRFY_SPI | 0x20910096 | U1 | - | - | Output rate of the UBX-TIM-VRFY message on port SPI |
| CFG-MSGOUT-UBX_TIM_VRFY_UART1 | 0x20910093 | U1 | - | - | Output rate of the UBX-TIM-VRFY message on port UART1 |

Table 18: CFG-MSGOUT configuration items

## 4.9.11 CFG-NAVMASK: Satellite Mask Configuration

This can be used to mask out defined satellites to not be used.

The satellites can be specified directly by GNSS system, or when entering a part of sky view in given azimuth and elevation coordinates.

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 145 of 181

<!-- page 146 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

It can be used to block certain portions of sky that are known to introduce signal distortions such as multi-path, etc.

Recommended to be used for receivers that are stationary.

Please note, the satellites may be blocked also by CFG-NAVSPG-INFIL\_MINELEV key.

SBAS can be specified using CFG-SBAS-PRNSCANMASK key.

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-NAVMASK-SV_MASK_GPS</td><td>0x50180013</td><td>X8</td><td>-</td><td>-</td><td>Satellite mask for the GPS system</td></tr><tr><td colspan="6">Every bit corresponds to a satellite in GPS system.</td></tr><tr><td colspan="6">Each bit of a mask corresponds to a satellite with same number:</td></tr><tr><td colspan="6">bit 0: reserved</td></tr><tr><td colspan="6">bit 1: SV #1</td></tr><tr><td colspan="6">bit 2: SV #2</td></tr><tr><td colspan="6">...</td></tr><tr><td colspan="6">bit 31: SV #31</td></tr><tr><td colspan="6">bit 32: SV #32</td></tr><tr><td colspan="6">bit 33 - 63: reserved</td></tr><tr><td colspan="6">The meaning of the bits:</td></tr><tr><td colspan="6">1: SV allowed</td></tr><tr><td colspan="6">0: SV blocked</td></tr><tr><td colspan="6">See Table 20 below for a list of possible constants for this item.</td></tr><tr><td>CFG-NAVMASK-SV_MASK_GAL</td><td>0x50180014</td><td>X8</td><td>-</td><td>-</td><td>Satellite mask for the GALILEO system</td></tr><tr><td colspan="6">Every bit corresponds to a satellite in GALILEO system.</td></tr><tr><td colspan="6">Each bit of a mask corresponds to a satellite with same number:</td></tr><tr><td colspan="6">bit 0: reserved</td></tr><tr><td colspan="6">bit 1: SV #1</td></tr><tr><td colspan="6">bit 2: SV #2</td></tr><tr><td colspan="6">...</td></tr><tr><td colspan="6">bit 35: SV #35</td></tr><tr><td colspan="6">bit 36: SV #36</td></tr><tr><td colspan="6">bit 37 - 63: reserved</td></tr><tr><td colspan="6">The meaning of the bits:</td></tr><tr><td colspan="6">1: SV allowed</td></tr><tr><td colspan="6">0: SV blocked</td></tr><tr><td colspan="6">See Table 21 below for a list of possible constants for this item.</td></tr><tr><td>CFG-NAVMASK-SV_MASK_BDS</td><td>0x50180016</td><td>X8</td><td>-</td><td>-</td><td>Satellite mask for the BeiDou system</td></tr><tr><td colspan="6">Every bit corresponds to a satellite in BeiDou system.</td></tr><tr><td colspan="6">Each bit of a mask corresponds to a satellite with same number:</td></tr><tr><td colspan="6">bit 0: reserved</td></tr><tr><td colspan="6">bit 1: SV #1</td></tr><tr><td colspan="6">bit 2: SV #2</td></tr><tr><td colspan="6">...</td></tr><tr><td colspan="6">bit 62: SV #62</td></tr><tr><td colspan="6">bit 63: SV #63</td></tr><tr><td colspan="6">The meaning of the bits:</td></tr><tr><td colspan="6">1: SV allowed</td></tr><tr><td colspan="6">0: SV blocked</td></tr><tr><td colspan="6">See Table 22 below for a list of possible constants for this item.</td></tr></table>

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 146 of 181

<!-- page 147 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-NAVMASK-SV_MASK_QZSS</td><td>0x50180017</td><td>X8</td><td>-</td><td>-</td><td>Satellite mask for the QZSS system</td></tr><tr><td colspan="6">Every bit corresponds to a satellite in QZSS system.</td></tr><tr><td colspan="6">Each bit of a mask corresponds to a satellite with same number:</td></tr><tr><td colspan="6">bit 0: reserved</td></tr><tr><td colspan="6">bit 1: SV #1</td></tr><tr><td colspan="6">bit 2: SV #2</td></tr><tr><td colspan="6">...</td></tr><tr><td colspan="6">bit 9: SV #9</td></tr><tr><td colspan="6">bit 10: SV #10</td></tr><tr><td colspan="6">bit 11 - 63: reserved</td></tr><tr><td colspan="6">The meaning of the bits:</td></tr><tr><td colspan="6">1: SV allowed</td></tr><tr><td colspan="6">0: SV blocked</td></tr><tr><td colspan="6">See Table 23 below for a list of possible constants for this item.</td></tr><tr><td>CFG-NAVMASK-SV_MASK_NAVIC</td><td>0x50180018</td><td>X8</td><td>-</td><td>-</td><td>Satellite mask for the NavIC system</td></tr><tr><td colspan="6">Every bit corresponds to a satellite in NavIC system.</td></tr><tr><td colspan="6">Each bit of a mask corresponds to a satellite with same number:</td></tr><tr><td colspan="6">bit 0: reserved</td></tr><tr><td colspan="6">bit 1: SV #1</td></tr><tr><td colspan="6">bit 2: SV #2</td></tr><tr><td colspan="6">...</td></tr><tr><td colspan="6">bit 13: SV #13</td></tr><tr><td colspan="6">bit 14: SV #14</td></tr><tr><td colspan="6">bit 15 - 63: reserved</td></tr><tr><td colspan="6">The meaning of the bits:</td></tr><tr><td colspan="6">1: SV allowed</td></tr><tr><td colspan="6">0: SV blocked</td></tr><tr><td colspan="6">See Table 24 below for a list of possible constants for this item.</td></tr></table>

See Table 24 below for a list of possible constants for this item.

## Table 19: CFG-NAVMASK configuration items

| Constant | Value | Description |
| --- | --- | --- |
| EMPTY | 0xffffffffffffffff | All GPS satellites allowed |

## Table 20: Constants for CFG-NAVMASK-SV\_MASK\_GPS

| Constant | Value | Description |
| --- | --- | --- |
| EMPTY | 0xffffffffffffffff | All GALILEO satellites allowed |

## Table 21: Constants for CFG-NAVMASK-SV\_MASK\_GAL

| Constant | Value | Description |
| --- | --- | --- |
| EMPTY | 0xffffffffffffffff | All BeiDou satellites allowed |

## Table 22: Constants for CFG-NAVMASK-SV\_MASK\_BDS

| Constant | Value | Description |
| --- | --- | --- |
| EMPTY | 0xffffffffffffffff | All QZSS satellites allowed |

## Table 23: Constants for CFG-NAVMASK-SV\_MASK\_QZSS

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 147 of 181

<!-- page 148 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Constant | Value | Description |
| --- | --- | --- |
| EMPTY | 0xffffffffffffffff | All NavIC satellites allowed |

Table 24: Constants for CFG-NAVMASK-SV\_MASK\_NAVIC

## 4.9.12 CFG-NAVSPG: Standard precision navigation configuration

This group contains configuration items related to the operation of the receiver at standard precision, including configuring position fix mode, ionospheric model selection and other related items.

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-NAVSPG-FIXMODE</td><td>0x20110011</td><td>E1</td><td>-</td><td>-</td><td>Position fix mode</td></tr><tr><td colspan="6">See Table 26 below for a list of possible constants for this item.</td></tr><tr><td>CFG-NAVSPG-INIFIX3D</td><td>0x10110013</td><td>L</td><td>-</td><td>-</td><td>Initial fix must be a 3D fix</td></tr><tr><td>CFG-NAVSPG-WKNROLLOVER</td><td>0x30110017</td><td>U2</td><td>-</td><td>-</td><td>GPS week rollover number</td></tr><tr><td colspan="6">GPS week numbers are set correctly from this week up to 1024 weeks after this week.The range is from 1 to 4096.</td></tr><tr><td>CFG-NAVSPG-UTCSTANDARD</td><td>0x2011001c</td><td>E1</td><td>-</td><td>-</td><td>UTC standard to be used</td></tr><tr><td colspan="6">See section GNSS time base in the integration manual.See Table 27 below for a list of possible constants for this item.</td></tr><tr><td>CFG-NAVSPG-DYNMODEL</td><td>0x20110021</td><td>E1</td><td>-</td><td>-</td><td>Dynamic platform model</td></tr><tr><td colspan="6">See Table 28 below for a list of possible constants for this item.</td></tr><tr><td>CFG-NAVSPG-ACKAIDING</td><td>0x10110025</td><td>L</td><td>-</td><td>-</td><td>Acknowledge assistance input messages</td></tr><tr><td>CFG-NAVSPG-USE_USRDAT</td><td>0x10110061</td><td>L</td><td>-</td><td>-</td><td>Use user geodetic datum parametersUser specified geodetic system can be used instead of the receiver&#x27;s default WGS84 ellipsoid. All of the CFG-NAVSPG-USERDAT_* user geodetic datum parameters listed here must be configured before enabling the user specified geodetic system.</td></tr><tr><td>CFG-NAVSPG-USRDAT_MAJA</td><td>0x50110062</td><td>R8</td><td>-</td><td>m</td><td>Geodetic datum semi-major axis</td></tr><tr><td colspan="6">Accepted range is from 6,300,000.0 to 6,500,000.0 meters</td></tr><tr><td>CFG-NAVSPG-USRDAT_FLAT</td><td>0x50110063</td><td>R8</td><td>-</td><td>-</td><td>Geodetic datum 1.0 / flattening</td></tr><tr><td colspan="6">Accepted range is 0.0 to 500.0.</td></tr><tr><td>CFG-NAVSPG-USRDAT_DX</td><td>0x40110064</td><td>R4</td><td>-</td><td>m</td><td>Geodetic datum X axis shift at the origin</td></tr><tr><td colspan="6">Accepted range is +/- 5000.0 meters.</td></tr><tr><td>CFG-NAVSPG-USRDAT_DY</td><td>0x40110065</td><td>R4</td><td>-</td><td>m</td><td>Geodetic datum Y axis shift at the origin</td></tr><tr><td colspan="6">Accepted range is +/- 5000.0 meters.</td></tr><tr><td>CFG-NAVSPG-USRDAT_DZ</td><td>0x40110066</td><td>R4</td><td>-</td><td>m</td><td>Geodetic datum Z axis shift at the origin</td></tr><tr><td colspan="6">Accepted range is +/- 5000.0 meters.</td></tr><tr><td>CFG-NAVSPG-USRDAT_ROTX</td><td>0x40110067</td><td>R4</td><td>-</td><td>arcsec</td><td>Geodetic datum rotation about the X axis</td></tr><tr><td colspan="6">Accepted range is +/- 20.0 milli arc seconds.</td></tr><tr><td>CFG-NAVSPG-USRDAT_ROTY</td><td>0x40110068</td><td>R4</td><td>-</td><td>arcsec</td><td>Geodetic datum rotation about the Y axis ()</td></tr><tr><td colspan="6">Accepted range is +/- 20.0 milli-arc seconds.</td></tr><tr><td>CFG-NAVSPG-USRDAT_ROTZ</td><td>0x40110069</td><td>R4</td><td>-</td><td>arcsec</td><td>Geodetic datum rotation about the Z axis</td></tr><tr><td colspan="6">Accepted range is +/- 20.0 milli-arc seconds.</td></tr><tr><td>CFG-NAVSPG-USRDAT_SCALE</td><td>0x4011006a</td><td>R4</td><td>-</td><td>ppm</td><td>Geodetic datum scale factor</td></tr><tr><td colspan="6">Accepted range is 0.0 to 50.0 parts per million.</td></tr><tr><td>CFG-NAVSPG-INFIL_MINSVS</td><td>0x201100a1</td><td>U1</td><td>-</td><td>-</td><td>Minimum number of satellites for navigation</td></tr></table>

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 148 of 181

<!-- page 149 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- |
| CFG-NAVSPG-INFIL_MAXSVS | 0x201100a2 U1 | - | - | Maximum number of satellites for navigation |
| CFG-NAVSPG-INFIL_MINCNO | 0x201100a3 U1 | - | dBHz | Minimum satellite signal level for navigation |
| CFG-NAVSPG-INFIL_MINELEV | 0x201100a4 I1 | - | deg | Minimum elevation for a GNSS satellite to be used in navigation |
| CFG-NAVSPG-INFIL_NCNOTHRS | 0x201100aa U1 | - | - | Number of satellites required to have C/N0 above CFG-NAVSPG-INFIL_CNOTHRS for a fix to be attempted |
| CFG-NAVSPG-INFIL_CNOTHRS | 0x201100ab U1 | - | - | C/N0 threshold for deciding whether to attempt a fix |
| CFG-NAVSPG-OUTFIL_PDOP | 0x301100b1 U2 | 0.1 | - | Output filter position DOP mask (threshold) |
| CFG-NAVSPG-OUTFIL_TDOP | 0x301100b2 U2 | 0.1 | - | Output filter time DOP mask (threshold) |
| CFG-NAVSPG-OUTFIL_PACC | 0x301100b3 U2 | - | m | Output filter position accuracy mask (threshold) |
| CFG-NAVSPG-OUTFIL_TACC | 0x301100b4 U2 | - | m | Output filter time accuracy mask (threshold) |
| CFG-NAVSPG-OUTFIL_FACC | 0x301100b5 U2 | 0.01 | m/s | Output filter frequency accuracy mask (threshold) |
| CFG-NAVSPG-CONSTR_ALT | 0x401100c1 I4 | 0.01 | m | Fixed altitude (mean sea level) for 2D fix mode |
| CFG-NAVSPG-CONSTR_ALTVAR | 0x401100c2 U4 | 0.0001 | m^2 | Fixed altitude variance for 2D mode |
| CFG-NAVSPG-CONSTR_DGNSSTO | 0x201100c4 U1 | - | s | DGNSS timeout |
| CFG-NAVSPG-SIGATTCOMP | 0x201100d6 E1 | - | - | Permanently attenuated signal compensation mode |

See Table 29 below for a list of possible constants for this item.

<table><tr><td>CFG-NAVSPG-PL_ENA</td><td>0x101100d7</td><td>L</td><td>-</td><td>-</td><td>Enable Protection level</td></tr><tr><td colspan="6">If enabled, protection level computing is on.</td></tr></table>

## Table 25: CFG-NAVSPG configuration items

| Constant | Value | Description |
| --- | --- | --- |
| 2DONLY | 1 | 2D only |
| 3DONLY | 2 | 3D only |
| AUTO | 3 | Auto2D/3D |

Table 26: Constants for CFG-NAVSPG-FIXMODE

| Constant | Value | Description |
| --- | --- | --- |
| AUTO | 0 | Automatic; receiver selects based on GNSS configuration |
| USNO | 3 | UTC as operated by the U.S. Naval Observatory (USNO); derived from GPS time |
| EU | 5 | UTC as combined from multiple European laboratories; derived from Galileo time |
| SU | 6 | UTC as operated by the former Soviet Union (SU); derived from GLONASS time |
| NTSC | 7 | UTC as operated by the National Time Service Center (NTSC), China; derived from BeiDou time |
| NPLI | 8 | UTC as operated by the National Physics Laboratory, India (NPLI); derived from NavIC time |
| NICT | 9 | UTC as operated by the National Institute of Information and Communications Technology, Japan (NICT); derived from QZSS time |

Table 27: Constants for CFG-NAVSPG-UTCSTANDARD

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 149 of 181

<!-- page 150 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Constant | Value | Description |
| --- | --- | --- |
| PORT | 0 | Portable |
| STAT | 2 | Stationary |
| PED | 3 | Pedestrian |
| AUTOMOT | 4 | Automotive |
| SEA | 5 | Sea |
| AIR1 | 6 | Airborne with &lt;1g acceleration |
| AIR2 | 7 | Airborne with &lt;2g acceleration |
| AIR4 | 8 | Airborne with &lt;4g acceleration |
| WRIST | 9 | Wrist-worn watch (not available in all products) |
| BIKE | 10 | Motorbike (not available in all products) |
| MOWER | 11 | Robotic lawn mower (not available in all products) |
| ESCOOTER | 12 | E-scooter (not available in all products) |

Table 28: Constants for CFG-NAVSPG-DYNMODEL

| Constant | Value | Description |
| --- | --- | --- |
| DIS | 0 | Disable signal attenuation compensation |
| AUTO | 255 | Automatic signal attenuation compensation |
| 01DBHZ | 1 | Maximum expected C/NO level is 1 dBHz |
| 02DBHZ | 2 | Maximum expected C/NO level is 2 dBHz |
| 03DBHZ | 3 | Maximum expected C/NO level is 3 dBHz |
| 04DBHZ | 4 | Maximum expected C/NO level is 4 dBHz |
| 05DBHZ | 5 | Maximum expected C/NO level is 5 dBHz |
| 06DBHZ | 6 | Maximum expected C/NO level is 6 dBHz |
| 07DBHZ | 7 | Maximum expected C/NO level is 7 dBHz |
| 08DBHZ | 8 | Maximum expected C/NO level is 8 dBHz |
| 09DBHZ | 9 | Maximum expected C/NO level is 9 dBHz |
| 10DBHZ | 10 | Maximum expected C/NO level is 10 dBHz |
| 11DBHZ | 11 | Maximum expected C/NO level is 11 dBHz |
| 12DBHZ | 12 | Maximum expected C/NO level is 12 dBHz |
| 13DBHZ | 13 | Maximum expected C/NO level is 13 dBHz |
| 14DBHZ | 14 | Maximum expected C/NO level is 14 dBHz |
| 15DBHZ | 15 | Maximum expected C/NO level is 15 dBHz |
| 16DBHZ | 16 | Maximum expected C/NO level is 16 dBHz |
| 17DBHZ | 17 | Maximum expected C/NO level is 17 dBHz |
| 18DBHZ | 18 | Maximum expected C/NO level is 18 dBHz |
| 19DBHZ | 19 | Maximum expected C/NO level is 19 dBHz |
| 20DBHZ | 20 | Maximum expected C/NO level is 20 dBHz |
| 21DBHZ | 21 | Maximum expected C/NO level is 21 dBHz |
| 22DBHZ | 22 | Maximum expected C/NO level is 22 dBHz |
| 23DBHZ | 23 | Maximum expected C/NO level is 23 dBHz |
| 24DBHZ | 24 | Maximum expected C/NO level is 24 dBHz |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 150 of 181

<!-- page 151 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Constant | Value | Description |
| --- | --- | --- |
| 25DBHZ | 25 | Maximum expected C/NO level is 25 dBHz |
| 26DBHZ | 26 | Maximum expected C/NO level is 26 dBHz |
| 27DBHZ | 27 | Maximum expected C/NO level is 27 dBHz |
| 28DBHZ | 28 | Maximum expected C/NO level is 28 dBHz |
| 29DBHZ | 29 | Maximum expected C/NO level is 29 dBHz |
| 30DBHZ | 30 | Maximum expected C/NO level is 30 dBHz |
| 31DBHZ | 31 | Maximum expected C/NO level is 31 dBHz |
| 32DBHZ | 32 | Maximum expected C/NO level is 32 dBHz |
| 33DBHZ | 33 | Maximum expected C/NO level is 33 dBHz |
| 34DBHZ | 34 | Maximum expected C/NO level is 34 dBHz |
| 35DBHZ | 35 | Maximum expected C/NO level is 35 dBHz |
| 36DBHZ | 36 | Maximum expected C/NO level is 36 dBHz |
| 37DBHZ | 37 | Maximum expected C/NO level is 37 dBHz |
| 38DBHZ | 38 | Maximum expected C/NO level is 38 dBHz |
| 39DBHZ | 39 | Maximum expected C/NO level is 39 dBHz |
| 40DBHZ | 40 | Maximum expected C/NO level is 40 dBHz |
| 41DBHZ | 41 | Maximum expected C/NO level is 41 dBHz |
| 42DBHZ | 42 | Maximum expected C/NO level is 42 dBHz |
| 43DBHZ | 43 | Maximum expected C/NO level is 43 dBHz |
| 44DBHZ | 44 | Maximum expected C/NO level is 44 dBHz |
| 45DBHZ | 45 | Maximum expected C/NO level is 45 dBHz |
| 46DBHZ | 46 | Maximum expected C/NO level is 46 dBHz |
| 47DBHZ | 47 | Maximum expected C/NO level is 47 dBHz |
| 48DBHZ | 48 | Maximum expected C/NO level is 48 dBHz |
| 49DBHZ | 49 | Maximum expected C/NO level is 49 dBHz |
| 50DBHZ | 50 | Maximum expected C/NO level is 50 dBHz |
| 51DBHZ | 51 | Maximum expected C/NO level is 51 dBHz |
| 52DBHZ | 52 | Maximum expected C/NO level is 52 dBHz |
| 53DBHZ | 53 | Maximum expected C/NO level is 53 dBHz |
| 54DBHZ | 54 | Maximum expected C/NO level is 54 dBHz |
| 55DBHZ | 55 | Maximum expected C/NO level is 55 dBHz |
| 56DBHZ | 56 | Maximum expected C/NO level is 56 dBHz |
| 57DBHZ | 57 | Maximum expected C/NO level is 57 dBHz |
| 58DBHZ | 58 | Maximum expected C/NO level is 58 dBHz |
| 59DBHZ | 59 | Maximum expected C/NO level is 59 dBHz |
| 60DBHZ | 60 | Maximum expected C/NO level is 60 dBHz |
| 61DBHZ | 61 | Maximum expected C/NO level is 61 dBHz |
| 62DBHZ | 62 | Maximum expected C/NO level is 62 dBHz |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 151 of 181

<!-- page 152 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Constant | Value | Description |
| --- | --- | --- |
| 63DBHZ | 63 | Maximum expected C/NO level is 63 dBHz |

Table 29: Constants for CFG-NAVSPG-SIGATTCOMP

## 4.9.13 CFG-NMEA: NMEA protocol configuration

This group configures the NMEA protocol. See section NMEA protocol configuration for a detailed description of the configuration effects on NMEA output.

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-NMEA-PROTVER</td><td>0x20930001</td><td>E1</td><td>-</td><td>-</td><td>NMEA protocol version</td></tr><tr><td colspan="6">See Table 31 below for a list of possible constants for this item.</td></tr><tr><td>CFG-NMEA-MAXSVS</td><td>0x20930002</td><td>E1</td><td>-</td><td>-</td><td>Maximum number of SVs to report per Talker ID</td></tr><tr><td colspan="6">See Table 32 below for a list of possible constants for this item.</td></tr><tr><td>CFG-NMEA-COMPAT</td><td>0x10930003</td><td>L</td><td>-</td><td>-</td><td>Enable compatibility mode</td></tr><tr><td colspan="6">This might be needed for certain applications, e.g. for an NMEA parser that expects a fixed number of digits in position coordinates.</td></tr><tr><td>CFG-NMEA-CONSIDER</td><td>0x10930004</td><td>L</td><td>-</td><td>-</td><td>Enable considering mode</td></tr><tr><td colspan="6">This affects the way the used satellite count in NMEA output is calculated. If set, also considered but rejected satellites (e.g. RAIMED) are counted as used satellites as well.</td></tr><tr><td>CFG-NMEA-LIMIT82</td><td>0x10930005</td><td>L</td><td>-</td><td>-</td><td>Enable strict limit to 82 characters maximum NMEA message length</td></tr><tr><td>CFG-NMEA-HIGHPREC</td><td>0x10930006</td><td>L</td><td>-</td><td>-</td><td>Enable high precision mode</td></tr><tr><td colspan="6">This flag cannot be set in conjunction with either CFG-NMEA-COMPAT or CFG-NMEA-LIMIT82 mode.</td></tr><tr><td>CFG-NMEA-SVNUMBERING</td><td>0x20930007</td><td>E1</td><td>-</td><td>-</td><td>Display configuration for SVs that do not have value defined in NMEA</td></tr><tr><td colspan="6">Configures the display of satellites that do not have an NMEA-defined value.</td></tr><tr><td colspan="6">Note: this does not apply to satellites with an unknown ID.</td></tr></table>

See also Satellite Numbering.

See Table 33 below for a list of possible constants for this item.

| CFG-NMEA-FILT_GPS | 0x10930011 | L | - | - | Disable reporting of GPS satellites |
| --- | --- | --- | --- | --- | --- |
| CFG-NMEA-FILT_SBAS | 0x10930012 | L | - | - | Disable reporting of SBAS satellites |
| CFG-NMEA-FILT_GAL | 0x10930013 | L | - | - | Disable reporting of Galileo satellites |
| CFG-NMEA-FILT_QZSS | 0x10930015 | L | - | - | Disable reporting of QZSS satellites |
| CFG-NMEA-FILT_BDS | 0x10930017 | L | - | - | Disable reporting of BeiDou satellites |
| CFG-NMEA-FILT_NAVIC | 0x10930018 | L | - | - | Disable reporting of NavIC satellites |
| CFG-NMEA-OUT_INVFIX | 0x10930021 | L | - | - | Enable position output for failed or invalid fixes |
| CFG-NMEA-OUT_MSKFIX | 0x10930022 | L | - | - | Enable position output for invalid fixes |
| CFG-NMEA-OUT_INVTIME | 0x10930023 | L | - | - | Enable time output for invalid times |
| CFG-NMEA-OUT_INVDATE | 0x10930024 | L | - | - | Enable date output for invalid dates |
| CFG-NMEA-OUT_ONLYGPS | 0x10930025 | L | - | - | Restrict output to GPS satellites only |
| CFG-NMEA-OUT_FROZENCOG | 0x10930026 | L | - | - | Enable course over ground output even if it is frozen |
| CFG-NMEA-MAINTALKERID | 0x20930031 | E1 | - | - | Main Talker ID |

By default the main Talker ID (i.e. the Talker ID used for all messages other than GSV) is determined by the GNSS assignment of the receiver's channels (see CFG-SIGNAL).

See Table 34 below for a list of possible constants for this item.

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 152 of 181

<!-- page 153 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID | Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- | --- |
| CFG-NMEA-GSVTALKERID | 0x20930032 | E1 | - | - | Talker ID for GSV NMEA messages |

By default the Talker ID for GSV messages is GNSS-specific (as defined by NMEA).

This field enables the GSV Talker ID to be overridden.

See Table 35 below for a list of possible constants for this item.

| CFG-NMEA-BDSTALKERID | 0x30930033 | U2 | - | - | BeiDou Talker ID |
| --- | --- | --- | --- | --- | --- |

Sets the two ASCII characters that should be used for the BeiDou Talker ID.

If these are set to zero, the receiver uses the default BeiDou Talker ID.

## Table 30: CFG-NMEA configuration items

| Constant | Value | Description |
| --- | --- | --- |
| V21 | 21 | NMEA protocol version 2.1 |
| V23 | 23 | NMEA protocol version 2.3 |
| V40 | 40 | NMEA protocol version 4.0 (not available in all products) |
| V41 | 41 | NMEA protocol version 4.10 (not available in all products) |
| V411 | 42 | NMEA protocol version 4.11 (not available in all products) |

Table 31: Constants for CFG-NMEA-PROTVER

| Constant | Value | Description |
| --- | --- | --- |
| UNLIM | 0 | Unlimited |
| 8SVS | 8 | 8SVs |
| 12SVS | 12 | 12SVs |
| 16SVS | 16 | 16SVs |

Table 32: Constants for CFG-NMEA-MAXSVS

| Constant | Value | Description |
| --- | --- | --- |
| STRICT | 0 | Strict - satellites are not output |
| EXTENDED | 1 | Extended - use proprietary numbering |

Table 33: Constants for CFG-NMEA-SVNUMBERING

| Constant | Value | Description |
| --- | --- | --- |
| AUTO | 0 | Main Talker ID is not overridden |
| GP | 1 | Set main Talker ID to 'GP' |
| GL | 2 | Set main Talker ID to 'GL' |
| GN | 3 | Set main Talker ID to 'GN' |
| GA | 4 | Set main Talker ID to 'GA' (not available in all products) |
| GB | 5 | Set main Talker ID to 'GB' (not available in all products) |
| GQ | 7 | Set main Talker ID to 'GQ' (not available in all products) |

Table 34: Constants for CFG-NMEA-MAINTALKERID

| Constant | Value | Description |
| --- | --- | --- |
| GNSS | 0 | Use GNSS-specific Talker ID (as defined by NMEA) |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 153 of 181

<!-- page 154 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Constant | Value | Description |
| --- | --- | --- |
| MAIN | 1 | UsethemainTalkerID |

Table 35: Constants for CFG-NMEA-GSVTALKERID

## 4.9.14 CFG-ODO: Odometer and low-speed course over ground filter configuration

The items in this group allow the user to configure the Odometer feature and Low-Speed Course Over Ground Filter.

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-ODO-USE_ODO</td><td>0x10220001</td><td>L</td><td>-</td><td>-</td><td>Use odometer</td></tr><tr><td>CFG-ODO-USE_COG</td><td>0x10220002</td><td>L</td><td>-</td><td>-</td><td>Use low-speed course over ground filter</td></tr><tr><td>CFG-ODO-OUTLPVEL</td><td>0x10220003</td><td>L</td><td>-</td><td>-</td><td>Output low-pass filtered velocity</td></tr><tr><td>CFG-ODO-OUTLPCOG</td><td>0x10220004</td><td>L</td><td>-</td><td>-</td><td>Output low-pass filtered course over ground (heading)</td></tr><tr><td>CFG-ODO-PROFILE</td><td>0x20220005</td><td>E1</td><td>-</td><td>-</td><td>Odometer profile configuration</td></tr><tr><td colspan="6">See Table 37 below for a list of possible constants for this item.</td></tr><tr><td>CFG-ODO-COGMAXSPEED</td><td>0x20220021</td><td>U1</td><td>1e-1</td><td>m/s</td><td>Upper speed limit for low-speed course over ground filter</td></tr><tr><td>CFG-ODO-COGMAXPOSACC</td><td>0x20220022</td><td>U1</td><td>-</td><td>-</td><td>Maximum acceptable position accuracy for computing low-speed filtered course over ground</td></tr><tr><td>CFG-ODO-VELLPGAIN</td><td>0x20220031</td><td>U1</td><td>-</td><td>-</td><td>Velocity low-pass filter level</td></tr><tr><td colspan="6">Range is from 0 to 255.</td></tr><tr><td>CFG-ODO-COGLPGAIN</td><td>0x20220032</td><td>U1</td><td>-</td><td>-</td><td>Course over ground low-pass filter level (at speed &lt; 8 m/s)</td></tr></table>

Range is from 0 to 255.

Table 36: CFG-ODO configuration items

| Constant | Value | Description |
| --- | --- | --- |
| RUN | 0 | Running |
| CYCL | 1 | Cycling |
| SWIM | 2 | Swimming |
| CAR | 3 | Car |
| CUSTOM | 4 | Custom |

Table 37: Constants for CFG-ODO-PROFILE

## 4.9.15 CFG-QZSS: QZSS system configuration

Note that enabling and disabling of individual GNSS is done via the CFG-SIGNAL configuration group.

| Configuration item | Key ID | Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- | --- |
| CFG-QZSS-USE_SLAS_DGNSS | 0x10370005 | L | - | - | Apply QZSS SLAS DGNSS corrections |
| CFG-QZSS-USE_SLAS_TESTMODE | 0x10370006 | L | - | - | Use QZSS SLAS data when it is in test mode (SLAS msg 0) |
| CFG-QZSS-USE_SLAS_RAIM_ | 0x10370007 | L | - | - | Raim out measurements that are not corrected |
| UNCORR |  |  |  |  | by QZSS SLAS, if at least 5 measurements are corrected |
| CFG-QZSS-SLAS_MAX_BASELINE | 0x30370008 | U2 | - | km | Maximum baseline distance to closest GMS |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 154 of 181

<!-- page 155 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td colspan="6">SLAS corrections are only applied if the receiver is at most this far away from the closest ground monitoring station (GMS). Note that due to the nature of the service, the usefulness of corrections degrades with distance. When far away from GMS, SBAS may be a better correction source.</td></tr></table>

Table 38: CFG-QZSS configuration items

## 4.9.16 CFG-RATE: Navigation and measurement rate configuration

The configuration items in this group allow the user to alter the rate at which navigation solutions (and the measurements that they depend on) are generated by the receiver. The calculation of the navigation solution is aligned to the top of a second zero (first second of the week) of the configured reference time system. The navigation period is an integer multiple of the measurement period.

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-RATE-MEAS</td><td>0x30210001</td><td>U2</td><td>0.001</td><td>s</td><td>Nominal time between GNSS measurements</td></tr><tr><td colspan="6">E.g. 100 ms results in 10 Hz measurement rate, 1000 ms = 1 Hz measurement rate.</td></tr><tr><td>CFG-RATE-NAV</td><td>0x30210002</td><td>U2</td><td>-</td><td>-</td><td>Ratio of number of measurements to number of navigation solutions</td></tr><tr><td colspan="6">E.g. 5 means five measurements for every navigation solution. The minimum value is 1. The maximum value is 127.</td></tr><tr><td>CFG-RATE-TIMEREF</td><td>0x20210003</td><td>E1</td><td>-</td><td>-</td><td>Time system to which measurements are aligned</td></tr></table>

See Table 40 below for a list of possible constants for this item.

Table 39: CFG-RATE configuration items

| Constant | Value | Description |
| --- | --- | --- |
| UTC | 0 | Align measurements to UTC time |
| GPS | 1 | Align measurements to GPS time |
| GLO | 2 | Align measurements to GLONASS time |
| BDS | 3 | Align measurements to BeiDou time |
| GAL | 4 | Align measurements to Galileo time |
| NAVIC | 5 | Align measurements to NavIC time |

Table 40: Constants for CFG-RATE-TIMEREF

## 4.9.17 CFG-RINV: Remote inventory

The remote inventory enables storing user-defined data in the receiver's non-volatile memory. The data can be either binary or a string of ASCII characters. In the latter case, it can optionally be output at startup after the boot screen.

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-RINV-DUMP</td><td>0x10c70001</td><td>L</td><td>-</td><td>-</td><td>Dump data at startup</td></tr><tr><td colspan="6">When true, data is dumped to the interface at startup, unless CFG-RINV-BINARY is set.</td></tr><tr><td>CFG-RINV-BINARY</td><td>0x10c70002</td><td>L</td><td>-</td><td>-</td><td>Data is binary</td></tr><tr><td colspan="6">When true, the data is treated as binary data.</td></tr><tr><td>CFG-RINV-DATA_SIZE</td><td>0x20c70003</td><td>U1</td><td>-</td><td>-</td><td>Size of data</td></tr><tr><td colspan="6">Size of data to store/stored in the remote inventory (maximum 30 bytes).</td></tr><tr><td>CFG-RINV-CHUNK0</td><td>0x50c70004</td><td>X8</td><td>-</td><td>-</td><td>Data bytes 1-8 (LSB)</td></tr><tr><td colspan="6">Data to store/stored in remote inventory - max 8 bytes, left-most in LSB, e.g. string ABCD will appear as 0x44434241.</td></tr><tr><td>CFG-RINV-CHUNK1</td><td>0x50c70005</td><td>X8</td><td>-</td><td>-</td><td>Data bytes 9-16</td></tr><tr><td colspan="6">Data to store/stored in remote inventory - max 8 bytes, left-most in LSB, e.g. string ABCD will appear as 0x44434241.</td></tr></table>

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 155 of 181

<!-- page 156 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-RINV-CHUNK2</td><td>0x50c70006</td><td>X8</td><td>-</td><td>-</td><td>Data bytes 17-24</td></tr><tr><td colspan="6">Data to store/stored in remote inventory - max 8 bytes, left-most in LSB, e.g. string ABCD will appear as 0x44434241.</td></tr><tr><td>CFG-RINV-CHUNK3</td><td>0x50c70007</td><td>X8</td><td>-</td><td>-</td><td>Data bytes 25-30 (MSB)</td></tr><tr><td colspan="6">Data to store/stored in remote inventory - max 6 bytes, left-most in LSB, e.g. string ABCD will appear as 0x44434241.</td></tr></table>

Table 41: CFG-RINV configuration items

## 4.9.18 CFG-SBAS: SBAS configuration

This group configures the SBAS receiver subsystem (i.e. WAAS, EGNOS, MSAS). See SBAS configuration settings description in the integration manual for a detailed description of how these settings affect receiver operation.

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td colspan="2">Unit Description</td></tr><tr><td>CFG-SBAS-USE_TESTMODE</td><td>0x10360002</td><td>L</td><td>-</td><td>-</td><td>Use SBAS data when it is in test mode (SBAS msg 0)</td></tr><tr><td>CFG-SBAS-USE_RANGING</td><td>0x10360003</td><td>L</td><td>-</td><td>-</td><td>Use SBAS GEOs as a ranging source (for navigation)</td></tr><tr><td>CFG-SBAS-USE_DIFFCORR</td><td>0x10360004</td><td>L</td><td>-</td><td>-</td><td>Use SBAS differential corrections</td></tr><tr><td>CFG-SBAS-USE_INTEGRITY</td><td>0x10360005</td><td>L</td><td>-</td><td>-</td><td>Use SBAS integrity information</td></tr></tbody></table>

| CFG-SBAS-ACCEPT_NOT_IN_PRNMASK | 0x30360008 | X2 | - | - | Accept corrections from SBAS SV, even if not self included in PRN MASK (Message Type 1) |
| --- | --- | --- | --- | --- | --- |

If enabled, the receiver will still use the SBAS data, even when the SBAS SV itself is not included in its PRN MASK. This is only useful for BDSBAS and not compatible whith current EGNOS implementation.

See Table 43 below for a list of possible constants for this item.

| CFG-SBAS-USE_IONOONLY | 0x10360007 | L | - | - | Use SBAS ionosphere correction only |
| --- | --- | --- | --- | --- | --- |
| CFG-SBAS-PRNSCANMASK | 0x50360006 | X8 | - | - | SBAS PRN search configuration |

This configuration item determines which SBAS PRNs should be searched. Setting it to 0 indicates auto-scanning all SBAS PRNs. For non-zero values the bits correspond to the allocated SBAS PRNs ranging from PRN120 (bit 0) to PRN158 (bit 38), where a bit set enables searching for the corresponding PRN. See Table 44 below for a list of possible constants for this item See Table 44 below for a list of possible constants for this item.

Table 42: CFG-SBAS configuration items

| Constant | Value | Description |
| --- | --- | --- |
| WAAS | 0x01 | WAAS bit |
| 1 = Use WAAS provider Id. |  |  |
| EGNOS | 0x02 | EGNOS bit |
| 1 = Use EGNOS provider Id. |  |  |
| MSAS | 0x04 | MSAS bit |
| 1 = Use MSAS provider Id. |  |  |
| GAGAN | 0x08 | GAGAN bit |
| 1 = Use GAGAN provider Id. |  |  |
| SDCM | 0x10 | SDCM bit |
| 1 = Use SDCM provider Id. |  |  |
| BDSBAS | 0x20 | BDSBAS bit |
| 1 = Use BDSBAS provider Id. |  |  |
| KASS | 0x40 | KASS bit |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 156 of 181

<!-- page 157 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tbody><tr><td colspan="3">Table 43: Constants for CFG-SBAS-ACCEPT_NOT_IN_PRNMASK</td></tr><tr><td>Constant</td><td>Value</td><td>Description</td></tr><tr><td>ALL</td><td>0x0000000000000000</td><td>Enable search for all SBAS PRNs</td></tr><tr><td>PRN120</td><td>0x0000000000000001</td><td>Enable search for SBAS PRN120</td></tr><tr><td>PRN121</td><td>0x0000000000000002</td><td>Enable search for SBAS PRN121</td></tr><tr><td>PRN122</td><td>0x0000000000000004</td><td>Enable search for SBAS PRN122</td></tr><tr><td>PRN123</td><td>0x0000000000000008</td><td>Enable search for SBAS PRN123</td></tr><tr><td>PRN124</td><td>0x0000000000000010</td><td>Enable search for SBAS PRN124</td></tr><tr><td>PRN125</td><td>0x0000000000000020</td><td>Enable search for SBAS PRN125</td></tr><tr><td>PRN126</td><td>0x0000000000000040</td><td>Enable search for SBAS PRN126</td></tr><tr><td>PRN127</td><td>0x0000000000000080</td><td>Enable search for SBAS PRN127</td></tr><tr><td>PRN128</td><td>0x0000000000000100</td><td>Enable search for SBAS PRN128</td></tr><tr><td>PRN129</td><td>0x0000000000000200</td><td>Enable search for SBAS PRN129</td></tr><tr><td>PRN130</td><td>0x0000000000000400</td><td>Enable search for SBAS PRN130</td></tr><tr><td>PRN131</td><td>0x0000000000000800</td><td>Enable search for SBAS PRN131</td></tr><tr><td>PRN132</td><td>0x0000000000001000</td><td>Enable search for SBAS PRN132</td></tr><tr><td>PRN133</td><td>0x0000000000002000</td><td>Enable search for SBAS PRN133</td></tr><tr><td>PRN134</td><td>0x0000000000004000</td><td>Enable search for SBAS PRN134</td></tr><tr><td>PRN135</td><td>0x0000000000008000</td><td>Enable search for SBAS PRN135</td></tr><tr><td>PRN136</td><td>0x0000000000010000</td><td>Enable search for SBAS PRN136</td></tr><tr><td>PRN137</td><td>0x0000000000020000</td><td>Enable search for SBAS PRN137</td></tr><tr><td>PRN138</td><td>0x0000000000040000</td><td>Enable search for SBAS PRN138</td></tr><tr><td>PRN139</td><td>0x0000000000080000</td><td>Enable search for SBAS PRN139</td></tr><tr><td>PRN140</td><td>0x0000000000100000</td><td>Enable search for SBAS PRN140</td></tr><tr><td>PRN141</td><td>0x0000000000200000</td><td>Enable search for SBAS PRN141</td></tr><tr><td>PRN142</td><td>0x0000000000400000</td><td>Enable search for SBAS PRN142</td></tr><tr><td>PRN143</td><td>0x0000000000800000</td><td>Enable search for SBAS PRN143</td></tr><tr><td>PRN144</td><td>0x0000000001000000</td><td>Enable search for SBAS PRN144</td></tr><tr><td>PRN145</td><td>0x0000000002000000</td><td>Enable search for SBAS PRN145</td></tr><tr><td>PRN146</td><td>0x0000000004000000</td><td>Enable search for SBAS PRN146</td></tr><tr><td>PRN147</td><td>0x0000000008000000</td><td>Enable search for SBAS PRN147</td></tr><tr><td>PRN148</td><td>0x0000000010000000</td><td>Enable search for SBAS PRN148</td></tr><tr><td>PRN149</td><td>0x0000000020000000</td><td>Enable search for SBAS PRN149</td></tr><tr><td>PRN150</td><td>0x0000000040000000</td><td>Enable search for SBAS PRN150</td></tr><tr><td>PRN151</td><td>0x0000000080000000</td><td>Enable search for SBAS PRN151</td></tr><tr><td>PRN152</td><td>0x0000000100000000</td><td>Enable search for SBAS PRN152</td></tr><tr><td>PRN153</td><td>0x0000000200000000</td><td>Enable search for SBAS PRN153</td></tr><tr><td>PRN154</td><td>0x0000000400000000</td><td>Enable search for SBAS PRN154</td></tr><tr><td>PRN155</td><td>0x0000000800000000</td><td>Enable search for SBAS PRN155</td></tr></tbody></table>

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 157 of 181

<!-- page 158 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Constant | Value | Description |
| --- | --- | --- |
| PRN156 | 0x0000001000000000 | Enable search for SBAS PRN156 |
| PRN157 | 0x0000002000000000 | Enable search for SBAS PRN157 |
| PRN158 | 0x0000004000000000 | Enable search for SBAS PRN158 |

Table 44: Constants for CFG-SBAS-PRNSCANMASK

## 4.9.19 CFG-SEC: Security configuration

Security configuration.

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-SEC-CFG_LOCK</td><td>0x10f60009</td><td>L</td><td>-</td><td>-</td><td>Configuration lockdown</td></tr><tr><td colspan="6">When set, the receiver configuration is locked and cannot be changed any more.</td></tr><tr><td>CFG-SEC-CFG_LOCK_UNLOCKGRP1</td><td>0x30f6000a</td><td>U2</td><td>-</td><td>-</td><td>Configuration lockdown exempted group 1</td></tr><tr><td colspan="6">This item can be set before enabling the configuration lockdown. It enables writing to the specified group even after the configuration lockdown has been enabled.</td></tr><tr><td>CFG-SEC-CFG_LOCK_UNLOCKGRP2</td><td>0x30f6000b</td><td>U2</td><td>-</td><td>-</td><td>Configuration lockdown exempted group 2</td></tr><tr><td colspan="6">This item can be set before enabling the configuration lockdown. It enables writing to the specified group even after the configuration lockdown has been enabled.</td></tr></table>

Table 45: CFG-SEC configuration items

## 4.9.20 CFG-SIGNAL: Satellite systems (GNSS) signal configuration

The enable items for individual signals are governed by their corresponding constellation enable item. It is necessary that at least one signal from a major GNSS constellation is enabled. See GNSS signal configuration in the integration manual for more details.

Configuration specific to a GNSS system is available in other groups (e.g. CFG-SBAS).

Note that changes to any items within this group triggers a reset to the GNSS subsystem. The reset takes some time, so wait first for the acknowledgement from the receiver and then 0.5 seconds before sending the next command.

<table><tbody><tr><td>Configurationitem</td><td>KeyID</td><td colspan="2">Type Scale</td><td colspan="2">Unit Description</td></tr><tr><td>CFG-SIGNAL-GPS_ENA</td><td>0x1031001f</td><td>L</td><td>-</td><td>-</td><td>GPSenable</td></tr><tr><td>CFG-SIGNAL-GPS_L1CA_ENA</td><td>0x10310001</td><td>L</td><td>-</td><td>-</td><td>GPSL1C/A</td></tr><tr><td>CFG-SIGNAL-GPS_L5_ENA</td><td>0x10310004</td><td>L</td><td>-</td><td>-</td><td>GPSL5</td></tr><tr><td>CFG-SIGNAL-SBAS_ENA</td><td>0x10310020</td><td>L</td><td>-</td><td>-</td><td>SBASenable</td></tr><tr><td>CFG-SIGNAL-SBAS_L1CA_ENA</td><td>0x10310005</td><td>L</td><td>-</td><td>-</td><td>SBASL1C/A</td></tr><tr><td>CFG-SIGNAL-GAL_ENA</td><td>0x10310021</td><td>L</td><td>-</td><td>-</td><td>Galileo enable</td></tr><tr><td>CFG-SIGNAL-GAL_E1_ENA</td><td>0x10310007</td><td>L</td><td>-</td><td>-</td><td>Galileo E1</td></tr><tr><td>CFG-SIGNAL-GAL_E5A_ENA</td><td>0x10310009</td><td>L</td><td>-</td><td>-</td><td>Galileo E5a</td></tr><tr><td>CFG-SIGNAL-BDS_ENA</td><td>0x10310022</td><td>L</td><td>-</td><td>-</td><td>BeiDouEnable</td></tr><tr><td>CFG-SIGNAL-BDS_B1_ENA</td><td>0x1031000d</td><td>L</td><td>-</td><td>-</td><td>BeiDouB1I</td></tr><tr><td>CFG-SIGNAL-BDS_B1C_ENA</td><td>0x1031000f</td><td>L</td><td>-</td><td>-</td><td>BeiDouB1C</td></tr><tr><td>CFG-SIGNAL-BDS_B2A_ENA</td><td>0x10310028</td><td>L</td><td>-</td><td>-</td><td>BeiDouB2a</td></tr><tr><td>CFG-SIGNAL-QZSS_ENA</td><td>0x10310024</td><td>L</td><td>-</td><td>-</td><td>QZSSenable</td></tr><tr><td>CFG-SIGNAL-QZSS_L1CA_ENA</td><td>0x10310012</td><td>L</td><td>-</td><td>-</td><td>QZSSL1C/A</td></tr><tr><td>CFG-SIGNAL-QZSS_L1S_ENA</td><td>0x10310014</td><td>L</td><td>-</td><td>-</td><td>QZSSL1S</td></tr></tbody></table>

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 158 of 181

<!-- page 159 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tbody><tr><td>Configurationitem</td><td>KeyID</td><td colspan="2">Type Scale</td><td colspan="2">Unit Description</td></tr><tr><td>CFG-SIGNAL-QZSS_L5_ENA</td><td>0x10310017</td><td>L</td><td>-</td><td>-</td><td>QZSSL5</td></tr><tr><td>CFG-SIGNAL-NAVIC_ENA</td><td>0x10310026</td><td>L</td><td>-</td><td>-</td><td>NavICenable</td></tr><tr><td>CFG-SIGNAL-NAVIC_L5_ENA</td><td>0x1031001d</td><td>L</td><td>-</td><td>-</td><td>NavICL5</td></tr></tbody></table>

Table 46: CFG-SIGNAL configuration items

## 4.9.21 CFG-SPI: Configuration of the SPI interface

Settings needed to configure the SPI communication interface.

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-SPI-MAXFF</td><td>0x20640001</td><td>U1</td><td>-</td><td>-</td><td>Number of bytes containing 0xFF to receive before switching off reception. Range: 0 (mechanism off) - 63</td></tr><tr><td>CFG-SPI-CPOLARITY</td><td>0x10640002</td><td>L</td><td>-</td><td>-</td><td>Clock polarity select: 0: Active Hight Clock, SCLK idles low, 1: Active Low Clock, SCLK idles high</td></tr><tr><td>CFG-SPI-CPHASE</td><td>0x10640003</td><td>L</td><td>-</td><td>-</td><td>Clock phase select: 0: Data captured on first edge of SCLK, 1: Data captured on second edge of SCLK</td></tr><tr><td>CFG-SPI-EXTENDEDTIMEOUT</td><td>0x10640005</td><td>L</td><td>-</td><td>-</td><td>Flag to disable timeouting the interface after 1.5s</td></tr><tr><td>CFG-SPI-ENABLED</td><td>0x10640006</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if the SPI interface should be enabled</td></tr></tbody></table>

Table 47: CFG-SPI configuration items

## 4.9.22 CFG-SPIINPROT: Input protocol configuration of the SPI interface Input protocol enable flags of the SPI interface.

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td colspan="2">Unit Description</td></tr><tr><td>CFG-SPIINPROT-UBX</td><td>0x10790001</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if UBX should be an input protocol on SPI</td></tr><tr><td>CFG-SPIINPROT-NMEA</td><td>0x10790002</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if NMEA should be an input protocol on SPI</td></tr></tbody></table>

Table 48: CFG-SPIINPROT configuration items

## 4.9.23 CFG-SPIOUTPROT: Output protocol configuration of the SPI interface Output protocol enable flags of the SPI interface.

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td colspan="2">Unit Description</td></tr><tr><td>CFG-SPIOUTPROT-UBX</td><td>0x107a0001</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if UBX should be an output protocol on SPI</td></tr><tr><td>CFG-SPIOUTPROT-NMEA</td><td>0x107a0002</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if NMEA should be an output protocol on SPI</td></tr></tbody></table>

Table 49: CFG-SPIOUTPROT configuration items

## 4.9.24 CFG-TP: Time pulse configuration

Use this group to configure the generation of time pulses.

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-TP-PULSE_DEF</td><td>0x20050023</td><td>E1</td><td>-</td><td>-</td><td>Determines whether the time pulse is interpreted as frequency or period</td></tr><tr><td colspan="6">See Table 51 below for a list of possible constants for this item.</td></tr></table>

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 159 of 181

<!-- page 160 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-TP-PULSE_LENGTH_DEF</td><td>0x20050030</td><td>E1</td><td>-</td><td>-</td><td>Determines whether the time pulse length is interpreted as length[us] or pulse ratio[%]</td></tr><tr><td colspan="6">See Table 52 below for a list of possible constants for this item.</td></tr><tr><td>CFG-TP-ANT_CABLEDELAY</td><td>0x30050001</td><td>I2</td><td>1e-9</td><td>s</td><td>Antenna cable delay in [ns]</td></tr><tr><td>CFG-TP-PERIOD_TP1</td><td>0x40050002</td><td>U4</td><td>1e-6</td><td>s</td><td>Time pulse period (TP1) in [us]</td></tr><tr><td colspan="6">This is used only if CFG-TP-PULSE_DEF=PERIOD.</td></tr><tr><td>CFG-TP-PERIOD_LOCK_TP1</td><td>0x40050003</td><td>U4</td><td>1e-6</td><td>s</td><td>Time pulse period when locked to GNSS time (TP1) in [us]</td></tr><tr><td colspan="6">Only used if CFG-TP-PULSE_DEF=PERIOD and CFG-TP-USE_LOCKED_TP1 is set.</td></tr><tr><td>CFG-TP-FREQ_TP1</td><td>0x40050024</td><td>U4</td><td>-</td><td>Hz</td><td>Time pulse frequency (TP1) in [Hz]</td></tr><tr><td colspan="6">This is used only if CFG-TP-PULSE_DEF=FREQ.</td></tr><tr><td>CFG-TP-FREQ_LOCK_TP1</td><td>0x40050025</td><td>U4</td><td>-</td><td>Hz</td><td>Time pulse frequency when locked to GNSS time (TP1) in [Hz]</td></tr><tr><td colspan="6">Only used if CFG-TP-PULSE_DEF=FREQ and CFG-TP-USE_LOCKED_TP1 is set.</td></tr><tr><td>CFG-TP-LEN_TP1</td><td>0x40050004</td><td>U4</td><td>1e-6</td><td>s</td><td>Time pulse length (TP1) in [us]</td></tr><tr><td colspan="6">Only used if CFG-TP-PULSE_LENGTH_DEF=LENGTH is set.</td></tr><tr><td>CFG-TP-LEN_LOCK_TP1</td><td>0x40050005</td><td>U4</td><td>1e-6</td><td>s</td><td>Time pulse length when locked to GNSS time (TP1) in [us]</td></tr><tr><td colspan="6">Only used if CFG-TP-PULSE_LENGTH_DEF=LENGTH and CFG-TP-USE_LOCKED_TP1 is set.</td></tr><tr><td>CFG-TP-DUTY_TP1</td><td>0x5005002a</td><td>R8</td><td>-</td><td>%</td><td>Time pulse duty cycle (TP1) in [%]</td></tr><tr><td colspan="6">Only used if CFG-TP-PULSE_LENGTH_DEF=RATIO is set.</td></tr><tr><td>CFG-TP-DUTY_LOCK_TP1</td><td>0x5005002b</td><td>R8</td><td>-</td><td>%</td><td>Time pulse duty cycle when locked to GNSS time (TP1) in [%]</td></tr><tr><td colspan="6">Only used if CFG-TP-PULSE_LENGTH_DEF=RATIO and CFG-TP-USE_LOCKED_TP1 are set.</td></tr><tr><td>CFG-TP-USER_DELAY_TP1</td><td>0x40050006</td><td>I4</td><td>1e-9</td><td>s</td><td>User-configurable time pulse delay (TP1) in [ns]</td></tr><tr><td>CFG-TP-TP1_ENA</td><td>0x10050007</td><td>L</td><td>-</td><td>-</td><td>Enable the time pulse (TP1)</td></tr><tr><td colspan="6">if pin associated with time pulse is assigned for another function, the other function takes precedence. Must be set for frequency-time products.</td></tr><tr><td>CFG-TP-SYNC_GNSS_TP1</td><td>0x10050008</td><td>L</td><td>-</td><td>-</td><td>Sync time pulse to GNSS time or local clock (TP1)</td></tr><tr><td colspan="6">If set, sync to GNSS if GNSS time is valid. Otherwise, use local clock. This flag can be unset only in Timing product variants.</td></tr><tr><td>CFG-TP-USE_LOCKED_TP1</td><td>0x10050009</td><td>L</td><td>-</td><td>-</td><td>Use locked parameters when possible (TP1)</td></tr><tr><td colspan="6">If set, use CFG-TP-PERIOD_LOCK_TP1 and CFG-TP-LEN_LOCK_TP1 as soon as GNSS time is valid. Otherwise, use CFG-TP-PERIOD_TP1 and CFG-TP-LEN_TP1.</td></tr><tr><td>CFG-TP-ALIGN_TO_TOW_TP1</td><td>0x1005000a</td><td>L</td><td>-</td><td>-</td><td>Align time pulse to top of second (TP1)</td></tr><tr><td colspan="6">To use this feature, CFG-TP-SYNC_GNSS_TP1 must be set. Time pulse period must be an integer fraction of 1 second.</td></tr><tr><td>CFG-TP-POL_TP1</td><td>0x1005000b</td><td>L</td><td>-</td><td>-</td><td>Set time pulse polarity (TP1)</td></tr><tr><td colspan="6">false (0): falling edge at top of second. true (1): rising edge at top of second.</td></tr><tr><td>CFG-TP-TIMEGRID_TP1</td><td>0x2005000c</td><td>E1</td><td>-</td><td>-</td><td>Time grid to use (TP1)</td></tr></table>

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 160 of 181

<!-- page 161 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td colspan="6">Only relevant if CFG-TP-SYNC_GNSS_TP1 is set.</td></tr></table>

Note that configured GNSS time is estimated by the receiver if locked to any GNSS system. If the receiver has a valid GNSS fix it attempts to steer the TP to the specified time grid even if the specified time is not based on information from the constellation's satellites. To ensure timing based purely on a given GNSS, restrict the supported constellations in CFG-SIGNAL-\*.

No TP is generated if the selected GNSS constellation is not configured.

See Table 53 below for a list of possible constants for this item.

| CFG-TP-DRSTR_TP1 | 0x20050035 | E1 | - | - | Set drive strength of TP1 |
| --- | --- | --- | --- | --- | --- |

Time Pulse pin 1 (TP1) can support 4 possible drive strength cases: 2, 4, 8 and 12 mA

See Table 54 below for a list of possible constants for this item.

Table 50: CFG-TP configuration items

| Constant | Value | Description |
| --- | --- | --- |
| PERIOD | 0 | Time pulse period [us] |
| FREQ | 1 | Time pulse frequency [Hz] |

Table 51: Constants for CFG-TP-PULSE\_DEF

| Constant | Value | Description |
| --- | --- | --- |
| RATIO | 0 | Timepulseratio |
| LENGTH | 1 | Timepulselength |

**Table 52: Constants for CFG-TP-PULSE\_LENGTH\_DEF**

| Constant | Value | Description |
| --- | --- | --- |
| UTC | 0 | UTC time reference |
| GPS | 1 | GPS time reference |
| GLO | 2 | GLONASS time reference |
| BDS | 3 | BeiDou time reference |
| GAL | 4 | Galileo time reference |
| NAVIC | 5 | NavIC time reference |

Table 53: Constants for CFG-TP-TIMEGRID\_TP1

| Constant | Value | Description |
| --- | --- | --- |
| DRIVE_STRENGTH_2MA | 0 | 2mAdrivestrength |
| DRIVE_STRENGTH_4MA | 1 | 4mAdrivestrength |
| DRIVE_STRENGTH_8MA | 2 | 8mAdrivestrength |
| DRIVE_STRENGTH_12MA | 3 | 12mAdrivestrength |

Table 54: Constants for CFG-TP-DRSTR\_TP1

## 4.9.25 CFG-TXREADY: TX ready configuration

Configuration of the TX ready pin.

| Configuration item | Key ID | Type | Scale | Unit | Description |
| --- | --- | --- | --- | --- | --- |
| CFG-TXREADY-ENABLED | 0x10a20001 | L | - | - | Flag to indicate if TX ready pin mechanism should be enabled |
| CFG-TXREADY-POLARITY | 0x10a20002 | L | - | - | The polarity of the TX ready pin: false:high-active, true:low-active |
| CFG-TXREADY-PIN | 0x20a20003 | U1 | - | - | Pin number to use for the TX ready functionality |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 161 of 181

<!-- page 162 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tr><td>Configuration item</td><td>Key ID</td><td>Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-TXREADY-THRESHOLD</td><td>0x30a20004</td><td>U2</td><td>-</td><td>-</td><td>Amount of data that should be ready on the interface before triggering the TX ready pin</td></tr><tr><td colspan="6">The value is amount of 8-byte chunks. For example, value of 250 sets the trigger to 2000 bytes.</td></tr><tr><td>CFG-TXREADY-INTERFACE</td><td>0x20a20005</td><td>E1</td><td>-</td><td>-</td><td>Interface where the TX ready feature should be linked to</td></tr></table>

See Table 56 below for a list of possible constants for this item.

Table 55: CFG-TXREADY configuration items

| Constant | Value | Description |
| --- | --- | --- |
| I2C | 0 | I2Cinterface |
| SPI | 1 | SPIinterface |

Table 56: Constants for CFG-TXREADY-INTERFACE

## 4.9.26 CFG-UART1: Configuration of the UART1 interface

Settings needed to configure the UART1 communication interface.

<table><tbody><tr><td>Configuration item Key ID Type</td><td>Scale</td><td>Unit</td><td>Description</td></tr><tr><td>CFG-UART1-BAUDRATE 0x40520001 U4</td><td>-</td><td>-</td><td rowspan="2">The baud rate that should be configured on the UART1 Number of stopbits that should be used on UART1</td></tr><tr><td>CFG-UART1-STOPBITS 0x20520002 E1</td><td>-</td><td>-</td></tr></tbody></table>

See Table 58 below for a list of possible constants for this item.

<table><tr><td>CFG-UART1-DATABITS</td><td>0x20520003</td><td>E1</td><td>-</td><td>-</td><td>Number of databits that should be used on UART1</td></tr><tr><td colspan="6">See Table 59 below for a list of possible constants for this item.</td></tr><tr><td>CFG-UART1-PARITY</td><td>0x20520004</td><td>E1</td><td>-</td><td>-</td><td>Parity mode that should be used on UART1</td></tr><tr><td colspan="6">See Table 60 below for a list of possible constants for this item.</td></tr><tr><td>CFG-UART1-ENABLED</td><td>0x10520005</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if the UART1 should be enabled</td></tr></table>

Table 57: CFG-UART1 configuration items

| Constant | Value | Description |
| --- | --- | --- |
| HALF | 0 | 0.5 stopbits |
| ONE | 1 | 1.0 stopbits |
| ONEHALF | 2 | 1.5 stopbits |
| TWO | 3 | 2.0 stopbits |

Table 58: Constants for CFG-UART1-STOPBITS

| Constant | Value | Description |
| --- | --- | --- |
| EIGHT | 0 | 8databits |
| SEVEN | 1 | 7databits |

Table 59: Constants for CFG-UART1-DATABITS

| Constant | Value | Description |
| --- | --- | --- |
| NONE | 0 | Noparitybit |
| ODD | 1 | Addanoddparitybit |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 162 of 181

<!-- page 163 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Constant | Value | Description |
| --- | --- | --- |
| EVEN | 2 | Addanevenparitybit |

Table 60: Constants for CFG-UART1-PARITY

## 4.9.27 CFG-UART1INPROT: Input protocol configuration of the UART1 interface Input protocol enable flags of the UART1 interface.

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td colspan="2">Unit Description</td></tr><tr><td>CFG-UART1INPROT-UBX</td><td>0x10730001</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if UBX should be an input protocol on UART1</td></tr><tr><td>CFG-UART1INPROT-NMEA</td><td>0x10730002</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if NMEA should be an input protocol on UART1</td></tr></tbody></table>

Table 61: CFG-UART1INPROT configuration items

## 4.9.28 CFG-UART1OUTPROT: Output protocol configuration of the UART1 interface

Output protocol enable flags of the UART1 interface.

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td colspan="2">Unit Description</td></tr><tr><td>CFG-UART1OUTPROT-UBX</td><td>0x10740001</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if UBX should be an output protocol on UART1</td></tr><tr><td>CFG-UART1OUTPROT-NMEA</td><td>0x10740002</td><td>L</td><td>-</td><td>-</td><td>Flag to indicate if NMEA should be an output protocol on UART1</td></tr></tbody></table>

Table 62: CFG-UART1OUTPROT configuration items

## 4.10 Legacy UBX message fields reference

The following table lists the legacy UBX message fields and the corresponding configuration item. Note that the mapping from UBX-CFG message fields to configuration items is not necessarily 1:1 and that that some legacy UBX-CFG messages may not be available for certain products.

| UBX message and field | Configuration item(s) |
| --- | --- |
| UBX-CFG-ANT |  |
| UBX-CFG-ANT.ocd | CFG-HW-ANT_CFG_OPENDET |
| UBX-CFG-ANT.pdwnOnSCD | CFG-HW-ANT_CFG_PWRDOWN |
| UBX-CFG-ANT.pinOCD | CFG-HW-ANT_SUP_OPEN_PIN |
| UBX-CFG-ANT.pinSCD | CFG-HW-ANT_SUP_SHORT_PIN |
| UBX-CFG-ANT.pinSwitch | CFG-HW-ANT_SUP_SWITCH_PIN |
| UBX-CFG-ANT.recovery | CFG-HW-ANT_CFG_RECOVER |
| UBX-CFG-ANT.scd | CFG-HW-ANT_CFG_SHORTDET |
| UBX-CFG-ANT.svcs | CFG-HW-ANT_CFG_VOLTCTRL |
| UBX-CFG-DAT |  |
| UBX-CFG-DAT.dX | CFG-NAVSPG-USRDAT_DX |
| UBX-CFG-DAT.dY | CFG-NAVSPG-USRDAT_DY |
| UBX-CFG-DAT.dZ | CFG-NAVSPG-USRDAT_DZ |
| UBX-CFG-DAT.flat | CFG-NAVSPG-USRDAT_FLAT |
| UBX-CFG-DAT.majA | CFG-NAVSPG-USE_USRDAT, CFG-NAVSPG-USRDAT_MAJA |
| UBX-CFG-DAT.rotX | CFG-NAVSPG-USRDAT_ROTX |
| UBX-CFG-DAT.rotY | CFG-NAVSPG-USRDAT_ROTY |
| UBX-CFG-DAT.rotZ | CFG-NAVSPG-USRDAT_ROTZ |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 163 of 181

<!-- page 164 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| UBX message and field | Configuration item(s) |
| --- | --- |
| UBX-CFG-DAT.scale | CFG-NAVSPG-USRDAT_SCALE |
| UBX-CFG-GNSS |  |
| UBX-CFG-GNSS.gnssId | CFG-SIGNAL-GPS_ENA, CFG-SIGNAL-SBAS_ENA, CFG-SIGNAL-BDS_ENA, CFG-SIGNAL-QZSS_ENA |
| UBX-CFG-INF |  |
| UBX-CFG-INF.infMsgMask | CFG-INFMSG-UBX_I2C, CFG-INFMSG-UBX_UART1, CFG-INFMSG-UBX_SPI, CFG-INFMSG-NMEA_I2C, CFG-INFMSG-NMEA_UART1, CFG-INFMSG-NMEA_SPI |
| UBX-CFG-INF.protocolID | CFG-INFMSG-UBX_UART1, CFG-INFMSG-UBX_SPI, CFG-INFMSG-NMEA_I2C, CFG-INFMSG-NMEA_UART1, CFG-INFMSG-NMEA_SPI |
| UBX-CFG-ITFM |  |
| UBX-CFG-ITFM.antSetting | CFG-ITFM-ANTSETTING |
| UBX-CFG-ITFM.bbThreshold | CFG-ITFM-BBTHRESHOLD |
| UBX-CFG-ITFM.cwThreshold | CFG-ITFM-CWTHRESHOLD |
| UBX-CFG-ITFM.enable | CFG-ITFM-ENABLE |
| UBX-CFG-ITFM.enable2 | CFG-ITFM-ENABLE_AUX |
| UBX-CFG-MOT |  |
| UBX-CFG-MOT.gnssDistThdl | CFG-MOT-GNSSDIST_THRS |
| UBX-CFG-MOT.gnssSpeedThdl | CFG-MOT-GNSSSPEED_THRS |
| UBX-CFG-NAV5 |  |
| UBX-CFG-NAV5.cnoThresh | CFG-NAVSPG-INFIL_CNOTHRS |
| UBX-CFG-NAV5.cnoThreshNumSVs | CFG-NAVSPG-INFIL_NCNOTHRS |
| UBX-CFG-NAV5.dgnssTimeout | CFG-NAVSPG-CONSTR_DGNSSTO |
| UBX-CFG-NAV5.dynModel | CFG-NAVSPG-DYNMODEL |
| UBX-CFG-NAV5.fixMode | CFG-NAVSPG-FIXMODE |
| UBX-CFG-NAV5.fixedAlt | CFG-NAVSPG-CONSTR_ALT |
| UBX-CFG-NAV5.fixedAltVar | CFG-NAVSPG-CONSTR_ALTVAR |
| UBX-CFG-NAV5.minElev | CFG-NAVSPG-INFIL_MINELEV |
| UBX-CFG-NAV5.pAcc | CFG-NAVSPG-OUTFIL_PACC |
| UBX-CFG-NAV5.pDop | CFG-NAVSPG-OUTFIL_PDOP |
| UBX-CFG-NAV5.staticHoldMaxDist | CFG-MOT-GNSSDIST_THRS |
| UBX-CFG-NAV5.staticHoldThresh | CFG-MOT-GNSSSPEED_THRS |
| UBX-CFG-NAV5.tAcc | CFG-NAVSPG-OUTFIL_TACC, CFG-NAVSPG-OUTFIL_FACC |
| UBX-CFG-NAV5.tDop | CFG-NAVSPG-OUTFIL_TDOP |
| UBX-CFG-NAV5.utcStandard | CFG-NAVSPG-UTCSTANDARD |
| UBX-CFG-NAVX5 |  |
| UBX-CFG-NAVX5.ackAiding | CFG-NAVSPG-ACKAIDING |
| UBX-CFG-NAVX5.aopOrbMaxErr | CFG-ANA-ORBMAXERR |
| UBX-CFG-NAVX5.iniFix3D | CFG-NAVSPG-INIFIX3D |
| UBX-CFG-NAVX5.maxSVs | CFG-NAVSPG-INFIL_MAXSVS |
| UBX-CFG-NAVX5.minCNO | CFG-NAVSPG-INFIL_MINCNO |
| UBX-CFG-NAVX5.minSVs | CFG-NAVSPG-INFIL_MINSVS |
| UBX-CFG-NAVX5.sigAttenCompMode | CFG-NAVSPG-SIGATTCOMP |
| UBX-CFG-NAVX5.useAOP | CFG-ANA-USE_ANA |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 164 of 181

<!-- page 165 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| UBX message and field | Configuration item(s) |
| --- | --- |
| UBX-CFG-NAVX5.wknRollover | CFG-NAVSPG-WKNROLLOVER |
| UBX-CFG-NMEA |  |
| UBX-CFG-NMEA.bdsTalkerId | CFG-NMEA-BDSTALKERID |
| UBX-CFG-NMEA.beidou | CFG-NMEA-FILT_BDS |
| UBX-CFG-NMEA.compat | CFG-NMEA-COMPAT |
| UBX-CFG-NMEA.consider | CFG-NMEA-CONSIDER |
| UBX-CFG-NMEA.dateFilt | CFG-NMEA-OUT_INVDATE |
| UBX-CFG-NMEA.galileo | CFG-NMEA-FILT_GAL |
| UBX-CFG-NMEA.gps | CFG-NMEA-FILT_GPS |
| UBX-CFG-NMEA.gpsOnlyFilter | CFG-NMEA-OUT_ONLYGPS |
| UBX-CFG-NMEA.gsvTalkerId | CFG-NMEA-GSVTALKERID |
| UBX-CFG-NMEA.highPrec | CFG-NMEA-HIGHPREC |
| UBX-CFG-NMEA.limit82 | CFG-NMEA-LIMIT82 |
| UBX-CFG-NMEA.mainTalkerId | CFG-NMEA-MAINTALKERID |
| UBX-CFG-NMEA.mskPosFilt | CFG-NMEA-OUT_MSKFIX |
| UBX-CFG-NMEA.nmeaVersion | CFG-NMEA-PROTVER |
| UBX-CFG-NMEA.numSV | CFG-NMEA-MAXSVS |
| UBX-CFG-NMEA.posFilt | CFG-NMEA-OUT_INVFIX |
| UBX-CFG-NMEA.qzss | CFG-NMEA-FILT_QZSS |
| UBX-CFG-NMEA.sbas | CFG-NMEA-FILT_SBAS |
| UBX-CFG-NMEA.svNumbering | CFG-NMEA-SVNUMBERING |
| UBX-CFG-NMEA.timeFilt | CFG-NMEA-OUT_INVTIME |
| UBX-CFG-NMEA.trackFilt | CFG-NMEA-OUT_FROZENCOG |
| UBX-CFG-ODO |  |
| UBX-CFG-ODO.cogLpGain | CFG-ODO-COGLPGAIN |
| UBX-CFG-ODO.cogMaxPosAcc | CFG-ODO-COGMAXPOSACC |
| UBX-CFG-ODO.cogMaxSpeed | CFG-ODO-COGMAXSPEED |
| UBX-CFG-ODO.outLPCog | CFG-ODO-OUTLPCOG |
| UBX-CFG-ODO.outLPVel | CFG-ODO-OUTLPVEL |
| UBX-CFG-ODO.profile | CFG-ODO-PROFILE |
| UBX-CFG-ODO.useCOG | CFG-ODO-USE_COG |
| UBX-CFG-ODO.useODO | CFG-ODO-USE_ODO |
| UBX-CFG-ODO.velLpGain | CFG-ODO-VELLPGAIN |
| UBX-CFG-PRT |  |
| UBX-CFG-PRT.en | CFG-TXREADY-ENABLED |
| UBX-CFG-PRT.extendedTxTimeout | CFG-I2C-EXTENDEDTIMEOUT |
| UBX-CFG-PRT.inNmea | CFG-I2CINPROT-NMEA |
| UBX-CFG-PRT.inProtoMask | CFG-I2C-ENABLED |
| UBX-CFG-PRT.inUbx | CFG-I2CINPROT-UBX |
| UBX-CFG-PRT.outNmea | CFG-I2COUTPROT-NMEA |
| UBX-CFG-PRT.outProtoMask | CFG-I2C-ENABLED |
| UBX-CFG-PRT.outUbx | CFG-I2COUTPROT-UBX |
| UBX-CFG-PRT.pin | CFG-TXREADY-PIN |
| UBX-CFG-PRT.pol | CFG-TXREADY-POLARITY |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 165 of 181

<!-- page 166 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| UBX message and field | Configuration item(s) |
| --- | --- |
| UBX-CFG-PRT.slaveAddr | CFG-I2C-ADDRESS |
| UBX-CFG-PRT.thres | CFG-TXREADY-THRESHOLD |
| UBX-CFG-PRT.en | CFG-TXREADY-ENABLED |
| UBX-CFG-PRT.extendedTxTimeout | CFG-SPI-EXTENDEDTIMEOUT |
| UBX-CFG-PRT.ffCnt | CFG-SPI-MAXFF |
| UBX-CFG-PRT.inNmea | CFG-SPIINPROT-NMEA |
| UBX-CFG-PRT.inProtoMask | CFG-SPI-ENABLED |
| UBX-CFG-PRT.inUbx | CFG-SPIINPROT-UBX |
| UBX-CFG-PRT.outNmea | CFG-SPIOUTPROT-NMEA |
| UBX-CFG-PRT.outProtoMask | CFG-SPI-ENABLED |
| UBX-CFG-PRT.outUbx | CFG-SPIOUTPROT-UBX |
| UBX-CFG-PRT.pin | CFG-TXREADY-PIN |
| UBX-CFG-PRT.pol | CFG-TXREADY-POLARITY |
| UBX-CFG-PRT.spiMode | CFG-SPI-CPOLARITY, CFG-SPI-CPHASE |
| UBX-CFG-PRT.thres | CFG-TXREADY-THRESHOLD |
| UBX-CFG-PRT.baudRate | CFG-UART1-BAUDRATE |
| UBX-CFG-PRT.charLen | CFG-UART1-DATABITS |
| UBX-CFG-PRT.inNmea | CFG-UART1INPROT-NMEA |
| UBX-CFG-PRT.inProtoMask | CFG-UART1-ENABLED |
| UBX-CFG-PRT.inUbx | CFG-UART1INPROT-UBX |
| UBX-CFG-PRT.nStopBits | CFG-UART1-STOPBITS |
| UBX-CFG-PRT.outNmea | CFG-UART1OUTPROT-NMEA |
| UBX-CFG-PRT.outProtoMask | CFG-UART1-ENABLED |
| UBX-CFG-PRT.outUbx | CFG-UART1OUTPROT-UBX |
| UBX-CFG-PRT.parity | CFG-UART1-PARITY |
| UBX-CFG-RATE |  |
| UBX-CFG-RATE.measRate | CFG-RATE-MEAS |
| UBX-CFG-RATE.navRate | CFG-RATE-NAV |
| UBX-CFG-RATE.timeRef | CFG-RATE-TIMEREF |
| UBX-CFG-RINV |  |
| UBX-CFG-RINV.data | CFG-RINV-DATA_SIZE, CFG-RINV-CHUNK0, CFG-RINV-CHUNK1, CFG-RINV-CHUNK2, CFG-RINV-CHUNK3 |
| UBX-CFG-RINV.flags | CFG-RINV-DUMP, CFG-RINV-BINARY |
| UBX-CFG-SBAS |  |
| UBX-CFG-SBAS.diffCorr | CFG-SBAS-USE_DIFFCORR |
| UBX-CFG-SBAS.integrity | CFG-SBAS-USE_INTEGRITY |
| UBX-CFG-SBAS.range | CFG-SBAS-USE_RANGING |
| UBX-CFG-SBAS.scanmode1 | CFG-SBAS-PRNSCANMASK |
| UBX-CFG-SBAS.test | CFG-SBAS-USE_TESTMODE |
| UBX-CFG-SLAS |  |
| UBX-CFG-SLAS.enabled | CFG-QZSS-USE_SLAS_DGNSS |
| UBX-CFG-SLAS.raim | CFG-QZSS-USE_SLAS_RAIM_UNCORR |
| UBX-CFG-SLAS.test | CFG-QZSS-USE_SLAS_TESTMODE |
| UBX-CFG-TP5 |  |

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 166 of 181

<!-- page 167 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| UBX message and field | Configuration item(s) |
| --- | --- |
| UBX-CFG-TP5.active | CFG-TP-TP1_ENA |
| UBX-CFG-TP5.alignToTow | CFG-TP-ALIGN_TO_TOW_TP1 |
| UBX-CFG-TP5.antCableDelay | CFG-TP-ANT_CABLEDELAY |
| UBX-CFG-TP5.freqPeriod | CFG-TP-PERIOD_TP1, CFG-TP-FREQ_TP1 |
| UBX-CFG-TP5.freqPeriodLock | CFG-TP-PERIOD_LOCK_TP1, CFG-TP-FREQ_LOCK_TP1 |
| UBX-CFG-TP5.gridUtcGnss | CFG-TP-TIMEGRID_TP1 |
| UBX-CFG-TP5.isFreq | CFG-TP-PULSE_DEF |
| UBX-CFG-TP5.isLength | CFG-TP-PULSE_LENGTH_DEF |
| UBX-CFG-TP5.lockGnssFreq | CFG-TP-SYNC_GNSS_TP1 |
| UBX-CFG-TP5.lockedOtherSet | CFG-TP-USE_LOCKED_TP1 |
| UBX-CFG-TP5.polarity | CFG-TP-POL_TP1 |
| UBX-CFG-TP5.pulseLenRatio | CFG-TP-LEN_TP1, CFG-TP-DUTY_TP1 |
| UBX-CFG-TP5.pulseLenRatioLock | CFG-TP-LEN_LOCK_TP1, CFG-TP-DUTY_LOCK_TP1 |
| UBX-CFG-TP5.userConfigDelay | CFG-TP-USER_DELAY_TP1 |

## Table 63: Legacy UBX message fields and the corresponding configuration items

UBX-23002975 - R02 C1-Public

4 Configuration interface

Page 167 of 181

<!-- page 168 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## Configuration defaults

The following tables contain the configuration defaults for the firmware. Some of these values may be changed in production. Refer to the integration manual for product-specific details.

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-ANA-USE_ANA | 0x10230001 | L | - | - | 0 (false) |
| CFG-ANA-ORBMAXERR | 0x30230002 | U2 | - | m | 100 |

Table 64: CFG-ANA configuration defaults

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-BDS-USE_GEO_PRN | 0x10340014 | L | - | - | 1 (true) |

Table 65: CFG-BDS configuration defaults

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-HW-ANT_CFG_VOLTCTRL | 0x10a3002e | L | - | - | 0 (false) |
| CFG-HW-ANT_CFG_SHORTDET | 0x10a3002f | L | - | - | 0 (false) |
| CFG-HW-ANT_CFG_SHORTDET_POL | 0x10a30030 | L | - | - | 1 (true) |
| CFG-HW-ANT_CFG_OPENDET | 0x10a30031 | L | - | - | 0 (false) |
| CFG-HW-ANT_CFG_OPENDET_POL | 0x10a30032 | L | - | - | 1 (true) |
| CFG-HW-ANT_CFG_PWRDOWN | 0x10a30033 | L | - | - | 0 (false) |
| CFG-HW-ANT_CFG_PWRDOWN_POL | 0x10a30034 | L | - | - | 1 (true) |
| CFG-HW-ANT_CFG_RECOVER | 0x10a30035 | L | - | - | 0 (false) |
| CFG-HW-ANT_SUP_SWITCH_PIN | 0x20a30036 | U1 | - | - | 7 |
| CFG-HW-ANT_SUP_SHORT_PIN | 0x20a30037 | U1 | - | - | 6 |
| CFG-HW-ANT_SUP_OPEN_PIN | 0x20a30038 | U1 | - | - | 5 |
| CFG-HW-ANT_ON_SHORT_US | 0x30a3003c | U2 | - | - | 500 |
| CFG-HW-ANT_SUP_ENGINE | 0x20a30054 | E1 | - | - | 0 (EXT) |
| CFG-HW-ANT_SUP_SHORT_THR | 0x20a30055 | U1 | - | mV | 0 |
| CFG-HW-ANT_SUP_OPEN_THR | 0x20a30056 | U1 | - | mV | 0 |
| CFG-HW-RF_LNA_MODE | 0x20a30057 | E1 | - | - | 0 (NORMAL) |

Table 66: CFG-HW configuration defaults

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-I2C-ADDRESS | 0x20510001 | U1 | - | - | 132 |
| CFG-I2C-EXTENDEDTIMEOUT | 0x10510002 | L | - | - | 0 (false) |
| CFG-I2C-ENABLED | 0x10510003 | L | - | - | 0 (false) |

Table 67: CFG-I2C configuration defaults

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td>Unit</td><td>Default value</td></tr><tr><td>CFG-I2CINPROT-UBX</td><td>0x10710001</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-I2CINPROT-NMEA</td><td>0x10710002</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr></tbody></table>

Table 68: CFG-I2CINPROT configuration defaults

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-I2COUTPROT-UBX | 0x10720001 | L | - | - | 1 (true) |

UBX-23002975 - R02 C1-Public

Configuration defaults

Page 168 of 181

<!-- page 169 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-I2COUTPROT-NMEA | 0x10720002 | L | - | - | 1 (true) |

Table 69: CFG-I2COUTPROT configuration defaults

| Configuration item | Key ID Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- |
| CFG-INFMSG-UBX_I2C | 0x20920001 X1 | - | - | 0x00 |
| CFG-INFMSG-UBX_UART1 | 0x20920002 X1 | - | - | 0x00 |
| CFG-INFMSG-UBX_SPI | 0x20920005 X1 | - | - | 0x00 |
| CFG-INFMSG-NMEA_I2C | 0x20920006 X1 | - | - | 0x07 (ERROR \| WARNING \| NOTICE) |
| CFG-INFMSG-NMEA_UART1 | 0x20920007 X1 | - | - | 0x07 (ERROR \| WARNING \| NOTICE) |
| CFG-INFMSG-NMEA_SPI | 0x2092000a X1 | - | - | 0x07 (ERROR \| WARNING \| NOTICE) |

Table 70: CFG-INFMSG configuration defaults

| Configuration item CFG-ITFM-BBTHRESHOLD | Key ID 0x20410001 | TypeU1 | Scale- | Unit- | Default value3 |
| --- | --- | --- | --- | --- | --- |
| CFG-ITFM-CWTHRESHOLD | 0x20410002 | U1 | - | - | 15 |
| CFG-ITFM-ENABLE | 0x1041000d | L | - | - | 0 (false) |
| CFG-ITFM-ANTSETTING | 0x20410010 | E1 | - | - | 0 (UNKNOWN) |
| CFG-ITFM-ENABLE_AUX | 0x10410013 | L | - | - | 0 (false) |

Table 71: CFG-ITFM configuration defaults

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-MOT-GNSSSPEED_THRS | 0x20250038 | U1 | 0.01 | m/s | 0 |
| CFG-MOT-GNSSDIST_THRS | 0x3025003b | U2 | 1.0 | m | 0 |

Table 72: CFG-MOT configuration defaults

| Configuration item | Key ID Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- |
| CFG-MSGOUT-NMEA_ID_DTM_I2C | 0x209100a6 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_DTM_SPI | 0x209100aa U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_DTM_UART1 | 0x209100a7 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_GBS_I2C | 0x209100dd U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_GBS_SPI | 0x209100e1 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_GBS_UART1 | 0x209100de U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_GGA_I2C | 0x209100ba U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_GGA_SPI | 0x209100be U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_GGA_UART1 | 0x209100bb U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_GLL_I2C | 0x209100c9 U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_GLL_SPI | 0x209100cd U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_GLL_UART1 | 0x209100ca U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_GNS_I2C | 0x209100b5 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_GNS_SPI | 0x209100b9 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_GNS_UART1 | 0x209100b6 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_GRS_I2C | 0x209100ce U1 | - | - | 0 |

UBX-23002975 - R02 C1-Public

Configuration defaults

Page 169 of 181

<!-- page 170 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- |
| CFG-MSGOUT-NMEA_ID_GRS_SPI | 0x209100d2 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_GRS_UART1 | 0x209100cf U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_GSA_I2C | 0x209100bf U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_GSA_SPI | 0x209100c3 U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_GSA_UART1 | 0x209100c0 U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_GST_I2C | 0x209100d3 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_GST_SPI | 0x209100d7 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_GST_UART1 | 0x209100d4 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_GSV_I2C | 0x209100c4 U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_GSV_SPI | 0x209100c8 U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_GSV_UART1 | 0x209100c5 U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_RLM_I2C | 0x20910400 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_RLM_SPI | 0x20910404 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_RLM_UART1 | 0x20910401 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_RMC_I2C | 0x209100ab U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_RMC_SPI | 0x209100af U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_RMC_UART1 | 0x209100ac U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_VLW_I2C | 0x209100e7 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_VLW_SPI | 0x209100eb U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_VLW_UART1 | 0x209100e8 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_VTG_I2C | 0x209100b0 U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_VTG_SPI | 0x209100b4 U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_VTG_UART1 | 0x209100b1 U1 | - | - | 1 |
| CFG-MSGOUT-NMEA_ID_ZDA_I2C | 0x209100d8 U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_ZDA_SPI | 0x209100dc U1 | - | - | 0 |
| CFG-MSGOUT-NMEA_ID_ZDA_UART1 | 0x209100d9 U1 | - | - | 0 |
| CFG-MSGOUT-PUBX_ID_POLYP_I2C | 0x209100ec U1 | - | - | 0 |
| CFG-MSGOUT-PUBX_ID_POLYP_SPI | 0x209100f0 U1 | - | - | 0 |
| CFG-MSGOUT-PUBX_ID_POLYP_UART1 | 0x209100ed U1 | - | - | 0 |
| CFG-MSGOUT-PUBX_ID_POLYS_I2C | 0x209100f1 U1 | - | - | 0 |
| CFG-MSGOUT-PUBX_ID_POLYS_SPI | 0x209100f5 U1 | - | - | 0 |
| CFG-MSGOUT-PUBX_ID_POLYS_UART1 | 0x209100f2 U1 | - | - | 0 |
| CFG-MSGOUT-PUBX_ID_POLYT_I2C | 0x209100f6 U1 | - | - | 0 |
| CFG-MSGOUT-PUBX_ID_POLYT_SPI | 0x209100fa U1 | - | - | 0 |
| CFG-MSGOUT-PUBX_ID_POLYT_UART1 | 0x209100f7 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_COMMS_I2C | 0x2091034f U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_COMMS_SPI | 0x20910353 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_COMMS_UART1 | 0x20910350 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_HW2_I2C | 0x209101b9 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_HW2_SPI | 0x209101bd U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_HW2_UART1 | 0x209101ba U1 | - | - | 0 |

UBX-23002975 - R02 C1-Public

Configuration defaults

Page 170 of 181

<!-- page 171 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- |
| CFG-MSGOUT-UBX_MON_HW3_I2C | 0x20910354 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_HW3_SPI | 0x20910358 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_HW3_UART1 | 0x20910355 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_HW_I2C | 0x209101b4 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_HW_SPI | 0x209101b8 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_HW_UART1 | 0x209101b5 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_IO_I2C | 0x209101a5 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_IO_SPI | 0x209101a9 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_IO_UART1 | 0x209101a6 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_MSGPP_I2C | 0x20910196 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_MSGPP_SPI | 0x2091019a U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_MSGPP_UART1 | 0x20910197 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_RF_I2C | 0x20910359 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_RF_SPI | 0x2091035d U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_RF_UART1 | 0x2091035a U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_RXR_I2C | 0x20910187 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_RXR_SPI | 0x2091018b U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_RXR_UART1 | 0x20910188 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_SPAN_I2C | 0x2091038b U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_SPAN_SPI | 0x2091038f U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_SPAN_UART1 | 0x2091038c U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_SYS_I2C | 0x2091069d U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_SYS_SPI | 0x209106a1 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_MON_SYS_UART1 | 0x2091069e U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_AOPSTATUS_I2C | 0x20910079 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_AOPSTATUS_SPI | 0x2091007d U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_AOPSTATUS_UART1 | 0x2091007a U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_CLOCK_I2C | 0x20910065 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_CLOCK_SPI | 0x20910069 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_CLOCK_UART1 | 0x20910066 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_COV_I2C | 0x20910083 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_COV_SPI | 0x20910087 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_COV_UART1 | 0x20910084 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_DOP_I2C | 0x20910038 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_DOP_SPI | 0x2091003c U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_DOP_UART1 | 0x20910039 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_EOE_I2C | 0x2091015f U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_EOE_SPI | 0x20910163 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_EOE_UART1 | 0x20910160 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_ODO_I2C | 0x2091007e U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_ODO_SPI | 0x20910082 U1 | - | - | 0 |

UBX-23002975 - R02 C1-Public

Configuration defaults

Page 171 of 181

<!-- page 172 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- |
| CFG-MSGOUT-UBX_NAV_ODO_UART1 | 0x2091007f U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_ORB_I2C | 0x20910010 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_ORB_SPI | 0x20910014 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_ORB_UART1 | 0x20910011 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_PL_I2C | 0x20910415 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_PL_SPI | 0x20910419 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_PL_UART1 | 0x20910416 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_POSECEF_I2C | 0x20910024 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_POSECEF_SPI | 0x20910028 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_POSECEF_UART1 | 0x20910025 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_POSLLH_I2C | 0x20910029 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_POSLLH_SPI | 0x2091002d U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_POSLLH_UART1 | 0x2091002a U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_PVT_I2C | 0x20910006 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_PVT_SPI | 0x2091000a U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_PVT_UART1 | 0x20910007 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_SAT_I2C | 0x20910015 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_SAT_SPI | 0x20910019 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_SAT_UART1 | 0x20910016 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_SBAS_I2C | 0x2091006a U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_SBAS_SPI | 0x2091006e U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_SBAS_UART1 | 0x2091006b U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_SIG_I2C | 0x20910345 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_SIG_SPI | 0x20910349 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_SIG_UART1 | 0x20910346 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_SLAS_I2C | 0x20910336 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_SLAS_SPI | 0x2091033a U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_SLAS_UART1 | 0x20910337 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_STATUS_I2C | 0x2091001a U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_STATUS_SPI | 0x2091001e U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_STATUS_UART1 | 0x2091001b U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEBDS_I2C | 0x20910051 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEBDS_SPI | 0x20910055 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEBDS_UART1 | 0x20910052 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEGAL_I2C | 0x20910056 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEGAL_SPI | 0x2091005a U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEGAL_UART1 | 0x20910057 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEGPS_I2C | 0x20910047 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEGPS_SPI | 0x2091004b U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEGPS_UART1 | 0x20910048 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMELS_I2C | 0x20910060 U1 | - | - | 0 |

UBX-23002975 - R02 C1-Public

Configuration defaults

Page 172 of 181

<!-- page 173 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- |
| CFG-MSGOUT-UBX_NAV_TIMELS_SPI | 0x20910064 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMELS_UART1 | 0x20910061 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMENAVIC_I2C | 0x209106a2 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMENAVIC_SPI | 0x209106a6 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMENAVIC_UART1 | 0x209106a3 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEQZSS_I2C | 0x20910386 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEQZSS_SPI | 0x2091038a U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEQZSS_UART1 | 0x20910387 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEUTC_I2C | 0x2091005b U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEUTC_SPI | 0x2091005f U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1 | 0x2091005c U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_VELECEF_I2C | 0x2091003d U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_VELECEF_SPI | 0x20910041 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_VELECEF_UART1 | 0x2091003e U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_VELNED_I2C | 0x20910042 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_VELNED_SPI | 0x20910046 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_NAV_VELNED_UART1 | 0x20910043 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_RXM_MEASX_I2C | 0x20910204 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_RXM_MEASX_SPI | 0x20910208 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_RXM_MEASX_UART1 | 0x20910205 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_RXM_RLM_I2C | 0x2091025e U1 | - | - | 0 |
| CFG-MSGOUT-UBX_RXM_RLM_SPI | 0x20910262 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_RXM_RLM_UART1 | 0x2091025f U1 | - | - | 0 |
| CFG-MSGOUT-UBX_RXM_SFRBX_I2C | 0x20910231 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_RXM_SFRBX_SPI | 0x20910235 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_RXM_SFRBX_UART1 | 0x20910232 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_SEC_SIGLOG_I2C | 0x20910689 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_SEC_SIGLOG_SPI | 0x2091068d U1 | - | - | 0 |
| CFG-MSGOUT-UBX_SEC_SIGLOG_UART1 | 0x2091068a U1 | - | - | 0 |
| CFG-MSGOUT-UBX_SEC_SIG_I2C | 0x20910634 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_SEC_SIG_SPI | 0x20910638 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_SEC_SIG_UART1 | 0x20910635 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_TIM_TM2_I2C | 0x20910178 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_TIM_TM2_SPI | 0x2091017c U1 | - | - | 0 |
| CFG-MSGOUT-UBX_TIM_TM2_UART1 | 0x20910179 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_TIM_TP_I2C | 0x2091017d U1 | - | - | 0 |
| CFG-MSGOUT-UBX_TIM_TP_SPI | 0x20910181 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_TIM_TP_UART1 | 0x2091017e U1 | - | - | 0 |
| CFG-MSGOUT-UBX_TIM_VRFY_I2C | 0x20910092 U1 | - | - | 0 |
| CFG-MSGOUT-UBX_TIM_VRFY_SPI | 0x20910096 U1 | - | - | 0 |

UBX-23002975 - R02 C1-Public

Configuration defaults

Page 173 of 181

<!-- page 174 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-MSGOUT-UBX_TIM_VRFY_UART1 | 0x20910093 | U1 | - | - | 0 |

Table 73: CFG-MSGOUT configuration defaults

| Configuration item | Key ID Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- |
| CFG-NAVMASK-SV_MASK_GPS | 0x50180013 X8 | - | - | 0xffffffffffffffff (EMPTY) |
| CFG-NAVMASK-SV_MASK_GAL | 0x50180014 X8 | - | - | 0xffffffffffffffff (EMPTY) |
| CFG-NAVMASK-SV_MASK_BDS | 0x50180016 X8 | - | - | 0xffffffffffffffff (EMPTY) |
| CFG-NAVMASK-SV_MASK_QZSS | 0x50180017 X8 | - | - | 0xffffffffffffffff (EMPTY) |
| CFG-NAVMASK-SV_MASK_NAVIC | 0x50180018 X8 | - | - | 0xffffffffffffffff (EMPTY) |

Table 74: CFG-NAVMASK configuration defaults

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td>Unit</td><td>Default value</td></tr><tr><td>CFG-NAVSPG-FIXMODE</td><td>0x20110011</td><td>E1</td><td>-</td><td>-</td><td>3 (AUTO)</td></tr><tr><td>CFG-NAVSPG-INIFIX3D</td><td>0x10110013</td><td>L</td><td>-</td><td>-</td><td>0 (false)</td></tr><tr><td>CFG-NAVSPG-WKNROLLOVER</td><td>0x30110017</td><td>U2</td><td>-</td><td>-</td><td>2280</td></tr><tr><td>CFG-NAVSPG-UTCSTANDARD</td><td>0x2011001c</td><td>E1</td><td>-</td><td>-</td><td>0 (AUTO)</td></tr><tr><td>CFG-NAVSPG-DYNMODEL</td><td>0x20110021</td><td>E1</td><td>-</td><td>-</td><td>0 (PORT)</td></tr><tr><td>CFG-NAVSPG-ACKAIDING</td><td>0x10110025</td><td>L</td><td>-</td><td>-</td><td>0 (false)</td></tr><tr><td>CFG-NAVSPG-USE_USRDAT</td><td>0x10110061</td><td>L</td><td>-</td><td>-</td><td>0 (false)</td></tr><tr><td>CFG-NAVSPG-USRDAT_MAJA</td><td>0x50110062</td><td>R8</td><td>-</td><td>m</td><td>6378137</td></tr><tr><td>CFG-NAVSPG-USRDAT_FLAT</td><td>0x50110063</td><td>R8</td><td>-</td><td>-</td><td>298.25722356300002502</td></tr><tr><td>CFG-NAVSPG-USRDAT_DX</td><td>0x40110064</td><td>R4</td><td>-</td><td>m</td><td>0</td></tr><tr><td>CFG-NAVSPG-USRDAT_DY</td><td>0x40110065</td><td>R4</td><td>-</td><td>m</td><td>0</td></tr><tr><td>CFG-NAVSPG-USRDAT_DZ</td><td>0x40110066</td><td>R4</td><td>-</td><td>m</td><td>0</td></tr><tr><td>CFG-NAVSPG-USRDAT_ROTX</td><td>0x40110067</td><td>R4</td><td>-</td><td>arcsec</td><td>0</td></tr><tr><td>CFG-NAVSPG-USRDAT_ROTY</td><td>0x40110068</td><td>R4</td><td>-</td><td>arcsec</td><td>0</td></tr><tr><td>CFG-NAVSPG-USRDAT_ROTZ</td><td>0x40110069</td><td>R4</td><td>-</td><td>arcsec</td><td>0</td></tr><tr><td>CFG-NAVSPG-USRDAT_SCALE</td><td>0x4011006a</td><td>R4</td><td>-</td><td>ppm</td><td>0</td></tr><tr><td>CFG-NAVSPG-INFIL_MINSVS</td><td>0x201100a1</td><td>U1</td><td>-</td><td>-</td><td>3</td></tr><tr><td>CFG-NAVSPG-INFIL_MAXSVS</td><td>0x201100a2</td><td>U1</td><td>-</td><td>-</td><td>32</td></tr><tr><td>CFG-NAVSPG-INFIL_MINCNO</td><td>0x201100a3</td><td>U1</td><td>-</td><td>dBHz</td><td>6</td></tr><tr><td>CFG-NAVSPG-INFIL_MINELEV</td><td>0x201100a4</td><td>I1</td><td>-</td><td>deg</td><td>5</td></tr><tr><td>CFG-NAVSPG-INFIL_NCNOTHRS</td><td>0x201100aa</td><td>U1</td><td>-</td><td>-</td><td>0</td></tr><tr><td>CFG-NAVSPG-INFIL_CNOTHRS</td><td>0x201100ab</td><td>U1</td><td>-</td><td>-</td><td>0</td></tr><tr><td>CFG-NAVSPG-OUTFIL_PDOP</td><td>0x301100b1</td><td>U2</td><td>0.1</td><td>-</td><td>250</td></tr><tr><td>CFG-NAVSPG-OUTFIL_TDOP</td><td>0x301100b2</td><td>U2</td><td>0.1</td><td>-</td><td>250</td></tr><tr><td>CFG-NAVSPG-OUTFIL_PACC</td><td>0x301100b3</td><td>U2</td><td>-</td><td>m</td><td>100</td></tr><tr><td>CFG-NAVSPG-OUTFIL_TACC</td><td>0x301100b4</td><td>U2</td><td>-</td><td>m</td><td>350</td></tr><tr><td>CFG-NAVSPG-OUTFIL_FACC</td><td>0x301100b5</td><td>U2</td><td>0.01</td><td>m/s</td><td>150</td></tr><tr><td>CFG-NAVSPG-CONSTR_ALT</td><td>0x401100c1</td><td>I4</td><td>0.01</td><td>m</td><td>0</td></tr><tr><td>CFG-NAVSPG-CONSTR_ALTVAR</td><td>0x401100c2</td><td>U4</td><td>0.0001</td><td>m^2</td><td>10000</td></tr><tr><td>CFG-NAVSPG-CONSTR_DGNSSTO</td><td>0x201100c4</td><td>U1</td><td>-</td><td>s</td><td>60</td></tr></tbody></table>

UBX-23002975 - R02 C1-Public

Configuration defaults

Page 174 of 181

<!-- page 175 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tbody><tr><td>Configuration item Key ID Type</td><td>Scale</td><td>Unit</td><td>Default value</td></tr><tr><td>CFG-NAVSPG-SIGATTCOMP 0x201100d6 E1</td><td>-</td><td>-</td><td rowspan="2">0 (DIS) 1 (true)</td></tr><tr><td>CFG-NAVSPG-PL_ENA 0x101100d7 L</td><td>-</td><td>-</td></tr></tbody></table>

Table 75: CFG-NAVSPG configuration defaults

| Configuration item CFG-NMEA-PROTVER | Key ID 0x20930001 | TypeE1 | Scale- | Unit- | Default value 42 (V411) |
| --- | --- | --- | --- | --- | --- |
| CFG-NMEA-MAXSVS | 0x20930002 | E1 | - | - | 0 (UNLIM) |
| CFG-NMEA-COMPAT | 0x10930003 | L | - | - | 0 (false) |
| CFG-NMEA-CONSIDER | 0x10930004 | L | - | - | 1 (true) |
| CFG-NMEA-LIMIT82 | 0x10930005 | L | - | - | 0 (false) |
| CFG-NMEA-HIGHPREC | 0x10930006 | L | - | - | 0 (false) |
| CFG-NMEA-SVNUMBERING | 0x20930007 | E1 | - | - | 0 (STRICT) |
| CFG-NMEA-FILT_GPS | 0x10930011 | L | - | - | 0 (false) |
| CFG-NMEA-FILT_SBAS | 0x10930012 | L | - | - | 0 (false) |
| CFG-NMEA-FILT_GAL | 0x10930013 | L | - | - | 0 (false) |
| CFG-NMEA-FILT_QZSS | 0x10930015 | L | - | - | 0 (false) |
| CFG-NMEA-FILT_BDS | 0x10930017 | L | - | - | 0 (false) |
| CFG-NMEA-FILT_NAVIC | 0x10930018 | L | - | - | 0 (false) |
| CFG-NMEA-OUT_INVFIX | 0x10930021 | L | - | - | 0 (false) |
| CFG-NMEA-OUT_MSKFIX | 0x10930022 | L | - | - | 0 (false) |
| CFG-NMEA-OUT_INVTIME | 0x10930023 | L | - | - | 0 (false) |
| CFG-NMEA-OUT_INVDATE | 0x10930024 | L | - | - | 0 (false) |
| CFG-NMEA-OUT_ONLYGPS | 0x10930025 | L | - | - | 0 (false) |
| CFG-NMEA-OUT_FROZENCOG | 0x10930026 | L | - | - | 0 (false) |
| CFG-NMEA-MAINTALKERID | 0x20930031 | E1 | - | - | 0 (AUTO) |
| CFG-NMEA-GSVTALKERID | 0x20930032 | E1 | - | - | 0 (GNSS) |
| CFG-NMEA-BDSTALKERID | 0x30930033 | U2 | - | - | 0 |

Table 76: CFG-NMEA configuration defaults

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-ODO-USE_ODO | 0x10220001 | L | - | - | 0 (false) |
| CFG-ODO-USE_COG | 0x10220002 | L | - | - | 0 (false) |
| CFG-ODO-OUTLPVEL | 0x10220003 | L | - | - | 0 (false) |
| CFG-ODO-OUTLPCOG | 0x10220004 | L | - | - | 0 (false) |
| CFG-ODO-PROFILE | 0x20220005 | E1 | - | - | 0 (RUN) |
| CFG-ODO-COGMAXSPEED | 0x20220021 | U1 | 1e-1 | m/s | 10 |
| CFG-ODO-COGMAXPOSACC | 0x20220022 | U1 | - | - | 50 |
| CFG-ODO-VELLPGAIN | 0x20220031 | U1 | - | - | 153 |
| CFG-ODO-COGLPGAIN | 0x20220032 | U1 | - | - | 76 |

Table 77: CFG-ODO configuration defaults

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-QZSS-USE_SLAS_DGNSS | 0x10370005 | L | - | - | 1 (true) |

UBX-23002975 - R02 C1-Public

Configuration defaults

Page 175 of 181

<!-- page 176 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-QZSS-USE_SLAS_TESTMODE | 0x10370006 | L | - | - | 0 (false) |
| CFG-QZSS-USE_SLAS_RAIM_UNCORR | 0x10370007 | L | - | - | 0 (false) |
| CFG-QZSS-SLAS_MAX_BASELINE | 0x30370008 | U2 | - | km | 350 |

Table 78: CFG-QZSS configuration defaults

| Configuration item | Key ID Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- |
| CFG-RATE-MEAS | 0x30210001 U2 | 0.001 | s | 1000 |
| CFG-RATE-NAV | 0x30210002 U2 | - | - | 1 |
| CFG-RATE-TIMEREF | 0x20210003 E1 | - | - | 1 (GPS) |

Table 79: CFG-RATE configuration defaults

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-RINV-DUMP | 0x10c70001 | L | - | - | 0 (false) |
| CFG-RINV-BINARY | 0x10c70002 | L | - | - | 0 (false) |
| CFG-RINV-DATA_SIZE | 0x20c70003 | U1 | - | - | 22 |
| CFG-RINV-CHUNK0 | 0x50c70004 | X8 | - | - | 0x203a656369746f4e ("Notice: ") |
| CFG-RINV-CHUNK1 | 0x50c70005 | X8 | - | - | 0x2061746164206f6e ("no data ") |
| CFG-RINV-CHUNK2 | 0x50c70006 | X8 | - | - | 0x0000216465766173("saved!\\0\\0") |
| CFG-RINV-CHUNK3 | 0x50c70007 | X8 | - | - | 0x0000000000000000 |

Table 80: CFG-RINV configuration defaults

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-SBAS-USE_TESTMODE | 0x10360002 | L | - | - | 0 (false) |
| CFG-SBAS-USE_RANGING | 0x10360003 | L | - | - | 1 (true) |
| CFG-SBAS-USE_DIFFCORR | 0x10360004 | L | - | - | 1 (true) |
| CFG-SBAS-USE_INTEGRITY | 0x10360005 | L | - | - | 0 (false) |
| CFG-SBAS-ACCEPT_NOT_IN_PRNMASK | 0x30360008 | X2 | - | - | 0x0000 |
| CFG-SBAS-USE_IONOONLY | 0x10360007 | L | - | - | 0 (false) |
| CFG-SBAS-PRNSCANMASK | 0x50360006 | X8 | - | - | 0x000000000003ab88 (ALL \| PRN123 \| PRN127 \| PRN128 \| PRN129 \| PRN131 \| PRN133 \| PRN135 \| PRN136 \| PRN137) |

Table 81: CFG-SBAS configuration defaults

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-SEC-CFG_LOCK | 0x10f60009 | L | - | - | 0 (false) |
| CFG-SEC-CFG_LOCK_UNLOCKGRP1 | 0x30f6000a | U2 | - | - | 0 |
| CFG-SEC-CFG_LOCK_UNLOCKGRP2 | 0x30f6000b | U2 | - | - | 0 |

Table 82: CFG-SEC configuration defaults

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td>Unit</td><td>Default value</td></tr><tr><td>CFG-SIGNAL-GPS_ENA</td><td>0x1031001f</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-GPS_L1CA_ENA</td><td>0x10310001</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-GPS_L5_ENA</td><td>0x10310004</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr></tbody></table>

UBX-23002975 - R02 C1-Public

Configuration defaults

Page 176 of 181

<!-- page 177 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td>Unit</td><td>Default value</td></tr><tr><td>CFG-SIGNAL-SBAS_ENA</td><td>0x10310020</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-SBAS_L1CA_ENA</td><td>0x10310005</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-GAL_ENA</td><td>0x10310021</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-GAL_E1_ENA</td><td>0x10310007</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-GAL_E5A_ENA</td><td>0x10310009</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-BDS_ENA</td><td>0x10310022</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-BDS_B1_ENA</td><td>0x1031000d</td><td>L</td><td>-</td><td>-</td><td>0 (false)</td></tr><tr><td>CFG-SIGNAL-BDS_B1C_ENA</td><td>0x1031000f</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-BDS_B2A_ENA</td><td>0x10310028</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-QZSS_ENA</td><td>0x10310024</td><td>L</td><td>-</td><td>-</td><td>0 (false)</td></tr><tr><td>CFG-SIGNAL-QZSS_L1CA_ENA</td><td>0x10310012</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-QZSS_L1S_ENA</td><td>0x10310014</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-QZSS_L5_ENA</td><td>0x10310017</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SIGNAL-NAVIC_ENA</td><td>0x10310026</td><td>L</td><td>-</td><td>-</td><td>0 (false)</td></tr><tr><td>CFG-SIGNAL-NAVIC_L5_ENA</td><td>0x1031001d</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr></tbody></table>

Table 83: CFG-SIGNAL configuration defaults

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td>Unit</td><td>Default value</td></tr><tr><td>CFG-SPI-MAXFF</td><td>0x20640001</td><td>U1</td><td>-</td><td>-</td><td>50</td></tr><tr><td>CFG-SPI-CPOLARITY</td><td>0x10640002</td><td>L</td><td>-</td><td>-</td><td>0 (false)</td></tr><tr><td>CFG-SPI-CPHASE</td><td>0x10640003</td><td>L</td><td>-</td><td>-</td><td>0 (false)</td></tr><tr><td>CFG-SPI-EXTENDEDTIMEOUT</td><td>0x10640005</td><td>L</td><td>-</td><td>-</td><td>0 (false)</td></tr><tr><td>CFG-SPI-ENABLED</td><td>0x10640006</td><td>L</td><td>-</td><td>-</td><td>0 (false)</td></tr></tbody></table>

Table 84: CFG-SPI configuration defaults

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td>Unit</td><td>Default value</td></tr><tr><td>CFG-SPIINPROT-UBX</td><td>0x10790001</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SPIINPROT-NMEA</td><td>0x10790002</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr></tbody></table>

Table 85: CFG-SPIINPROT configuration defaults

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td>Unit</td><td>Default value</td></tr><tr><td>CFG-SPIOUTPROT-UBX</td><td>0x107a0001</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-SPIOUTPROT-NMEA</td><td>0x107a0002</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr></tbody></table>

Table 86: CFG-SPIOUTPROT configuration defaults

| Configuration item | Key ID Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- |
| CFG-TP-PULSE_DEF | 0x20050023 E1 | - | - | 0 (PERIOD) |
| CFG-TP-PULSE_LENGTH_DEF | 0x20050030 E1 | - | - | 1 (LENGTH) |
| CFG-TP-ANT_CABLEDELAY | 0x30050001 I2 | 1e-9 | s | 50 |
| CFG-TP-PERIOD_TP1 | 0x40050002 U4 | 1e-6 | s | 1000000 |
| CFG-TP-PERIOD_LOCK_TP1 | 0x40050003 U4 | 1e-6 | s | 1000000 |
| CFG-TP-FREQ_TP1 | 0x40050024 U4 | - | Hz | 1 |
| CFG-TP-FREQ_LOCK_TP1 | 0x40050025 U4 | - | Hz | 1 |
| CFG-TP-LEN_TP1 | 0x40050004 U4 | 1e-6 | s | 0 |

UBX-23002975 - R02 C1-Public

Configuration defaults

Page 177 of 181

<!-- page 178 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-TP-LEN_LOCK_TP1 | 0x40050005 | U4 | 1e-6 | s | 100000 |
| CFG-TP-DUTY_TP1 | 0x5005002a | R8 | - | % | 0 |
| CFG-TP-DUTY_LOCK_TP1 | 0x5005002b | R8 | - | % | 10 |
| CFG-TP-USER_DELAY_TP1 | 0x40050006 | I4 | 1e-9 | s | 0 |
| CFG-TP-TP1_ENA | 0x10050007 | L | - | - | 1 (true) |
| CFG-TP-SYNC_GNSS_TP1 | 0x10050008 | L | - | - | 1 (true) |
| CFG-TP-USE_LOCKED_TP1 | 0x10050009 | L | - | - | 1 (true) |
| CFG-TP-ALIGN_TO_TOW_TP1 | 0x1005000a | L | - | - | 1 (true) |
| CFG-TP-POL_TP1 | 0x1005000b | L | - | - | 1 (true) |
| CFG-TP-TIMEGRID_TP1 | 0x2005000c | E1 | - | - | 0 (UTC) |
| CFG-TP-DRSTR_TP1 | 0x20050035 | E1 | - | - | 1 (DRIVE_STRENGTH_4MA) |

Table 87: CFG-TP configuration defaults

| Configuration item | Key ID | Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- | --- |
| CFG-TXREADY-ENABLED | 0x10a20001 | L | - | - | 0 (false) |
| CFG-TXREADY-POLARITY | 0x10a20002 | L | - | - | 0 (false) |
| CFG-TXREADY-PIN | 0x20a20003 | U1 | - | - | 0 |
| CFG-TXREADY-THRESHOLD | 0x30a20004 | U2 | - | - | 0 |
| CFG-TXREADY-INTERFACE | 0x20a20005 | E1 | - | - | 0 (I2C) |

Table 88: CFG-TXREADY configuration defaults

| Configuration item | Key ID Type | Scale | Unit | Default value |
| --- | --- | --- | --- | --- |
| CFG-UART1-BAUDRATE | 0x40520001 U4 | - | - | 38400 |
| CFG-UART1-STOPBITS | 0x20520002 E1 | - | - | 1 (ONE) |
| CFG-UART1-DATABITS | 0x20520003 E1 | - | - | 0 (EIGHT) |
| CFG-UART1-PARITY | 0x20520004 E1 | - | - | 0 (NONE) |
| CFG-UART1-ENABLED | 0x10520005 L | - | - | 1 (true) |

Table 89: CFG-UART1 configuration defaults

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td>Unit</td><td>Default value</td></tr><tr><td>CFG-UART1INPROT-UBX</td><td>0x10730001</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-UART1INPROT-NMEA</td><td>0x10730002</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr></tbody></table>

## Table 90: CFG-UART1INPROT configuration defaults

<table><tbody><tr><td>Configuration item</td><td>Key ID</td><td colspan="2">Type Scale</td><td>Unit</td><td>Default value</td></tr><tr><td>CFG-UART1OUTPROT-UBX</td><td>0x10740001</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr><tr><td>CFG-UART1OUTPROT-NMEA</td><td>0x10740002</td><td>L</td><td>-</td><td>-</td><td>1 (true)</td></tr></tbody></table>

## Table 91: CFG-UART1OUTPROT configuration defaults

UBX-23002975 - R02 C1-Public

Configuration defaults

Page 178 of 181

<!-- page 179 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## Related documents

**[1]** Receiver Data sheet

**[2]** Receiver Integration manual

**[3]** NMEA 0183 Standard for Interfacing Marine Electronic Devices, Version 4.11, November 2018

![Image block](doc:a53f2ec/tier:standard/page:179/block:7)

For regular updates to u-blox documentation and to receive product change notifications please register on our homepage ([https://www.u-blox.com](https://www.u-blox.com)).

UBX-23002975 - R02 C1-Public

Related documents

Page 179 of 181

<!-- page 180 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

Revision history

| Revision | Date | Status/Comments |
| --- | --- | --- |
| R01 | 09-Nov-2023 | Initial release |
| R02 | 13-Jun-2024 | New message UBX-MON-RCVRSTAT |

UBX-23002975 - R02 C1-Public

Revision history

Page 180 of 181

<!-- page 181 of 181 -->

Cblox

u-blox F10 SPG 6.00 - Interface description

## Contact

**u-blox AG**

Address: Zürcherstrasse 68

8800 Thalwil

Switzerland

For further support and contact information, visit us at [www.u-blox.com/support](https://www.u-blox.com/support).

UBX-23002975 - R02 C1-Public

Page 181 of 181