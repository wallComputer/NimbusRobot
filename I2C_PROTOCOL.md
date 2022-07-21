# Raspberry Pi - Nimbus Robot I2C Communication

## Register Map- Quick Reference

The Raspberry Pi can send the following commands to control the RP2040

| Register           | Address | Functionality                                |  Read/Write  |  Unit   | Values                             |
|:-------------------|:--------|:---------------------------------------------|:------------:|:-------:|:-----------------------------------|
| [NR](#NR-Register) | 0x01    | Reset the Nimbus Robot                       |  Write Only  |    -    | Any Value Resets the RP2040        |
 | NBLV               | 0x02    | Read Battery Voltage                         |  Read Only   |    V    | [3.6, 4.2] for healthy battery.    |
 | NPBV               | 0x03    | Read Power Booster Voltage                   |  Write Only  |    V    | [5.02, 5.17] for sufficient power. |
 | NBCV               | 0x04    | Cutoff Voltage for Battery                   | Read & Write |    V    | [3.6,3.9] for safe operation.      |
 | NMLTC              | 0x05    | Current for Motor Left Top                   |  Read Only   |   mA    | [0,300]                            |
 | NMLBC              | 0x06    | Current for Motor Left Bottom                |  Read Only   |   mA    | [0,300]                            |
 | NMRTC              | 0x07    | Current for Motor Right Top                  |  Read Only   |   mA    | [0,300]                            |
 | NMRBC              | 0x08    | Current for Motor Right Bottom               |  Read Only   |   mA    | [0,300]                            |
 | NLED               | 0x09    | Set or Reset to BLUE LED                     | Read & Write |    -    | 1: Set, 0: Reset                   |
 | NNPXL              | 0x0A    | Set or Read Neopixel LED                     | Read & Write |    -    | RGB Value from [0,255]             |
 | NMSC               | 0x0B    | Set or Reset Motor Controller Switch         | Read & Write |    -    | 1: Set, 0: Reset                   |
 | NMLTS              | 0x0C    | Set or Read Motor Left Top Speed             | Read & Write |    %    | [-100,100]                         |
 | NMLBS              | 0x0D    | Set or Read Motor Left Bottom Speed          | Read & Write |    %    | [-100,100]                         |
 | NMLTS              | 0x0E    | Set or Read Motor Right Top Speed            | Read & Write |    %    | [-100,100]                         |
 | NMLBS              | 0x0F    | Set or Read Motor Right Bottom Speed         | Read & Write |    %    | [-100,100]                         |
 | NMLTT              | 0x10    | Set or Read Motor Left Top Encoder Ticks     | Read & Write |    -    | [0,65535]                          |
 | NMLTT              | 0x11    | Set or Read Motor Left Bottom Encoder Ticks  | Read & Write |    -    | [0,65535]                          |
 | NMLTT              | 0x12    | Set or Read Motor Right Top Encoder Ticks    | Read & Write |    -    | [0,65535]                          |
 | NMLTT              | 0x13    | Set or Read Motor Right Bottom Encoder Ticks | Read & Write |    -    | [0,65535]                          |
 | NMTSP              | 0x14    | Set or Read Robot Twist Speed                | Read & Write |    %    | [-100,100]                         |
 | NMTAR              | 0x15    | Set or Read Robot Twist Angle                | Read & Write | radians | [0,2π]                             |
 | NMTTS              | 0x16    | Set or Read Robot Twist Time                 | Read & Write | seconds | [0,255]                            |            


## Register Details

### NR Register

The reset register. Resets all registers to default value