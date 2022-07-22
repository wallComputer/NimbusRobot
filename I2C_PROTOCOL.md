# Raspberry Pi - Nimbus Robot I2C Communication

## Register Map- Quick Reference

The Raspberry Pi can send the following commands to control the RP2040

| Register           | Address | Functionality                                |  Read/Write  |  Unit   | Values                             |   Size   |
|:-------------------|:--------|:---------------------------------------------|:------------:|:-------:|:-----------------------------------|:--------:|
| [NR](#NR-Register) | 0x01    | Reset the Nimbus Robot                       |  Write Only  |    -    | Any Value Resets the RP2040        |   0x00   |
 | NBLV               | 0x02    | Read Battery Voltage                         |  Read Only   |    V    | [3.6, 4.2] for healthy battery.    |  0x0000  |
 | NPBV               | 0x04    | Read Power Booster Voltage                   |  Write Only  |    V    | [5.02, 5.17] for sufficient power. |  0x0000  |
 | NBCV               | 0x06    | Cutoff Voltage for Battery                   | Read & Write |    V    | [3.6,3.9] for safe operation.      |  0x0000  |
 | NMLTC              | 0x08    | Current for Motor Left Top                   |  Read Only   |   mA    | [0,300]                            |  0x0000  |
 | NMLBC              | 0x0A    | Current for Motor Left Bottom                |  Read Only   |   mA    | [0,300]                            |  0x0000  |
 | NMRTC              | 0x0C    | Current for Motor Right Top                  |  Read Only   |   mA    | [0,300]                            |  0x0000  |
 | NMRBC              | 0x0E    | Current for Motor Right Bottom               |  Read Only   |   mA    | [0,300]                            |  0x0000  |
 | NNPXL              | 0x10    | Set or Read Neopixel LED                     | Read & Write |    -    | RGB Value from [0,255]             | 0x000000 |
 | NLED               | 0x13    | Set or Reset to BLUE LED                     | Read & Write |    -    | 1: Set, 0: Reset                   |   0x00   |
 | NMSC               | 0x14    | Set or Reset Motor Controller Switch         | Read & Write |    -    | 1: Set, 0: Reset                   |   0x00   |
 | NMLTS              | 0x15    | Set or Read Motor Left Top Speed             | Read & Write |    %    | [-100,100]                         |   0x00   |
 | NMLBS              | 0x16    | Set or Read Motor Left Bottom Speed          | Read & Write |    %    | [-100,100]                         |   0x00   |
 | NMRTS              | 0x17    | Set or Read Motor Right Top Speed            | Read & Write |    %    | [-100,100]                         |   0x00   |
 | NMRBS              | 0x18    | Set or Read Motor Right Bottom Speed         | Read & Write |    %    | [-100,100]                         |   0x00   |
 | NMLTT              | 0x19    | Set or Read Motor Left Top Encoder Ticks     | Read & Write |    -    | [0,65535]                          |  0x0000  |
 | NMLBT              | 0x1B    | Set or Read Motor Left Bottom Encoder Ticks  | Read & Write |    -    | [0,65535]                          |  0x0000  |
 | NMRTT              | 0x1D    | Set or Read Motor Right Top Encoder Ticks    | Read & Write |    -    | [0,65535]                          |  0x0000  |
 | NMRBT              | 0x1F    | Set or Read Motor Right Bottom Encoder Ticks | Read & Write |    -    | [0,65535]                          |  0x0000  |
 | NMTSP              | 0x21    | Set or Read Robot Twist Speed                | Read & Write |    %    | [-100,100]                         |   0x00   |
 | NMTAR              | 0x22    | Set or Read Robot Twist Angle                | Read & Write | radians | [0,2π]                             |  0x0000  |
 | NMTTS              | 0x24    | Set or Read Robot Twist Time                 | Read & Write | seconds | [0,255]                            |   0x00   |
 | NBTN               | 0x25    | Set or Read Robot Twist Time                 | Read & Write | seconds | [0,255]                            |   0x00   |
 | **DC**             | **37**  | **Number of Read/Write Registers**           | **Neither**  |  **-**  | **-**                              | **37**   |




## Register Details

### NR Register

The reset register. Resets all registers to default value