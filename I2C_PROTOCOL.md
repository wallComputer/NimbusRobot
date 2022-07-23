# Raspberry Pi - Nimbus Robot I2C Communication

## Register Map- Quick Reference

The Raspberry Pi can send the following commands to control the RP2040

|     | Register           | Address | Functionality                         |  Read/Write  |  Unit   | Values                                        |    Size    | Byte Count |
|:---:|:-------------------|:--------|:--------------------------------------|:------------:|:-------:|:----------------------------------------------|:----------:|:----------:|
 | 1.  | [NR](#NR-Register) | 0x01    | Reset the Nimbus Robot                |  Write Only  |    -    | Any Value Resets the RP2040                   |    0x00    |     1      |
 | 2.  | NBLV               | 0x02    | Read Battery Voltage                  |  Read Only   |    V    | [3.6, 4.2] for healthy battery.               |   0x0000   |     2      |
 | 3.  | NPBV               | 0x04    | Read Power Booster Voltage            |  Write Only  |    V    | [5.02, 5.17] for sufficient power.            |   0x0000   |     2      |
 | 4.  | NBCV               | 0x06    | Cutoff Voltage for Battery            | Read & Write |    V    | [3.6,3.9] for safe operation.                 |   0x0000   |     2      |
 | 5.  | NMLTC              | 0x08    | Current for Motor Left Top            |  Read Only   |   mA    | [0,300]                                       |   0x0000   |     2      |
 | 6.  | NMLBC              | 0x0A    | Current for Motor Left Bottom         |  Read Only   |   mA    | [0,300]                                       |   0x0000   |     2      |
 | 7.  | NMRTC              | 0x0C    | Current for Motor Right Top           |  Read Only   |   mA    | [0,300]                                       |   0x0000   |     2      |
 | 8.  | NMRBC              | 0x0E    | Current for Motor Right Bottom        |  Read Only   |   mA    | [0,300]                                       |   0x0000   |     2      |
 | 9.  | NNPXL              | 0x10    | Set or Read Neopixel LED              | Read & Write |    -    | RGB Value from [0,255]                        |  0x000000  |     3      |
 | 10. | NLED               | 0x13    | Set or Reset to BLUE LED              | Read & Write |    -    | 1: On, 0: Off                                 |    0x00    |     1      |
 | 11. | NMSC               | 0x14    | Set or Reset Motor Controller Switch  | Read & Write |    -    | 1: On, 0: Off                                 |    0x00    |     1      |
 | 12. | NMAS               | 0x15    | Set or Read All Motor Speeds          | Read & Write |    %    | [[-100,100],[-100,100],[-100,100],[-100,100]] | 0x00000000 |     4      |
 | 13. | NMLTTS             | 0x19    | Set Motor Left Top Encoder Ticks      |  Write Only  |    -    | [-2^31-1,2^31-1]                              | 0x00000000 |     4      |
 | 14. | NMLBTS             | 0x1D    | Set Motor Left Bottom Encoder Ticks   |  Write Only  |    -    | [-2^31-1,2^31-1]                              | 0x00000000 |     4      |
 | 15. | NMRTTS             | 0x21    | Set Motor Right Top Encoder Ticks     |  Write Only  |    -    | [-2^31-1,2^31-1]                              | 0x00000000 |     4      |
 | 16. | NMRBTS             | 0x25    | Set Motor Right Bottom Encoder Ticks  |  Write Only  |    -    | [-2^31-1,2^31-1]                              | 0x00000000 |     4      |
 | 17. | NMLTTR             | 0x29    | Read Motor Left Top Encoder Ticks     |  Read Only   |    -    | [-2^31-1,2^31-1]                              | 0x00000000 |     4      |
 | 18. | NMLBTR             | 0x2D    | Read Motor Left Bottom Encoder Ticks  |  Read Only   |    -    | [-2^31-1,2^31-1]                              | 0x00000000 |     4      |
 | 19. | NMRTTR             | 0x31    | Read Motor Right Top Encoder Ticks    |  Read Only   |    -    | [-2^31-1,2^31-1]                              | 0x00000000 |     4      |
 | 20. | NMRBTR             | 0x35    | Read Motor Right Bottom Encoder Ticks |  Read Only   |    -    | [-2^31-1,2^31-1]                              | 0x00000000 |     4      |
 | 21. | NRTSP              | 0x39    | Set or Read Robot Twist Speed         | Read & Write |    %    | [0,100]                                       |    0x00    |     1      |
 | 22. | NRTAD              | 0x3A    | Set or Read Robot Twist Angle         | Read & Write | Degrees | [-180,180]                                    |   0x0000   |     2      |
 | 23. | NRTTS              | 0x3C    | Set or Read Robot Twist Time          | Read & Write | seconds | [0,255]                                       |    0x00    |     1      |
 | 24. | NBTN               | 0x3D    | Read Button State                     |  Read only.  |    -    | 1: Not Pressed, 0: Pressed                    |    0x00    |     1      |
 |  .  | **DC**             | **61**  | **Number of Read/Write Registers**    | **Neither**  |  **-**  | **-**                                         |   **61**   |   **61**   |




## Register Details

### NR Register

The reset register. Resets all registers to default value