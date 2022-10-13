# Raspberry Pi - Nimbus Robot I2C Communication

## Register Map- Quick Reference

The Raspberry Pi can send the following commands to control the RP2040

|    | Register           | Address | Functionality                       |  Read/Write  | Unit  |               Values                |    Datatype    |     Size | Byte Count |
|:--:|:-------------------|:--------|:------------------------------------|:------------:|:-----:|:-----------------------------------:|:--------------:|---------:|:----------:|
 | 1. | [NR](#NR-Register) | 0x00    | Reset the Nimbus Robot              |  Write Only  |   -   |    Any Value Resets the RP2040.     |      byte      |   1 Byte |     1      |
 | 2. | NLED               | 0x01    | Set or Reset to BLUE LED            | Read & Write |   -   |            1: On, 0: Off            |      byte      |   1 Byte |     1      |
 | 3. | NMSC               | 0x02    | Set or Reset Motor Switch Pin       | Read & Write |   -   |            1: On, 0: Off            |      byte      |   1 Byte |     1      |
 | 4. | NBTN               | 0x03    | Read Button State                   |  Read only.  |   -   |     1: Not Pressed, 0: Pressed      |      byte      |   1 Byte |     1      |
 | 5. | NNPXL              | 0x04    | Set or Read Neopixel LED            | Read & Write |   -   |       RGB Value from [0,255]        | byte array[3]  |  3 Bytes |     3      |
 | 6. | NBLV               | 0x07    | Read Battery Voltage                |  Read Only   |   V   |   [3.6, 4.2] for healthy battery.   |     float      |  4 Bytes |     4      |
 | 7. | NPBV               | 0x0B    | Read Power Booster Voltage          |  Write Only  |   V   | [5.02, 5.17] for sufficient power.  |     float      |  4 Bytes |     4      |
 | 8. | NBCV               | 0x0F    | Cutoff Voltage for Battery          | Read & Write |   V   |    [3.7,3.9] for safe operation.    |     float      |  4 Bytes |     4      |
 | 9. | NMLTC              | 0x13    | Current for Motor Left Top          |  Read Only   |  mA   |               [0,300]               |     float      |  4 Bytes |     4      |
 | 10. | NMLBC              | 0x17    | Current for Motor Left Bottom       |  Read Only   |  mA   |               [0,300]               |     float      |  4 Bytes |     4      |
 | 11. | NMRTC              | 0x1B    | Current for Motor Right Top         |  Read Only   |  mA   |               [0,300]               |     float      |  4 Bytes |     4      |
 | 12. | NMRBC              | 0x1F    | Current for Motor Right Bottom      |  Read Only   |  mA   |               [0,300]               |     float      |  4 Bytes |     4      |
 | 13. | NMLTS              | 0x23    | Speed for Motor Left Top            | Read & Write |   -   |           [-65535,65535]            |     float      |  4 Bytes |     4      |
 | 14. | NMLBS              | 0x27    | Speed for Motor Left Bottom         | Read & Write |   -   |           [-65535,65535]            |     float      |  4 Bytes |     4      |
 | 15. | NMRTS              | 0x2B    | Speed for Motor Right Top           | Read & Write |   -   |           [-65535,65535]            |     float      |  4 Bytes |     4      |
 | 16. | NMRBS              | 0x2F    | Speed for Motor Right Bottom        | Read & Write |   -   |           [-65535,65535]            |     float      |  4 Bytes |     4      |
 | 17. | NMTVPT             | 0x33    | Set Theta, Velocity, and Phi        | Read & Write | mixed | [(-π,π],[0,65535],(-π,π],[200ms, ]] | float array[3] | 12 Bytes |     12     |
 | 18. | NMT                | 0x3F    | Set Default Timeout for all motors. | Read & Write |   -   |            [200ms,65535]            |     float      |  4 Bytes |     4      |
 | 19. | NMTB               | 0x43    | Set Default Timeout Behaviour.      | Read & Write |   -   |      1: Use Timer, 0: Free Run      |      byte      |  1 Bytes |     1      |
 |  . | **DC**             | **68**  | **Number of Read/Write Registers**  |    **-**     | **-** |                **-**                |       -        |   **68** |   **68**   |




## Register Details

### NR Register

The reset register. Resets all registers to default value