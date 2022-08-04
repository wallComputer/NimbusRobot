# Raspberry Pi - Nimbus Robot I2C Communication

## Register Map- Quick Reference

The Raspberry Pi can send the following commands to control the RP2040

|     | Register           | Address | Functionality                      |  Read/Write  | Unit  | Values                             |   Datatype    |    Size | Byte Count |
|:---:|:-------------------|:--------|:-----------------------------------|:------------:|:-----:|:-----------------------------------|:-------------:|--------:|:----------:|
 | 1.  | [NR](#NR-Register) | 0x00    | Reset the Nimbus Robot             |  Write Only  |   -   | Any Value Resets the RP2040.       |     byte      |  1 Byte |     1      |
 | 10. | NLED               | 0x01    | Set or Reset to BLUE LED           | Read & Write |   -   | 1: On, 0: Off                      |     byte      |  1 Byte |     1      |
 | 11. | NMSC               | 0x02    | Set or Reset Motor Switch Pin      | Read & Write |   -   | 1: On, 0: Off                      |     byte      |  1 Byte |     1      |
 | 12. | NBTN               | 0x03    | Read Button State                  |  Read only.  |   -   | 1: Not Pressed, 0: Pressed         |     byte      |  1 Byte |     1      |
 | 9.  | NNPXL              | 0x04    | Set or Read Neopixel LED           | Read & Write |   -   | RGB Value from [0,255]             | byte array[3] | 3 Bytes |     3      |
 | 2.  | NBLV               | 0x07    | Read Battery Voltage               |  Read Only   |   V   | [3.6, 4.2] for healthy battery.    | byte array[2] | 2 Bytes |     2      |
 | 3.  | NPBV               | 0x09    | Read Power Booster Voltage         |  Write Only  |   V   | [5.02, 5.17] for sufficient power. | byte array[2] | 2 Bytes |     2      |
 | 4.  | NBCV               | 0x0B    | Cutoff Voltage for Battery         | Read & Write |   V   | [3.6,3.9] for safe operation.      | byte array[2] | 2 Bytes |     2      |
 | 5.  | NMLTC              | 0x0D    | Current for Motor Left Top         |  Read Only   |  mA   | [0,300]                            | byte array[2] | 2 Bytes |     2      |
 | 6.  | NMLBC              | 0x0F    | Current for Motor Left Bottom      |  Read Only   |  mA   | [0,300]                            | byte array[2] | 2 Bytes |     2      |
 | 7.  | NMRTC              | 0x11    | Current for Motor Right Top        |  Read Only   |  mA   | [0,300]                            | byte array[2] | 2 Bytes |     2      |
 | 8.  | NMRBC              | 0x13    | Current for Motor Right Bottom     |  Read Only   |  mA   | [0,300]                            | byte array[2] | 2 Bytes |     2      |
 |  .  | **DC**             | **21**  | **Number of Read/Write Registers** | **Neither**  | **-** | **-**                              |       -       |  **21** |   **21**   |




## Register Details

### NR Register

The reset register. Resets all registers to default value