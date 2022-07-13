# Pinout for Nimbus Robot

## Motor Connections

###
## Motor Control Switch
| Functionality | RP2040 Pin# |       nrpd.py    |
| :------------ | ----------: |-----------------:|
| Motor Switch  |      GPIO28 | MOTOR_SWITCH_PIN |

### Left Side Motors

#### Output Direction Pins

| Functionality    | RP2040 Pin# | nrpd.py |
| :--------------- | ----------: |--------:|
| Motor Top IN1    |       GPIO8 |  MLTIN1 |
| Motor Top IN2    |       GPIO9 |  MLTIN2 |
| Motor Bottom IN1 |      GPIO11 |  MLBIN1 |
| Motor Bottom IN2 |      GPIO10 |  MLBIN2 |

#### Encoder Input Pins

| Functionality    | RP2040 Pin# | nrpd.py |
| :--------------- | ----------: |--------:|
| Motor Top ENA    |      GPIO14 |    ELTA |
| Motor Top ENB    |      GPIO15 |    ELTB |
| Motor Bottom ENA |      GPIO12 |    ELBA |
| Motor Bottom ENB |      GPIO13 |    ELBB |

### Right Side Motors 

#### Output Direction Pins

| Functionality    | RP2040 Pin# | nrpd.py |
| :--------------- | ----------: |--------:|
| Motor Top IN1    |       GPIO7 |  MRTIN1 |
| Motor Top IN2    |       GPIO6 |  MRTIN2 |
| Motor Bottom IN1 |       GPIO4 |  MRBIN1 |
| Motor Bottom IN2 |       GPIO5 |  MRBIN2 |

#### Encoder Input Pins

| Functionality    | RP2040 Pin# | nrpd.py |
| :--------------- | ----------: |--------:|
| Motor Top ENA    |       GPIO0 |    ERTA |
| Motor Top ENB    |       GPIO1 |    ERTB |
| Motor Bottom ENA |       GPIO3 |    ERBA |
| Motor Bottom ENB |       GPIO2 |    ERBB |
---

## LEDs
| Functionality | RP2040 Pin# |      nrpd.py |
| :------------ | ----------: |-------------:|
| RGB NEOPIXEL  |      GPIO18 | NEOPIXEL_PIN |
| BLUE LED      |      GPIO19 |  BLUELED_PIN |
---

## User Button
| Functionality | RP2040 Pin# |    nrpd.py |
| :------------ | ----------: |-----------:|
| USER BUTTON   |      GPIO23 | BUTTON_PIN |
---

## UART
| Functionality | RP2040 Pin# | nrpd.py |
| :------------ | ----------: |--------:|
| UART Rx       |      GPIO17 | UART_RX |
| UART Tx       |      GPIO16 | UART_TX |
---

## SWD
| Functionality | RP2040 Pin# |
| :------------ | ----------: |
| SWDIO         |      GPIO25 |
| SWCLK         |      GPIO24 |
---

## I2C
| Functionality | RP2040 Pin# |     nrpd.py   |
| :------------ | ----------: |--------------:|
| SDA           |      GPIO20 |       I2C_SDA |
| SCL           |      GPIO21 |       I2C_SCL |
| INT           |      GPIO19 | INTERRUPT_PIN |
---

## SERVOS
| Functionality | RP2040 Pin# |    nrpd.py |
|:--------------|------------:|-----------:|
 | Servo 1 Pin   |      GPIO27 | SERVO1_PIN |
 | Servo 2 Pin   |      GPIO26 | SERVO2_PIN |
---



## ADC

### ADC Input Pin
| Functionality | RP2040 Pin# |    nrpd.py |
| :------------ | ----------: |-----------:|
| ADC READ      |      GPIO29 | ADC_IN_PIN |

### ADC Control Pins
| Functionality | RP2040 Pin# |   nrpd.py |
| :------------ | ----------: |----------:|
| ADC SEL0      |      GPIO24 | ADC_SEL_0 |
| ADC SEL1      |      GPIO25 | ADC_SEL_1 |
| ADC SEL2      |      GPIO22 | ADC_SEL_2 |

### ADC Channels
| Functionality                    | ADC Channel# |
| :------------------------------- | -----------: |
| FAULT RIGHT                      |      ADC IN0 |
| Motor Right Bottom Current Sense |      ADC IN1 |
| Motor Right Top Current Sense    |      ADC IN2 |
| Battery Level                    |      ADC IN3 |
| Motor Left Top Current Sense     |      ADC IN4 |
| PowerBooster Output Level        |      ADC IN5 |
| FAULT Left                       |      ADC IN6 |
| Motor Left Bottom Current Sense  |      ADC IN7 |
---
