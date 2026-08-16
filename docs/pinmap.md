# Pin map

Fill this in from the physical wiring, then make `board_config.h` match it.
This table is the record of what is actually on the bench; the header is
the code's copy of it. If they disagree, this table wins and the header is
wrong.

## Board A -- elevator controller

| Signal | GPIO | Direction | Peripheral | Notes |
|---|---|---|---|---|
| STEPPER_EN | | | | |
| STEPPER_STEP | | | | |
| STEPPER_DIR | | | | |
| STEPPER_TX | | | | |
| STEPPER_RX | | | | |
| HCSR04_TRIG | | | | |
| HCSR04_ECHO | | | | |
| I2C_SDA | | | | |
| I2C_SCL | | | | |
| SPI_SCK | | | | |
| SPI_MOSI | | | | |
| SPI_MISO | | | | |
| RC522_CS | | | | |
| RC522_RST | | | | |
| BTN_CAR_0 | | | | |
| BTN_CAR_1 | | | | |
| BTN_CAR_2 | | | | |

## Board B -- fall detector

| Signal | GPIO | Direction | Peripheral | Notes |
|---|---|---|---|---|
| I2C_SDA | | | | |
| I2C_SCL | | | | |
| MPU_INT | | | | |

## Reserved pins -- do not assign

| Range | Reason |
|---|---|
| GPIO26-32 | SPI flash |
| GPIO33-37 | Octal PSRAM, N16R8 module |
| GPIO0, 3, 45, 46 | Strapping pins |
| GPIO19, 20 | Native USB |
| GPIO43, 44 | UART0 console |
| GPIO38 | On-board RGB LED |

## Board assignment

The two DevKitC-1 boards are physically identical and must be told apart by
label, not by memory. Put a piece of tape on each one.

| Board | Role | Who holds it |
|---|---|---|
| **B** | Fall detector: MPU6050 only, transmits over ESP-NOW | **Hongyi** (this is the board the IMU bring-up was done on) |
| A | Elevator controller: stepper, encoder, ultrasonic, LCD, RFID, buttons | to be confirmed |

Getting this backwards costs an evening: a swapped peer MAC makes
`esp_now_send()` report SUCCESS while nothing ever arrives, and the first
things anyone suspects are the antenna, the channel, and the code.

## Hardware addresses

| Item | Value | Filled by |
|---|---|---|
| Board A STA MAC | `80:B5:4E:E3:22:50` | mac_print, 2026-08-15 -- **RECHECK: confirm this is the controller board, not the one now carrying the IMU** |
| Board B STA MAC | | |
| LCD I2C address | | |
| IMU I2C address | | |
| WiFi channel | 1 | mac_print, 2026-08-15 |
