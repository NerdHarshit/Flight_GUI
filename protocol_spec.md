# AvioPro Ground Station & PCB Interface Contract

This document outlines the exact CSV line formats, binary structures, and command protocols required for the Ground Station PCB firmware (Arduino / C++ / ESP32 / RP2040) to communicate seamlessly with the AvioPro Flight GUI over Serial (USB/UART).

---

## 1. Overview & Ground Station PCB Responsibilities

The Ground Station PCB acts as a dual-frequency radio gateway and pre-processor:
1. **Frequency Hopping / Dual-Receiver**: Listens for Avionics (Controller A+B) on one frequency and CanSat on another.
2. **CRC Verification**: Verifies incoming radio packet CRC.
3. **Attitude Computation**: Computes Pitch, Roll, and Yaw from IMU data if not already pre-calculated by flight software.
4. **Radio Telemetry Metrics**: Appends local ground-station metrics (`rssi`, `snr`, `packet_loss`) to each packet.
5. **Serial Output**: Formats each telemetry payload as a **Comma-Separated ASCII String (CSV)** terminated with `\n` (newline) and sends it over Serial (115200 baud).

---

## 2. Downlink: Telemetry Packet CSV Formats

### 2.1 Avionics Packet (Controller A + B) — Prefix: `CAB`

- **Line Identifier**: `CAB`
- **Total CSV Fields**: 38

| Index | Field Name | Type | Description |
|---|---|---|---|
| 0 | `packetType` | String | Must be `"CAB"` (or `"A"`) |
| 1 | `missionTime` | uint32_t | Milliseconds since boot (`ms`) |
| 2 | `teamID` | uint8_t | Competition Team ID |
| 3 | `altitude` | float | Filtered barometric altitude (`m`) |
| 4 | `maxAltitude` | float | Peak recorded altitude (`m`) |
| 5 | `accelX` | float | Controller A Acceleration X (`m/s²`) |
| 6 | `accelY` | float | Controller A Acceleration Y (`m/s²`) |
| 7 | `accelZ` | float | Controller A Acceleration Z (`m/s²`) |
| 8 | `accelMag` | float | Controller A Accel Magnitude (`G` or `m/s²`) |
| 9 | `gyroX` | float | Controller A Gyro X (`rad/s` or `deg/s`) |
| 10 | `gyroY` | float | Controller A Gyro Y (`rad/s` or `deg/s`) |
| 11 | `gyroZ` | float | Controller A Gyro Z (`rad/s` or `deg/s`) |
| 12 | `temperature` | float | Temperature (`°C`) |
| 13 | `pressure` | float | Barometric pressure (`hPa`) |
| 14 | `pitch` | float | Controller A Pitch (`deg`) |
| 15 | `roll` | float | Controller A Roll (`deg`) |
| 16 | `yaw` | float | Controller A Yaw (`deg`) |
| 17 | `batteryVoltage` | float | Controller A Battery Voltage (`V`) |
| 18 | `gpsLat` | float | GPS Latitude (decimal degrees) |
| 19 | `gpsLon` | float | GPS Longitude (decimal degrees) |
| 20 | `gpsAltitude` | float | GPS Altitude (`m`) |
| 21 | `gpsSats` | uint8_t | Number of visible GPS satellites |
| 22 | `gpsStale` | uint8_t | `1` if fix is old/invalid, `0` if valid |
| 23 | `A_state` | uint8_t | Controller A FSM State ID |
| 24 | `A_launched` | uint8_t | `1` if launched, `0` otherwise |
| 25 | `A_apogee` | uint8_t | `1` if apogee reached, `0` otherwise |
| 26 | `A_separated` | uint8_t | `1` if payload separated, `0` otherwise |
| 27 | `A_landed` | uint8_t | `1` if landed, `0` otherwise |
| 28 | `controllerBAlive`| uint8_t | `1` if Controller B is online, `0` if dead |
| 29 | `B_ax` | float | Controller B Acceleration X (`m/s²`) |
| 30 | `B_ay` | float | Controller B Acceleration Y (`m/s²`) |
| 31 | `B_az` | float | Controller B Acceleration Z (`m/s²`) |
| 32 | `B_baro_alt` | float | Controller B Barometric Altitude (`m`) |
| 33 | `B_state` | uint8_t | Controller B FSM State ID |
| 34 | `battLow` | uint8_t | `1` if battery is low, `0` otherwise |
| 35 | `rssi` | int16_t | Radio Received Signal Strength (`dBm`) |
| 36 | `snr` | int16_t | Radio Signal-to-Noise Ratio (`dB`) |
| 37 | `packet_loss` | uint16_t | Ground Station Packet Loss (`%`) |

#### Arduino C++ Formatting Example (`CAB`):
```cpp
snprintf(buffer, sizeof(buffer),
  "CAB,%lu,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.1f,%.1f,%.2f,%.6f,%.6f,%.1f,%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%u,%u,%d,%d,%u\n",
  missionTime, teamID, altitude, maxAltitude,
  accelX, accelY, accelZ, accelMag,
  gyroX, gyroY, gyroZ,
  temperature, pressure,
  pitch, roll, yaw,
  batteryVoltage, gpsLat, gpsLon, gpsAltitude, gpsSats, gpsStale ? 1 : 0,
  A_state, A_launched ? 1:0, A_apogee ? 1:0, A_separated ? 1:0, A_landed ? 1:0,
  controllerBAlive ? 1:0, B_ax, B_ay, B_az, B_baro_alt, B_state, battLow ? 1:0,
  rssi, snr, packet_loss
);
Serial.println(buffer);
```

---

### 2.2 CanSat Payload Packet — Prefix: `CANSAT`

- **Line Identifier**: `CANSAT` (or `CAN`)
- **Total CSV Fields**: 26

| Index | Field Name | Type | Description |
|---|---|---|---|
| 0 | `packetType` | String | Must be `"CANSAT"` (or `"CAN"`) |
| 1 | `teamID` | uint8_t | Competition Team ID |
| 2 | `timestamp` | uint32_t | Milliseconds since boot (`ms`) |
| 3 | `state` | uint8_t | CanSat FSM State ID |
| 4 | `ax` | float | CanSat Accel X (`m/s²`) |
| 5 | `ay` | float | CanSat Accel Y (`m/s²`) |
| 6 | `az` | float | CanSat Accel Z (`m/s²`) |
| 7 | `h_raw` | float | Raw altitude (`m`) — *Displayed in GUI as main height* |
| 8 | `h_filtered` | float | Filtered altitude (`m`) — *Logged to CSV* |
| 9 | `gx` | float | Gyro X (`rad/s` or `deg/s`) |
| 10 | `gy` | float | Gyro Y (`rad/s` or `deg/s`) |
| 11 | `gz` | float | Gyro Z (`rad/s` or `deg/s`) |
| 12 | `magX` | float | Magnetometer X (`uT`) — *Logged to CSV* |
| 13 | `magY` | float | Magnetometer Y (`uT`) — *Logged to CSV* |
| 14 | `magZ` | float | Magnetometer Z (`uT`) — *Logged to CSV* |
| 15 | `lat` | float | GPS Latitude (decimal degrees) |
| 16 | `lon` | float | GPS Longitude (decimal degrees) |
| 17 | `alt` | float | GPS Altitude (`m`) |
| 18 | `satcnt` | uint8_t | GPS Satellite Count |
| 19 | `COG` | float | Course Over Ground (`deg`) |
| 20 | `headingErr` | float | Heading Error (`deg`) |
| 21 | `distToTarget` | float | Distance to Target (`m`) |
| 22 | `batteryVoltage` | float | CanSat Battery Voltage (`V`) — *Logged to CSV* |
| 23 | `rssi` | int16_t | Radio RSSI (`dBm`) |
| 24 | `snr` | int16_t | Radio SNR (`dB`) |
| 25 | `packet_loss` | uint16_t | Radio Packet Loss (`%`) |

#### Arduino C++ Formatting Example (`CANSAT`):
```cpp
snprintf(buffer, sizeof(buffer),
  "CANSAT,%u,%lu,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.1f,%u,%.1f,%.1f,%.1f,%.2f,%d,%d,%u\n",
  teamID, timestamp, state,
  ax, ay, az,
  h_raw, h_filtered,
  gx, gy, gz,
  magX, magY, magZ,
  lat, lon, alt, satcnt,
  COG, headingErr, distToTarget,
  batteryVoltage, rssi, snr, packet_loss
);
Serial.println(buffer);
```

---

### 2.3 Ground Station Status Packet (Optional Heartbeat) — Prefix: `STATUS`

Format:
`STATUS,Marker,BmpOK,BnoOK,SdOK,FlashOK,LoraOK,GpsSearch,CtrlB,BattV,FlightNum,RSSI,SNR`

Example:
`STATUS,GS1,1,1,1,1,1,0,1,12.4,1,-65,9.5`

---

## 3. Uplink: Command Protocol (GUI → Ground Station PCB)

When an operator interacts with command controls in the GUI, binary command frames are emitted over Serial. The Ground Station PCB reads these frames and relays them over LoRa.

### Command Frame Structure
```
[SYNC_1: 0xAA] [SYNC_2: 0x55] [CMD_TYPE: 1 Byte] [CMD_ID_LSB: 1 Byte] [CMD_ID_MSB: 1 Byte] [PAYLOAD_BYTES...]
```

### Command Types (`CMD_TYPE`)

| Hex Code | Command Name | Payload Size | Description |
|---|---|---|---|
| `0x01` | `CMD_START_TELEMETRY` | 0 Bytes | Enable high-frequency telemetry downlink |
| `0x02` | `CMD_STOP_TELEMETRY` | 0 Bytes | Pause telemetry downlink |
| `0x03` | `CMD_SENSOR_ZERO` | 0 Bytes | Calibrate altitude baseline (zero sensors) |
| `0x04` | `CMD_RESET_TIMER` | 0 Bytes | Reset mission clock |
| `0x05` | `CMD_REQUEST_STATUS` | 0 Bytes | Request instant health report |
| `0x09` | `CMD_BUZZER_PLAY` | 0 Bytes | Trigger locator buzzer audio pattern |
| `0x0C` | `CMD_RESET_CONTROLLER`| 0 Bytes | Command software reboot (Launchpad state only) |
| `0x10` | `CMD_ENABLE_SERVOS` | 0 Bytes | Unlock CanSat servo controls |
| `0x11` | `CMD_DISABLE_SERVOS` | 0 Bytes | Lock CanSat servo controls |
| `0x12` | `CMD_SERVO_L_POS` | 0 Bytes | Nudge Left Servo positive |
| `0x13` | `CMD_SERVO_L_NEG` | 0 Bytes | Nudge Left Servo negative |
| `0x14` | `CMD_SERVO_R_POS` | 0 Bytes | Nudge Right Servo positive |
| `0x15` | `CMD_SERVO_R_NEG` | 0 Bytes | Nudge Right Servo negative |
| `0x16` | `CMD_SET_TARGET_GPS` | 8 Bytes | `float32` Latitude, `float32` Longitude (Little-Endian) |
| `0x17` | `CMD_SET_PD_CONSTANTS`| N Bytes | Array of `float32` control parameters |
| `0x18` | `CMD_SPIRAL_DOWN` | 0 Bytes | Emergency spiral descent descent trigger |

### Acknowledgment (ACK) Back channel
To acknowledge command reception, the PCB sends back:
`ACK id=<CMD_ID>` over Serial to confirm delivery and stop GUI command retries.
