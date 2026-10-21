/**
 * ============================================================
 *  AvioPro Ground Station Firmware
 *  Hardware : Raspberry Pi Pico + RFM95 (SX1276) LoRa module
 *  Library  : Sandeep Mistry "LoRa" library
 *  Protocol : AvioPro Ground Station & PCB Interface Contract
 *             (see protocol_spec.md)
 * ============================================================
 *
 *  Serial output (USB, 115200 baud) → AvioPro Flight GUI
 *    • CAB,…    – 38-field avionics CSV  (Controller A+B)
 *    • CANSAT,… – 26-field CanSat CSV
 *    • STATUS,… – periodic heartbeat
 *    • ACK id=<CMD_ID> – command acknowledgement
 *
 *  Command uplink (binary frame from GUI over Serial):
 *    [0xAA][0x55][CMD_TYPE][CMD_ID_LSB][CMD_ID_MSB][PAYLOAD…]
 *
 *  Frequency hopping:
 *    • AVIONICS_FREQ (MHz) → Controller A + B
 *    • CANSAT_FREQ   (MHz) → CanSat payload
 * ============================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <math.h>

// ─── Pin Mapping (RP2040 / Pico) ────────────────────────────
// Adjust to match your PCB schematic.
#define LORA_NSS   17   // SPI chip-select (GP17)
#define LORA_RST   20   // Reset          (GP20)
#define LORA_DIO0  22   // IRQ / DIO0     (GP22)
// SPI: default SPI0 → SCK=18, MISO=16, MOSI=19 (Pico default)

// ─── Mission Configuration ───────────────────────────────────
#define TEAM_ID             42      // Change to your competition team ID

#define AVIONICS_FREQ       868E6   // Hz – Controller A+B frequency
#define CANSAT_FREQ         869E6   // Hz – CanSat frequency

#define LORA_SF             9       // Spreading Factor (7–12)
#define LORA_BW             125E3   // Bandwidth in Hz
#define LORA_TX_POWER       20      // dBm (2–20)
#define LORA_CODING_RATE    5       // 4/5

// Frequency-hop dwell time (ms per channel before switching)
#define HOP_DWELL_MS        150

// STATUS heartbeat interval (ms)
#define STATUS_INTERVAL_MS  2000

// ─── Command sync bytes ──────────────────────────────────────
#define CMD_SYNC1  0xAA
#define CMD_SYNC2  0x55

// Maximum command payload bytes (CMD_SET_TARGET_GPS = 8 bytes)
#define CMD_MAX_PAYLOAD  32

// ─── Command type codes (mirror command_manager.py) ──────────
#define CMD_START_TELEMETRY   0x01
#define CMD_STOP_TELEMETRY    0x02
#define CMD_SENSOR_ZERO       0x03
#define CMD_RESET_TIMER       0x04
#define CMD_REQUEST_STATUS    0x05
#define CMD_BUZZER_PLAY       0x09
#define CMD_RESET_CONTROLLER  0x0C
#define CMD_SET_QNH           0x0D
#define CMD_ENABLE_SERVOS     0x10
#define CMD_DISABLE_SERVOS    0x11
#define CMD_SERVO_L_POS       0x12
#define CMD_SERVO_L_NEG       0x13
#define CMD_SERVO_R_POS       0x14
#define CMD_SERVO_R_NEG       0x15
#define CMD_SET_TARGET_GPS    0x16
#define CMD_SET_PD_CONSTANTS  0x17
#define CMD_SPIRAL_DOWN       0x18

// ─── Enums (declared early so Arduino auto-prototypes can resolve them) ──
enum Channel { CH_AVIONICS, CH_CANSAT };

enum CmdParseState {
    WAIT_SYNC1,
    WAIT_SYNC2,
    WAIT_CMD,
    WAIT_ID_LSB,
    WAIT_ID_MSB,
    WAIT_PAYLOAD
};

// ─── Forward declarations ────────────────────────────────────
void handleCommand(uint8_t type, uint16_t id, uint8_t* payload, uint8_t payLen);
void switchChannel(Channel ch);

// ─── CanSat-targeted commands (relayed on CANSAT_FREQ) ───────
static bool isCanSatCommand(uint8_t cmd) {
    switch (cmd) {
        case CMD_ENABLE_SERVOS:
        case CMD_DISABLE_SERVOS:
        case CMD_SERVO_L_POS:
        case CMD_SERVO_L_NEG:
        case CMD_SERVO_R_POS:
        case CMD_SERVO_R_NEG:
        case CMD_SET_TARGET_GPS:
        case CMD_SET_PD_CONSTANTS:
        case CMD_SPIRAL_DOWN:
            return true;
        default:
            return false;
    }
}

// ─── Avionics (CAB) raw packet layout ────────────────────────
// Transmitted by Controller A over LoRa; Controller B data is
// embedded within the same packet.
struct __attribute__((packed)) AvionicsPacket {
    uint8_t  packetType;        // 'A'
    uint32_t missionTime;       // ms since boot
    uint8_t  teamID;
    float    altitude;          // filtered baro alt, m
    float    maxAltitude;       // peak altitude, m

    float    accelX;            // m/s²
    float    accelY;
    float    accelZ;
    float    accelMag;          // magnitude (G or m/s²)

    float    gyroX;             // rad/s
    float    gyroY;
    float    gyroZ;

    float    temperature;       // °C
    float    pressure;          // hPa

    // Pitch/Roll/Yaw — computed by GS if zero
    float    pitch;
    float    roll;
    float    yaw;

    float    batteryVoltage;    // V

    float    gpsLat;
    float    gpsLon;
    float    gpsAltitude;
    uint8_t  gpsSats;
    uint8_t  gpsStale;          // 1 = stale/invalid

    uint8_t  A_state;
    uint8_t  A_launched;
    uint8_t  A_apogee;
    uint8_t  A_separated;
    uint8_t  A_landed;

    uint8_t  controllerBAlive;
    float    B_ax;
    float    B_ay;
    float    B_az;
    float    B_baro_alt;
    uint8_t  B_state;
    uint8_t  battLow;

    uint16_t crc;               // CRC-16 over all preceding bytes
};

// ─── CanSat raw packet layout ────────────────────────────────
struct __attribute__((packed)) CanSatPacket {
    uint8_t  packetType;        // 'C'
    uint8_t  teamID;
    uint32_t timestamp;         // ms since boot
    uint8_t  state;

    float    ax;                // m/s²
    float    ay;
    float    az;

    float    h_raw;             // raw baro alt, m  (displayed in GUI)
    float    h_filtered;        // filtered alt, m  (logged to CSV only)

    float    gx;                // rad/s or deg/s
    float    gy;
    float    gz;

    float    magX;              // µT (logged to CSV)
    float    magY;
    float    magZ;

    float    lat;
    float    lon;
    float    alt;               // GPS altitude, m
    uint8_t  satcnt;

    float    COG;               // Course Over Ground, deg
    float    headingErr;        // deg
    float    distToTarget;      // m

    float    batteryVoltage;    // V (logged to CSV)

    uint16_t crc;               // CRC-16 over all preceding bytes
};

// ─── Ground-station state ─────────────────────────────────────
Channel    currentChannel    = CH_AVIONICS;
uint32_t   lastHopTime       = 0;
uint32_t   lastStatusTime    = 0;
bool       telemetryEnabled  = true;

// Packet-loss tracking per channel
uint32_t   avioRxCount       = 0;
uint32_t   avioExpected      = 0;
uint16_t   avioLastId        = 0;
bool       avioFirstPkt      = true;

uint32_t   canRxCount        = 0;
uint32_t   canExpected       = 0;
uint16_t   canLastId         = 0;
bool       canFirstPkt       = true;

// Latest radio metrics (updated on each received packet)
int16_t    lastRSSI          = 0;
float      lastSNR           = 0.0f;
uint16_t   lastPacketLoss    = 0;

// Serial command parser state
CmdParseState cmdState       = WAIT_SYNC1;
uint8_t       cmdType        = 0;
uint16_t      cmdId          = 0;
uint8_t       cmdPayload[CMD_MAX_PAYLOAD];
uint8_t       cmdPayloadLen  = 0;
uint8_t       cmdPayloadIdx  = 0;

// ─── CRC-16/CCITT ────────────────────────────────────────────
uint16_t crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc <<= 1;
        }
    }
    return crc;
}

// ─── Attitude computation ─────────────────────────────────────
// Computes pitch and roll from normalised accelerometer.
// Yaw requires magnetometer — left as 0 at GS level (no mag on GS).
void computeAttitude(float ax, float ay, float az,
                     float &pitch, float &roll, float &yaw) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) { pitch = roll = yaw = 0.0f; return; }
    ax /= norm; ay /= norm; az /= norm;
    pitch = -(asinf(ax)       * 180.0f / (float)M_PI);
    roll  =  (atan2f(ay, az)  * 180.0f / (float)M_PI);
    yaw   = 0.0f;  // magnetometer not available at GS
}

// ─── Channel switching ────────────────────────────────────────
void switchChannel(Channel ch) {
    LoRa.idle();
    LoRa.setFrequency(ch == CH_AVIONICS ? AVIONICS_FREQ : CANSAT_FREQ);
    currentChannel = ch;
    LoRa.receive();
}

// ─── Packet-loss calculator ───────────────────────────────────
uint16_t calcPacketLoss(uint16_t currentId,
                        uint16_t &lastId,
                        uint32_t &rxCount,
                        uint32_t &expected,
                        bool &isFirst) {
    if (isFirst) {
        lastId   = currentId;
        rxCount  = 1;
        expected = 1;
        isFirst  = false;
        return 0;
    }
    rxCount++;
    uint16_t delta = (uint16_t)(currentId - lastId);
    if (delta == 0 || delta > 1000) delta = 1;  // wrap / jump guard
    expected += delta;
    lastId = currentId;
    if (expected == 0) return 0;
    uint32_t lost = (expected > rxCount) ? (expected - rxCount) : 0;
    return (uint16_t)((lost * 100UL) / expected);
}

// ─── Avionics packet handler ──────────────────────────────────
void handleAvionicsPacket(uint8_t* buf, int size) {
    if ((size_t)size < sizeof(AvionicsPacket)) return;

    AvionicsPacket* pkt = (AvionicsPacket*)buf;

    // CRC verification — covers all bytes before the crc field
    size_t   crcLen  = sizeof(AvionicsPacket) - sizeof(uint16_t);
    uint16_t calcCRC = crc16(buf, crcLen);
    if (calcCRC != pkt->crc) {
        Serial.println("GS_WARN: Avionics CRC mismatch — packet dropped");
        return;
    }

    // Radio metrics from Sandeep Mistry LoRa library
    lastRSSI = (int16_t)LoRa.packetRssi();
    lastSNR  = LoRa.packetSnr();

    // Packet-loss — use lower 16 bits of missionTime as rolling ID
    uint16_t pseudoId  = (uint16_t)(pkt->missionTime & 0xFFFF);
    lastPacketLoss = calcPacketLoss(pseudoId,
                                    avioLastId, avioRxCount,
                                    avioExpected, avioFirstPkt);

    // Attitude — compute from accel if the flight SW didn't send it
    float pitch = pkt->pitch;
    float roll  = pkt->roll;
    float yaw   = pkt->yaw;
    if (pitch == 0.0f && roll == 0.0f && yaw == 0.0f) {
        computeAttitude(pkt->accelX, pkt->accelY, pkt->accelZ,
                        pitch, roll, yaw);
    }

    if (!telemetryEnabled) return;

    // ── CAB CSV — 38 fields (protocol_spec §2.1) ───────────────
    char line[512];
    snprintf(line, sizeof(line),
        "CAB,"
        "%lu,%u,"           // [1] missionTime, [2] teamID
        "%.2f,%.2f,"        // [3] altitude, [4] maxAltitude
        "%.2f,%.2f,%.2f,%.2f,"  // [5-8] accelX/Y/Z/Mag
        "%.2f,%.2f,%.2f,"   // [9-11] gyroX/Y/Z
        "%.1f,%.2f,"        // [12] temperature, [13] pressure
        "%.1f,%.1f,%.1f,"   // [14-16] pitch, roll, yaw
        "%.2f,"             // [17] batteryVoltage
        "%.6f,%.6f,%.1f,"   // [18-20] gpsLat, gpsLon, gpsAlt
        "%u,%u,"            // [21] gpsSats, [22] gpsStale
        "%u,"               // [23] A_state
        "%u,%u,%u,%u,"      // [24-27] A_launched/apogee/separated/landed
        "%u,"               // [28] controllerBAlive
        "%.2f,%.2f,%.2f,%.2f," // [29-32] B_ax/ay/az, B_baro_alt
        "%u,"               // [33] B_state
        "%u,"               // [34] battLow
        "%d,%d,%u",         // [35-37] rssi, snr, packet_loss
        (unsigned long)pkt->missionTime,
        (unsigned)TEAM_ID,
        pkt->altitude,    pkt->maxAltitude,
        pkt->accelX,      pkt->accelY,   pkt->accelZ,  pkt->accelMag,
        pkt->gyroX,       pkt->gyroY,    pkt->gyroZ,
        pkt->temperature, pkt->pressure,
        pitch, roll, yaw,
        pkt->batteryVoltage,
        pkt->gpsLat,   pkt->gpsLon,   pkt->gpsAltitude,
        (unsigned)pkt->gpsSats, (unsigned)pkt->gpsStale,
        (unsigned)pkt->A_state,
        (unsigned)pkt->A_launched, (unsigned)pkt->A_apogee,
        (unsigned)pkt->A_separated,(unsigned)pkt->A_landed,
        (unsigned)pkt->controllerBAlive,
        pkt->B_ax,  pkt->B_ay,  pkt->B_az,  pkt->B_baro_alt,
        (unsigned)pkt->B_state,
        (unsigned)pkt->battLow,
        (int)lastRSSI, (int)lastSNR, (unsigned)lastPacketLoss
    );
    Serial.println(line);
}

// ─── CanSat packet handler ────────────────────────────────────
void handleCanSatPacket(uint8_t* buf, int size) {
    if ((size_t)size < sizeof(CanSatPacket)) return;

    CanSatPacket* pkt = (CanSatPacket*)buf;

    // CRC verification
    size_t   crcLen  = sizeof(CanSatPacket) - sizeof(uint16_t);
    uint16_t calcCRC = crc16(buf, crcLen);
    if (calcCRC != pkt->crc) {
        Serial.println("GS_WARN: CanSat CRC mismatch — packet dropped");
        return;
    }

    lastRSSI = (int16_t)LoRa.packetRssi();
    lastSNR  = LoRa.packetSnr();

    uint16_t pseudoId  = (uint16_t)(pkt->timestamp & 0xFFFF);
    lastPacketLoss = calcPacketLoss(pseudoId,
                                    canLastId, canRxCount,
                                    canExpected, canFirstPkt);

    if (!telemetryEnabled) return;

    // ── CANSAT CSV — 26 fields (protocol_spec §2.2) ────────────
    char line[512];
    snprintf(line, sizeof(line),
        "CANSAT,"
        "%u,%lu,%u,"        // [1] teamID, [2] timestamp, [3] state
        "%.2f,%.2f,%.2f,"   // [4-6] ax/ay/az
        "%.2f,%.2f,"        // [7] h_raw (displayed), [8] h_filtered (logged)
        "%.2f,%.2f,%.2f,"   // [9-11] gx/gy/gz
        "%.2f,%.2f,%.2f,"   // [12-14] magX/Y/Z (logged)
        "%.6f,%.6f,%.1f,%u," // [15-18] lat, lon, alt, satcnt
        "%.1f,%.1f,%.1f,"   // [19-21] COG, headingErr, distToTarget
        "%.2f,"             // [22] batteryVoltage (logged)
        "%d,%d,%u",         // [23-25] rssi, snr, packet_loss
        (unsigned)TEAM_ID,
        (unsigned long)pkt->timestamp,
        (unsigned)pkt->state,
        pkt->ax,  pkt->ay,  pkt->az,
        pkt->h_raw, pkt->h_filtered,
        pkt->gx,  pkt->gy,  pkt->gz,
        pkt->magX, pkt->magY, pkt->magZ,
        pkt->lat,  pkt->lon,  pkt->alt, (unsigned)pkt->satcnt,
        pkt->COG,  pkt->headingErr, pkt->distToTarget,
        pkt->batteryVoltage,
        (int)lastRSSI, (int)lastSNR, (unsigned)lastPacketLoss
    );
    Serial.println(line);
}

// ─── STATUS heartbeat (protocol_spec §2.3) ───────────────────
// Fields: Marker, BmpOK, BnoOK, SdOK, FlashOK, LoraOK,
//         GpsSearch, CtrlB, BattV, FlightNum, RSSI, SNR
// GS PCB has no BMP/BNO/SD/Flash sensors — always report 1 (not applicable).
void sendStatus() {
    char line[128];
    snprintf(line, sizeof(line),
        "STATUS,GS1,1,1,1,1,1,0,1,0.0,1,%d,%.1f",
        (int)lastRSSI, lastSNR
    );
    Serial.println(line);
}

// ─── Command relay over LoRa ──────────────────────────────────
void relayCommand(uint8_t type, uint16_t id,
                  uint8_t* payload, uint8_t payLen) {
    // Rebuild the exact binary frame the GUI sent
    uint8_t frame[5 + CMD_MAX_PAYLOAD];
    frame[0] = CMD_SYNC1;
    frame[1] = CMD_SYNC2;
    frame[2] = type;
    frame[3] = (uint8_t)(id & 0xFF);
    frame[4] = (uint8_t)(id >> 8);
    if (payLen > 0 && payload != nullptr) {
        memcpy(&frame[5], payload, payLen);
    }
    uint8_t totalLen = 5 + payLen;

    // Switch to the appropriate TX frequency
    bool toCanSat = isCanSatCommand(type);
    LoRa.idle();
    LoRa.setFrequency(toCanSat ? CANSAT_FREQ : AVIONICS_FREQ);

    LoRa.beginPacket();
    LoRa.write(frame, totalLen);
    LoRa.endPacket(false);   // blocking send

    // Restore listening channel
    LoRa.setFrequency(currentChannel == CH_CANSAT ? CANSAT_FREQ : AVIONICS_FREQ);
    LoRa.receive();

    // ── ACK back to GUI (protocol_spec §3) ─────────────────────
    char ack[32];
    snprintf(ack, sizeof(ack), "ACK id=%u", (unsigned)id);
    Serial.println(ack);
}

// ─── Command dispatcher ───────────────────────────────────────
void handleCommand(uint8_t type, uint16_t id,
                   uint8_t* payload, uint8_t payLen) {
    switch (type) {
        case CMD_START_TELEMETRY:
            telemetryEnabled = true;
            relayCommand(type, id, payload, payLen);
            break;

        case CMD_STOP_TELEMETRY:
            telemetryEnabled = false;
            relayCommand(type, id, payload, payLen);
            break;

        case CMD_REQUEST_STATUS:
            // Respond with a fresh STATUS right away, then relay
            sendStatus();
            relayCommand(type, id, payload, payLen);
            break;

        default:
            relayCommand(type, id, payload, payLen);
            break;
    }
}

// ─── Serial command parser (byte-at-a-time state machine) ────
void parseSerialByte(uint8_t b) {
    switch (cmdState) {
        case WAIT_SYNC1:
            if (b == CMD_SYNC1) cmdState = WAIT_SYNC2;
            break;

        case WAIT_SYNC2:
            cmdState = (b == CMD_SYNC2) ? WAIT_CMD : WAIT_SYNC1;
            break;

        case WAIT_CMD:
            cmdType       = b;
            cmdPayloadLen = 0;
            cmdPayloadIdx = 0;
            cmdState      = WAIT_ID_LSB;
            break;

        case WAIT_ID_LSB:
            cmdId    = b;
            cmdState = WAIT_ID_MSB;
            break;

        case WAIT_ID_MSB:
            cmdId |= ((uint16_t)b << 8);
            // Determine expected payload length from command type
            if      (cmdType == CMD_SET_TARGET_GPS)    cmdPayloadLen = 8;  // 2x float32
            else if (cmdType == CMD_SET_QNH)           cmdPayloadLen = 4;  // 1x float32
            else if (cmdType == CMD_SET_PD_CONSTANTS)  cmdPayloadLen = 8;  // 2x float32 stub
            else                                       cmdPayloadLen = 0;

            if (cmdPayloadLen == 0) {
                handleCommand(cmdType, cmdId, nullptr, 0);
                cmdState = WAIT_SYNC1;
            } else {
                cmdState = WAIT_PAYLOAD;
            }
            break;

        case WAIT_PAYLOAD:
            if (cmdPayloadIdx < CMD_MAX_PAYLOAD) {
                cmdPayload[cmdPayloadIdx++] = b;
            }
            if (cmdPayloadIdx >= cmdPayloadLen) {
                handleCommand(cmdType, cmdId, cmdPayload, cmdPayloadLen);
                cmdState = WAIT_SYNC1;
            }
            break;
    }
}

// ─── setup() ─────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    // Wait for USB serial to enumerate (max 3 s)
    while (!Serial && millis() < 3000);

    Serial.println("GS_INFO: AvioPro Ground Station v1.0 booting");

    // ── LoRa initialisation ───────────────────────────────────
    LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(AVIONICS_FREQ)) {
        Serial.println("GS_ERR: LoRa module not found — check wiring. Halting.");
        while (true);
    }

    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.enableCrc();        // LoRa hardware CRC (extra protection layer)

    LoRa.receive();          // continuous RX mode

    currentChannel = CH_AVIONICS;
    lastHopTime    = millis();
    lastStatusTime = millis();

    Serial.println("GS_INFO: LoRa OK");
    Serial.print("GS_INFO: Avionics channel = ");
    Serial.print(AVIONICS_FREQ / 1E6, 3);
    Serial.println(" MHz");
    Serial.print("GS_INFO: CanSat channel   = ");
    Serial.print(CANSAT_FREQ / 1E6, 3);
    Serial.println(" MHz");
    Serial.println("GS_INFO: Ready — listening");
}

// ─── loop() ──────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();

    // ── 1. Frequency hopping ─────────────────────────────────
    if (now - lastHopTime >= HOP_DWELL_MS) {
        lastHopTime = now;
        switchChannel(currentChannel == CH_AVIONICS ? CH_CANSAT : CH_AVIONICS);
    }

    // ── 2. LoRa receive check ────────────────────────────────
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
        static uint8_t rxBuf[256];
        int len = 0;
        while (LoRa.available() && len < (int)sizeof(rxBuf)) {
            rxBuf[len++] = (uint8_t)LoRa.read();
        }
        if (len > 0) {
            if (currentChannel == CH_AVIONICS) {
                handleAvionicsPacket(rxBuf, len);
            } else {
                handleCanSatPacket(rxBuf, len);
            }
        }
    }

    // ── 3. Serial command parser ─────────────────────────────
    while (Serial.available()) {
        parseSerialByte((uint8_t)Serial.read());
    }

    // ── 4. Periodic STATUS heartbeat ────────────────────────
    if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
        lastStatusTime = now;
        sendStatus();
    }
}
