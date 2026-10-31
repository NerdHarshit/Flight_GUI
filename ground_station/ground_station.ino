/**
 * ============================================================
 *  AvioPro Ground Station Firmware (Full Command Tester) v2.3
 *  Hardware : Raspberry Pi Pico + RFM95 (SX1276) LoRa module
 * ============================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <math.h>
#include <stddef.h>

// ─── Pin Mapping (RP2040 / Pico) ────────────────────────────
#define LORA_NSS   17   // SPI chip-select (GP17)
#define LORA_RST   20   // Reset          (GP20)
#define LORA_DIO0  22   // IRQ / DIO0     (GP22)

// ─── Mission Configuration ───────────────────────────────────
#define TEAM_ID             18

#define AVIONICS_FREQ       866E6   // 866 MHz (Controller A)
#define CANSAT_FREQ         867.75E6   // 868 MHz (CanSat)

#define AVIONICS_SF         8       // Spreading Factor for Controller A
#define CANSAT_SF           8       // Spreading Factor for CanSat

#define AVIONICS_SYNC_WORD  0x92    // Sync Word for Controller A
#define CANSAT_SYNC_WORD    0x92    // Sync Word for CanSat

#define LORA_BW             125E3   // 125 kHz Bandwidth
#define LORA_TX_POWER       20      // dBm
#define LORA_CODING_RATE    5       // 4/5

// 1200ms Dwell to ensure 257ms packets can complete without being cut off
#define HOP_DWELL_MS        1200    
#define STATUS_INTERVAL_MS  2000    

// Uplink reliability: A is deaf ~257 ms of every 500 ms while it transmits
// telemetry, and a command frame is ~72 ms on air. One shot only lands ~1/3 of
// the time, so each command is sent several times with the SAME seq number
// (A drops duplicates by seq, so repeats are harmless).
#define CMD_REPEATS_AVIONICS  5
#define CMD_REPEATS_CANSAT    3
#define CMD_REPEAT_GAP_MS     80    // + ~72 ms airtime => ~150 ms between starts

// ─── Protocol Sync Bytes & Packet Identifiers ────────────────
#define LORA_PKT_SYNC_0     0xA5
#define LORA_PKT_SYNC_1     0x5A
#define PKT_STATUS          0xA1
#define PKT_TELEMETRY       0xA2

#define CMD_SYNC1           0xAA
#define CMD_SYNC2           0x55
#define CMD_MAX_PAYLOAD     32

// ─── Telemetry Wire Flags ────────────────────────────────────
#define TLM_FLAG_LAUNCHED     (1u << 0)
#define TLM_FLAG_APOGEE       (1u << 1)
#define TLM_FLAG_SEPARATED    (1u << 2)
#define TLM_FLAG_LANDED       (1u << 3)
#define TLM_FLAG_CTRLB_ALIVE  (1u << 4)
#define TLM_FLAG_BATT_LOW     (1u << 5)
#define TLM_FLAG_GPS_STALE    (1u << 6)

// ─── Command Codes Matched to Flight Hardware ────────────────
#define CMD_START_TX          0x02
#define CMD_STOP_TX           0x03
#define CMD_SET_ZERO          0x04
#define CMD_BEEP              0x05
#define CMD_ERASE_FLASH       0x09
#define CMD_GET_STATUS        0x0A
#define CMD_SET_SEALEVEL_HPA  0x0B
#define CMD_SERVO_PARACHUTE_OPEN   0x0C   // parachute latch -> open (stays open)
#define CMD_SERVO_PAYLOAD_OPEN     0x0D   // CanSat latch    -> open (stays open)
#define CMD_SERVO_PARACHUTE_CLOSE  0x0E   // parachute latch -> closed (pad / after landing only)
#define CMD_SERVO_PAYLOAD_CLOSE    0x0F   // CanSat latch    -> closed (pad / after landing only)
#define CMD_RESET_MCU         0x10
#define CMD_SET_TARGET        0x11
#define CMD_SET_BENCH_MODE    0x12   // payload: 1 byte, 1 = bench ON, 0 = bench OFF (flight)

enum Channel { CH_AVIONICS, CH_CANSAT };

enum CmdParseState {
    WAIT_SYNC1,
    WAIT_SYNC2,
    WAIT_CMD,
    WAIT_ID_LSB,
    WAIT_ID_MSB,
    WAIT_PAYLOAD
};

// ─── Avionics On-Air Packet Layout (78 bytes) ────────────────
struct __attribute__((packed)) TelemetryWireAvionics {
    uint32_t missionTime;
    float    altitude;
    float    maxAltitude;
    float    accelX;
    float    accelY;
    float    accelZ;
    float    accelMag;
    float    temperature;
    float    pressure;
    float    pitch;
    float    roll;
    float    yaw;
    float    batteryVoltage;
    float    gpsLat;
    float    gpsLon;
    float    gpsAltitude;
    uint8_t  gpsSats;
    uint8_t  state;
    uint8_t  flags;
    uint8_t  bState;
    float    bAltitude;
    float    bAccelMag;
    uint16_t crc;
};

// ─── CanSat On-Air Packet Layout (99 bytes) ──────────────────
struct __attribute__((packed)) TelemetryWireCanSat {
    uint32_t missionTime;
    float    altitude;
    float    maxAltitude;
    float    accelX;
    float    accelY;
    float    accelZ;
    float    accelMag;
    float    temperature;
    float    pressure;
    float    pitch;
    float    roll;
    float    yaw;
    float    batteryVoltage;
    float    gpsLat;
    float    gpsLon;
    float    gpsAltitude;
    uint8_t  gpsSats;
    uint8_t  state;
    uint8_t  flags;
    uint8_t  bState;
    float    bAltitude;
    float    bAccelMag;
    float    gpsCourse;
    float    gpsSpeed;
    float    headingError;
    float    guidanceU;
    float    targetDistance;
    uint8_t  guideFlags;
    uint16_t crc;
};

// ─── State Variables ─────────────────────────────────────────
Channel    currentChannel    = CH_AVIONICS;
uint32_t   lastHopTime       = 0;
uint32_t   lastStatusTime    = 0;
bool       telemetryEnabled  = true;
uint8_t    gsCmdSeq          = 0;

int16_t    lastRSSI          = 0;
float      lastSNR           = 0.0f;

CmdParseState cmdState       = WAIT_SYNC1;
uint8_t       cmdType        = 0;
uint16_t      cmdId          = 0;
uint8_t       cmdPayload[CMD_MAX_PAYLOAD];
uint8_t       cmdPayloadLen  = 0;
uint8_t       cmdPayloadIdx  = 0;

// ─── Forward Declarations ────────────────────────────────────
void switchChannel(Channel ch);
void handleCommand(uint8_t type, uint16_t id, uint8_t* payload, uint8_t payLen);

// ─── CRC-16/CCITT-FALSE ──────────────────────────────────────
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

// ─── Dynamic Channel Switching ───────────────────────────────
void switchChannel(Channel ch) {
    LoRa.idle();
    if (ch == CH_AVIONICS) {
        LoRa.setFrequency(AVIONICS_FREQ);
        LoRa.setSpreadingFactor(AVIONICS_SF);
        LoRa.setSyncWord(AVIONICS_SYNC_WORD);
    } else {
        LoRa.setFrequency(CANSAT_FREQ);
        LoRa.setSpreadingFactor(CANSAT_SF);
        LoRa.setSyncWord(CANSAT_SYNC_WORD);
    }
    currentChannel = ch;
    LoRa.receive();
}

// ─── Avionics Status Packet (PKT_STATUS) ─────────────────────
// Must match Controller A's StatusPacket exactly (NOT packed on A, so the
// natural ARM layout with padding is what goes on air: 16 bytes).
// Frame: [0xA5][0x5A][0xA1][ StatusPacket ]  (no seq byte, 19 bytes total)
struct AvionicsStatus {
    uint8_t  marker;
    bool     bmpOK;
    bool     bnoOK;
    bool     sdOK;
    bool     flashOK;
    bool     loraOK;
    bool     gpsSearching;
    bool     ctrlBAlive;
    float    battV;
    uint8_t  flightNumber;
    bool     benchMode;          // fills the old padding byte (crc stays at offset 14)
    uint16_t crc;
};
static_assert(sizeof(AvionicsStatus) == 16, "AvionicsStatus must match Controller A's StatusPacket (16 bytes)");
static_assert(offsetof(AvionicsStatus, crc) == 14, "AvionicsStatus crc must be at offset 14");

void handleAvionicsStatus(uint8_t* buf, int size) {
    if (size < 3 + (int)sizeof(AvionicsStatus)) return;

    AvionicsStatus st;
    memcpy(&st, buf + 3, sizeof(st));   // memcpy: buf+3 is not 4-byte aligned

    uint16_t calcCRC = crc16((const uint8_t*)&st, offsetof(AvionicsStatus, crc));
    if (calcCRC != st.crc || st.marker != 0x55) {
        Serial.println("GS_WARN: Avionics STATUS CRC mismatch — packet dropped");
        return;
    }

    lastRSSI = (int16_t)LoRa.packetRssi();
    lastSNR  = LoRa.packetSnr();

    // Same field order as the GS heartbeat line the GUI already parses, but
    // with Controller A's REAL values instead of hardcoded ones.
    char line[128];
    snprintf(line, sizeof(line),
        "STATUS,A,%u,%u,%u,%u,%u,%u,%u,%.2f,%u,%d,%.1f",
        (unsigned)st.bmpOK, (unsigned)st.bnoOK, (unsigned)st.sdOK,
        (unsigned)st.flashOK, (unsigned)st.loraOK,
        (unsigned)st.gpsSearching, (unsigned)st.ctrlBAlive,
        st.battV, (unsigned)st.flightNumber,
        (int)lastRSSI, lastSNR);
    Serial.println(line);

    // Separate line so the existing STATUS,... field layout the GUI parses is unchanged.
    Serial.print("BENCH,");
    Serial.println((unsigned)st.benchMode);
}

// ─── Avionics Packet Handler ──────────────────────────────────
void handleAvionicsPacket(uint8_t* buf, int size) {
    if (size < 3) return;
    if (buf[0] != LORA_PKT_SYNC_0 || buf[1] != LORA_PKT_SYNC_1) return;

    // FIX: status frames were silently discarded (only telemetry was handled).
    if (buf[2] == PKT_STATUS) { handleAvionicsStatus(buf, size); return; }

    if (buf[2] != PKT_TELEMETRY) return;
    if (size < 4 + (int)sizeof(TelemetryWireAvionics)) return;

    TelemetryWireAvionics* pkt = (TelemetryWireAvionics*)(buf + 4);

    size_t crcLen = offsetof(TelemetryWireAvionics, crc);
    uint16_t calcCRC = crc16(buf + 4, crcLen);
    if (calcCRC != pkt->crc) {
        Serial.println("GS_WARN: Avionics CRC mismatch — packet dropped");
        return;
    }

    lastRSSI = (int16_t)LoRa.packetRssi();
    lastSNR  = LoRa.packetSnr();

    if (!telemetryEnabled) return;

    bool launched  = (pkt->flags & TLM_FLAG_LAUNCHED) != 0;
    bool apogee    = (pkt->flags & TLM_FLAG_APOGEE) != 0;
    bool separated = (pkt->flags & TLM_FLAG_SEPARATED) != 0;
    bool landed    = (pkt->flags & TLM_FLAG_LANDED) != 0;
    bool ctrlB     = (pkt->flags & TLM_FLAG_CTRLB_ALIVE) != 0;
    bool battLow   = (pkt->flags & TLM_FLAG_BATT_LOW) != 0;
    bool gpsStale  = (pkt->flags & TLM_FLAG_GPS_STALE) != 0;

    char line[512];
    snprintf(line, sizeof(line),
        "CAB,%lu,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,0.00,0.00,0.00,%.1f,%.2f,%.1f,%.1f,%.1f,%.2f,%.6f,%.6f,%.1f,%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%u,%u,%d,%d,0",
        (unsigned long)pkt->missionTime, (unsigned)TEAM_ID,
        pkt->altitude, pkt->maxAltitude,
        pkt->accelX, pkt->accelY, pkt->accelZ, pkt->accelMag,
        pkt->temperature, pkt->pressure,
        pkt->pitch, pkt->roll, pkt->yaw,
        pkt->batteryVoltage,
        pkt->gpsLat, pkt->gpsLon, pkt->gpsAltitude,
        (unsigned)pkt->gpsSats, (unsigned)gpsStale,
        (unsigned)pkt->state,
        (unsigned)launched, (unsigned)apogee, (unsigned)separated, (unsigned)landed,
        (unsigned)ctrlB,
        0.0f, 0.0f, 0.0f, pkt->bAltitude,
        (unsigned)pkt->bState, (unsigned)battLow,
        (int)lastRSSI, (int)lastSNR
    );
    Serial.println(line);
}

// ─── CanSat Packet Handler ────────────────────────────────────
void handleCanSatPacket(uint8_t* buf, int size) {
    if (size < 4 + (int)sizeof(TelemetryWireCanSat)) return;

    if (buf[0] != LORA_PKT_SYNC_0 || buf[1] != LORA_PKT_SYNC_1) return;
    if (buf[2] != PKT_TELEMETRY) return;

    TelemetryWireCanSat* pkt = (TelemetryWireCanSat*)(buf + 4);

    size_t crcLen = offsetof(TelemetryWireCanSat, crc);
    uint16_t calcCRC = crc16(buf + 4, crcLen);
    if (calcCRC != pkt->crc) {
        Serial.println("GS_WARN: CanSat CRC mismatch — packet dropped");
        return;
    }

    lastRSSI = (int16_t)LoRa.packetRssi();
    lastSNR  = LoRa.packetSnr();

    if (!telemetryEnabled) return;

    char line[512];
    snprintf(line, sizeof(line),
        "CANSAT,%u,%lu,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,0.00,0.00,0.00,%.6f,%.6f,%.1f,%u,%.1f,%.1f,%.1f,%.2f,%d,%d,0",
        (unsigned)TEAM_ID, (unsigned long)pkt->missionTime, (unsigned)pkt->state,
        pkt->accelX, pkt->accelY, pkt->accelZ,
        pkt->altitude, pkt->altitude,
        pkt->pitch, pkt->roll, pkt->yaw,
        pkt->gpsLat, pkt->gpsLon, pkt->gpsAltitude, (unsigned)pkt->gpsSats,
        pkt->gpsCourse, pkt->headingError, pkt->targetDistance,
        pkt->batteryVoltage,
        (int)lastRSSI, (int)lastSNR
    );
    Serial.println(line);
}

// ─── STATUS Heartbeat ─────────────────────────────────────────
void sendStatus() {
    char line[128];
    snprintf(line, sizeof(line),
        "STATUS,GS1,1,1,1,1,1,0,1,0.0,1,%d,%.1f",
        (int)lastRSSI, lastSNR
    );
    Serial.println(line);
}

// ─── Command Relay Over LoRa ──────────────────────────────────
// Sends one frame on the given channel `repeats` times. endPacket() with the
// default argument is the BLOCKING send in this library, so each copy has fully
// left the antenna before the next one (or the retune to the CanSat channel).
static void sendFrameOnChannel(uint32_t freq, uint8_t sf, uint8_t sync,
                               const uint8_t* frame, uint8_t len, uint8_t repeats) {
    LoRa.idle();
    LoRa.setFrequency(freq);
    LoRa.setSpreadingFactor(sf);
    LoRa.setSyncWord(sync);

    for (uint8_t r = 0; r < repeats; r++) {
        if (LoRa.beginPacket()) {
            LoRa.write(frame, len);
            LoRa.endPacket();            // blocking: returns when TX is done
        }
        if (r + 1 < repeats) delay(CMD_REPEAT_GAP_MS);
    }
}

void relayCommand(uint8_t type, uint16_t id, uint8_t* payload, uint8_t payLen) {
    if (payLen > CMD_MAX_PAYLOAD) payLen = CMD_MAX_PAYLOAD;

    uint8_t frame[64];
    frame[0] = LORA_PKT_SYNC_0; // 0xA5
    frame[1] = LORA_PKT_SYNC_1; // 0x5A
    frame[2] = type;
    frame[3] = gsCmdSeq++;      // same seq for every repeat => A de-duplicates
    frame[4] = payLen;

    if (payLen > 0 && payload != nullptr) {
        memcpy(&frame[5], payload, payLen);
    }

    uint8_t hdrPlusPayLen = 5 + payLen;
    uint16_t cmdCRC = crc16(frame, hdrPlusPayLen);

    frame[hdrPlusPayLen]     = (uint8_t)(cmdCRC >> 8);
    frame[hdrPlusPayLen + 1] = (uint8_t)(cmdCRC & 0xFF);
    uint8_t totalLen = hdrPlusPayLen + 2;

    // 1. Controller A (866 MHz)
    sendFrameOnChannel(AVIONICS_FREQ, AVIONICS_SF, AVIONICS_SYNC_WORD,
                       frame, totalLen, CMD_REPEATS_AVIONICS);

    delay(CMD_REPEAT_GAP_MS);

    // 2. CanSat (868 MHz)
    sendFrameOnChannel(CANSAT_FREQ, CANSAT_SF, CANSAT_SYNC_WORD,
                       frame, totalLen, CMD_REPEATS_CANSAT);

    // Restore listening channel and restart the hop timer so we don't
    // immediately hop away after this (long, blocking) transmit sequence.
    switchChannel(currentChannel);
    lastHopTime = millis();

    // NOTE: this only means "sent", not "A received it". A has no ACK packet yet.
    char ack[32];
    snprintf(ack, sizeof(ack), "ACK id=%u", (unsigned)id);
    Serial.println(ack);
}

void handleCommand(uint8_t type, uint16_t id, uint8_t* payload, uint8_t payLen) {
    // FIX: START/STOP used to flip telemetryEnabled locally, which muted the GS
    // output on STOP whether or not A actually heard the command (false
    // "STOP works"). The GS now always shows what A is really sending, so
    // telemetry only stops when A really stops. telemetryEnabled stays true.

    if (type == CMD_GET_STATUS) sendStatus();   // GS's own heartbeat line;
                                                // A's real status arrives as a
                                                // STATUS,A,... line after the relay
    relayCommand(type, id, payload, payLen);
}

// ─── Serial Command Parser State Machine ──────────────────────
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
            if      (cmdType == CMD_SET_SEALEVEL_HPA) cmdPayloadLen = 4;
            else if (cmdType == CMD_SET_TARGET)       cmdPayloadLen = 8;
            else if (cmdType == CMD_SET_BENCH_MODE)   cmdPayloadLen = 1;
            else                                      cmdPayloadLen = 0;

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

// ─── Setup ───────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("GS_INFO: AvioPro Ground Station v2.3 (Full Command Tester)");
    Serial.println("GS_INFO: Commands -> s:Start, t:Stop, z:Zero, b:Beep, p:Parachute OPEN, k:Parachute CLOSE, c:CanSat OPEN, l:CanSat CLOSE, m:Bench ON, n:Bench OFF, g:Status, r:Reset, q:Set QNH, x:Set Target");

    LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(AVIONICS_FREQ)) {
        Serial.println("GS_ERR: LoRa module not found — check wiring. Halting.");
        while (true);
    }

    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.enableCrc();

    switchChannel(CH_AVIONICS);

    lastHopTime    = millis();
    lastStatusTime = millis();
}

// ─── Main Loop ───────────────────────────────────────────────
void loop() {
    uint32_t now = millis();

    // 1. Channel Hopping (1200ms Dwell)
    if (now - lastHopTime >= HOP_DWELL_MS) {
        lastHopTime = now;
        switchChannel(currentChannel == CH_AVIONICS ? CH_CANSAT : CH_AVIONICS);
    }

    // 2. Telemetry Reception
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

    // 3. Serial Command Handling
    while (Serial.available()) {
        uint8_t b = Serial.peek();
        
        // If the byte is part of a GUI binary frame, pass to the parser
        if (b == CMD_SYNC1 || cmdState != WAIT_SYNC1) {
            parseSerialByte((uint8_t)Serial.read());
            continue;
        }
        
        // Otherwise, process ASCII keyboard shortcuts
        b = Serial.read();
        switch (b) {
            case 's': case 'S':
                Serial.println("GS_INFO: [0x02] CMD_START_TX");
                handleCommand(CMD_START_TX, 1, nullptr, 0);
                break;
            case 't': case 'T':
                Serial.println("GS_INFO: [0x03] CMD_STOP_TX");
                handleCommand(CMD_STOP_TX, 1, nullptr, 0);
                break;
            case 'z': case 'Z':
                Serial.println("GS_INFO: [0x04] CMD_SET_ZERO");
                handleCommand(CMD_SET_ZERO, 1, nullptr, 0);
                break;
            case 'b': case 'B':
                Serial.println("GS_INFO: [0x05] CMD_BEEP");
                handleCommand(CMD_BEEP, 1, nullptr, 0);
                break;
            case 'e': case 'E':
                Serial.println("GS_INFO: [0x09] CMD_ERASE_FLASH");
                handleCommand(CMD_ERASE_FLASH, 1, nullptr, 0);
                break;
            case 'g': case 'G':
                Serial.println("GS_INFO: [0x0A] CMD_GET_STATUS");
                handleCommand(CMD_GET_STATUS, 1, nullptr, 0);
                break;
            case 'q': case 'Q': {
                Serial.println("GS_INFO: [0x0B] CMD_SET_SEALEVEL_HPA (Test Payload: 1013.25 hPa)");
                float testQNH = 1013.25f;
                handleCommand(CMD_SET_SEALEVEL_HPA, 1, (uint8_t*)&testQNH, sizeof(float));
                break;
            }
            case 'p': case 'P':
                Serial.println("GS_INFO: [0x0C] CMD_SERVO_PARACHUTE_OPEN");
                handleCommand(CMD_SERVO_PARACHUTE_OPEN, 1, nullptr, 0);
                break;
            case 'k': case 'K':
                Serial.println("GS_INFO: [0x0E] CMD_SERVO_PARACHUTE_CLOSE");
                handleCommand(CMD_SERVO_PARACHUTE_CLOSE, 1, nullptr, 0);
                break;
            case 'c': case 'C':
                Serial.println("GS_INFO: [0x0D] CMD_SERVO_PAYLOAD_OPEN");
                handleCommand(CMD_SERVO_PAYLOAD_OPEN, 1, nullptr, 0);
                break;
            case 'l': case 'L':
                Serial.println("GS_INFO: [0x0F] CMD_SERVO_PAYLOAD_CLOSE");
                handleCommand(CMD_SERVO_PAYLOAD_CLOSE, 1, nullptr, 0);
                break;
            case 'm': case 'M': {
                Serial.println("GS_INFO: [0x12] CMD_SET_BENCH_MODE = 1 (ON, pad/landed only)");
                uint8_t on = 1;
                handleCommand(CMD_SET_BENCH_MODE, 1, &on, 1);
                break;
            }
            case 'n': case 'N': {
                Serial.println("GS_INFO: [0x12] CMD_SET_BENCH_MODE = 0 (OFF, flight behaviour)");
                uint8_t off = 0;
                handleCommand(CMD_SET_BENCH_MODE, 1, &off, 1);
                break;
            }
            case 'r': case 'R':
                Serial.println("GS_INFO: [0x10] CMD_RESET_MCU");
                handleCommand(CMD_RESET_MCU, 1, nullptr, 0);
                break;
            case 'x': case 'X': {
                Serial.println("GS_INFO: [0x11] CMD_SET_TARGET (Test Payload: 19.0, 72.0)");
                int32_t latE7 = 190000000;
                int32_t lonE7 = 720000000;
                uint8_t targetPayload[8];
                memcpy(targetPayload, &latE7, 4);
                memcpy(targetPayload + 4, &lonE7, 4);
                handleCommand(CMD_SET_TARGET, 1, targetPayload, 8);
                break;

            case 'o': case 'O':
                Serial.println("GS_INFO: [0x12] CMD_GYRO_ON (CanSat Only)");
                handleCommand(CMD_GYRO_ON, 1, nullptr, 0);
                break;
            case 'f': case 'F':
                Serial.println("GS_INFO: [0x13] CMD_GYRO_OFF (CanSat Only)");
                handleCommand(CMD_GYRO_OFF, 1, nullptr, 0);
                break;
            case 'w': case 'W':
                Serial.println("GS_INFO: [0x14] CMD_BURN_WIRE (CanSat Only)");
                handleCommand(CMD_BURN_WIRE, 1, nullptr, 0);
                break;
                
            }
            default:
                break;
        }
    }

    // 4. Periodic Ground Station Heartbeat
    if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
        lastStatusTime = now;
        sendStatus();
    }
}