// ================================================================
//  AVIOPRO V0 — GROUND STATION (Raspberry Pi Pico W)
//  Receives Telemetry and Status packets, verifies CRC, and
//  outputs CSV with RSSI & SNR for GUI parsing.
// ================================================================

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// ================================================================
//  PIN MAP (Raspberry Pi Pico W)
//  Using standard SPI0. Adjust pins to match your GS wiring.
// ================================================================
#define LORA_MISO_PIN   16
#define LORA_CS_PIN     17
#define LORA_SCK_PIN    18
#define LORA_MOSI_PIN   19
#define LORA_IRQ_PIN    20
#define LORA_RST_PIN    21

// ================================================================
//  LORA TUNING CONSTANTS (Matched to Avionics)
// ================================================================
#define LORA_FREQUENCY         433E6
#define LORA_SPREADING_FACTOR  7
#define LORA_BANDWIDTH         500E3
#define LORA_CODING_RATE       5
#define LORA_SYNC_WORD         0xAF
#define LORA_PKT_SYNC_0        0xA5
#define LORA_PKT_SYNC_1        0x5A

// ================================================================
//  STRUCTS (Matched to Avionics)
// ================================================================
struct TelemetryPacket {
    uint32_t missionTime;
    float    altitude;
    float    maxAltitude;
    float    accelX;
    float    accelY;
    float    accelZ;
    float    accelMag;
    float    gyroX;
    float    gyroY;
    float    gyroZ;
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
    bool     gpsStale;
    uint8_t  state;
    bool     launched;
    bool     apogee;
    bool     separated;
    bool     landed;
    bool     controllerBAlive;
    bool     battLow;
    uint16_t crc;
};

struct StatusPacket {
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
    uint16_t crc;
};

// ================================================================
//  UTILITIES
// ================================================================
// CRC-16/CCITT-FALSE — polynomial 0x1021, init 0xFFFF.
uint16_t calculateCRC(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000); 

    Serial.println("GS_INIT,Starting Ground Station...");

    SPI.setRX(LORA_MISO_PIN);
    SPI.setTX(LORA_MOSI_PIN);
    SPI.setSCK(LORA_SCK_PIN);
    SPI.begin();

    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_IRQ_PIN);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("GS_ERROR,LoRa initialization failed!");
        while (1) { delay(1000); }
    }

    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();

    Serial.println("GS_READY,Listening for telemetry...");
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
        
        if (packetSize < 2) return; 

        uint8_t sync0 = LoRa.read();
        uint8_t sync1 = LoRa.read();
        
        if (sync0 != LORA_PKT_SYNC_0 || sync1 != LORA_PKT_SYNC_1) {
            Serial.println("GS_WARN,Invalid Sync Bytes");
            return;
        }

        int payloadSize = packetSize - 2; 
        uint8_t payload[128];
        int bytesRead = 0;
        
        while (LoRa.available() && bytesRead < sizeof(payload)) {
            payload[bytesRead++] = LoRa.read();
        }

        int rssi = LoRa.packetRssi();
        float snr = LoRa.packetSnr();

        // ----------------------------------------------------------------
        // IDENTIFY PACKET BY LENGTH
        // Telemetry includes a sequence byte (1) + struct size (67) = 68 bytes
        // Status lacks sequence byte (struct size = 14) = 14 bytes
        // ----------------------------------------------------------------
        
        if (payloadSize == (sizeof(TelemetryPacket) + 1)) {
            uint8_t seq = payload[0];
            TelemetryPacket t;
            memcpy(&t, &payload[1], sizeof(TelemetryPacket));

            uint16_t expectedCRC = calculateCRC((const uint8_t*)&t, sizeof(TelemetryPacket) - sizeof(uint16_t));
            
            if (t.crc == expectedCRC) {
                // OUTPUT FORMAT: TELEM,Seq,MissionTime,State,Alt,MaxAlt,AccelMag,Pitch,Roll,Yaw,Temp,Press,Batt,Lat,Lon,GpsAlt,Sats,Stale,Launch,Apogee,Sep,Landed,RSSI,SNR
                Serial.print("TELEM,");
                Serial.print(seq);                   Serial.print(",");
                Serial.print(t.missionTime);         Serial.print(",");
                Serial.print(t.state);               Serial.print(",");
                Serial.print(t.altitude, 2);         Serial.print(",");
                Serial.print(t.maxAltitude, 2);      Serial.print(",");
                Serial.print(t.accelMag, 3);         Serial.print(",");
                Serial.print(t.pitch, 1);            Serial.print(",");
                Serial.print(t.roll, 1);             Serial.print(",");
                Serial.print(t.yaw, 1);              Serial.print(",");
                Serial.print(t.temperature, 1);      Serial.print(",");
                Serial.print(t.pressure, 1);         Serial.print(",");
                Serial.print(t.batteryVoltage, 2);   Serial.print(",");
                Serial.print(t.gpsLat, 6);           Serial.print(",");
                Serial.print(t.gpsLon, 6);           Serial.print(",");
                Serial.print(t.gpsAltitude, 1);      Serial.print(",");
                Serial.print(t.gpsSats);             Serial.print(",");
                Serial.print(t.gpsStale);            Serial.print(",");
                Serial.print(t.launched);            Serial.print(",");
                Serial.print(t.apogee);              Serial.print(",");
                Serial.print(t.separated);           Serial.print(",");
                Serial.print(t.landed);              Serial.print(",");
                Serial.print(rssi);                  Serial.print(",");
                Serial.println(snr, 1);
            } else {
                Serial.println("GS_WARN,Telemetry CRC Failure");
            }
        } 
        else if (payloadSize == sizeof(StatusPacket)) {
            StatusPacket s;
            memcpy(&s, payload, sizeof(StatusPacket));

            uint16_t expectedCRC = calculateCRC((const uint8_t*)&s, sizeof(StatusPacket) - sizeof(uint16_t));

            if (s.crc == expectedCRC) {
                // OUTPUT FORMAT: STATUS,Marker,BmpOK,BnoOK,SdOK,FlashOK,LoraOK,GpsSearch,CtrlB,BattV,FlightNum,RSSI,SNR
                Serial.print("STATUS,");
                Serial.print(s.marker, HEX);   Serial.print(",");
                Serial.print(s.bmpOK);         Serial.print(",");
                Serial.print(s.bnoOK);         Serial.print(",");
                Serial.print(s.sdOK);          Serial.print(",");
                Serial.print(s.flashOK);       Serial.print(",");
                Serial.print(s.loraOK);        Serial.print(",");
                Serial.print(s.gpsSearching);  Serial.print(",");
                Serial.print(s.ctrlBAlive);    Serial.print(",");
                Serial.print(s.battV, 2);      Serial.print(",");
                Serial.print(s.flightNumber);  Serial.print(",");
                Serial.print(rssi);            Serial.print(",");
                Serial.println(snr, 1);
            } else {
                Serial.println("GS_WARN,Status CRC Failure");
            }
        } else {
            Serial.print("GS_WARN,Unknown Payload Size: ");
            Serial.println(payloadSize);
        }
    }
    
    // Commands to send Uplink via GUI can be processed here using Serial.available() 
}