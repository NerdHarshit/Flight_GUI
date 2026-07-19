// ================================================================
//  AVIOPRO V0 — GROUND STATION GUI SIMULATOR / TEST SCRIPT
//  Simulates a full flight profile and outputs CSV data via Serial.
//  GPS coordinates are kept fixed at exactly 18.99°N 72.81°E.
// ================================================================

#include <Arduino.h>

// Exact Fixed Location Requested
const float SIM_LAT = 18.990000f;
const float SIM_LON = 72.810000f;

// Simulation variables
uint32_t missionTime = 0;
uint8_t seq = 0;
uint8_t flightState = 2; // Starts at LAUNCH_PAD (2)
float currentAltitude = 0.0f;
float maxAltitude = 0.0f;
float accelMag = 1.0f; // 1G at rest
float pitch = 90.0f;   // Vertical on pad
float roll = 0.0f;
float yaw = 0.0f;
float batteryVoltage = 8.4f;

// Simulation timeline tracking
uint32_t lastUpdateTime = 0;
uint32_t stateStartTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000); 
    
    // Broadcast initial status packets to simulate boot sequence
    for(int i = 0; i < 2; i++) {
        // FORMAT: STATUS,Marker,BmpOK,BnoOK,SdOK,FlashOK,LoraOK,GpsSearch,CtrlB,BattV,FlightNum,RSSI,SNR
        Serial.print("STATUS,55,1,1,1,1,1,0,0,8.38,1,-45,10.5\n");
        delay(500);
    }
    
    stateStartTime = millis();
    lastUpdateTime = millis();
}

void loop() {
    uint32_t now = millis();
    
    // Run the simulation update at 10Hz (every 100ms) to match your flight loop rate
    if (now - lastUpdateTime >= 100) {
        lastUpdateTime = now;
        missionTime += 100;
        seq++;
        
        uint32_t timeInState = now - stateStartTime;
        
        // --- FLIGHT PROFILE STATE MACHINE SIMULATION ---
        switch (flightState) {
            case 2: // LAUNCH_PAD
                currentAltitude = random(-50, 50) / 100.0f; // Minor ground noise
                accelMag = 1.0f + (random(-5, 5) / 100.0f);
                pitch = 89.5f;
                if (timeInState > 5000) { // Stay on pad for 5 seconds
                    flightState = 3; // Transition to ASCENT
                    stateStartTime = now;
                }
                break;
                
            case 3: // ASCENT
                accelMag = 4.5f; // High Gs during boost phase
                currentAltitude += 12.5f; // Climbing quickly
                pitch = 85.0f - (currentAltitude * 0.02f); // Slight gravity turn simulation
                roll += 2.0f; // Spin stabilization simulation
                
                if (currentAltitude > maxAltitude) maxAltitude = currentAltitude;
                
                if (timeInState > 6000) { // Reach apogee at 6 seconds
                    flightState = 4; // PAYLOAD_SEP
                    stateStartTime = now;
                }
                break;
                
            case 4: // PAYLOAD_SEP (Momentary state)
                accelMag = 0.2f; // Ejection shock / momentary weightlessness
                flightState = 5; // Instantly move to DESCENT
                stateStartTime = now;
                break;
                
            case 5: // DESCENT
                accelMag = 1.0f; 
                currentAltitude -= 3.2f; // Falling under main parachute
                pitch = random(-20, 20); // Swaying/dangling under chute
                roll += 0.5f;
                
                if (currentAltitude <= 0.0f) {
                    currentAltitude = 0.0f;
                    flightState = 6; // IMPACT
                    stateStartTime = now;
                }
                break;
                
            case 6: // IMPACT
                accelMag = 1.0f;
                pitch = -75.0f; // Rocket laying flat on the ground
                roll = 45.0f;
                if (timeInState > 10000) { // Stay at impact for 10 seconds, then reset loop
                    flightState = 2; 
                    currentAltitude = 0.0f;
                    maxAltitude = 0.0f;
                    stateStartTime = now;
                    missionTime = 0;
                }
                break;
        }

        // Slowly drain battery over time
        batteryVoltage -= 0.0005f;
        if (batteryVoltage < 6.5f) batteryVoltage = 8.4f;

        // --- GENERATE AND PRINT TELEMETRY STRING ---
        // Format: TELEM,Seq,MissionTime,State,Alt,MaxAlt,AccelMag,Pitch,Roll,Yaw,Temp,Press,Batt,Lat,Lon,GpsAlt,Sats,Stale,Launch,Apogee,Sep,Landed,RSSI,SNR
        Serial.print("TELEM,");
        Serial.print(seq);                                      Serial.print(",");
        Serial.print(missionTime);                              Serial.print(",");
        Serial.print(flightState);                              Serial.print(",");
        Serial.print(currentAltitude, 2);                       Serial.print(",");
        Serial.print(maxAltitude, 2);                           Serial.print(",");
        Serial.print(accelMag, 3);                              Serial.print(",");
        Serial.print(pitch, 1);                                 Serial.print(",");
        Serial.print(roll, 1);                                  Serial.print(",");
        Serial.print(yaw, 1);                                   Serial.print(",");
        Serial.print(24.5f + (currentAltitude * -0.0065f), 1);  // Temperature drop simulation
        Serial.print(1013.25f - (currentAltitude * 0.12f), 1);  // Barometric pressure drop simulation
        Serial.print(batteryVoltage, 2);                        Serial.print(",");
        
        // Exact Fixed Location Requested: 18.99°N 72.81°E
        Serial.print(SIM_LAT, 6);                               Serial.print(","); 
        Serial.print(SIM_LON, 6);                               Serial.print(",");
        
        Serial.print(currentAltitude, 1);                       Serial.print(","); // GPS Alt tracks baro alt for test mapping
        Serial.print(8);                                        Serial.print(","); // 8 Satellites connected
        Serial.print(0);                                        Serial.print(","); // GPS not stale (0 = false)
        
        // Boolean Flight flags derived from flightState
        Serial.print(flightState >= 3 ? 1 : 0);                 Serial.print(","); // launched
        Serial.print(flightState >= 4 ? 1 : 0);                 Serial.print(","); // apogee
        Serial.print(flightState >= 5 ? 1 : 0);                 Serial.print(","); // separated
        Serial.print(flightState == 6 ? 1 : 0);                 Serial.print(","); // landed
        
        // Mock LoRa Signal Environment Metrics
        Serial.print(random(-65, -40));                         Serial.print(","); // RSSI
        Serial.println(10.5f, 1);                                                  // SNR
    }
}