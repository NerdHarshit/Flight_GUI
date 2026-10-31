// ================================================================
//  AVIOPRO V0 — CONTROLLER A — SINGLE-CONTROLLER TEST LAUNCH BUILD
//
//  Merged from:
//    - Test_launch_singleController.ino  (debugged SdFat + raw-SPI
//      flash driver + MPU6500_WE + remapped Wire1/SPI1/SPI0 pins)
//    - controllerA_v1.ino                (full flight logic: state
//      machine, CRC, LoRa commands, GPS, deployment, self-check)
//
//  GPS is not physically connected yet but will be wired before
//  launch — GPS init/drain/validate are live so it "just works"
//  once the module is attached; no code changes needed later.
//
//  New actuator pins for this board revision (not present in the
//  debugged file — wired in per hardware confirmation):
//    BUZZER_PIN      = GPIO27
//    SERVO_SEP_PIN   = GPIO19  (CanSat bay door latch servo)
//    PARACHUTE_PIN   = GPIO21  (parachute bay door latch servo — see below)
//
//  ----------------------------------------------------------------
//  COMPETITION-READINESS PASS (this revision) — summary of changes:
//    - PARACHUTE_PIN reassigned from a MOSFET pyro gate to a second
//      door-latch SERVO (ASSUMPTION — flag if hardware disagrees).
//    - Startup: per-component OK/FAIL beep on each init step; wait
//      for Controller B is now bounded to 30s (was infinite) and
//      plays a distinct timeout tone, then continues single-controller.
//    - Ends with a GO (3 quick beeps) / NO-GO (1 long beep) verdict.
//    - Nothing transmits over LoRa until CMD_START_TX is received.
//      The very first START_TX after boot sends the sensor-health
//      packet first, then telemetry; a later stop/start cycle goes
//      straight to telemetry. CMD_GET_STATUS re-sends live health on
//      demand at any time, independent of TX state.
//    - Uplink command frame is now variable-length (header+payload+
//      CRC) instead of fixed 6 bytes, to carry command arguments.
//    - Downlink frames now carry an explicit PKT_STATUS/PKT_TELEMETRY
//      type byte so the ground station can tell them apart regardless
//      of when they arrive.
//    - New commands: CMD_GET_STATUS, CMD_SET_SEALEVEL_HPA (float
//      payload, falls back to the built-in constant if bad/missing),
//      CMD_SERVO_*_OPEN / CMD_SERVO_*_CLOSE (see below),
//      CMD_RESET_MCU (honored ONLY from LAUNCH_PAD, GUI confirms first).
//    - int16 telemetry compaction was explicitly asked to be held off
//      until radio-range testing decides if it's needed — NOT done here.
//
//  SECOND PASS:
//    - ARM removed entirely (CMD_ARM, `armed`, the launch-detection
//      gate, and the armed checks on deployment). Launch detection is
//      live from LAUNCH_PAD as soon as the board is up.
//    - TEST_MODE state and CMD_TEST_MODE / CMD_SIM_LAUNCH /
//      CMD_SIM_APOGEE removed. Retired IDs/state numbers are left as
//      gaps so every remaining value is unchanged.
//    - Servo commands: individual OPEN and CLOSE per door (0x0C/0x0D open,
//      0x0E/0x0F close). No automatic 0->90->0 cycle any more. Manual
//      commands never move both servos within SERVO_STAGGER_MS of each
//      other; manual CLOSE is refused in flight (only LAUNCH_PAD / IMPACT).
//    - CMD_SET_BENCH_MODE (0x12): runtime bench-mode flag, see above.
//    - On-air telemetry is now the packed 78-byte TelemetryWire: gyro
//      dropped (ground derives rates from pitch/roll/yaw), 7 flags in a
//      bitmask, and B's state / baro altitude / accel magnitude added.
//  ----------------------------------------------------------------
// ================================================================

#include <Arduino.h>
#include <Wire.h>
#include <stddef.h>                // offsetof() — see CRC bugfix notes below
#include <string.h>                // memcpy() — command payload parsing
#include <SPI.h>
#include <Servo.h>
#include "SdFat.h"                // SD logging
#include <Adafruit_BMP3XX.h>
#include <MPU6500_WE.h>
#include <TinyGPSPlus.h>
#include <stddef.h>                // offsetof() for the StatusPacket layout check
#include <EEPROM.h>                // flash-backed EEPROM emulation on Pico
#include "hardware/watchdog.h"     // Pico SDK — RP2350 hardware watchdog
#include <LoRa.h>                  // Sandeep Mistry LoRa library

// ----------------------------------------------------------------
//  DEBUG SERIAL — left ON for this test build so you can watch it
//  work over USB. Comment out #define DEBUG_SERIAL for a flight
//  build once this test has been validated, to shave serial
//  overhead in the hot loop.
// ----------------------------------------------------------------
#define DEBUG_SERIAL

// ----------------------------------------------------------------
//  BENCH MODE — now a RUNTIME setting (CMD_SET_BENCH_MODE 0x12, uplink),
//  stored in EEPROM so it survives a reboot. No reflash needed at the pad.
//    ON  : every boot forces a clean slate (ignores flight state saved in
//          EEPROM by the previous run) — what you want on the bench / pad.
//    OFF : flight behaviour — after a mid-flight reset the saved flight
//          state is RESUMED (deployment logic keeps working).
//  Rules (enforced in processCommand):
//    - OFF is accepted in any state.
//    - ON is accepted only on the pad (LAUNCH_PAD) or after landing (IMPACT).
//    - Launch detection automatically switches bench mode OFF and saves
//      it, so a forgotten flag can never wipe the flight state mid-flight.
//  BENCH_MODE_DEFAULT is what a never-configured board starts with.
//
//  The #define below now only selects the buzzer wiring: ON routes the beep
//  to GPIO26 (onboard LED stand-in) as well as the real buzzer on GPIO27.
//  It no longer changes any flight/EEPROM behaviour.
//  SAFETY: bench mode does NOT disable deployment. If apogee is detected,
//  both bay-door servos still move for real. Disconnect the servo
//  linkage before running a full-sequence test on the bench.
// ----------------------------------------------------------------
#define BENCH_TEST_MODE

#ifdef DEBUG_SERIAL
  #define DEBUG_PRINT(x)      Serial.print(x)
  #define DEBUG_PRINTLN(x)    Serial.println(x)
  #define DEBUG_PRINT2(x,y)   Serial.print(x,y)
  #define DEBUG_PRINTLN2(x,y) Serial.println(x,y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT2(x,y)
  #define DEBUG_PRINTLN2(x,y)
#endif

// ================================================================
//  PIN MAP
//  Flash + SD share SPI1.  LoRa is on default SPI0.
//  BMP + MPU are on Wire1.  GPS is on UART0 (Serial1).
// ================================================================
#define FLASH_CS_PIN   22   // SPI flash chip-select
#define SD_MOSI_PIN    11
#define SD_MISO_PIN    12
#define SD_SCK_PIN     10
#define SD_CS_PIN      13   // SD card chip-select
#define BATT_ADC_PIN   28   // voltage divider -> ADC2
#define LORA_CS_PIN     5   // LoRa module chip-select
// NOTE: GPIO5's alt-function is SPI0 CSn *and* I2C0 SCL — same physical pin
// as the default I2C0 (Wire) clock line used for the Controller B link below.
// We move the B-link off GPIO4/5 instead of touching this (see I2C_B_SDA_PIN /
// I2C_B_SCL_PIN) so the LoRa wiring/PCB traces don't need to change.
// #define LORA_RST_PIN 16  // LoRa reset — pulled high to 3.3V via 10k resistor, no GPIO needed
#define LORA_IRQ_PIN   16   // LoRa DIO0

#define BMP_IMU_SCL     7   // Wire1
#define BMP_IMU_SDA     6   // Wire1

// ----------------------------------------------------------------
//  I2C LINK TO CONTROLLER B (Wire / I2C0)
//  BUGFIX: Wire.begin() was being called with no explicit pins, so it
//  silently used the core default I2C0 pins — GPIO4 (SDA) / GPIO5 (SCL).
//  Those exact pins are ALSO claimed later in setup() by the LoRa radio:
//  SPI.setRX(4) steals SDA, and LORA_CS_PIN=5 steals SCL. Whichever inits
//  last wins the pin mux, which either breaks LoRa or silently kills the
//  Controller B I2C link (matches "B not responding" / hang symptoms).
//  Moved the B-link to GPIO8/9 (also a valid I2C0 pin pair) so it no
//  longer overlaps the LoRa SPI0 pins. Requires moving the physical
//  SDA/SCL jumper (+ pull-ups) between the two boards from GPIO4/5 to
//  GPIO8/9 — Controller B's pin map below must match.
// ----------------------------------------------------------------
#define I2C_B_SDA_PIN   8
#define I2C_B_SCL_PIN   9

#define GPS_TX          0   // UART0 (Serial1) TX pin
#define GPS_RX          1   // UART0 (Serial1) RX pin

#define IMU_INT        14
#define BMP_INT        17
#define SD_DET         15

#ifdef BENCH_TEST_MODE
  #define BUZZER_PIN   26   // onboard LED (plain Pico 2) — stands in for the buzzer 
  #define BUZZER_PIN_actual   27
#else
  // FLIGHT BUILD: BUZZER_PIN was only ever #defined inside BENCH_TEST_MODE,
  // so simply removing that #define (as instructed above) would leave
  // BUZZER_PIN undefined and fail to compile. Both macros now point at the
  // real physical buzzer pin for a real flight build.
  #define BUZZER_PIN          27
  #define BUZZER_PIN_actual   27
#endif
//#define BUZZER_PIN_actual   27  // active buzzer

// BZ1 (Murata PKLCS 12mm, footprint PKLCS_472x472_MUR) on GP27 via R5 (100E)
// is a PASSIVE piezo sounder, not a self-driving buzzer: it has no internal
// oscillator, so holding the pin at a steady HIGH only produces a single
// click as the disc deflects once, not a tone. It must be driven with a
// square wave (tone()/noTone()) and is loudest at its mechanical
// resonance, ~4 kHz for this part family. See buzzer_check.ino.
#define BUZZER_RESONANT_HZ  4000
#define SERVO_SEP_PIN  19   // CanSat bay door latch servo signal
// ASSUMPTION (competition rebuild): the parachute channel is now a second
// door-latch SERVO, not the old pyro/MOSFET gate — matches "avionics
// controls only the door-latch servos for parachute and CanSat bays" and
// the manual-servo-as-last-resort-parachute-open requirement. If this pin
// is still wired to a MOSFET/pyro channel on the actual board, say so and
// this needs to go back to digitalWrite() instead of Servo.write().
#define PARACHUTE_PIN  18   // parachute bay door latch servo signal

#define SERVO_CLOSED_POS   0    // both bay-door servos: latched / closed
#define SERVO_OPEN_POS     90   // both bay-door servos: open — simple 90 deg throw, no angle param needed
// Old names kept as aliases so nothing else in the file needs touching.
#define SERVO_SAFE_POS    SERVO_CLOSED_POS
#define SERVO_DEPLOY_POS  SERVO_OPEN_POS

// ================================================================
//  TUNING CONSTANTS
// ================================================================
#define SEALEVELPRESSURE_HPA 1012.50f

#define BATT_VOLTAGE_DIVIDER 5.70f
#define BATT_ADC_REF          3.3f
#define BATT_LOW_THRESHOLD_V  7.0f

#define LOW_PASS_ALPHA 0.12f

#define LAUNCH_ACCEL_THRESHOLD_G   3.0f    // G-force to confirm launch
#define LAUNCH_CONFIRM_MS          60      // must hold for this long
#define APOGEE_DROP_THRESHOLD_M    1.2f    // altitude drop to flag apogee
#define APOGEE_NEG_VEL_THRESH      -0.5f   // m/s — velocity must also be negative
#define APOGEE_CONFIRM_COUNT       5       // consecutive readings needed
#define MIN_ASCENT_TIME_MS         2500    // ignore apogee before this
#define BACKUP_APOGEE_TIMEOUT_MS   17000   // hard apogee trigger if not detected
#define LANDING_VELOCITY_THRESH    0.5f    // m/s — consider landed below this
#define LANDING_CONFIRM_MS         5000    // must hold for this long

// -- GPS --
#define GPS_BAUD_RATE       9600    // standard u-blox default
#define GPS_MAX_FIX_AGE_MS  2000    // reject fixes older than 2s
#define GPS_MIN_SATS        4       // minimum satellites for trust
#define GPS_LOST_TIMEOUT_MS 10000   // log a warning after 10s without fix

// -- Logging --
#define LOG_RING_SIZE            64
#define LOG_SD_FLUSH_INTERVAL    450
#define LOG_FLASH_FLUSH_INTERVAL 550
#define LOG_SD_FAIL_LIMIT        20
#define LOG_PREALLOCATE_KB       512
#define FLASH_LOG_START_ADDR     0x001000
#define FLASH_LOG_END_ADDR       0x020000
#define FLASH_MAGIC_ADDR         0x000000
#define FLASH_MAGIC_VAL          0xAF1010AF

// -- LoRa --
// ---- COMPETITION RADIO SETTINGS (865-868 MHz India licence-free band) ----
//  Frequency 866.00 MHz, LoRa, BW 125 kHz (mandatory), SF8 (this pad's row),
//  CR 4/5, TX power +10 dBm on the pad / +14 dBm in flight.
//  Sync word = 0x40 + Team ID. The RFM95 (SX1276) sync word is ONE byte, so
//  it is the sum: 0x40 + 0x52 = 0x92.  !! CONFIRM THIS READING WITH THE
//  ORGANISERS, and give the ground-station receiver the same value. !!
#define LORA_FREQUENCY         866E6
#define LORA_TX_POWER_PAD      10      // dBm — LAUNCH_PAD, IMPACT, BOOT
#define LORA_TX_POWER_FLIGHT   14      // dBm — ASCENT, PAYLOAD_SEP, DESCENT
#define LORA_SPREADING_FACTOR  8
#define LORA_BANDWIDTH         125E3
#define LORA_CODING_RATE       5
#define LORA_SYNC_BASE         0x40
#define LORA_TEAM_ID           0x52
#define LORA_SYNC_WORD         (LORA_SYNC_BASE + LORA_TEAM_ID)   // 0x92
// AIRTIME at SF8 / BW125 / CR4/5 / 8-symbol preamble / CRC on:
//   82-byte telemetry frame ~ 257 ms   (was ~32 ms at SF7/BW500)
//   19-byte status frame    ~ 103 ms
// The old 100 ms (10 Hz) TX interval is therefore impossible. Telemetry is
// sent every LORA_TX_INTERVAL_MS; the rest of the cycle is the window in
// which the radio listens for uplink commands. Max possible rate ~3.9 Hz.
#define LORA_TX_INTERVAL_MS    500     // 2 Hz downlink, ~243 ms listening gap per cycle
#define LORA_RX_POLL_MS        10      // how often the RX side polls the radio
#define LORA_PKT_SYNC_0        0xA5
#define LORA_PKT_SYNC_1        0x5A
// Uplink command frame, now variable length to carry command arguments:
//   [sync0][sync1][cmdId][seq][len][payload...(len bytes)][crc_hi][crc_lo]
// CRC covers everything before the crc bytes, i.e. the first (5+len) bytes.
// LORA_CMD_HDR_LEN = sync0+sync1+cmdId+seq+len = 5 bytes, +2 crc bytes = 7 with no payload.
#define LORA_CMD_HDR_LEN       5
#define LORA_CMD_MAX_PAYLOAD   8
#define LORA_CMD_MIN_LEN       (LORA_CMD_HDR_LEN + 2)   // header + crc, zero payload

#define I2C_TIMEOUT_MS         25
#define WATCHDOG_TIMEOUT_MS    4000
#define WATCHDOG_PET_INTERVAL  500

// BUGFIX: was used in readControllerB() but never defined — compile error.
// B's heartbeat to A arrives every 200ms (taskHeartbeat on B's side);
// 1000ms allows for ~5 missed beats before A calls B's link dead.
#define HEARTBEAT_TIMEOUT_MS   1000

// -- EEPROM --
#define EEPROM_MAGIC           0xA510DEAD
#define EEPROM_MAGIC_ADDR      0
#define EEPROM_STATE_ADDR      4
#define EEPROM_FLIGHTNUM_ADDR  64
#define EEPROM_BENCH_ADDR      72      // 1 byte: BENCH_TAG_ON / BENCH_TAG_OFF (anything else = default)
#define BENCH_TAG_OFF          0xB0
#define BENCH_TAG_ON           0xB1
#define BENCH_MODE_DEFAULT     true    // value for a board that was never configured by uplink

bool benchMode = BENCH_MODE_DEFAULT;   // runtime flag, loaded from EEPROM in setup()

// ----------------------------------------------------------------
//  I2C ADDRESSES
// ----------------------------------------------------------------
#define CONTROLLER_B_ADDR      0x42

// ----------------------------------------------------------------
//  CONTROLLER B PACKET
//  Full status packet B sends over I2C every 200ms.
//  A reads it as I2C master and validates CRC before trusting.
// ----------------------------------------------------------------
struct ControllerBPacket {
    float    altitude;
    float    accelMag;
    bool     apogeeFlag;
    bool     deploymentTriggered;  // B fired its own deployment channel
    uint8_t  state;
    uint8_t  sensorHealth;         // bitmask: bit0=baro OK, bit1=IMU OK
    uint16_t crc;
};
// ----------------------------------------------------------------
//  CONTROLLER B STATUS — parsed and validated state of B
//  Populated by readControllerB(). All flight logic reads from here,
//  never directly from the raw packet buffer.
// ----------------------------------------------------------------
struct ControllerBStatus {
    bool     alive;                  // heartbeat healthy
    bool     apogeeDetected;         // B declared apogee
    bool     deploymentTriggered;    // B fired deployment
    bool     baroHealthy;            // B baro sensor OK
    bool     imuHealthy;             // B IMU sensor OK
    uint8_t  state;                  // B flight state
    float    altitude;               // B baro altitude
    float    accelMag;               // B accel magnitude, G (forwarded in A's downlink)
    uint32_t lastHeartbeatTime;      // millis() of last valid packet
};

ControllerBStatus ctrlB = {false, false, false, false, false, 0, 0.0f, 0.0f, 0};

// ----------------------------------------------------------------
//  HEARTBEAT PACKET — A transmits this over I2C so B can monitor A
//  B reads A's heartbeat to decide when to act autonomously.
//  Sent every 200ms from Controller A as I2C broadcast.
// ----------------------------------------------------------------
struct HeartbeatPacket {
    uint32_t missionTime;
    uint8_t  state;
    bool     launched;
    bool     apogeeDetected;
    bool     separated;
    bool     baroHealthy;
    bool     imuHealthy;
    bool     armed;
    uint16_t crc;
};

// ================================================================
//  ENUMS
// ================================================================
enum FlightState : uint8_t {
    BOOT        = 0,
    // 1 retired (was TEST_MODE) — numbering of the other states is intentionally unchanged
    LAUNCH_PAD  = 2,
    ASCENT      = 3,
    PAYLOAD_SEP = 4,
    DESCENT     = 5,
    IMPACT      = 6,
    SAFE_MODE   = 7
};

enum LoRaCmd : uint8_t {
    // 0x01 retired (was CMD_ARM — arming removed, launch detection always live on the pad)
    CMD_START_TX        = 0x02,  // begin LoRa telemetry downlink
    CMD_STOP_TX         = 0x03,  // pause LoRa downlink (logging continues)
    CMD_SET_ZERO        = 0x04,  // calibrate altitude baseline to current reading
    CMD_BEEP            = 0x05,  // trigger locator beep sequence
    // 0x06-0x08 retired (CMD_TEST_MODE / CMD_SIM_LAUNCH / CMD_SIM_APOGEE removed)
    CMD_ERASE_FLASH     = 0x09,  // erase SPI flash (only from LAUNCH_PAD)

    // -- added for competition GUI-verification workflow --
    CMD_GET_STATUS       = 0x0A,  // on-demand sensor-health / go-no-go packet, any time, any state
    CMD_SET_SEALEVEL_HPA = 0x0B,  // payload = float32 hPa. Bad/missing payload -> keep fallback constant
    CMD_SERVO_PARACHUTE_OPEN  = 0x0C,  // parachute bay latch -> SERVO_OPEN_POS (stays open)
    CMD_SERVO_PAYLOAD_OPEN    = 0x0D,  // CanSat bay latch    -> SERVO_OPEN_POS (stays open)
    CMD_SERVO_PARACHUTE_CLOSE = 0x0E,  // parachute bay latch -> SERVO_CLOSED_POS (LAUNCH_PAD / IMPACT only)
    CMD_SERVO_PAYLOAD_CLOSE   = 0x0F,  // CanSat bay latch    -> SERVO_CLOSED_POS (LAUNCH_PAD / IMPACT only)
    CMD_RESET_MCU        = 0x10,  // last-resort reset. Honored ONLY from LAUNCH_PAD (GUI confirms first)
    // 0x11 is used by the ground station for CMD_SET_TARGET (CanSat) — A ignores it
    CMD_SET_BENCH_MODE   = 0x12,  // payload = 1 byte: 1 = bench ON (clean slate each boot), 0 = OFF (flight)
};

// Downlink frames now carry an explicit type byte (3rd byte, right after the
// two sync bytes) so the ground station can tell a status reply apart from a
// telemetry packet regardless of when/why it was sent — needed now that
// CMD_GET_STATUS can arrive at any moment, not just once at boot.
enum LoRaPacketType : uint8_t {
    PKT_STATUS    = 0xA1,
    PKT_TELEMETRY = 0xA2,
};

// ================================================================
//  STRUCTS
// ================================================================

// On-board working telemetry record (logging, CSV debug, and the source
// the on-air packet is built from). This is NOT what goes over the radio —
// see TelemetryWire below for the compact on-air layout.
struct TelemetryPacket {
    uint32_t missionTime;       // ms since boot

    float altitude;             // filtered baro altitude, m
    float maxAltitude;          // peak recorded altitude, m

    float accelX;                // m/s^2
    float accelY;
    float accelZ;
    float accelMag;              // magnitude in G

    float temperature;           // deg C
    float pressure;               // hPa

    float pitch;                  // degrees
    float roll;
    float yaw;

    float batteryVoltage;         // V

    float gpsLat;
    float gpsLon;
    float gpsAltitude;
    uint8_t gpsSats;
    bool  gpsStale;                // true if fix is old or invalid

    uint8_t state;                 // FlightState enum value

    bool launched;
    bool apogee;
    bool separated;
    bool landed;
    bool controllerBAlive;         // No Controller B in this test — always false
    bool battLow;

    uint16_t crc;                  // CRC-16/CCITT over all preceding bytes
};

// ----------------------------------------------------------------
//  ON-AIR TELEMETRY PACKET (PKT_TELEMETRY payload) — compact, packed.
//  Frame: [0xA5][0x5A][0xA2][seq][ TelemetryWire, 78 bytes ]
//  Little-endian, no padding (packed). CRC-16/CCITT-FALSE (poly 0x1021,
//  init 0xFFFF) over the first 76 bytes (everything before `crc`).
//
//  Differences from the old on-air struct:
//   - Gyro XYZ removed. Ground station derives angular rate by
//     differentiating pitch/roll/yaw over missionTime.
//   - Seven bool flags packed into the `flags` bitmask (below).
//   - Controller B's state/baro-altitude/accel-magnitude appended
//     (B has no radio of its own; it rides on A's link).
// ----------------------------------------------------------------
#define TLM_FLAG_LAUNCHED     (1u << 0)
#define TLM_FLAG_APOGEE       (1u << 1)
#define TLM_FLAG_SEPARATED    (1u << 2)
#define TLM_FLAG_LANDED       (1u << 3)
#define TLM_FLAG_CTRLB_ALIVE  (1u << 4)   // if 0, the b* fields below are last-known/stale
#define TLM_FLAG_BATT_LOW     (1u << 5)
#define TLM_FLAG_GPS_STALE    (1u << 6)
// bit 7 reserved, always 0

struct __attribute__((packed)) TelemetryWire {
    uint32_t missionTime;       // ms since boot
    float    altitude;          // m, filtered baro
    float    maxAltitude;       // m
    float    accelX;            // m/s^2
    float    accelY;
    float    accelZ;
    float    accelMag;          // G
    float    temperature;       // deg C
    float    pressure;          // hPa
    float    pitch;             // deg
    float    roll;              // deg
    float    yaw;               // deg
    float    batteryVoltage;    // V
    float    gpsLat;
    float    gpsLon;
    float    gpsAltitude;
    uint8_t  gpsSats;
    uint8_t  state;             // A's FlightState
    uint8_t  flags;             // TLM_FLAG_* bitmask
    uint8_t  bState;            // B's flight state (b_state)
    float    bAltitude;         // B baro altitude, m (b_hbaro)
    float    bAccelMag;         // B accel magnitude, G (b_accel)
    uint16_t crc;
};
static_assert(sizeof(TelemetryWire) == 78, "TelemetryWire must be exactly 78 bytes — ground station depends on it");

struct PersistentState {
    FlightState state;
    float       maxAltitude;
    float       altitudeAtLaunch;
    uint32_t    launchElapsedMs;
    bool        launched;
    bool        apogeeDetected;
    bool        separated;
    bool        landed;
    bool        deploymentFired;
    uint16_t    crc;
};

// Lightweight GO/NO-GO packet broadcast once at boot.
struct StatusPacket {
    uint8_t  marker;          // 0x55 = status packet identifier
    bool     bmpOK;
    bool     bnoOK;
    bool     sdOK;
    bool     flashOK;
    bool     loraOK;
    bool     gpsSearching;    // true = module alive but no fix yet
    bool     ctrlBAlive;      // No Controller B in this test — always false
    float    battV;
    uint8_t  flightNumber;
    bool     benchMode;       // fills the old padding byte: struct stays 16 bytes, crc stays at offset 14
    uint16_t crc;
};
static_assert(sizeof(StatusPacket) == 16, "StatusPacket must stay 16 bytes — ground station mirrors it");
static_assert(offsetof(StatusPacket, crc) == 14, "StatusPacket crc offset must stay 14");

struct LogEntry {
    uint32_t missionTime;
    float    altitude;
    float    maxAltitude;
    float    vertVelocity;
    float    accelMag;
    float    pitch;
    float    roll;
    float    yaw;
    float    temperature;
    float    pressure;
    float    battV;
    float    gpsLat;
    float    gpsLon;
    uint8_t  gpsSats;
    bool     gpsStale;
    uint8_t  state;
    bool     launched;
    bool     apogee;
    bool     separated;
};  // 44 bytes per record

struct RingBuffer {
    LogEntry records[LOG_RING_SIZE];
    uint8_t head;
    uint8_t sdTail;
    uint8_t flashTail;
};

struct Task {
    uint32_t lastRun;
    uint32_t intervalMs;
};

inline bool runTask(Task& t, uint32_t now) {
    if (now - t.lastRun >= t.intervalMs) {
        if (now - t.lastRun >= t.intervalMs * 2) {
            t.lastRun = now;
        } else {
            t.lastRun += t.intervalMs;
        }
        return true;
    }
    return false;
}

// ================================================================
//  GLOBALS
// ================================================================
FlightState state = BOOT;

TelemetryPacket telemetry;
RingBuffer      ringBuf = { { { 0 } }, 0, 0, 0 };

float verticalVelocity  = 0.0f;
float previousAltitude  = 0.0f;
float altitudeFiltered  = 0.0f;
float altitudeRaw       = 0.0f;
float altitudeAtLaunch  = 0.0f;

uint32_t launchElapsedMs  = 0;
uint32_t launchTime       = 0;
uint32_t apogeeTime       = 0;
uint32_t missionStartTime = 0;

// -- device status flags --
bool aBaroHealthy = false;
bool aImuHealthy  = false;
bool sdReady      = false;
bool flashReady   = false;
uint8_t sdFailCount = 0;
uint32_t flashWriteAddr = FLASH_LOG_START_ADDR;
uint8_t flightNumber = 0;

// -- flight flags --
bool launched        = false;
bool apogeeDetected  = false;
bool separated       = false;
bool landed          = false;
bool deploymentFired = false;   // latching guard — deployment executes exactly once

// -- GPS --
float    gpsLastLat     = 0.0f;
float    gpsLastLon     = 0.0f;
float    gpsLastAlt     = 0.0f;
uint8_t  gpsLastSats    = 0;
bool     gpsHadFix      = false;
uint32_t gpsLastFixTime = 0;
TinyGPSPlus gps;

// -- LoRa --
bool     loraReady      = false;
bool     loraTxEnabled  = false;
uint8_t  loraTxSeq      = 0;
uint8_t  loraLastRxSeq  = 0xFF;
uint32_t loraLastTxTime = 0;
int8_t   loraTxPowerNow = 0;   // power currently programmed into the radio, dBm

// -- buzzer --
uint32_t buzzerEndTime = 0;   // 0 = buzzer currently off

// -- competition GUI-verification additions --
float    seaLevelPressureHPA = SEALEVELPRESSURE_HPA;  // GUI-settable QNH; falls back to the built-in constant
bool     firstTxSinceBoot    = true;   // true only until the very first CMD_START_TX after boot
                                         // (that first START_TX sends the health packet before telemetry;
                                         // any later stop/start cycle goes straight to telemetry)

// -- devices --
Adafruit_BMP3XX bmp;
#define MPU6500_ADDR 0x68
MPU6500_WE mpu(&Wire1, MPU6500_ADDR);
Servo separationServo;   // CanSat bay door latch
Servo parachuteServo;    // Parachute bay door latch (was a MOSFET gate — see PARACHUTE_PIN note above)

SdFat sd;
FsFile logFile;
const uint32_t PRE_ALLOC_SIZE = (uint32_t)LOG_PREALLOCATE_KB * 1024UL;

// -- tasks --
Task taskSensors     = { 0,  20 };
Task taskBuildTelem   = { 0, 100 };
Task taskSerialCSV    = { 0, 200 };
Task taskWatchdog     = { 0, 500 };
Task taskSDFlush      = { 0, 450 };
Task taskFlashFlush   = { 0, 550 };
Task taskGPSDrain     = { 0,  20 };   //  50Hz UART drain — prevents overflow
Task taskGPSValidate  = { 0, 200 };   //   5Hz fix validation + telemetry update
Task taskLoRaTx       = { 0, LORA_TX_INTERVAL_MS };  // 2Hz telemetry downlink (257 ms airtime each)
Task taskLoRaRx       = { 0, LORA_RX_POLL_MS };      // 100Hz poll for uplink commands
Task taskControllerB = {0, 200};   // 200ms =  5Hz  read B heartbeat
Task taskHeartbeat   = {0, 200};   // 200ms =  5Hz  transmit A heartbeat to B

// ================================================================
//  UTILITIES
// ================================================================
float lowPass(float newVal, float prevFiltered, float alpha) {
    return prevFiltered + alpha * (newVal - prevFiltered);
}

float readBatteryVoltage() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        sum += analogRead(BATT_ADC_PIN);
    }
    float voltage = ((sum / 4.0f) / 4095.0f) * BATT_ADC_REF * BATT_VOLTAGE_DIVIDER;
    return voltage;
}

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

// ================================================================
//  BUZZER
// ================================================================
void scheduleBeep(uint16_t durationMs) {
    uint32_t newEndTime = millis() + durationMs;
    if (newEndTime > buzzerEndTime) {              // never cut a running beep short
        buzzerEndTime = newEndTime;
    }
    digitalWrite(BUZZER_PIN, HIGH);               // LED stand-in — DC is fine here
    tone(BUZZER_PIN_actual, BUZZER_RESONANT_HZ);   // passive piezo needs a square wave
}

void taskBuzzerTick(uint32_t now) {
    if (buzzerEndTime > 0 && now >= buzzerEndTime) {
        digitalWrite(BUZZER_PIN, LOW);
        noTone(BUZZER_PIN_actual);
        // noTone() leaves the pin wherever the waveform stopped; park it
        // low so the piezo isn't left with a DC bias between beeps.
        digitalWrite(BUZZER_PIN_actual, LOW);
        buzzerEndTime = 0;
    }
}

// Blocking version for setup() ONLY. During flight use scheduleBeep().
void beepBlocking(uint16_t durationMs) {
    digitalWrite(BUZZER_PIN, HIGH);
    tone(BUZZER_PIN_actual, BUZZER_RESONANT_HZ);
    delay(durationMs);
    digitalWrite(BUZZER_PIN, LOW);
    noTone(BUZZER_PIN_actual);
    digitalWrite(BUZZER_PIN_actual, LOW);
}

// -- startup self-test beeps (setup() only, blocking) --
// One short beep = this component initialized OK. Two short beeps = it
// failed. Count-based so the pattern is unambiguous even if you can't see
// the serial monitor on the pad.
void beepComponentOK() {
    beepBlocking(80);
    delay(150);
}
void beepComponentFail() {
    beepBlocking(90); delay(90);
    beepBlocking(90);
    delay(150);
}
// Full-init verdict, played once setup() finishes deciding GO/NO-GO.
void beepStartupGo() {
    for (uint8_t i = 0; i < 3; i++) { beepBlocking(90); delay(90); }
}
void beepStartupNoGo() {
    beepBlocking(1200);
}
// Controller B not found within the wait window — distinct from both of the above.
void beepControllerBTimeout() {
    beepBlocking(400); delay(200);
    beepBlocking(400);
}

// ================================================================
//  STATE MACHINE HELPERS + EEPROM PERSISTENCE
// ================================================================
void saveState();  // fwd decl (used by setState before its own definition)

void setState(FlightState newState) {
    if (newState == state) return;

    DEBUG_PRINT(F("STATE "));
    DEBUG_PRINT((uint8_t)state);
    DEBUG_PRINT(F(" -> "));
    DEBUG_PRINTLN((uint8_t)newState);

    state = newState;
    saveState();
}

void saveState() {
    PersistentState ps;
    ps.state            = state;
    ps.maxAltitude       = telemetry.maxAltitude;
    ps.altitudeAtLaunch  = altitudeAtLaunch;
    ps.launchElapsedMs   = launchElapsedMs;
    ps.launched          = launched;
    ps.apogeeDetected    = apogeeDetected;
    ps.separated         = separated;
    ps.landed            = landed;
    ps.deploymentFired   = deploymentFired;

    ps.crc = calculateCRC(
        (const uint8_t*)&ps,
        sizeof(PersistentState) - sizeof(uint16_t)
    );

    uint32_t magic = EEPROM_MAGIC;
    EEPROM.put(EEPROM_MAGIC_ADDR, magic);
    EEPROM.put(EEPROM_STATE_ADDR, ps);
    EEPROM.commit();
    DEBUG_PRINT(F("EEPROM saved state="));
    DEBUG_PRINTLN((uint8_t)state);
}

bool loadState() {
    uint32_t magic = 0;
    EEPROM.get(EEPROM_MAGIC_ADDR, magic);

    if (magic != EEPROM_MAGIC) {
        DEBUG_PRINTLN(F("EEPROM: no valid state — fresh boot"));
        return false;
    }

    PersistentState ps;
    EEPROM.get(EEPROM_STATE_ADDR, ps);

    uint16_t expectedCRC = calculateCRC(
        (const uint8_t*)&ps,
        sizeof(PersistentState) - sizeof(uint16_t)
    );

    if (ps.crc != expectedCRC) {
        DEBUG_PRINTLN(F("EEPROM: CRC mismatch — fresh boot"));
        return false;
    }

    if (ps.state == BOOT || ps.state > SAFE_MODE) {
        DEBUG_PRINTLN(F("EEPROM: stored state invalid — fresh boot"));
        return false;
    }

    state             = ps.state;
    telemetry.maxAltitude = ps.maxAltitude;
    altitudeAtLaunch  = ps.altitudeAtLaunch;
    launchElapsedMs   = ps.launchElapsedMs;
    launched          = ps.launched;
    apogeeDetected    = ps.apogeeDetected;
    separated         = ps.separated;
    landed            = ps.landed;
    deploymentFired   = ps.deploymentFired;

    DEBUG_PRINT(F("EEPROM: resumed state="));
    DEBUG_PRINT((uint8_t)state);
    DEBUG_PRINT(F(" maxAlt="));
    DEBUG_PRINTLN2(ps.maxAltitude, 1);

    return true;
}

void clearSavedState() {
    uint32_t blank = 0x00000000;
    EEPROM.put(EEPROM_MAGIC_ADDR, blank);
    EEPROM.commit();
    DEBUG_PRINTLN(F("EEPROM: cleared"));
}

// Bench-mode flag lives in its own EEPROM byte, so clearSavedState() (which
// only blanks the magic word) never touches it.
void loadBenchFlag() {
    uint8_t tag = 0;
    EEPROM.get(EEPROM_BENCH_ADDR, tag);
    if      (tag == BENCH_TAG_ON)  benchMode = true;
    else if (tag == BENCH_TAG_OFF) benchMode = false;
    else                           benchMode = BENCH_MODE_DEFAULT;
}

void saveBenchFlag(bool on) {
    uint8_t tag = on ? BENCH_TAG_ON : BENCH_TAG_OFF;
    EEPROM.put(EEPROM_BENCH_ADDR, tag);
    EEPROM.commit();
}

// ================================================================
//  SD CARD (SdFat, shared SPI1 with flash)
// ================================================================
void initSD() {
    SPI1.endTransaction();

    pinMode(SD_DET, INPUT_PULLUP);

    // SD_DET has proven unreliable on this hardware (reads steady HIGH
    // regardless of whether a card is inserted — confirmed by testing
    // both with and without a card). Rather than guess at polarity, we
    // no longer gate initialization on this pin at all. It's still read
    // and printed here purely for diagnostic visibility; sd.begin()'s
    // own return value is the sole authority on whether the card works.
    uint8_t highCount = 0;
    const uint8_t SAMPLES = 8;
    for (uint8_t i = 0; i < SAMPLES; i++) {
        if (digitalRead(SD_DET) == HIGH) highCount++;
        delay(5);
    }
    DEBUG_PRINT(F("SD_DET raw HIGH count (diagnostic only, not gating init): "));
    DEBUG_PRINT(highCount);
    DEBUG_PRINT(F("/"));
    DEBUG_PRINTLN(SAMPLES);

    if (!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, 16000000, &SPI1))) {
        DEBUG_PRINTLN(F("SD: init failed — flash only"));
        #ifdef DEBUG_SERIAL
        sd.initErrorPrint(&Serial);
        #endif
        sdReady = false;
        return;
    }

    char filename[16];
    snprintf(filename, sizeof(filename), "flt_%03u.bin", (unsigned)flightNumber);

    if (sd.exists(filename)) {
        sd.remove(filename);
    }

    if (!logFile.open(filename, O_WRONLY | O_CREAT | O_TRUNC)) {
        DEBUG_PRINTLN(F("SD: open failed"));
        sdReady = false;
        return;
    }

    if (!logFile.preAllocate(PRE_ALLOC_SIZE)) {
        DEBUG_PRINTLN(F("SD: FAILED allocation! Card might be fragmented or full."));
        logFile.close();
        sdReady = false;
        return;
    }

    sdReady = true;
    sdFailCount = 0;
    DEBUG_PRINTLN(F("SD: ready (contiguous space pre-allocated)"));
}

// ================================================================
//  SPI FLASH — RAW BIT-BANGED DRIVER (SPI1, shared with SD)
// ================================================================
void flashWaitReady() {
    SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(FLASH_CS_PIN, LOW);
    SPI1.transfer(0x05); // Read Status Register 1

    uint32_t startCheck = millis();
    while (SPI1.transfer(0x00) & 0x01) {
        watchdog_update();
        if (millis() - startCheck > 100) {
            break;
        }
    }
    digitalWrite(FLASH_CS_PIN, HIGH);
    SPI1.endTransaction();
}

void flashWriteEnable() {
    SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(FLASH_CS_PIN, LOW);
    SPI1.transfer(0x06);  // Write Enable
    digitalWrite(FLASH_CS_PIN, HIGH);
    SPI1.endTransaction();
}

void rawFlashRead(uint32_t addr, uint8_t* buf, size_t len) {
    SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(FLASH_CS_PIN, LOW);
    SPI1.transfer(0x03);  // Read Data
    SPI1.transfer((addr >> 16) & 0xFF);
    SPI1.transfer((addr >> 8) & 0xFF);
    SPI1.transfer(addr & 0xFF);
    for (size_t i = 0; i < len; i++) {
        buf[i] = SPI1.transfer(0x00);
    }
    digitalWrite(FLASH_CS_PIN, HIGH);
    SPI1.endTransaction();
}

void rawFlashWritePage(uint32_t addr, uint8_t* buf, size_t len) {
    size_t i = 0;
    while (i < len) {
        size_t space = 256 - ((addr + i) & 0xFF);
        size_t chunk = (len - i < space) ? (len - i) : space;

        flashWriteEnable();
        SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        digitalWrite(FLASH_CS_PIN, LOW);
        SPI1.transfer(0x02);
        SPI1.transfer(((addr + i) >> 16) & 0xFF);
        SPI1.transfer(((addr + i) >> 8) & 0xFF);
        SPI1.transfer((addr + i) & 0xFF);
        for (size_t j = 0; j < chunk; j++) {
            SPI1.transfer(buf[i + j]);
        }
        digitalWrite(FLASH_CS_PIN, HIGH);
        SPI1.endTransaction();

        delayMicroseconds(5);
        flashWaitReady();
        i += chunk;
    }
}

void rawFlashEraseSector(uint32_t addr) {
    flashWriteEnable();
    SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(FLASH_CS_PIN, LOW);
    SPI1.transfer(0x20);  // Sector Erase (4KB)
    SPI1.transfer((addr >> 16) & 0xFF);
    SPI1.transfer((addr >> 8) & 0xFF);
    SPI1.transfer(addr & 0xFF);
    digitalWrite(FLASH_CS_PIN, HIGH);
    SPI1.endTransaction();
    flashWaitReady();
}

void flashEraseLogRegion() {
    for (uint32_t a = 0; a < FLASH_LOG_END_ADDR; a += 4096) {
        rawFlashEraseSector(a);
    }
}

uint32_t flashScanForEnd() {
    uint8_t buf[sizeof(LogEntry)];
    uint32_t a = FLASH_LOG_START_ADDR;
    for (; a + sizeof(LogEntry) <= FLASH_LOG_END_ADDR; a += sizeof(LogEntry)) {
        rawFlashRead(a, buf, sizeof(buf));
        bool erased = true;
        for (size_t i = 0; i < sizeof(buf); i++) {
            if (buf[i] != 0xFF) { erased = false; break; }
        }
        if (erased) return a;
    }
    return a;
}

void initFlash() {
    DEBUG_PRINTLN(F("Inside initFlash..."));

    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);

    pinMode(FLASH_CS_PIN, OUTPUT);
    digitalWrite(FLASH_CS_PIN, HIGH);

    SPI1.setRX(SD_MISO_PIN);
    SPI1.setTX(SD_MOSI_PIN);
    SPI1.setSCK(SD_SCK_PIN);
    SPI1.begin();
    DEBUG_PRINTLN(F("SPI1 instance started."));

    SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(FLASH_CS_PIN, LOW);
    SPI1.transfer(0x9F);  // Read JEDEC ID
    uint32_t mid      = SPI1.transfer(0x00);
    uint32_t memType  = SPI1.transfer(0x00);
    uint32_t capacity = SPI1.transfer(0x00);
    digitalWrite(FLASH_CS_PIN, HIGH);
    SPI1.endTransaction();

    uint32_t jedec_id = (mid << 16) | (memType << 8) | capacity;

    if (jedec_id == 0 || jedec_id == 0xFFFFFF) {
        DEBUG_PRINTLN(F("Error: No valid flash chip detected. Check your wiring."));
        flashReady = false;
        return;
    } else {
        DEBUG_PRINT(F("JEDEC ID Detected: 0x"));
        DEBUG_PRINTLN2(jedec_id, HEX);
    }

    uint32_t magic = 0;
    rawFlashRead(FLASH_MAGIC_ADDR, (uint8_t*)&magic, sizeof(magic));

    if (magic == FLASH_MAGIC_VAL) {
        flashWriteAddr = flashScanForEnd();
        DEBUG_PRINT(F("Flash: resumed addr=0x"));
        DEBUG_PRINTLN2(flashWriteAddr, HEX);
    } else {
        flashEraseLogRegion();
        magic = FLASH_MAGIC_VAL;
        rawFlashWritePage(FLASH_MAGIC_ADDR, (uint8_t*)&magic, sizeof(magic));
        flashWriteAddr = FLASH_LOG_START_ADDR;
        DEBUG_PRINTLN(F("Flash: initialised fresh"));
    }

    flashReady = true;
}

// Only from LAUNCH_PAD (enforced in processCommand). Sector-erases
// the first 128KB region used for the magic header + active log.
void eraseFlashForFlight() {
    if (!flashReady) return;

    flashEraseLogRegion();

    uint32_t magic = FLASH_MAGIC_VAL;
    rawFlashWritePage(FLASH_MAGIC_ADDR, (uint8_t*)&magic, sizeof(magic));
    flashWriteAddr = FLASH_LOG_START_ADDR;

    ringBuf.flashTail = ringBuf.head;
    DEBUG_PRINTLN(F("Flash: erased for flight"));
}

// ================================================================
//  RING BUFFER + FLUSH
// ================================================================
void pushToBuffer() {
    // Before writing, make sure neither active consumer's true unread
    // backlog is already at max usable capacity (reserving one slot so
    // "full" and "empty" are never ambiguous). If it is, advance that
    // specific consumer's tail first, sacrificing only its own oldest
    // unread record. A consumer that isn't ready (e.g. SD not detected)
    // is skipped entirely so it can never block or corrupt the other's
    // stream — this replaces the old shared 'count' latch, which forced
    // BOTH tails forward together regardless of real per-consumer
    // progress and caused stale/duplicate reads once the buffer filled.
    if (sdReady) {
        uint8_t sdBacklog = (ringBuf.head - ringBuf.sdTail + LOG_RING_SIZE) % LOG_RING_SIZE;
        if (sdBacklog >= LOG_RING_SIZE - 1) {
            ringBuf.sdTail = (ringBuf.sdTail + 1) % LOG_RING_SIZE;
        }
    }
    if (flashReady) {
        uint8_t flashBacklog = (ringBuf.head - ringBuf.flashTail + LOG_RING_SIZE) % LOG_RING_SIZE;
        if (flashBacklog >= LOG_RING_SIZE - 1) {
            ringBuf.flashTail = (ringBuf.flashTail + 1) % LOG_RING_SIZE;
        }
    }

    LogEntry entry;
    entry.missionTime  = telemetry.missionTime;
    entry.altitude     = telemetry.altitude;
    entry.maxAltitude  = telemetry.maxAltitude;
    entry.vertVelocity = verticalVelocity;
    entry.accelMag     = telemetry.accelMag;
    entry.pitch        = telemetry.pitch;
    entry.roll         = telemetry.roll;
    entry.yaw          = telemetry.yaw;
    entry.temperature  = telemetry.temperature;
    entry.pressure     = telemetry.pressure;
    entry.battV        = telemetry.batteryVoltage;
    entry.gpsLat       = telemetry.gpsLat;
    entry.gpsLon       = telemetry.gpsLon;
    entry.gpsSats      = telemetry.gpsSats;
    entry.gpsStale     = telemetry.gpsStale;
    entry.state        = telemetry.state;
    entry.launched     = telemetry.launched;
    entry.apogee       = telemetry.apogee;
    entry.separated    = telemetry.separated;

    ringBuf.records[ringBuf.head] = entry;
    ringBuf.head = (ringBuf.head + 1) % LOG_RING_SIZE;
}

void flushToSD() {
    if (!sdReady) return;
    uint8_t pending = (ringBuf.head - ringBuf.sdTail + LOG_RING_SIZE) % LOG_RING_SIZE;
    if (pending == 0) return;

    watchdog_update();
    uint8_t readIdx = ringBuf.sdTail;
    bool writeOk = true;

    for (uint8_t i = 0; i < pending; i++) {
        LogEntry& e = ringBuf.records[readIdx];

        if (logFile.curPosition() + sizeof(LogEntry) >= PRE_ALLOC_SIZE) {
            DEBUG_PRINTLN(F("\n[SUCCESS] SD pre-allocated boundary reached. Closing cleanly."));
            logFile.truncate();
            logFile.close();
            sdReady = false;
            break;
        }

        int bytesWritten = logFile.write((const uint8_t*)&e, sizeof(LogEntry));
        if (bytesWritten != sizeof(LogEntry)) {
            writeOk = false;
            break;
        }
        readIdx = (readIdx + 1) % LOG_RING_SIZE;
    }

    if (writeOk) {
        logFile.sync();
        ringBuf.sdTail = readIdx;
        sdFailCount = 0;
    } else {
        sdFailCount++;
        DEBUG_PRINT(F("SD: write fail #"));
        DEBUG_PRINTLN(sdFailCount);
        if (sdFailCount >= LOG_SD_FAIL_LIMIT) {
            sdReady = false;
            logFile.close();
            DEBUG_PRINTLN(F("SD: abandoned"));
        }
    }
    watchdog_update();
}

void flushToFlash() {
    if (!flashReady) return;

    uint8_t pending = (ringBuf.head - ringBuf.flashTail + LOG_RING_SIZE) % LOG_RING_SIZE;
    if (pending == 0) return;

    watchdog_update();

    uint8_t readIdx = ringBuf.flashTail;
    for (uint8_t i = 0; i < pending; i++) {
        if (flashWriteAddr + sizeof(LogEntry) > FLASH_LOG_END_ADDR) {
            flashReady = false;
            break;
        }

        LogEntry& e = ringBuf.records[readIdx];
        rawFlashWritePage(flashWriteAddr, (uint8_t*)&e, sizeof(LogEntry));
        flashWriteAddr += sizeof(LogEntry);
        readIdx = (readIdx + 1) % LOG_RING_SIZE;
    }

    ringBuf.flashTail = readIdx;
    watchdog_update();
}

// ================================================================
//  GPS — not physically connected yet, but wired up so it works
//  the moment the module is attached before launch.
// ================================================================
void initGPS() {
    Serial1.setTX(GPS_TX);
    Serial1.setRX(GPS_RX);
    Serial1.begin(GPS_BAUD_RATE);
    delay(100);   // blocking settle — acceptable in setup()
    DEBUG_PRINTLN(F("GPS UART started"));
}

// Called every 20ms (50Hz) — only feeds bytes to the parser.
void drainGPS() {
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }
}

// Called every 200ms (5Hz) — validates fix quality, updates telemetry.
void validateGPS() {
    bool fixValid = gps.location.isValid()
                 && (gps.location.age() < GPS_MAX_FIX_AGE_MS)
                 && gps.satellites.isValid()
                 && (gps.satellites.value() >= GPS_MIN_SATS);

    if (fixValid) {
        gpsLastLat     = (float)gps.location.lat();
        gpsLastLon     = (float)gps.location.lng();
        gpsLastSats    = (uint8_t)gps.satellites.value();
        gpsHadFix      = true;
        gpsLastFixTime = millis();

        if (gps.altitude.isValid()) {
            gpsLastAlt = (float)gps.altitude.meters();
        }

        telemetry.gpsLat      = gpsLastLat;
        telemetry.gpsLon      = gpsLastLon;
        telemetry.gpsAltitude = gpsLastAlt;
        telemetry.gpsSats     = gpsLastSats;
        telemetry.gpsStale    = false;

    } else {
        telemetry.gpsLat      = gpsLastLat;
        telemetry.gpsLon      = gpsLastLon;
        telemetry.gpsAltitude = gpsLastAlt;
        telemetry.gpsSats     = gpsLastSats;
        telemetry.gpsStale    = true;

        bool lostTooLong = gpsHadFix
                        && (millis() - gpsLastFixTime > GPS_LOST_TIMEOUT_MS);
        if (lostTooLong) {
            static uint32_t lastWarnTime = 0;
            if (millis() - lastWarnTime > GPS_LOST_TIMEOUT_MS) {
                DEBUG_PRINT(F("WARN: GPS fix lost age="));
                DEBUG_PRINTLN(gps.location.age());
                lastWarnTime = millis();
            }
        }

        if (!gpsHadFix) {
            telemetry.gpsSats = (gps.satellites.isValid())
                                ? (uint8_t)gps.satellites.value() : 0;
        }
    }
}

// ================================================================
//  CALIBRATION
// ================================================================
void calibrateAltitudeZero() {
    if (!bmp.performReading()) {
        DEBUG_PRINTLN(F("SET_ZERO: BMP read failed — baseline unchanged"));
        return;
    }
    altitudeAtLaunch      = bmp.readAltitude(seaLevelPressureHPA);
    altitudeFiltered      = 0.0f;
    previousAltitude      = 0.0f;
    telemetry.maxAltitude = 0.0f;
    saveState();
    DEBUG_PRINT(F("SET_ZERO: baseline="));
    DEBUG_PRINTLN2(altitudeAtLaunch, 2);
}

// ================================================================
//  SENSOR READ
//  NOTE: MPU6500_WE has no live "checkConnection()" — the library
//  only reports health at init(). aImuHealthy is therefore latched
//  once in setup() and is not re-verified every cycle here.
// ================================================================
void readSensors() {
    // --- Barometer ---
    if (bmp.performReading()) {
        aBaroHealthy     = true;
        altitudeRaw      = bmp.readAltitude(seaLevelPressureHPA) - altitudeAtLaunch;
        altitudeFiltered = lowPass(altitudeRaw, altitudeFiltered, LOW_PASS_ALPHA);

        telemetry.altitude    = altitudeFiltered;
        telemetry.pressure    = bmp.pressure / 100.0f;
        telemetry.temperature = bmp.temperature;
    } else {
        aBaroHealthy = false;
    }

    // --- IMU: MPU6500_WE ---
    if (aImuHealthy) {
        xyzFloat gVal = mpu.getGValues();
        telemetry.accelX   = gVal.x * 9.81f;
        telemetry.accelY   = gVal.y * 9.81f;
        telemetry.accelZ   = gVal.z * 9.81f;
        telemetry.accelMag = sqrtf(gVal.x * gVal.x + gVal.y * gVal.y + gVal.z * gVal.z);

        xyzFloat angles = mpu.getAngles();
        telemetry.pitch = angles.x;
        telemetry.roll  = angles.y;
        telemetry.yaw   = angles.z;
    }
    // telemetry fields retain last good values if unhealthy — do not zero mid-flight.

    // --- Battery ---
    telemetry.batteryVoltage = readBatteryVoltage();
    telemetry.battLow        = (telemetry.batteryVoltage < BATT_LOW_THRESHOLD_V);
}

// ================================================================
//  TELEMETRY PACKET BUILD + CSV DEBUG OUTPUT
// ================================================================
void buildTelemetry() {
    telemetry.missionTime      = millis() - missionStartTime;
    telemetry.state            = (uint8_t)state;
    telemetry.launched         = launched;
    telemetry.apogee           = apogeeDetected;
    telemetry.separated        = separated;
    telemetry.landed           = landed;
    telemetry.controllerBAlive = ctrlB.alive;   // BUGFIX: was hardcoded false, a leftover
                                                 // from the single-controller merge. This is
                                                 // the dual-controller build now — report B's
                                                 // real, live status to the ground station/logs.

    // Only track peak altitude once actually launched. Tracking this on
    // the pad lets sensor warm-up transients or ground noise permanently
    // inflate maxAltitude, which would make detectApogee()'s drop check
    // fire falsely as soon as ASCENT begins.
    if (launched && altitudeFiltered > telemetry.maxAltitude) {
        telemetry.maxAltitude = altitudeFiltered;
    }

    telemetry.crc = calculateCRC(
        (const uint8_t*)&telemetry,
        sizeof(TelemetryPacket) - sizeof(uint16_t)
    );
}

void sendTelemetryCSV() {
    Serial.print(telemetry.missionTime);      Serial.print(',');
    Serial.print(telemetry.state);            Serial.print(',');
    Serial.print(telemetry.altitude, 2);      Serial.print(',');
    Serial.print(telemetry.maxAltitude, 2);   Serial.print(',');
    Serial.print(telemetry.accelMag, 3);      Serial.print(',');
    Serial.print(telemetry.pitch, 1);         Serial.print(',');
    Serial.print(telemetry.roll,  1);         Serial.print(',');
    Serial.print(telemetry.yaw,   1);         Serial.print(',');
    Serial.print(telemetry.temperature, 1);   Serial.print(',');
    Serial.print(telemetry.pressure, 1);      Serial.print(',');
    Serial.print(telemetry.batteryVoltage, 2);Serial.print(',');
    Serial.print(telemetry.gpsLat, 6);        Serial.print(',');
    Serial.print(telemetry.gpsLon, 6);        Serial.print(',');
    Serial.print(telemetry.gpsAltitude, 1);   Serial.print(',');
    Serial.print(telemetry.gpsSats);          Serial.print(',');
    Serial.print(telemetry.gpsStale);         Serial.print(',');
    Serial.print(telemetry.launched);         Serial.print(',');
    Serial.print(telemetry.apogee);           Serial.print(',');
    Serial.print(telemetry.separated);        Serial.print(',');
    Serial.print(telemetry.landed);           Serial.print(',');
    Serial.print(telemetry.controllerBAlive); Serial.print(',');
    Serial.print(telemetry.battLow);          Serial.print(',');
    Serial.println(telemetry.crc, HEX);
}
// ================================================================
//  CONTROLLER B — READ STATUS PACKET OVER I2C
//  Parses into ctrlB struct. All flight logic reads from ctrlB.
//  Raw packet buffer is never exposed outside this function.
// ================================================================
void readControllerB() {
    const size_t pktSize = sizeof(ControllerBPacket);
    uint8_t buf[pktSize];

    Wire.requestFrom(CONTROLLER_B_ADDR, (uint8_t)pktSize);

    if (Wire.available() < (int)pktSize) {
        if (millis() - ctrlB.lastHeartbeatTime > HEARTBEAT_TIMEOUT_MS) {
            if (ctrlB.alive) {
                DEBUG_PRINTLN(F("WARN: Controller B heartbeat lost"));
            }
            ctrlB.alive = false;
        }
        return;
    }

    for (size_t i = 0; i < pktSize; i++) {
        buf[i] = Wire.read();
    }

    ControllerBPacket* bPkt = (ControllerBPacket*)buf;

    // BUGFIX: was `pktSize - sizeof(uint16_t)`. ControllerBPacket contains a
    // float, so the compiler pads sizeof() up to a 4-byte multiple (16, not
    // the 14 meaningful bytes) — the padding lands AFTER the crc field.
    // "sizeof - 2" therefore still includes the crc field itself in the
    // hashed range instead of stopping right before it, which made every
    // single packet fail CRC. offsetof() is padding-agnostic and always
    // points at exactly where crc starts.
    uint16_t expectedCRC = calculateCRC(buf, offsetof(ControllerBPacket, crc));
    if (bPkt->crc != expectedCRC) {
        DEBUG_PRINTLN(F("WARN: B CRC mismatch"));
        return;
    }

    // Valid packet — populate status struct
    ctrlB.alive                = true;
    ctrlB.apogeeDetected       = bPkt->apogeeFlag;
    ctrlB.deploymentTriggered  = bPkt->deploymentTriggered;
    ctrlB.baroHealthy          = (bPkt->sensorHealth & 0x01) != 0;
    ctrlB.imuHealthy           = (bPkt->sensorHealth & 0x02) != 0;
    ctrlB.state                = bPkt->state;
    ctrlB.altitude             = bPkt->altitude;
    ctrlB.accelMag             = bPkt->accelMag;   // was received but previously discarded
    ctrlB.lastHeartbeatTime    = millis();
}

// ================================================================
//  REDUNDANCY HELPER — evaluates B's apogee signal trustworthiness
//
//  Returns true if B's apogee signal should be acted upon.
//  Conditions: B heartbeat healthy AND B sensor suite healthy
//  AND B has not already triggered its own deployment (dedup).
//
//  This is called from the ASCENT state in updateStateMachine()
//  and encapsulates all redundancy logic in one testable function.
// ================================================================
bool shouldTrustBApogee() {
    if (!ctrlB.alive)              return false;  // heartbeat gone
    if (!ctrlB.baroHealthy)        return false;  // B baro fault
    if (ctrlB.deploymentTriggered) return false;  // B already fired — don't double deploy
    return ctrlB.apogeeDetected;
}

// ================================================================
//  HEARTBEAT TRANSMIT — A broadcasts status to B over I2C
//  Uses Wire as master to write to a known B register address.
//  B listens as I2C slave and reads these fields to decide
//  whether to act autonomously.
//
//  Sent every 200ms — lightweight, non-blocking (I2C at 400kHz
//  takes ~200µs for this 12-byte packet).
// ================================================================
void sendHeartbeat() {
    HeartbeatPacket hb;
    hb.missionTime    = millis() - missionStartTime;
    hb.state          = (uint8_t)state;
    hb.launched       = launched;
    hb.apogeeDetected = apogeeDetected;
    hb.separated      = separated;
    // BUGFIX: was hardcoded `true` regardless of actual sensor state, so A
    // always told B it was 100% healthy even after a sensor failure. Use
    // the real live health flags instead.
    hb.baroHealthy    = aBaroHealthy;
    hb.imuHealthy     = aImuHealthy;
    // Arming was removed from A. The field stays in the I2C struct so the
    // wire layout shared with Controller B's code doesn't change; A is now
    // permanently "armed", so report true. Reconcile when B's code is updated.
    hb.armed          = true;

    // BUGFIX: same padding issue as ControllerBPacket — use offsetof(), not
    // sizeof()-2, so the hash range stops exactly before the crc field
    // regardless of any compiler-inserted padding.
    hb.crc = calculateCRC(
        (const uint8_t*)&hb,
        offsetof(HeartbeatPacket, crc)
    );

    Wire.beginTransmission(CONTROLLER_B_ADDR);
    Wire.write((const uint8_t*)&hb, sizeof(HeartbeatPacket));
    Wire.endTransmission();
}


// ================================================================
//  FLIGHT EVENT DETECTORS
// ================================================================
bool detectLaunch() {
    // BUGFIX: this was permanently hardcoded to `return true;`, with the
    // real threshold/hold-time logic commented out below it — meaning A
    // would "launch" on the very first loop iteration after ARM,
    // regardless of actual acceleration. Restored the real logic.
    static uint32_t launchStartTime = 0;

    if (telemetry.accelMag >= LAUNCH_ACCEL_THRESHOLD_G) {
        if (launchStartTime == 0) {
            launchStartTime = millis();
        }
        if (millis() - launchStartTime >= LAUNCH_CONFIRM_MS) {
            return true;
        }
    } else {
        launchStartTime = 0;
    }
    return false;
}


bool detectApogee() {
    uint32_t timeSinceLaunch = millis() - launchTime;

    if (timeSinceLaunch >= BACKUP_APOGEE_TIMEOUT_MS) {
        DEBUG_PRINTLN(F("APOGEE: backup timer fired"));
        return true;
    }

    if (!aBaroHealthy) {
        return false;
    }

    if (timeSinceLaunch < MIN_ASCENT_TIME_MS) {
        return false;
    }

    static uint8_t apogeeCount = 0;

    bool altDropping = (altitudeFiltered < (telemetry.maxAltitude - APOGEE_DROP_THRESHOLD_M));
    bool velNegative  = (verticalVelocity < APOGEE_NEG_VEL_THRESH);

    if (altDropping && velNegative) {
        apogeeCount++;
        if (apogeeCount >= APOGEE_CONFIRM_COUNT) {
            apogeeCount = 0;
            return true;
        }
    } else {
        apogeeCount = 0;
    }

    return false;
}

bool detectLanding() {
    static uint32_t landStartTime = 0;

    float absVel = fabsf(verticalVelocity);

    if (absVel < LANDING_VELOCITY_THRESH) {
        if (landStartTime == 0) {
            landStartTime = millis();
        }
        if (millis() - landStartTime >= LANDING_CONFIRM_MS) {
            return true;
        }
    } else {
        landStartTime = 0;
    }
    return false;
}

// ================================================================
//  DEPLOYMENT ACTIONS
// ================================================================
// -- low-level bay-door actuators, no gating --
// Each one stamps its servo's last-move time so manual commands can keep the
// two servos from moving together (supply limit ~2 A; an MG996R can draw
// ~2.5 A stalled).
uint32_t lastCansatMoveMs    = 0;   // 0 = never moved
uint32_t lastParachuteMoveMs = 0;

void openCansatBayDoor() {
    separationServo.write(SERVO_OPEN_POS);
    lastCansatMoveMs = millis();
}
void closeCansatBayDoor() {
    separationServo.write(SERVO_CLOSED_POS);
    lastCansatMoveMs = millis();
}
void openParachuteBayDoor() {
    parachuteServo.write(SERVO_OPEN_POS);
    lastParachuteMoveMs = millis();
}
void closeParachuteBayDoor() {
    parachuteServo.write(SERVO_CLOSED_POS);
    lastParachuteMoveMs = millis();
}

// ----------------------------------------------------------------
//  MANUAL SERVO COMMANDS (uplink): one command = one move, no timers.
//  OPEN/CLOSE are separate commands per door; the latch stays where it
//  was sent until the next command.
// ----------------------------------------------------------------
// Autonomous deployment order: CanSat door first, parachute door second, never
// both servos moving at once. The chute door is only commanded this long after
// the CanSat door.
#define CHUTE_DEPLOY_DELAY_MS  800  // TUNE on the bench: CanSat servo travel + settle + margin
uint32_t cansatReleaseMs = 0;       // millis() when the CanSat door was last commanded open

// A manual command for one door is refused while the OTHER door was moved
// less than this long ago (the operator just resends).
#define SERVO_STAGGER_MS  CHUTE_DEPLOY_DELAY_MS

static bool otherServoSettled(uint32_t otherLastMoveMs) {
    return otherLastMoveMs == 0 || (millis() - otherLastMoveMs) >= SERVO_STAGGER_MS;
}

// Manual CLOSE would re-latch a door the autonomous logic just opened, so it
// is only allowed on the pad or after landing. Manual OPEN works in any state
// (doubles as the last-resort manual release).
static bool manualCloseAllowed() {
    return state == LAUNCH_PAD || state == IMPACT;
}

// -- autonomous deployment path (apogee-triggered): open and STOP --
// No close movement.
void triggerSeparation() {
    DEBUG_PRINTLN(F("SEP: opening CanSat bay door servo"));
    openCansatBayDoor();
    cansatReleaseMs = millis();
    separated = true;
    scheduleBeep(500);
}

void deployParachute() {
    DEBUG_PRINTLN(F("CHUTE: opening parachute bay door servo"));
    openParachuteBayDoor();
    scheduleBeep(500);
}

// ================================================================
//  STATE MACHINE
// ================================================================
void updateStateMachine() {
    switch (state) {

        case BOOT:
            // Sensor init and self-check happen in setup().
            break;

        case LAUNCH_PAD:
            // Launch detection is always live from the moment the board is on
            // the pad (no arming step): >= LAUNCH_ACCEL_THRESHOLD_G held for
            // LAUNCH_CONFIRM_MS.
            if (detectLaunch()) {
                launched         = true;
                launchTime       = millis();
                launchElapsedMs  = launchTime - missionStartTime;
                altitudeAtLaunch = altitudeFiltered;
                telemetry.maxAltitude = 0.0f;   // fresh peak tracking starts exactly at launch
                if (benchMode) {                // never carry bench mode into flight: a mid-flight
                    benchMode = false;          // reset must RESUME the saved state, not wipe it
                    saveBenchFlag(false);
                }
                saveState();
                setState(ASCENT);
                scheduleBeep(100);
                scheduleBeep(100);
            }
            break;

        case ASCENT: {
            // PRIMARY: A's own apogee detection
            bool aApogee = detectApogee();

            // REDUNDANT: B's apogee signal — only if B is healthy and trusted
            // shouldTrustBApogee() encapsulates all B trustworthiness checks.
            bool bApogee = shouldTrustBApogee();
            if (aApogee || bApogee) {
                if (!aApogee && bApogee) {
                    DEBUG_PRINTLN(F("APOGEE: triggered by Controller B"));
                }
                apogeeDetected = true;
                apogeeTime     = millis();
                //added cansat separation here as it must be released first before deploying rocket parachute.
                triggerSeparation();
                setState(PAYLOAD_SEP);
                scheduleBeep(100);
                scheduleBeep(100);
            }
            break;
        }

        case PAYLOAD_SEP:
            if (!deploymentFired) {
                // Non-blocking stagger: the CanSat door was opened at apogee.
                // Stay in PAYLOAD_SEP (loop keeps running at full rate) until
                // its servo has finished moving, THEN drive the chute servo.
                if (millis() - cansatReleaseMs < CHUTE_DEPLOY_DELAY_MS) break;

                deploymentFired = true;
                saveState();
                deployParachute();
                scheduleBeep(100);
            }
            setState(DESCENT);
            break;

        case DESCENT:
            if (detectLanding()) {
                landed = true;
                saveState();
                setState(IMPACT);
                scheduleBeep(200);
            }
            break;

        case IMPACT: {
             // Locator beep every 5s — non-blocking
            static uint32_t lastBeepTime = 0;
            if (millis() - lastBeepTime > 5000) {
                scheduleBeep(50);
                lastBeepTime = millis();
                scheduleBeep(100);
            }
            break;
        }

        case SAFE_MODE:
            separationServo.write(SERVO_SAFE_POS);
            break;
    }
}

// ================================================================
//  LORA
// ================================================================
void initLoRa() {
    LoRa.setPins(LORA_CS_PIN, -1, LORA_IRQ_PIN);  // reset pin pulled high via resistor, no GPIO

    if (!LoRa.begin(LORA_FREQUENCY)) {
        DEBUG_PRINTLN(F("LoRa: init failed"));
        loraReady = false;
        return;
    }

    LoRa.setTxPower(LORA_TX_POWER_PAD);
    loraTxPowerNow = LORA_TX_POWER_PAD;
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();

    loraReady     = true;
    loraTxEnabled = false;
    DEBUG_PRINTLN(F("LoRa: ready"));
}

// Last-resort reset for lag on the pad. Gated to LAUNCH_PAD by the caller
// (CMD_RESET_MCU handler) — never honored once off the pad, since a
// mid-flight reset would drop RAM state (EEPROM resume covers a WATCHDOG
// timeout, not an intentional command reset chosen at the wrong moment).
void resetMicrocontroller() {
    Serial.println(F("CMD: RESET_MCU — resetting now"));
    delay(50);  // let the serial line flush
    watchdog_reboot(0, 0, 0);
    while (true) {}  // watchdog_reboot fires almost immediately; wait for it
}

void sendStatusPacket();  // fwd decl — used by processCommand below, defined near runSelfCheck()

void processCommand(uint8_t cmdId, uint8_t seq, const uint8_t* payload, uint8_t payloadLen) {
    if (seq == loraLastRxSeq) {
        DEBUG_PRINTLN(F("LoRa: dup cmd ignored"));
        return;
    }
    loraLastRxSeq = seq;

    switch (cmdId) {

        case CMD_START_TX:
            loraTxEnabled = true;
            DEBUG_PRINTLN(F("CMD: START_TX"));
            // Very first START_TX since boot: send the sensor-health/go-no-go
            // packet immediately, ahead of the next telemetry tick, so it is
            // always the first thing the GUI sees. A later stop/start cycle
            // skips straight to telemetry (health is then only sent on the
            // separate CMD_GET_STATUS request).
            if (firstTxSinceBoot) {
                sendStatusPacket();
                firstTxSinceBoot = false;
            }
            break;

        case CMD_STOP_TX:
            loraTxEnabled = false;
            DEBUG_PRINTLN(F("CMD: STOP_TX"));
            break;

        case CMD_SET_ZERO:
            calibrateAltitudeZero();
            DEBUG_PRINTLN(F("CMD: SET_ZERO"));
            break;

        case CMD_BEEP:
            scheduleBeep(1500);
            DEBUG_PRINTLN(F("CMD: BEEP"));
            break;

        case CMD_GET_STATUS:
            // On-demand, live sensor-health packet — works regardless of TX
            // state or flight state, and any number of times, so the operator
            // can re-check health at any point before giving go-for-launch.
            sendStatusPacket();
            DEBUG_PRINTLN(F("CMD: GET_STATUS"));
            break;

        case CMD_SET_SEALEVEL_HPA: {
            if (payloadLen == sizeof(float)) {
                float hpa;
                memcpy(&hpa, payload, sizeof(float));
                // Sanity-bound: reject garbage values, keep the fallback constant.
                if (hpa > 800.0f && hpa < 1100.0f) {
                    seaLevelPressureHPA = hpa;
                    DEBUG_PRINT(F("CMD: SET_SEALEVEL_HPA = "));
                    DEBUG_PRINTLN2(hpa, 2);
                } else {
                    DEBUG_PRINTLN(F("CMD: SET_SEALEVEL_HPA rejected — out of range, keeping fallback"));
                }
            } else {
                DEBUG_PRINTLN(F("CMD: SET_SEALEVEL_HPA rejected — bad payload length"));
            }
            break;
        }

        case CMD_SERVO_PARACHUTE_OPEN:
            if (!otherServoSettled(lastCansatMoveMs)) {
                DEBUG_PRINTLN(F("CMD: SERVO_PARACHUTE_OPEN refused — CanSat servo still moving, resend"));
                break;
            }
            openParachuteBayDoor();
            scheduleBeep(100);
            DEBUG_PRINTLN(F("CMD: SERVO_PARACHUTE_OPEN"));
            break;

        case CMD_SERVO_PAYLOAD_OPEN:
            if (!otherServoSettled(lastParachuteMoveMs)) {
                DEBUG_PRINTLN(F("CMD: SERVO_PAYLOAD_OPEN refused — parachute servo still moving, resend"));
                break;
            }
            openCansatBayDoor();
            scheduleBeep(100);
            DEBUG_PRINTLN(F("CMD: SERVO_PAYLOAD_OPEN"));
            break;

        case CMD_SERVO_PARACHUTE_CLOSE:
            if (!manualCloseAllowed()) {
                DEBUG_PRINTLN(F("CMD: SERVO_PARACHUTE_CLOSE refused — only on pad / after landing"));
                break;
            }
            if (!otherServoSettled(lastCansatMoveMs)) {
                DEBUG_PRINTLN(F("CMD: SERVO_PARACHUTE_CLOSE refused — CanSat servo still moving, resend"));
                break;
            }
            closeParachuteBayDoor();
            scheduleBeep(100);
            DEBUG_PRINTLN(F("CMD: SERVO_PARACHUTE_CLOSE"));
            break;

        case CMD_SERVO_PAYLOAD_CLOSE:
            if (!manualCloseAllowed()) {
                DEBUG_PRINTLN(F("CMD: SERVO_PAYLOAD_CLOSE refused — only on pad / after landing"));
                break;
            }
            if (!otherServoSettled(lastParachuteMoveMs)) {
                DEBUG_PRINTLN(F("CMD: SERVO_PAYLOAD_CLOSE refused — parachute servo still moving, resend"));
                break;
            }
            closeCansatBayDoor();
            scheduleBeep(100);
            DEBUG_PRINTLN(F("CMD: SERVO_PAYLOAD_CLOSE"));
            break;

        case CMD_SET_BENCH_MODE: {
            if (payloadLen != 1 || payload[0] > 1) {
                DEBUG_PRINTLN(F("CMD: SET_BENCH_MODE rejected — payload must be 1 byte, 0 or 1"));
                break;
            }
            bool want = (payload[0] == 1);
            // Turning bench ON in flight would make a mid-flight reset wipe the
            // saved flight state, so ON is only honored on the pad / after landing.
            if (want && !(state == LAUNCH_PAD || state == IMPACT)) {
                DEBUG_PRINTLN(F("CMD: SET_BENCH_MODE ON refused — only on pad / after landing"));
                break;
            }
            benchMode = want;
            saveBenchFlag(want);
            scheduleBeep(want ? 300 : 100);
            DEBUG_PRINT(F("CMD: SET_BENCH_MODE = "));
            DEBUG_PRINTLN(want ? F("ON (clean slate every boot)") : F("OFF (flight: resume after reset)"));
            sendStatusPacket();   // status frame carries the flag so the ground station can confirm it
            break;
        }

        case CMD_RESET_MCU:
            if (state == LAUNCH_PAD) {
                resetMicrocontroller();   // does not return
            } else {
                DEBUG_PRINTLN(F("CMD: RESET_MCU refused — only honored from LAUNCH_PAD"));
            }
            break;

        case CMD_ERASE_FLASH:
            if (state == LAUNCH_PAD) {
                eraseFlashForFlight();
                DEBUG_PRINTLN(F("CMD: ERASE_FLASH done"));
            } else {
                DEBUG_PRINTLN(F("CMD: ERASE_FLASH refused — not on pad"));
            }
            break;

        default:
            DEBUG_PRINT(F("CMD: unknown 0x"));
            DEBUG_PRINTLN2(cmdId, HEX);
            break;
    }
}

// Packs the working telemetry record + Controller B's forwarded values into
// the compact on-air struct and stamps its CRC.
void buildTelemetryWire(TelemetryWire& w) {
    w.missionTime    = telemetry.missionTime;
    w.altitude       = telemetry.altitude;
    w.maxAltitude    = telemetry.maxAltitude;
    w.accelX         = telemetry.accelX;
    w.accelY         = telemetry.accelY;
    w.accelZ         = telemetry.accelZ;
    w.accelMag       = telemetry.accelMag;
    w.temperature    = telemetry.temperature;
    w.pressure       = telemetry.pressure;
    w.pitch          = telemetry.pitch;
    w.roll           = telemetry.roll;
    w.yaw            = telemetry.yaw;
    w.batteryVoltage = telemetry.batteryVoltage;
    w.gpsLat         = telemetry.gpsLat;
    w.gpsLon         = telemetry.gpsLon;
    w.gpsAltitude    = telemetry.gpsAltitude;
    w.gpsSats        = telemetry.gpsSats;
    w.state          = telemetry.state;

    uint8_t f = 0;
    if (telemetry.launched)         f |= TLM_FLAG_LAUNCHED;
    if (telemetry.apogee)           f |= TLM_FLAG_APOGEE;
    if (telemetry.separated)        f |= TLM_FLAG_SEPARATED;
    if (telemetry.landed)           f |= TLM_FLAG_LANDED;
    if (telemetry.controllerBAlive) f |= TLM_FLAG_CTRLB_ALIVE;
    if (telemetry.battLow)          f |= TLM_FLAG_BATT_LOW;
    if (telemetry.gpsStale)         f |= TLM_FLAG_GPS_STALE;
    w.flags = f;

    w.bState     = ctrlB.state;
    w.bAltitude  = ctrlB.altitude;
    w.bAccelMag  = ctrlB.accelMag;

    w.crc = calculateCRC((const uint8_t*)&w, offsetof(TelemetryWire, crc));
}

// +10 dBm on the pad, +14 dBm once the rocket is in flight. Only reprogrammed
// between packets (never while the radio is transmitting).
void updateLoRaTxPower() {
    if (!loraReady) return;
    int8_t want = (state == ASCENT || state == PAYLOAD_SEP || state == DESCENT)
                    ? LORA_TX_POWER_FLIGHT : LORA_TX_POWER_PAD;
    if (want != loraTxPowerNow) {
        LoRa.setTxPower(want);
        loraTxPowerNow = want;
        DEBUG_PRINT(F("LoRa: TX power -> "));
        DEBUG_PRINTLN((int)want);
    }
}

void sendLoRaTelemetry() {
    if (!loraReady || !loraTxEnabled) return;

    TelemetryWire wire;
    buildTelemetryWire(wire);

    LoRa.beginPacket();
    LoRa.write(LORA_PKT_SYNC_0);
    LoRa.write(LORA_PKT_SYNC_1);
    LoRa.write(PKT_TELEMETRY);
    LoRa.write(loraTxSeq++);
    LoRa.write((const uint8_t*)&wire, sizeof(TelemetryWire));
    // NOTE: in this library endPacket(false) is the BLOCKING send (async=false).
    // It returns only when the ~257 ms frame is fully on air, so the radio is
    // idle again by the time we get here and the RX side can be armed at once.
    LoRa.endPacket(false);

    loraLastTxTime = millis();
}

void checkLoRaRx() {
    // No "recently transmitted" guard here. TX is blocking (endPacket(false)),
    // so whenever this runs the radio is not transmitting and parsePacket()
    // can safely arm single-RX. The old 250 ms guard was measured from the
    // END of the blocking TX, which kept the receiver off for the whole
    // 243 ms gap between telemetry frames, i.e. A never listened while
    // downlink was on.
    if (!loraReady) return;

    int packetSize = LoRa.parsePacket();
    if (packetSize < LORA_CMD_MIN_LEN) return;

    // Header first: sync0, sync1, cmdId, seq, len
    uint8_t hdr[LORA_CMD_HDR_LEN];
    for (uint8_t i = 0; i < LORA_CMD_HDR_LEN; i++) {
        if (!LoRa.available()) return;
        hdr[i] = LoRa.read();
    }

    if (hdr[0] != LORA_PKT_SYNC_0 || hdr[1] != LORA_PKT_SYNC_1) {
        DEBUG_PRINTLN(F("LoRa RX: bad sync"));
        return;
    }

    uint8_t cmdId = hdr[2];
    uint8_t seq   = hdr[3];
    uint8_t len   = hdr[4];

    if (len > LORA_CMD_MAX_PAYLOAD || packetSize != LORA_CMD_HDR_LEN + len + 2) {
        DEBUG_PRINTLN(F("LoRa RX: bad length"));
        return;
    }

    uint8_t payload[LORA_CMD_MAX_PAYLOAD];
    for (uint8_t i = 0; i < len; i++) {
        if (!LoRa.available()) return;
        payload[i] = LoRa.read();
    }

    uint8_t crcHi = LoRa.available() ? LoRa.read() : 0xFF;
    uint8_t crcLo = LoRa.available() ? LoRa.read() : 0xFF;
    uint16_t receivedCRC = ((uint16_t)crcHi << 8) | crcLo;

    // CRC covers header + payload together (build one contiguous buffer).
    uint8_t crcBuf[LORA_CMD_HDR_LEN + LORA_CMD_MAX_PAYLOAD];
    memcpy(crcBuf, hdr, LORA_CMD_HDR_LEN);
    memcpy(crcBuf + LORA_CMD_HDR_LEN, payload, len);
    uint16_t expectedCRC = calculateCRC(crcBuf, LORA_CMD_HDR_LEN + len);

    if (receivedCRC != expectedCRC) {
        DEBUG_PRINTLN(F("LoRa RX: CRC fail"));
        return;
    }

    DEBUG_PRINT(F("LoRa RX: cmd=0x"));
    DEBUG_PRINT2(cmdId, HEX);
    DEBUG_PRINT(F(" seq="));
    DEBUG_PRINT(seq);
    DEBUG_PRINT(F(" len="));
    DEBUG_PRINT(len);
    DEBUG_PRINT(F(" RSSI="));
    DEBUG_PRINTLN(LoRa.packetRssi());

    processCommand(cmdId, seq, payload, len);
}

void taskLoRaTxTick() {
    updateLoRaTxPower();
    sendLoRaTelemetry();
}

void taskLoRaRxTick() {
    checkLoRaRx();
}

// ================================================================
//  SENSOR-HEALTH / GO-NO-GO STATUS PACKET
//  buildStatusPacket() always reflects CURRENT live health (aBaroHealthy /
//  aImuHealthy / ctrlB.alive are updated continuously by readSensors() and
//  readControllerB()), not a boot-time snapshot — CMD_GET_STATUS can be
//  requested at any point and must show what's true right now.
// ================================================================
StatusPacket buildStatusPacket() {
    StatusPacket sp;
    sp.marker       = 0x55;
    sp.bmpOK        = aBaroHealthy;
    sp.bnoOK        = aImuHealthy;
    sp.sdOK         = sdReady;
    sp.flashOK      = flashReady;
    sp.loraOK       = loraReady;
    sp.gpsSearching = !gpsHadFix;       // live: true until GPS gets its first fix
    sp.ctrlBAlive   = ctrlB.alive;
    sp.battV        = readBatteryVoltage();
    sp.flightNumber = flightNumber;
    sp.benchMode    = benchMode;

    sp.crc = calculateCRC(
        (const uint8_t*)&sp,
        sizeof(StatusPacket) - sizeof(uint16_t)
    );
    return sp;
}

// Transmits the live status packet over LoRa, tagged PKT_STATUS. Called by
// CMD_GET_STATUS (any time) and once automatically on the first CMD_START_TX
// after boot. Never called from setup() directly — no packet goes on air
// until the GUI asks for one, per the plug-and-play/no-surprise-traffic rule.
void sendStatusPacket() {
    StatusPacket sp = buildStatusPacket();

    Serial.print(F("STATUS bmp="));   Serial.print(sp.bmpOK);
    Serial.print(F(" bno="));         Serial.print(sp.bnoOK);
    Serial.print(F(" sd="));          Serial.print(sp.sdOK);
    Serial.print(F(" flash="));       Serial.print(sp.flashOK);
    Serial.print(F(" lora="));        Serial.print(sp.loraOK);
    Serial.print(F(" ctrlB="));       Serial.print(sp.ctrlBAlive);
    Serial.print(F(" batt="));        Serial.print(sp.battV, 2);
    Serial.print(F(" flt#"));         Serial.println(sp.flightNumber);

    if (loraReady) {
        LoRa.beginPacket();
        LoRa.write(LORA_PKT_SYNC_0);
        LoRa.write(LORA_PKT_SYNC_1);
        LoRa.write(PKT_STATUS);
        LoRa.write((const uint8_t*)&sp, sizeof(StatusPacket));
        LoRa.endPacket(false);   // false = blocking (~103 ms), see sendLoRaTelemetry()
        loraLastTxTime = millis();
    }
}

// Boot-time GO/NO-GO verdict for the startup beep sequence ONLY — does NOT
// transmit (see sendStatusPacket() above for why). GO requires the two
// flight-critical sensors and the radio link; SD/flash logging failures are
// still reported (serial + status packet) but don't block the audible GO,
// since you can still fly and telemeter without local logging.
bool runSelfCheck() {
    StatusPacket sp = buildStatusPacket();

    Serial.print(F("SELFCHECK bmp="));   Serial.print(sp.bmpOK);
    Serial.print(F(" bno="));            Serial.print(sp.bnoOK);
    Serial.print(F(" sd="));             Serial.print(sp.sdOK);
    Serial.print(F(" flash="));          Serial.print(sp.flashOK);
    Serial.print(F(" lora="));           Serial.print(sp.loraOK);
    Serial.print(F(" ctrlB="));          Serial.print(sp.ctrlBAlive);
    Serial.print(F(" batt="));           Serial.print(sp.battV, 2);
    Serial.print(F(" flt#"));            Serial.println(sp.flightNumber);

    return sp.bmpOK && sp.bnoOK && sp.loraOK;
}
// ================================================================
//  HANDSHAKE / WAIT FOR CONTROLLER B
//  Polls for Controller B on I2C for up to CONTROLLER_B_WAIT_TIMEOUT_MS.
//  Found -> one short OK beep, proceed. Not found within the window ->
//  distinct timeout beep, proceed anyway with ctrlB.alive left false —
//  the plug-and-play requirement means a dead/absent B must not hang
//  the whole board on the pad. All downstream logic already treats
//  ctrlB.alive==false safely (shouldTrustBApogee(), telemetry, etc).
// ================================================================
#define CONTROLLER_B_WAIT_TIMEOUT_MS  30000

bool waitForControllerB() {
    DEBUG_PRINTLN(F("Waiting for Controller B to power up and respond on I2C..."));
    DEBUG_PRINT(F("  Using SDA="));  DEBUG_PRINT(I2C_B_SDA_PIN);
    DEBUG_PRINT(F(" SCL="));         DEBUG_PRINTLN(I2C_B_SCL_PIN);

    uint32_t startTime = millis();
    uint32_t attempts   = 0;

    while (millis() - startTime < CONTROLLER_B_WAIT_TIMEOUT_MS) {
        watchdog_update(); // Keep the watchdog happy while waiting

        Wire.beginTransmission(CONTROLLER_B_ADDR);
        uint8_t error = Wire.endTransmission();

        // Error code 0 indicates success (Controller B ACK'd its address)
        if (error == 0) {
            DEBUG_PRINTLN(F("Controller B detected! Proceeding with setup."));
            beepComponentOK();
            return true;
        }

        attempts++;
        // Print the actual Wire error code every ~1s so a real bus fault (as
        // opposed to a wiring/pin mistake) is visible instead of looking
        // like a hang. Error meanings (Arduino Wire): 1=data too long,
        // 2=NACK on address (nothing ACKed — most common: B not booted/not
        // wired), 3=NACK on data, 4=other/bus error.
        if (attempts % 10 == 0) {
            DEBUG_PRINT(F("  still waiting... Wire.endTransmission() err="));
            DEBUG_PRINTLN(error);
        }

        delay(100); // Poll every 100ms
    }

    DEBUG_PRINTLN(F("Controller B NOT FOUND within 30s — skipping, continuing single-controller."));
    beepControllerBTimeout();
    return false;
}
// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    // BUGFIX (FLIGHT-CRITICAL): was `while (!Serial);` with no timeout.
    // Serial only becomes true once a USB host enumerates the port — with
    // no USB connection in flight, this would hang forever, before the
    // watchdog is even armed later in setup(). Bounded to 3s so the board
    // still boots and flies normally when untethered.
    while (!Serial && millis() - t < 3000);

    Serial.println(F("AVIOPRO V0 — Controller A (Single-Controller Test Build)"));

    if (watchdog_caused_reboot()) {
        Serial.println(F("*** WATCHDOG RESET ***"));
    }

    EEPROM.begin(256);
    loadBenchFlag();
    bool resumed = loadState();

    pinMode(BUZZER_PIN,    OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    pinMode(BUZZER_PIN_actual, OUTPUT);   // was never configured — DC digitalWrite()
    digitalWrite(BUZZER_PIN_actual, LOW); // calls were also silently no-op'ing on it
    separationServo.attach(SERVO_SEP_PIN);
    separationServo.write(SERVO_SAFE_POS);      // CanSat bay door — start closed
    parachuteServo.attach(PARACHUTE_PIN);
    parachuteServo.write(SERVO_CLOSED_POS);     // Parachute bay door — start closed

    // --- Wire0: I2C Master link to Controller B ---
    // BUGFIX: pins must be set explicitly and BEFORE begin() — the old code
    // relied on the GPIO4/5 default, which the LoRa SPI0/CS setup below
    // silently steals later in this same function.
    Wire.setSDA(I2C_B_SDA_PIN);
    Wire.setSCL(I2C_B_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);
    Wire.setTimeout(I2C_TIMEOUT_MS);
    
    // Wait (up to 30s) for Controller B before continuing — plug-and-play
    // requirement: a missing/dead B must not hang the board indefinitely.
    bool controllerBFound = waitForControllerB();
    
    // --- Wire1: BMP390 + MPU6500 ---
    Wire1.setSDA(BMP_IMU_SDA);
    Wire1.setSCL(BMP_IMU_SCL);
    Wire1.begin();
    Wire1.setClock(400000);
    Wire1.setTimeout(I2C_TIMEOUT_MS);
    DEBUG_PRINTLN(F("Wire1 init done"));

    // --- SPI0: LoRa radio ---
    SPI.setRX(4);
    SPI.setTX(3);
    SPI.setSCK(2);
    SPI.begin();
    DEBUG_PRINTLN(F("SPI0 init done"));

    // --- BMP390 ---
    if (!bmp.begin_I2C(0x77, &Wire1)) {
        Serial.println(F("WARN: BMP3XX failed to initialize! Continuing degraded."));
        aBaroHealthy = false;
    } else {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
        DEBUG_PRINTLN(F("BMP390 OK — warming up IIR filter..."));

        // The BMP390's internal IIR filter takes several samples to settle
        // after being configured. Early readings can be wildly off (large
        // transient spike that decays over ~1-2s). Discard readings here
        // so nothing downstream (altitude zero calibration, maxAltitude
        // tracking) ever sees the garbage transient.
        for (uint8_t i = 0; i < 100; i++) {
            bmp.performReading();
            delay(20);   // matches 50Hz sensor task rate — 100 x 20ms = 2s total
        }
        DEBUG_PRINTLN(F("BMP390 filter settled"));
        aBaroHealthy = true;
    }
    aBaroHealthy ? beepComponentOK() : beepComponentFail();

    // --- MPU6500 ---
    if (!mpu.init()) {
        Serial.println(F("WARN: MPU6500 failed to initialize! Continuing degraded."));
        aImuHealthy = false;
    } else {
        DEBUG_PRINTLN(F("MPU6500 found"));
        mpu.setAccRange(MPU6500_ACC_RANGE_16G);
        mpu.setGyrRange(MPU6500_GYRO_RANGE_500);
        mpu.setAccDLPF(MPU6500_DLPF_6);
        mpu.setGyrDLPF(MPU6500_DLPF_6);
        mpu.setSampleRateDivider(0);
        aImuHealthy = true;
    }
    aImuHealthy ? beepComponentOK() : beepComponentFail();

    // GPS module isn't physically wired up yet — starting the UART always
    // "succeeds" regardless of whether a module is present, so it gets no
    // pass/fail beep of its own; skip straight to the components that have
    // a real health flag.
    initGPS();

    initFlash();
    flashReady ? beepComponentOK() : beepComponentFail();

    initSD();
    sdReady ? beepComponentOK() : beepComponentFail();

    initLoRa();
    loraReady ? beepComponentOK() : beepComponentFail();

    analogReadResolution(12);
    telemetry.gpsStale = true;
    missionStartTime    = millis();

    // Flight number: increment on fresh boot, keep on watchdog resume.
    {
        uint8_t storedFlight = 0;
        EEPROM.get(EEPROM_FLIGHTNUM_ADDR, storedFlight);
        if (!resumed) {
            flightNumber = storedFlight + 1;
            EEPROM.put(EEPROM_FLIGHTNUM_ADDR, flightNumber);
            EEPROM.commit();
        } else {
            flightNumber = storedFlight;
        }
    }

    // One sensor cycle so self-check has real numbers.
    readSensors();
    // One Controller B status read so self-check reports B's real state
    // instead of the ctrlB struct's unpopulated default.
    readControllerB();

    // Full-init verdict: 3 quick beeps = GO for GUI commands, one long
    // beep = NO-GO (check the status packet / serial log for what failed).
    // Nothing is transmitted over LoRa here — see sendStatusPacket().
    bool go = runSelfCheck();
    if (go) {
        beepStartupGo();
    } else {
        beepStartupNoGo();
    }
    (void)controllerBFound;  // informational only — B being absent doesn't block GO

    // Runtime bench mode (uplink-settable, stored in EEPROM): when ON, always
    // force a clean slate on boot, ignoring whatever flight state EEPROM
    // remembers from the last run (otherwise a test that reached IMPACT would
    // resume into IMPACT forever). When OFF (flight), a reset RESUMES the saved
    // state, which is the correct behaviour after an in-flight reboot.
    if (benchMode) {
        resumed = false;
        Serial.println(F("BENCH MODE ON — clean slate every boot"));
    } else {
        Serial.println(F("BENCH MODE OFF — flight behaviour, saved state resumes after a reset"));
    }

    if (!resumed) {
        clearSavedState();
        setState(LAUNCH_PAD);
        Serial.println(F("Fresh boot — on pad, launch detection live"));
    } else {
        if (launched && launchElapsedMs > 0) {
            launchTime = missionStartTime - launchElapsedMs;
        }

        Serial.print(F("Resumed state="));
        Serial.println((uint8_t)state);

        if (state == PAYLOAD_SEP && !deploymentFired) {
            Serial.println(F("Re-firing deployment"));
            // setup() just re-closed both doors, so re-open the CanSat door
            // now; the state machine fires the chute door CHUTE_DEPLOY_DELAY_MS
            // later (never both servos at once, even after a reboot).
            triggerSeparation();
        } else if (state == PAYLOAD_SEP && deploymentFired) {
            setState(DESCENT);
        }
    }

    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    Serial.println(F("Watchdog armed (4000ms)"));
}

// ================================================================
//  LOOP — multi-rate non-blocking task scheduler
//
//  Execution order (deterministic, drift-corrected):
//    1.  Watchdog pet
//    2.  Buzzer tick
//    3.  GPS UART drain      — 50Hz, prevents UART overflow
//    4.  Sensors             — 50Hz baro + IMU
//    5.  Velocity            — derived from fresh altitude
//    6.  State machine       — 50Hz flight logic
//    7.  Build + push        — 10Hz packet assembly + ring write
//    8.  SD flush            — ~2Hz, watchdog pet inside
//    9.  Flash flush         — ~2Hz, watchdog pet inside
//    10. LoRa TX 2Hz + RX poll 100Hz
//    11. GPS validate        — 5Hz fix quality check
//    12. Serial CSV          — 5Hz debug output
// ================================================================
void loop() {
    uint32_t now = millis();

    if (runTask(taskWatchdog, now)) {
        watchdog_update();
    }

    taskBuzzerTick(now);

    if (runTask(taskGPSDrain, now)) {
        drainGPS();
    }

    if (runTask(taskSensors, now)) {
        readSensors();
        float dt = taskSensors.intervalMs / 1000.0f;
        verticalVelocity = (altitudeFiltered - previousAltitude) / dt;
        previousAltitude = altitudeFiltered;
    }

    updateStateMachine();//state machine is not using the task method rn

    if (runTask(taskBuildTelem, now)) {
        buildTelemetry();
        pushToBuffer();
    }

    if (runTask(taskSDFlush, now)) {
        flushToSD();
    }

    if (runTask(taskFlashFlush, now)) {
        flushToFlash();
    }

    if (runTask(taskLoRaTx, now)) {
        taskLoRaTxTick();
    }

    if (runTask(taskLoRaRx, now)) {
        taskLoRaRxTick();
    }

    if (runTask(taskGPSValidate, now)) {
        validateGPS();
    }

    if (runTask(taskSerialCSV, now)) {
        sendTelemetryCSV();
    }

    //  Read Controller B status — 5Hz
    if (runTask(taskControllerB, now)) {
        readControllerB();
    }

    //Transmit A heartbeat to Controller B — 5Hz
    if (runTask(taskHeartbeat, now)) {
        sendHeartbeat();
    }
}
