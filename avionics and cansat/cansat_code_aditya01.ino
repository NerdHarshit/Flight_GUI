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
//      CMD_SERVO_PARACHUTE / CMD_SERVO_PAYLOAD (see below),
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
//    - Servo commands: one GUI click = one command = one non-blocking
//      0->90->0 latch cycle (SERVO_CYCLE_HOLD_MS). The autonomous apogee
//      path just opens and stops, and cancels any cycle in progress.
//    - On-air telemetry is now the packed 78-byte TelemetryWire: gyro
//      dropped (ground derives rates from pitch/roll/yaw), 7 flags in a
//      bitmask, and B's state / baro altitude / accel magnitude added.
//
//  CANSAT PASS (this revision):
//    - Controller B link removed (no I2C0, no 30s wait at boot). The
//      b* telemetry fields and the ctrlB status/CSV columns are kept
//      in place, always 0, so ground-station offsets don't move.
//    - Bay-door servos and CMD_SERVO_PARACHUTE / CMD_SERVO_PAYLOAD
//      removed. GP21 was being driven as a servo, but on this board it
//      is the /PYRO2 MOSFET gate — servo pulses there would have pulsed
//      the burn wire.
//    - Mission sequence: apogee -> gyro on (/PYRO3, GP20). Descent
//      through BURNWIRE_ALT_M AGL -> burn wire (/PYRO2, GP21) for
//      BURNWIRE_ON_MS, parafoil opens, PARAFOIL_SETTLE_MS later the
//      guidance loop takes over the two brake servos (GP19 / GP18).
//      Landing -> gyro off, servos neutral.
//    - Guidance: great-circle bearing to target vs GPS course over
//      ground, PD on the heading error with a Mamdani fuzzy gain
//      scheduler, differential-brake mixer, dead-reckoning fallback
//      on GPS loss. Target is hardcoded (TARGET_LAT/LON_DEG) and can
//      be overridden from the pad with CMD_SET_TARGET.
//    - GPS: 38400 baud (verified on this board) and set to 5Hz /
//      airborne <4g at boot via UBX-CFG-VALSET.
//    - Logging: guidance fields added to LogEntry, flash log region
//      grown to the full 2MB W25Q16, SD pre-allocation to 8MB, pad
//      logging decimated to 1Hz so the pad wait can't fill storage.
//    - BUGFIX: launch re-zero overwrote a SET_ZERO baseline with ~0,
//      so AGL jumped to MSL at launch. Now shifts the baseline instead.
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
//  BENCH_TEST_MODE — LAB TESTING ONLY. REMOVE BEFORE REAL FLIGHT.
//  1) Forces a clean slate on every boot (ignores any flight state
//     saved in EEPROM from the previous run).
//  2) Buzzer output is redirected to the onboard LED (GPIO26 here)
//     since no physical buzzer is wired up on the bench.
//  (There is no ARM command any more — launch detection is always live
//  from LAUNCH_PAD, so this flag no longer auto-arms anything.)
//  SAFETY: this does NOT disable deployment. If apogee is detected,
//  both bay-door servos still move for real. Disconnect the servo
//  linkage before running a full-sequence test on the bench.
//  !! For the competition build this flag must be OFF so the startup
//  !! beeps come out of the real buzzer — see notes in the changelog.
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
//  PIN MAP — matches LASC 2026 Avionics_PCB (board.h)
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
// #define LORA_RST_PIN 16  // LoRa reset — pulled high to 3.3V via 10k resistor, no GPIO needed
#define LORA_IRQ_PIN   16   // LoRa DIO0

#define BMP_IMU_SCL     7   // Wire1
#define BMP_IMU_SDA     6   // Wire1

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

// Parafoil brake-line servos (MG996R), powered from the +5V_SERVO buck.
#define SERVO_LEFT_PIN     19   // /SERVO1 -> J10 — left brake line
#define SERVO_RIGHT_PIN    18   // /SERVO2 -> J14 — right brake line

// Both pyro channels are IRLZ44N low-side switches (load between VBAT and
// drain, 10k gate pulldown), so a Hi-Z / reset GPIO leaves the FET off.
// GP20 and GP21 are the two halves of the same PWM slice, so they always
// share one PWM frequency (PYRO_PWM_FREQ_HZ).
#define PYRO_BURNWIRE_PIN  21   // /PYRO2 -> J11 — parafoil burn wire
#define PYRO_GYRO_PIN      20   // /PYRO3 -> J12 — gyro motor (battery powered, PWM gate)

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
#define GPS_BAUD_RATE       38400   // TBSM10 (u-blox M10) on this board — verified, NOT 9600
#define GPS_NAV_RATE_MS     200     // 5Hz fix rate (CFG-RATE-MEAS) — guidance runs off each fix
#define GPS_DYNMODEL        8       // CFG-NAVSPG-DYNMODEL: 8 = airborne <4g
#define GPS_MAX_FIX_AGE_MS  2000    // reject fixes older than 2s
#define GPS_MIN_SATS        4       // minimum satellites for trust
#define GPS_LOST_TIMEOUT_MS 10000   // log a warning after 10s without fix

// -- Logging --
#define LOG_RING_SIZE            64
#define LOG_SD_FLUSH_INTERVAL    450
#define LOG_FLASH_FLUSH_INTERVAL 550
#define LOG_SD_FAIL_LIMIT        20
#define LOG_PREALLOCATE_KB       8192
#define LOG_PAD_DECIMATION       10         // on LAUNCH_PAD log 1 in 10 records (1Hz) so the pad wait can't fill storage
#define FLASH_LOG_START_ADDR     0x001000
#define FLASH_LOG_END_ADDR       0x200000   // whole W25Q16 (2MB) — the populated part, not the W25Q32 on the BOM
#define FLASH_BLOCK_SIZE         0x10000    // 64KB block erase (0xD8)
#define FLASH_BLOCK_ERASE_MAX_MS 3000       // W25Q16 datasheet max is 2000ms
#define FLASH_MAGIC_ADDR         0x000000
#define FLASH_MAGIC_VAL          0xAF1010AF

// -- LoRa --
#define LORA_FREQUENCY         868E6
#define LORA_TX_POWER          14
#define LORA_SPREADING_FACTOR  7
#define LORA_BANDWIDTH         125E3
#define LORA_CODING_RATE       5
#define LORA_SYNC_WORD         0xAF
#define LORA_RX_WINDOW_MS      80
#define LORA_TX_INTERVAL_MS    100
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

// -- EEPROM --
#define EEPROM_MAGIC           0xA510DEAD
#define EEPROM_MAGIC_ADDR      0
#define EEPROM_STATE_ADDR      4
#define EEPROM_FLIGHTNUM_ADDR  64

// ================================================================
//  CANSAT MISSION TUNING — everything set before a flight lives here
// ================================================================
// -- Pyro PWM (both channels share one PWM slice -> one frequency) --
#define PYRO_PWM_FREQ_HZ        1000
#define PYRO_PWM_RANGE          255
#define BURNWIRE_DUTY           255      // of PYRO_PWM_RANGE — 255 = 100%
#define BURNWIRE_ON_MS          3000     // single attempt, no retry
#define BURNWIRE_ALT_M          600.0f   // AGL, only after apogee (on descent)
#define BURNWIRE_CONFIRM_COUNT  5        // consecutive fresh baro samples at/below — 100ms at 50Hz
#define GYRO_DUTY               255      // of PYRO_PWM_RANGE — 255 = 100%
#define PARAFOIL_SETTLE_MS      2000     // after the burn wire switches off, before steering starts

// -- Target (great-circle bearing / haversine distance) --
// PLACEHOLDER. 0,0 means "no target": guidance holds the servos neutral
// until a real target is set here or with CMD_SET_TARGET on the pad.
#define TARGET_LAT_DEG          0.0
#define TARGET_LON_DEG          0.0
#define EARTH_RADIUS_M          6371000.0

// -- PD core: u = (Kp0 + dKp) * e + (Kd0 + dKd) * edot --
// u is servo degrees of differential brake; e in deg, edot in deg/s.
#define GUIDANCE_PERIOD_MS      200      // 5Hz — matches GPS_NAV_RATE_MS
#define KP0                     2.5f
#define KD0                     0.1f
#define U_MAX_DEG               90.0f    // full brake. Kp0=2.5 saturates at ~36 deg of error
#define E_DEADBAND_DEG          5.0f     // P term ignores |e| below this
#define EDOT_FILTER_ALPHA       0.5f     // low-pass on the raw de/dt
#define COG_MIN_SPEED_MPS       1.5f     // COG is noise below this — hold last servo command

// -- Fuzzy gain scheduler (Mamdani, 5x5, centroid) --
// Inputs are normalised to [-1, 1] against these ranges (and clamped).
#define FUZZY_E_RANGE_DEG       90.0f
#define FUZZY_EDOT_RANGE_DPS    30.0f
// Output universe [-1, 1] is scaled to these. Because NB/PB are half
// triangles at the universe edges, the centroid tops out at ~0.83 of it.
#define FUZZY_DKP_MAX           1.0f     // +/-40% of KP0
#define FUZZY_DKD_MAX           0.05f    // +/-50% of KD0
#define FUZZY_OUT_STEPS         101      // centroid resolution

// -- Dead reckoning (GPS-loss fallback) --
#define DR_ENGAGE_MS            2000     // no valid fix for this long -> dead reckon
#define GLIDE_SPEED_H_MPS       6.0f     // assumed parafoil horizontal speed
#define GLIDE_SPEED_V_MPS       2.0f     // expected parafoil sink rate (descent-rate sanity flag)
#define DESCENT_FAST_FACTOR     2.0f     // flag if sink rate > factor * GLIDE_SPEED_V_MPS
// Heading rate = GYRO_YAW_SIGN * (gz - bias). Heading is clockwise from
// north; with the board flat and the MPU6500 face up, +gz is
// counter-clockwise, hence -1. VERIFY ON THE BENCH for the real mounting.
#define GYRO_YAW_SIGN           -1.0f
#define GYRO_BIAS_ALPHA         0.01f    // gz bias tracked on the pad (rocket still on the rail)

// -- Differential-brake mixer --
// u > 0 (target to the right) pulls the right line, u < 0 the left.
// Both start at 90 (neutral); left pulls towards 180, right is mirrored
// and pulls towards 0.
#define SERVO_NEUTRAL_DEG       90
#define SERVO_LEFT_DIR          (+1)
#define SERVO_RIGHT_DIR         (-1)

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
    // 0x0C-0x0D retired (CMD_SERVO_PARACHUTE / CMD_SERVO_PAYLOAD — no bay doors on the CanSat)
    // 0x0E-0x0F retired (separate open/close commands replaced by the single-cycle commands above)
    CMD_RESET_MCU        = 0x10,  // last-resort reset. Honored ONLY from LAUNCH_PAD (GUI confirms first)
    CMD_SET_TARGET       = 0x11,  // payload = int32 lat*1e7, int32 lon*1e7 (LE). Honored ONLY from LAUNCH_PAD
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
    bool battLow;

    // -- CanSat guidance --
    float gpsCourse;               // deg, course over ground
    float gpsSpeed;                // m/s, ground speed
    float yawRate;                 // deg/s, bias-corrected heading rate from the gyro
    float descentRate;             // m/s, positive = sinking
    uint8_t guideFlags;            // GUIDE_FLAG_* bitmask

    uint16_t crc;                  // CRC-16/CCITT over all preceding bytes
};

// ----------------------------------------------------------------
//  ON-AIR TELEMETRY PACKET (PKT_TELEMETRY payload) — compact, packed.
//  (Original 78-byte layout; the CanSat build appends to it — see below.)
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
#define TLM_FLAG_CTRLB_ALIVE  (1u << 4)   // no Controller B on the CanSat — always 0
#define TLM_FLAG_BATT_LOW     (1u << 5)
#define TLM_FLAG_GPS_STALE    (1u << 6)
// bit 7 reserved, always 0

// CanSat guidance flags — `guideFlags` in TelemetryWire / LogEntry.
#define GUIDE_FLAG_GYRO_ON        (1u << 0)   // /PYRO3 gyro gate driven
#define GUIDE_FLAG_BURNWIRE_FIRED (1u << 1)   // burn wire has been fired (latched)
#define GUIDE_FLAG_BURNWIRE_ON    (1u << 2)   // burn wire gate driven right now
#define GUIDE_FLAG_ACTIVE         (1u << 3)   // guidance is steering the servos
#define GUIDE_FLAG_DEAD_RECKON    (1u << 4)   // position/heading from dead reckoning, not GPS
#define GUIDE_FLAG_TARGET_SET     (1u << 5)   // a real (non-placeholder) target is loaded
#define GUIDE_FLAG_LOW_SPEED_HOLD (1u << 6)   // ground speed < COG_MIN_SPEED_MPS, servos holding
#define GUIDE_FLAG_DESCENT_FAST   (1u << 7)   // sinking faster than DESCENT_FAST_FACTOR * GLIDE_SPEED_V_MPS

//  Frame: [0xA5][0x5A][0xA2][seq][ TelemetryWire, 99 bytes ]
//  CanSat fields are appended after bAccelMag, so every older offset is
//  unchanged; only the crc moved (now covers the first 97 bytes).
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
    float    gpsCourse;         // deg, course over ground
    float    gpsSpeed;          // m/s, ground speed
    float    headingError;      // deg, e = wrap180(bearing - COG)
    float    guidanceU;         // deg, PD output (servo degrees of differential brake)
    float    targetDistance;    // m, haversine distance to target
    uint8_t  guideFlags;        // GUIDE_FLAG_* bitmask
    uint16_t crc;
};
static_assert(sizeof(TelemetryWire) == 99, "TelemetryWire must be exactly 99 bytes — ground station depends on it");

struct PersistentState {
    FlightState state;
    float       maxAltitude;
    float       altitudeAtLaunch;
    uint32_t    launchElapsedMs;
    bool        launched;
    bool        apogeeDetected;
    bool        separated;
    bool        landed;
    bool        deploymentFired;    // apogee action (gyro on) done
    bool        burnWireFired;      // burn wire fired — never re-fired after a reset
    bool        targetSet;
    int32_t     targetLatE7;        // so a LoRa-set target survives an in-flight reset
    int32_t     targetLonE7;
    uint16_t    crc;
};
static_assert(EEPROM_STATE_ADDR + sizeof(PersistentState) <= EEPROM_FLIGHTNUM_ADDR,
              "PersistentState overlaps the flight-number slot");

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
    uint16_t crc;
};

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
    // -- CanSat guidance --
    double   navLat;           // position guidance used (GPS, or dead-reckoned)
    double   navLon;
    float    gpsCourse;        // deg
    float    gpsSpeed;         // m/s
    float    navHeading;       // deg, COG or dead-reckoned heading
    float    yawRate;          // deg/s
    float    bearing;          // deg to target
    float    targetDistance;   // m
    float    headingError;     // e, deg
    float    headingErrorRate; // edot, deg/s (filtered)
    float    kpEff;
    float    kdEff;
    float    guidanceU;        // deg
    uint8_t  servoLeft;        // commanded angle, deg
    uint8_t  servoRight;
    uint8_t  guideFlags;       // GUIDE_FLAG_* bitmask
};  // layout is the struct's natural (unpacked) one — decode with sizeof(LogEntry)

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
bool deploymentFired = false;   // latching guard — apogee action (gyro on) executes exactly once
bool burnWireFired   = false;   // latching guard — burn wire fires exactly once, never after a reset

// -- pyro channels --
bool     gyroOn            = false;
bool     burnWireOn        = false;
uint32_t burnWireStartMs   = 0;
uint32_t guidanceArmMs     = 0;    // millis() at which steering may start (0 = not armed yet)
uint8_t  burnWireAltCount  = 0;    // consecutive fresh baro samples at/below BURNWIRE_ALT_M
bool     baroSampleFresh   = false; // set by readSensors() on each new baro conversion

// -- IMU yaw rate (for dead-reckoned heading) --
float    gyroZBias         = 0.0f;  // deg/s, learned on the pad
float    yawRateDps        = 0.0f;  // GYRO_YAW_SIGN * (gz - bias)
float    yawIntegralDeg    = 0.0f;  // accumulated by the 50Hz sensor task, consumed by guidance

// -- GPS --
float    gpsLastLat     = 0.0f;
float    gpsLastLon     = 0.0f;
float    gpsLastAlt     = 0.0f;
uint8_t  gpsLastSats    = 0;
bool     gpsHadFix      = false;
uint32_t gpsLastFixTime = 0;
double   gpsLatD        = 0.0;     // full-precision copies for guidance
double   gpsLonD        = 0.0;
uint32_t gpsEpochMs     = 0;       // millis() the current fix was produced (millis() - age)
bool     gpsCourseValid = false;
float    gpsCourseDeg   = 0.0f;
float    gpsSpeedMps    = 0.0f;
TinyGPSPlus gps;

// -- guidance --
struct Guidance {
    bool     active;          // steering the servos
    bool     deadReckoning;
    bool     lowSpeedHold;
    bool     targetSet;
    double   targetLat;
    double   targetLon;
    double   navLat;          // GPS or dead-reckoned position
    double   navLon;
    float    navHeading;      // deg, COG or dead-reckoned
    bool     headingValid;    // at least one usable COG seen
    float    bearing;
    float    distance;
    float    e;
    float    ePrev;
    bool     ePrevValid;
    float    eDot;
    float    kpEff;
    float    kdEff;
    float    u;
    uint8_t  servoLeft;
    uint8_t  servoRight;
    uint32_t lastTickMs;
    uint32_t lastEpochMs;     // GPS epoch the last update used
    float    descentRate;     // m/s, positive = sinking
    float    lastAltitude;
};
Guidance guide = {};

// -- LoRa --
bool     loraReady      = false;
bool     loraTxEnabled  = false;
uint8_t  loraTxSeq      = 0;
uint8_t  loraLastRxSeq  = 0xFF;
uint32_t loraLastTxTime = 0;

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
Servo servoLeft;         // left brake line
Servo servoRight;        // right brake line

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
Task taskLoRa         = { 0, 100 };   //  10Hz LoRa TX + RX window
Task taskGuidance     = { 0, GUIDANCE_PERIOD_MS };   //   5Hz guidance loop
uint8_t padLogCounter = 0;            // LOG_PAD_DECIMATION counter

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
    ps.burnWireFired     = burnWireFired;
    ps.targetSet         = guide.targetSet;
    ps.targetLatE7       = (int32_t)lround(guide.targetLat * 1e7);
    ps.targetLonE7       = (int32_t)lround(guide.targetLon * 1e7);

    // offsetof, not sizeof()-2: the struct has trailing padding after crc,
    // so sizeof()-2 would hash part of the crc field itself.
    ps.crc = calculateCRC(
        (const uint8_t*)&ps,
        offsetof(PersistentState, crc)
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
        offsetof(PersistentState, crc)
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
    burnWireFired     = ps.burnWireFired;

    // A LoRa-set target is only carried across an IN-FLIGHT reset. On the
    // pad the hardcoded target is used, so a stale EEPROM value can never
    // silently override a reflashed TARGET_LAT/LON_DEG.
    if (ps.targetSet && ps.state != LAUNCH_PAD) {
        guide.targetSet = true;
        guide.targetLat = ps.targetLatE7 / 1e7;
        guide.targetLon = ps.targetLonE7 / 1e7;
    }

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
void flashWaitReady(uint32_t timeoutMs = 100) {
    SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(FLASH_CS_PIN, LOW);
    SPI1.transfer(0x05); // Read Status Register 1

    uint32_t startCheck = millis();
    while (SPI1.transfer(0x00) & 0x01) {
        watchdog_update();
        if (millis() - startCheck > timeoutMs) {
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

void rawFlashEraseBlock64(uint32_t addr) {
    flashWriteEnable();
    SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(FLASH_CS_PIN, LOW);
    SPI1.transfer(0xD8);  // Block Erase (64KB)
    SPI1.transfer((addr >> 16) & 0xFF);
    SPI1.transfer((addr >> 8) & 0xFF);
    SPI1.transfer(addr & 0xFF);
    digitalWrite(FLASH_CS_PIN, HIGH);
    SPI1.endTransaction();
    flashWaitReady(FLASH_BLOCK_ERASE_MAX_MS);
}

// 64KB block erases — 32 of them for 2MB, a few seconds total, instead of
// 512 sector erases. flashWaitReady() pets the watchdog while it waits.
void flashEraseLogRegion() {
    for (uint32_t a = 0; a < FLASH_LOG_END_ADDR; a += FLASH_BLOCK_SIZE) {
        rawFlashEraseBlock64(a);
    }
}

bool flashSlotErased(uint32_t addr) {
    uint8_t buf[sizeof(LogEntry)];
    rawFlashRead(addr, buf, sizeof(buf));
    for (size_t i = 0; i < sizeof(buf); i++) {
        if (buf[i] != 0xFF) return false;
    }
    return true;
}

// The log is a written prefix followed by erased slots, so binary-search
// for the first erased slot. A linear scan of 2MB would hold up a
// mid-flight watchdog resume for seconds.
uint32_t flashScanForEnd() {
    uint32_t lo = 0;
    uint32_t hi = (FLASH_LOG_END_ADDR - FLASH_LOG_START_ADDR) / sizeof(LogEntry);
    while (lo < hi) {
        uint32_t mid = lo + (hi - lo) / 2;
        if (flashSlotErased(FLASH_LOG_START_ADDR + mid * sizeof(LogEntry))) {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    return FLASH_LOG_START_ADDR + lo * sizeof(LogEntry);
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

    entry.navLat           = guide.navLat;
    entry.navLon           = guide.navLon;
    entry.gpsCourse        = telemetry.gpsCourse;
    entry.gpsSpeed         = telemetry.gpsSpeed;
    entry.navHeading       = guide.navHeading;
    entry.yawRate          = telemetry.yawRate;
    entry.bearing          = guide.bearing;
    entry.targetDistance   = guide.distance;
    entry.headingError     = guide.e;
    entry.headingErrorRate = guide.eDot;
    entry.kpEff            = guide.kpEff;
    entry.kdEff            = guide.kdEff;
    entry.guidanceU        = guide.u;
    entry.servoLeft        = guide.servoLeft;
    entry.servoRight       = guide.servoRight;
    entry.guideFlags       = telemetry.guideFlags;

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
// UBX-CFG-VALSET (0x06 0x8A), RAM layer only — the M10 dropped CFG-NAV5 /
// CFG-RATE. Same frames as Avio_Flight. Best effort: a wrong key is NAK'd
// and the module carries on at its defaults.
void ubxValset(const uint8_t* kv, uint8_t kvLen) {
    uint8_t frame[32];
    uint8_t n = 0;

    frame[n++] = 0xB5; frame[n++] = 0x62;   // sync
    frame[n++] = 0x06; frame[n++] = 0x8A;   // CFG-VALSET
    frame[n++] = 4 + kvLen; frame[n++] = 0x00;
    frame[n++] = 0x00;                      // version
    frame[n++] = 0x01;                      // layer: RAM
    frame[n++] = 0x00; frame[n++] = 0x00;   // reserved
    for (uint8_t i = 0; i < kvLen; i++) frame[n++] = kv[i];

    uint8_t ckA = 0, ckB = 0;
    for (uint8_t i = 2; i < n; i++) { ckA += frame[i]; ckB += ckA; }
    frame[n++] = ckA; frame[n++] = ckB;

    Serial1.write(frame, n);
    Serial1.flush();
}

void ubxSetU1(uint32_t key, uint8_t val) {
    uint8_t kv[5] = { (uint8_t)key, (uint8_t)(key >> 8),
                      (uint8_t)(key >> 16), (uint8_t)(key >> 24), val };
    ubxValset(kv, sizeof(kv));
}

void ubxSetU2(uint32_t key, uint16_t val) {
    uint8_t kv[6] = { (uint8_t)key, (uint8_t)(key >> 8),
                      (uint8_t)(key >> 16), (uint8_t)(key >> 24),
                      (uint8_t)val, (uint8_t)(val >> 8) };
    ubxValset(kv, sizeof(kv));
}

void initGPS() {
    Serial1.setTX(GPS_TX);
    Serial1.setRX(GPS_RX);
    Serial1.begin(GPS_BAUD_RATE);
    delay(100);   // blocking settle — acceptable in setup()

    ubxSetU2(0x30210001, GPS_NAV_RATE_MS);  // CFG-RATE-MEAS
    ubxSetU1(0x20110021, GPS_DYNMODEL);     // CFG-NAVSPG-DYNMODEL
    // Only GGA (fix/sats/alt) and RMC (course + speed) — at 5Hz the default
    // sentence set doesn't fit in 38400 baud.
    ubxSetU1(0x209100bb, 1);                // GGA on
    ubxSetU1(0x209100ac, 1);                // RMC on
    ubxSetU1(0x209100ca, 0);                // GLL off
    ubxSetU1(0x209100c0, 0);                // GSA off
    ubxSetU1(0x209100c5, 0);                // GSV off
    ubxSetU1(0x209100b1, 0);                // VTG off
    delay(100);
    DEBUG_PRINTLN(F("GPS UART started, 5Hz airborne config sent"));
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
        gpsEpochMs     = millis() - gps.location.age();
        gpsLatD        = gps.location.lat();
        gpsLonD        = gps.location.lng();
        gpsLastLat     = (float)gpsLatD;
        gpsLastLon     = (float)gpsLonD;
        gpsLastSats    = (uint8_t)gps.satellites.value();
        gpsHadFix      = true;
        gpsLastFixTime = millis();

        gpsCourseValid = gps.course.isValid() && gps.speed.isValid()
                      && (gps.course.age() < GPS_MAX_FIX_AGE_MS);
        if (gpsCourseValid) {
            gpsCourseDeg = (float)gps.course.deg();
            gpsSpeedMps  = (float)gps.speed.mps();
        }
        telemetry.gpsCourse = gpsCourseDeg;
        telemetry.gpsSpeed  = gpsSpeedMps;

        if (gps.altitude.isValid()) {
            gpsLastAlt = (float)gps.altitude.meters();
        }

        telemetry.gpsLat      = gpsLastLat;
        telemetry.gpsLon      = gpsLastLon;
        telemetry.gpsAltitude = gpsLastAlt;
        telemetry.gpsSats     = gpsLastSats;
        telemetry.gpsStale    = false;

    } else {
        gpsCourseValid        = false;
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
        baroSampleFresh  = true;
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

        // Heading rate for the dead-reckoned heading. The gz bias is learned
        // while the rocket sits on the rail and frozen from launch on.
        xyzFloat gyr = mpu.getGyrValues();
        if (state == LAUNCH_PAD) {
            gyroZBias = lowPass(gyr.z, gyroZBias, GYRO_BIAS_ALPHA);
        }
        yawRateDps      = GYRO_YAW_SIGN * (gyr.z - gyroZBias);
        yawIntegralDeg += yawRateDps * (taskSensors.intervalMs / 1000.0f);
        telemetry.yawRate = yawRateDps;
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
    telemetry.descentRate      = guide.descentRate;

    uint8_t g = 0;
    if (gyroOn)                g |= GUIDE_FLAG_GYRO_ON;
    if (burnWireFired)         g |= GUIDE_FLAG_BURNWIRE_FIRED;
    if (burnWireOn)            g |= GUIDE_FLAG_BURNWIRE_ON;
    if (guide.active)          g |= GUIDE_FLAG_ACTIVE;
    if (guide.deadReckoning)   g |= GUIDE_FLAG_DEAD_RECKON;
    if (guide.targetSet)       g |= GUIDE_FLAG_TARGET_SET;
    if (guide.lowSpeedHold)    g |= GUIDE_FLAG_LOW_SPEED_HOLD;
    if (burnWireFired && guide.descentRate > DESCENT_FAST_FACTOR * GLIDE_SPEED_V_MPS) {
        g |= GUIDE_FLAG_DESCENT_FAST;
    }
    telemetry.guideFlags = g;

    // Only track peak altitude once actually launched. Tracking this on
    // the pad lets sensor warm-up transients or ground noise permanently
    // inflate maxAltitude, which would make detectApogee()'s drop check
    // fire falsely as soon as ASCENT begins.
    if (launched && altitudeFiltered > telemetry.maxAltitude) {
        telemetry.maxAltitude = altitudeFiltered;
    }

    telemetry.crc = calculateCRC(
        (const uint8_t*)&telemetry,
        offsetof(TelemetryPacket, crc)
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
    Serial.print(0);                          Serial.print(',');   // was controllerBAlive — column kept
    Serial.print(telemetry.battLow);          Serial.print(',');
    Serial.print(telemetry.crc, HEX);         Serial.print(',');
    // -- CanSat guidance columns (appended) --
    Serial.print(telemetry.guideFlags, HEX);  Serial.print(',');
    Serial.print(telemetry.gpsCourse, 1);     Serial.print(',');
    Serial.print(telemetry.gpsSpeed, 2);      Serial.print(',');
    Serial.print(guide.bearing, 1);           Serial.print(',');
    Serial.print(guide.distance, 1);          Serial.print(',');
    Serial.print(guide.e, 1);                 Serial.print(',');
    Serial.print(guide.eDot, 2);              Serial.print(',');
    Serial.print(guide.u, 1);                 Serial.print(',');
    Serial.print(guide.servoLeft);            Serial.print(',');
    Serial.println(guide.servoRight);
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
//  PYRO CHANNELS — burn wire (/PYRO2, GP21) and gyro (/PYRO3, GP20)
// ================================================================
// duty 0 parks the pin as a plain LOW output (FET off). Anything else is
// PWM at PYRO_PWM_FREQ_HZ, duty out of PYRO_PWM_RANGE.
void pyroWrite(uint8_t pin, uint16_t duty) {
    if (duty == 0) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    } else {
        analogWrite(pin, duty);
    }
}

void initPyros() {
    pyroWrite(PYRO_BURNWIRE_PIN, 0);
    pyroWrite(PYRO_GYRO_PIN, 0);
    analogWriteFreq(PYRO_PWM_FREQ_HZ);
    analogWriteRange(PYRO_PWM_RANGE);
}

void startGyro() {
    DEBUG_PRINTLN(F("GYRO: on"));
    pyroWrite(PYRO_GYRO_PIN, GYRO_DUTY);
    gyroOn = true;
}

void stopGyro() {
    if (gyroOn) {
        DEBUG_PRINTLN(F("GYRO: off"));
    }
    pyroWrite(PYRO_GYRO_PIN, 0);
    gyroOn = false;
}

// Latches and saves burnWireFired BEFORE energising the wire, so a reset
// mid-burn can never fire it a second time.
void fireBurnWire() {
    if (burnWireFired) return;
    burnWireFired = true;
    saveState();

    DEBUG_PRINTLN(F("BURNWIRE: on"));
    pyroWrite(PYRO_BURNWIRE_PIN, BURNWIRE_DUTY);
    burnWireOn      = true;
    burnWireStartMs = millis();
    scheduleBeep(300);
}

// Non-blocking: switches the burn wire off after BURNWIRE_ON_MS and arms
// guidance PARAFOIL_SETTLE_MS after that, once the parafoil has inflated.
void updateBurnWire(uint32_t now) {
    if (burnWireOn && now - burnWireStartMs >= BURNWIRE_ON_MS) {
        pyroWrite(PYRO_BURNWIRE_PIN, 0);
        burnWireOn    = false;
        guidanceArmMs = now + PARAFOIL_SETTLE_MS;
        DEBUG_PRINTLN(F("BURNWIRE: off — parafoil settling"));
    }
}

// ================================================================
//  DIFFERENTIAL-BRAKE SERVO MIXER
//  u > 0 (target to the right) pulls the right line by |u| degrees,
//  u < 0 pulls the left. The other line stays at neutral.
// ================================================================
void writeServos(float u) {
    float mag = fabsf(u);
    if (mag > U_MAX_DEG) mag = U_MAX_DEG;

    int left  = SERVO_NEUTRAL_DEG;
    int right = SERVO_NEUTRAL_DEG;
    if (u > 0.0f) {
        right += SERVO_RIGHT_DIR * (int)lroundf(mag);
    } else if (u < 0.0f) {
        left  += SERVO_LEFT_DIR  * (int)lroundf(mag);
    }
    left  = constrain(left,  0, 180);
    right = constrain(right, 0, 180);

    servoLeft.write(left);
    servoRight.write(right);
    guide.servoLeft  = (uint8_t)left;
    guide.servoRight = (uint8_t)right;
}

void servosNeutral() {
    guide.u = 0.0f;
    writeServos(0.0f);
}

// ================================================================
//  GUIDANCE — geometry
// ================================================================
float wrap360(float a) {
    a = fmodf(a, 360.0f);
    return (a < 0.0f) ? a + 360.0f : a;
}

float wrap180(float a) {
    return wrap360(a + 180.0f) - 180.0f;
}

// 1) Initial great-circle bearing from (lat1, lon1) to (lat2, lon2), deg [0, 360).
float bearingTo(double lat1, double lon1, double lat2, double lon2) {
    double p1 = radians(lat1);
    double p2 = radians(lat2);
    double dl = radians(lon2 - lon1);
    double y  = sin(dl) * cos(p2);
    double x  = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    return wrap360((float)degrees(atan2(y, x)));
}

// 2) Haversine ground distance, m.
float distanceTo(double lat1, double lon1, double lat2, double lon2) {
    double p1 = radians(lat1);
    double p2 = radians(lat2);
    double dp = radians(lat2 - lat1);
    double dl = radians(lon2 - lon1);
    double a  = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    return (float)(2.0 * EARTH_RADIUS_M * atan2(sqrt(a), sqrt(1.0 - a)));
}

// 0,0 is the placeholder — "no target", guidance holds neutral.
void setTarget(double lat, double lon) {
    guide.targetLat = lat;
    guide.targetLon = lon;
    guide.targetSet = !(lat == 0.0 && lon == 0.0);
}

// ================================================================
//  GUIDANCE — fuzzy gain scheduler (Mamdani, 5x5, centroid)
//  Inputs e and edot are normalised to [-1, 1]. Five triangular sets
//  centred at -1, -0.5, 0, 0.5, 1 (half-width 0.5) on both inputs and
//  outputs. Rule strength = min(mu_e, mu_edot); each output set is
//  clipped at its strongest rule; centroid of the max-aggregate.
//
//  dKp: error growing (e and edot same sign) -> raise Kp; error already
//       shrinking -> cut Kp to avoid overshoot.
//  dKd: small error but fast-changing -> raise Kd to damp; large error
//       -> lower Kd so the turn isn't held back.
// ================================================================
enum FuzzySet : uint8_t { FZ_NB = 0, FZ_NS, FZ_ZE, FZ_PS, FZ_PB };

// Rows: e (NB..PB). Columns: edot (NB..PB).
static const uint8_t FUZZY_DKP_RULES[5][5] = {
    { FZ_PB, FZ_PB, FZ_PB, FZ_PS, FZ_ZE },
    { FZ_PS, FZ_PS, FZ_PS, FZ_ZE, FZ_NS },
    { FZ_NS, FZ_ZE, FZ_ZE, FZ_ZE, FZ_NS },
    { FZ_NS, FZ_ZE, FZ_PS, FZ_PS, FZ_PS },
    { FZ_ZE, FZ_PS, FZ_PB, FZ_PB, FZ_PB },
};

static const uint8_t FUZZY_DKD_RULES[5][5] = {
    { FZ_NB, FZ_NS, FZ_NS, FZ_NS, FZ_ZE },
    { FZ_ZE, FZ_ZE, FZ_NS, FZ_ZE, FZ_PS },
    { FZ_PB, FZ_PS, FZ_ZE, FZ_PS, FZ_PB },
    { FZ_PS, FZ_ZE, FZ_NS, FZ_ZE, FZ_ZE },
    { FZ_ZE, FZ_NS, FZ_NS, FZ_NS, FZ_NB },
};

float fuzzyTriangle(float x, uint8_t set) {
    float c = -1.0f + 0.5f * set;
    float m = 1.0f - fabsf(x - c) / 0.5f;
    return (m > 0.0f) ? m : 0.0f;
}

// Returns the defuzzified output in [-1, 1].
float fuzzyInfer(const uint8_t rules[5][5], const float muE[5], const float muD[5]) {
    float clip[5] = { 0 };
    for (uint8_t i = 0; i < 5; i++) {
        for (uint8_t j = 0; j < 5; j++) {
            float w = fminf(muE[i], muD[j]);
            uint8_t k = rules[i][j];
            if (w > clip[k]) clip[k] = w;
        }
    }

    float num = 0.0f;
    float den = 0.0f;
    for (uint16_t s = 0; s < FUZZY_OUT_STEPS; s++) {
        float z = -1.0f + 2.0f * s / (FUZZY_OUT_STEPS - 1);
        float agg = 0.0f;
        for (uint8_t k = 0; k < 5; k++) {
            float m = fminf(fuzzyTriangle(z, k), clip[k]);
            if (m > agg) agg = m;
        }
        num += z * agg;
        den += agg;
    }
    return (den > 0.0f) ? num / den : 0.0f;
}

void fuzzyGains(float e, float eDot, float& dKp, float& dKd) {
    float x = constrain(e / FUZZY_E_RANGE_DEG, -1.0f, 1.0f);
    float y = constrain(eDot / FUZZY_EDOT_RANGE_DPS, -1.0f, 1.0f);

    float muE[5], muD[5];
    for (uint8_t i = 0; i < 5; i++) {
        muE[i] = fuzzyTriangle(x, i);
        muD[i] = fuzzyTriangle(y, i);
    }
    dKp = FUZZY_DKP_MAX * fuzzyInfer(FUZZY_DKP_RULES, muE, muD);
    dKd = FUZZY_DKD_MAX * fuzzyInfer(FUZZY_DKD_RULES, muE, muD);
}

// ================================================================
//  GUIDANCE — 5Hz loop
//  Navigation (position + heading) is tracked from the first GPS fix so
//  a heading is ready the moment steering starts; the control law only
//  drives the servos once guide.active is set (burn wire off + settle).
//
//  Heading: the last good COG, carried forward between fixes by the
//  gyro's yaw rate. Position: the GPS fix, or dead-reckoned at
//  GLIDE_SPEED_H_MPS along that heading once the fix is DR_ENGAGE_MS old.
// ================================================================
void updateGuidance(uint32_t now) {
    float dt = (guide.lastTickMs == 0) ? (GUIDANCE_PERIOD_MS / 1000.0f)
                                       : (now - guide.lastTickMs) / 1000.0f;
    guide.lastTickMs = now;

    float yawDelta = yawIntegralDeg;
    yawIntegralDeg = 0.0f;

    // Sink rate, positive down — feeds the GUIDE_FLAG_DESCENT_FAST sanity check.
    guide.descentRate  = lowPass((guide.lastAltitude - altitudeFiltered) / dt, guide.descentRate, 0.3f);
    guide.lastAltitude = altitudeFiltered;

    if (guide.headingValid) {
        guide.navHeading = wrap360(guide.navHeading + yawDelta);
    }

    // ---- navigation: GPS fix, or dead reckoning ----
    bool  haveSample = false;
    float dtE        = dt;
    bool  fixFresh   = gpsHadFix && (now - gpsEpochMs < DR_ENGAGE_MS);

    if (fixFresh) {
        guide.deadReckoning = false;
        // Same fix as last tick -> nothing new. The half-period margin
        // absorbs the 1ms jitter in millis() - age().
        if (gpsEpochMs - guide.lastEpochMs > GPS_NAV_RATE_MS / 2) {
            if (guide.lastEpochMs != 0) {
                dtE = (gpsEpochMs - guide.lastEpochMs) / 1000.0f;
            }
            guide.lastEpochMs = gpsEpochMs;
            guide.navLat      = gpsLatD;
            guide.navLon      = gpsLonD;

            if (gpsCourseValid && gpsSpeedMps >= COG_MIN_SPEED_MPS) {
                guide.navHeading   = gpsCourseDeg;
                guide.headingValid = true;
                guide.lowSpeedHold = false;
                haveSample         = true;
            } else {
                guide.lowSpeedHold = true;
            }
        }
    } else if (guide.active && guide.headingValid) {
        // 6) Dead-reckoning position propagation.
        guide.deadReckoning = true;
        guide.lowSpeedHold  = false;
        double h = radians((double)guide.navHeading);
        guide.navLat += GLIDE_SPEED_H_MPS * dt * cos(h) / 111320.0;
        guide.navLon += GLIDE_SPEED_H_MPS * dt * sin(h) / (111320.0 * cos(radians(guide.navLat)));
        haveSample = true;
    }

    if (!guide.active) return;

    if (guide.lowSpeedHold) {
        guide.ePrevValid = false;   // the held gap must not become an edot spike
        return;                     // COG is noise at this speed — hold the last command
    }
    if (!haveSample) return;        // nothing new — hold the last command

    if (!guide.targetSet) {
        servosNeutral();
        return;
    }

    // ---- 1) 2) 3) bearing, distance, heading error ----
    guide.bearing  = bearingTo(guide.navLat, guide.navLon, guide.targetLat, guide.targetLon);
    guide.distance = distanceTo(guide.navLat, guide.navLon, guide.targetLat, guide.targetLon);
    guide.e        = wrap180(guide.bearing - guide.navHeading);

    if (dtE < 0.05f) dtE = 0.05f;
    if (guide.ePrevValid) {
        float eDotRaw = wrap180(guide.e - guide.ePrev) / dtE;
        guide.eDot    = lowPass(eDotRaw, guide.eDot, EDOT_FILTER_ALPHA);
    } else {
        guide.eDot = 0.0f;
    }
    guide.ePrev      = guide.e;
    guide.ePrevValid = true;

    // ---- 4) 5) PD with fuzzy-scheduled gains ----
    float dKp, dKd;
    fuzzyGains(guide.e, guide.eDot, dKp, dKd);
    guide.kpEff = KP0 + dKp;
    guide.kdEff = KD0 + dKd;

    float eP = (fabsf(guide.e) < E_DEADBAND_DEG) ? 0.0f : guide.e;
    float u  = guide.kpEff * eP + guide.kdEff * guide.eDot;
    guide.u  = constrain(u, -U_MAX_DEG, U_MAX_DEG);

    writeServos(guide.u);
}

void startGuidance() {
    DEBUG_PRINTLN(F("GUIDANCE: active"));
    guide.active     = true;
    guide.ePrevValid = false;
    guide.eDot       = 0.0f;
    scheduleBeep(100);
}

void stopGuidance() {
    guide.active = false;
    servosNeutral();
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
                // BUGFIX: was `altitudeAtLaunch = altitudeFiltered`, which threw
                // away a SET_ZERO baseline (altitudeFiltered is already relative
                // to it, so ~0) and made AGL jump to MSL at launch. Shift the
                // baseline by the current reading instead, and the filter state
                // with it, so altitude reads ~0 here either way.
                float padOffset   = altitudeFiltered;
                altitudeAtLaunch += padOffset;
                altitudeFiltered -= padOffset;
                previousAltitude -= padOffset;
                telemetry.maxAltitude = 0.0f;   // fresh peak tracking starts exactly at launch
                saveState();
                setState(ASCENT);
                scheduleBeep(100);
                scheduleBeep(100);
            }
            break;

        case ASCENT:
            if (detectApogee()) {
                apogeeDetected = true;
                apogeeTime     = millis();
                setState(PAYLOAD_SEP);
                scheduleBeep(100);
                scheduleBeep(100);
            }
            break;

        case PAYLOAD_SEP:
            // Apogee: the rocket ejects the CanSat and the gyro spins up.
            if (!deploymentFired) {
                deploymentFired = true;
                separated       = true;
                saveState();
                startGyro();
                scheduleBeep(100);
            }
            setState(DESCENT);
            break;

        case DESCENT: {
            uint32_t now = millis();

            // Burn wire: BURNWIRE_CONFIRM_COUNT consecutive fresh baro samples
            // at/below BURNWIRE_ALT_M. Counted per conversion, not per loop
            // pass, so a stale reading can't be counted twice.
            if (!burnWireFired && baroSampleFresh) {
                baroSampleFresh = false;
                if (aBaroHealthy && altitudeFiltered <= BURNWIRE_ALT_M) {
                    if (++burnWireAltCount >= BURNWIRE_CONFIRM_COUNT) {
                        fireBurnWire();
                    }
                } else {
                    burnWireAltCount = 0;
                }
            }

            if (!guide.active && guidanceArmMs != 0 && (int32_t)(now - guidanceArmMs) >= 0) {
                startGuidance();
            }

            if (detectLanding()) {
                landed = true;
                stopGuidance();
                stopGyro();
                saveState();
                setState(IMPACT);
                scheduleBeep(200);
            }
            break;
        }

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
            stopGuidance();
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

    LoRa.setTxPower(LORA_TX_POWER);
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

        case CMD_SET_TARGET: {
            if (state != LAUNCH_PAD) {
                DEBUG_PRINTLN(F("CMD: SET_TARGET refused — only honored from LAUNCH_PAD"));
                break;
            }
            if (payloadLen != 2 * sizeof(int32_t)) {
                DEBUG_PRINTLN(F("CMD: SET_TARGET rejected — bad payload length"));
                break;
            }
            int32_t latE7, lonE7;
            memcpy(&latE7, payload, sizeof(int32_t));
            memcpy(&lonE7, payload + sizeof(int32_t), sizeof(int32_t));
            double lat = latE7 / 1e7;
            double lon = lonE7 / 1e7;
            if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 || (latE7 == 0 && lonE7 == 0)) {
                DEBUG_PRINTLN(F("CMD: SET_TARGET rejected — out of range, keeping current target"));
                break;
            }
            setTarget(lat, lon);
            saveState();
            scheduleBeep(100);
            DEBUG_PRINT(F("CMD: SET_TARGET = "));
            DEBUG_PRINT2(lat, 7);
            DEBUG_PRINT(F(", "));
            DEBUG_PRINTLN2(lon, 7);
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
    if (telemetry.battLow)          f |= TLM_FLAG_BATT_LOW;
    if (telemetry.gpsStale)         f |= TLM_FLAG_GPS_STALE;
    w.flags = f;

    // No Controller B on the CanSat — fields kept so older offsets don't move.
    w.bState     = 0;
    w.bAltitude  = 0.0f;
    w.bAccelMag  = 0.0f;

    w.gpsCourse      = telemetry.gpsCourse;
    w.gpsSpeed       = telemetry.gpsSpeed;
    w.headingError   = guide.e;
    w.guidanceU      = guide.u;
    w.targetDistance = guide.distance;
    w.guideFlags     = telemetry.guideFlags;

    w.crc = calculateCRC((const uint8_t*)&w, offsetof(TelemetryWire, crc));
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
    LoRa.endPacket(false);   // non-blocking TX

    loraLastTxTime = millis();
}

void checkLoRaRx() {
    if (!loraReady) return;
    if (millis() - loraLastTxTime < LORA_RX_WINDOW_MS) return;

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

void taskLoRaTick() {
    sendLoRaTelemetry();
    checkLoRaRx();
}

// ================================================================
//  SENSOR-HEALTH / GO-NO-GO STATUS PACKET
//  buildStatusPacket() always reflects CURRENT live health (aBaroHealthy /
//  aImuHealthy are updated continuously by readSensors()), not a
//  boot-time snapshot — CMD_GET_STATUS can be
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
    sp.ctrlBAlive   = false;            // no Controller B on the CanSat — field kept for the GS layout
    sp.battV        = readBatteryVoltage();
    sp.flightNumber = flightNumber;

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
        LoRa.endPacket(false);
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
//  SETUP
// ================================================================
void setup() {
    // Pyro gates first — before anything that can stall (USB wait, I2C,
    // SD). The gate pulldowns already hold the FETs off; this makes it
    // an actively driven LOW.
    initPyros();

    Serial.begin(115200);
    uint32_t t = millis();
    // BUGFIX (FLIGHT-CRITICAL): was `while (!Serial);` with no timeout.
    // Serial only becomes true once a USB host enumerates the port — with
    // no USB connection in flight, this would hang forever, before the
    // watchdog is even armed later in setup(). Bounded to 3s so the board
    // still boots and flies normally when untethered.
    while (!Serial && millis() - t < 3000);

    Serial.println(F("AVIOPRO V0 — Controller A (CanSat build)"));

    if (watchdog_caused_reboot()) {
        Serial.println(F("*** WATCHDOG RESET ***"));
    }

    EEPROM.begin(256);
    setTarget(TARGET_LAT_DEG, TARGET_LON_DEG);   // loadState() may override with a LoRa-set target
    #ifdef BENCH_TEST_MODE
        // Lab testing only: always start from a clean slate. EEPROM is not
        // even loaded — loading it and then ignoring the state would still
        // carry stale flags (burnWireFired, deploymentFired, ...) into the
        // next bench run and silently skip those events. Real flight builds
        // must NOT do this — resuming after an in-flight reboot is intended.
        bool resumed = false;
    #else
        bool resumed = loadState();
    #endif

    pinMode(BUZZER_PIN,    OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    pinMode(BUZZER_PIN_actual, OUTPUT);   // was never configured — DC digitalWrite()
    digitalWrite(BUZZER_PIN_actual, LOW); // calls were also silently no-op'ing on it
    servoLeft.attach(SERVO_LEFT_PIN);
    servoRight.attach(SERVO_RIGHT_PIN);
    servosNeutral();                            // brake lines neutral until guidance starts

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

    // Full-init verdict: 3 quick beeps = GO for GUI commands, one long
    // beep = NO-GO (check the status packet / serial log for what failed).
    // Nothing is transmitted over LoRa here — see sendStatusPacket().
    bool go = runSelfCheck();
    if (go) {
        beepStartupGo();
    } else {
        beepStartupNoGo();
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
            Serial.println(F("Re-running apogee action"));
            deploymentFired = true;
            separated       = true;
            saveState();
        }
        if (state == PAYLOAD_SEP) {
            setState(DESCENT);
        }

        // Output pins don't survive a reset — re-drive what should be on.
        if (deploymentFired && !landed) {
            startGyro();
        }
        // The burn wire is never re-fired (a reset mid-burn ends that burn).
        // Steering resumes after a fresh settle period.
        if (burnWireFired && !landed) {
            guidanceArmMs = millis() + PARAFOIL_SETTLE_MS;
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
//    10. LoRa TX+RX          — 10Hz
//    11. GPS validate        — 5Hz fix quality check
//    12. Guidance            — 5Hz, right after the GPS validate
//    13. Serial CSV          — 5Hz debug output
// ================================================================
void loop() {
    uint32_t now = millis();

    if (runTask(taskWatchdog, now)) {
        watchdog_update();
    }

    taskBuzzerTick(now);
    updateBurnWire(now);

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
        // 1Hz on the pad, full rate from launch on.
        if (state != LAUNCH_PAD || ++padLogCounter >= LOG_PAD_DECIMATION) {
            padLogCounter = 0;
            pushToBuffer();
        }
    }

    if (runTask(taskSDFlush, now)) {
        flushToSD();
    }

    if (runTask(taskFlashFlush, now)) {
        flushToFlash();
    }

    if (runTask(taskLoRa, now)) {
        taskLoRaTick();
    }

    if (runTask(taskGPSValidate, now)) {
        validateGPS();
    }

    if (runTask(taskGuidance, now)) {
        updateGuidance(now);
    }

    if (runTask(taskSerialCSV, now)) {
        sendTelemetryCSV();
    }
}
