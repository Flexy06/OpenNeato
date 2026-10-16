#ifndef CONFIG_H
#define CONFIG_H

// Firmware version — passed via build flag (-DFIRMWARE_VERSION=...), fallback for local builds
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "0.0"
#endif

// Chip model — passed via build flag (-DCHIP_MODEL=...) from board section in platformio.ini
#ifndef CHIP_MODEL
#error "CHIP_MODEL must be defined (e.g. -DCHIP_MODEL=\\\"ESP32-C3\\\")"
#endif

// WiFi Configuration
#define DEFAULT_HOSTNAME "neato"
#define WIFI_DEFAULT_TX_POWER                                                                                          \
    60 // WiFi TX power in 0.25 dBm units (60 = 15 dBm, ~32 mW)
       // Lower values caused boot connection failures (4-way handshake
       // timeouts) at marginal signal levels (-70 dBm range).
       // Range: 8 (2 dBm) to 84 (21 dBm). Common values:
       //   34 = 8.5 dBm,  52 = 13 dBm,  60 = 15 dBm (recommended)
       //   68 = 17 dBm,  78 = 19.5 dBm
#define WIFI_MAX_RECONNECT_BACKOFF 30000 // Max backoff between reconnect attempts (ms)

// Pin Configuration — boot/reset button and default UART pins vary by chip.
// Original ESP32: BOOT is GPIO0, GPIO1/3 are the USB-UART bridge (U0TXD/U0RXD).
// ESP32-C3: BOOT is GPIO9, GPIO1/3 are free GPIOs.
// Seeed Studio XIAO ESP32C3: D6/TX is GPIO21, D7/RX is GPIO20.
// ESP32-S3: BOOT is GPIO0, GPIO19/20 are native USB — use free GPIOs for UART.
#if CONFIG_IDF_TARGET_ESP32
#define RESET_BUTTON_PIN 0
#define NEATO_DEFAULT_TX_PIN 17
#define NEATO_DEFAULT_RX_PIN 16
#define MAX_GPIO_PIN 39
#elif CONFIG_IDF_TARGET_ESP32C3
#define RESET_BUTTON_PIN 9
#if defined(OPENNEATO_BOARD_XIAO_ESP32C3)
#define NEATO_DEFAULT_TX_PIN 21
#define NEATO_DEFAULT_RX_PIN 20
#else
#define NEATO_DEFAULT_TX_PIN 3
#define NEATO_DEFAULT_RX_PIN 4
#endif
#define MAX_GPIO_PIN 21
#elif CONFIG_IDF_TARGET_ESP32S3
#define RESET_BUTTON_PIN 0
#define NEATO_DEFAULT_TX_PIN 17
#define NEATO_DEFAULT_RX_PIN 18
#define MAX_GPIO_PIN 48
#else
#error "Unsupported chip — add pin definitions for this target"
#endif

// Actual UART pins are stored in NVS and configurable via settings API.
#define NEATO_BAUD_RATE 115200
#define NEATO_UART_RX_BUFFER 4096 // Default 256 bytes overflows during GetLDSScan (~5KB response)

// Neato command queue timing (milliseconds)
#define NEATO_CMD_TIMEOUT_MS 3000
#define NEATO_INTER_CMD_DELAY_MS 50
#define NEATO_QUEUE_MAX_SIZE 16
#define NEATO_RESPONSE_TERMINATOR 0x1A // Ctrl-Z

// initSKey retry — GetVersion can fail at boot if the robot is still powering up.
// Retry with exponential backoff until the robot responds.
#define SKEY_RETRY_INITIAL_MS 2000 // First retry after 2s
#define SKEY_RETRY_MAX_MS 30000 // Cap backoff at 30s
#define SKEY_ATTEMPT_TIMEOUT_MS 15000 // Watchdog: retry an attempt whose result never arrived

// AsyncCache TTL values (milliseconds) — how long each response is considered fresh.
// Callers within the TTL window get the cached value instantly; concurrent requests
// during an in-flight fetch are coalesced (only one serial command dispatched).
#define CACHE_TTL_STATE 2000 // GetState + GetErr — polled every 2s
#define CACHE_TTL_CHARGER 30000 // GetCharger — battery data (30s)
#define CACHE_TTL_SENSORS 1000 // Analog/digital sensors, motors (1s — fast for manual clean safety)
#define CACHE_TTL_VERSION 300000 // GetVersion — rarely changes (5 min)
#define CACHE_TTL_LDS 1500 // LIDAR scan — 1.5s (scan takes ~800ms on serial)

// Manual clean safety
#define MANUAL_SAFETY_POLL_MS 500 // Poll bumpers every 500ms during manual clean
#define MANUAL_STALL_POLL_MS 500 // Poll wheel load every 500ms while wheels are moving
#define MANUAL_STALL_LOAD_PCT 60 // Wheel load % threshold — above this is considered stalled
#define MANUAL_STALL_COUNT 2 // Consecutive overloaded polls before stopping (2 × 500ms = 1s grace)
#define MANUAL_CLIENT_TIMEOUT_MS 5000 // Stop wheels if no API activity (any request) within this window
#define MANUAL_BRUSH_RPM 1200 // Default brush RPM in manual mode
#define MANUAL_VACUUM_SPEED_PCT 80 // Default vacuum speed (%) in manual mode
#define MANUAL_SIDE_BRUSH_POWER_MW 1500 // Default side brush power (mW) — universal Neato Botvac default

// Command completion status (for enhanced logging)
enum CommandStatus {
    CMD_SUCCESS, // Command succeeded, response received OK
    CMD_TIMEOUT, // No complete response within timeout (may have partial data)
    CMD_PARSE_FAILED, // Got response but parse failed (not used yet)
    CMD_SERIAL_ERROR, // UART error or other serial issue (e.g. response desync)
    CMD_UNSUPPORTED, // Robot responded with "Unknown Cmd" — command not available
    CMD_QUEUE_FULL // Command rejected because the serial queue was full
};

// Timing intervals (milliseconds)
#define WIFI_RECONNECT_INTERVAL 5000
#define RESET_BUTTON_HOLD_TIME 5000 // Hold for 5 seconds to reset

// Data logger
#define LOG_MAX_FILE_SIZE 32768 // 32 KB per file before rotation
#define LOG_MAX_FS_PERCENT 60 // Delete oldest logs when log dir exceeds this share of filesystem
#define LOG_MIN_FS_PERCENT 10 // Logs always get at least this % of filesystem, even if other data fills the rest
#define LOG_MAX_FILES 50 // Maximum number of archived log files to keep
#define LOG_DIR "/log"
#define LOG_CURRENT_FILE "/log/current.jsonl"
#define LOG_FLUSH_INTERVAL_MS 30000 // Flush write buffer to filesystem every 30 seconds (reduces flash wear)
#define LOG_FLUSH_MAX_LINES 128 // Also flush when buffer reaches this many lines
#define LOG_ENFORCE_LIMITS_MS 30000 // Check log dir size/count limits every 30s (not every 50ms tick)

// NVS (Non-Volatile Storage) — single shared namespace for all settings
#define NVS_NAMESPACE "neato"

// NVS keys — WiFi
#define NVS_KEY_WIFI_SSID "wifi_ssid"
#define NVS_KEY_WIFI_PASS "wifi_pass"
#define NVS_KEY_AP_FALLBACK "ap_fallback"

// Fallback Access Point (provisioning AP)
// SSID is derived from hostname: "<hostname>-ap". Network is open (no password).
// AP is automatic: always on when no STA credentials saved; on/off based on
// apFallbackOnDisconnect setting when STA connection drops.
#define AP_SSID_SUFFIX "-ap"
#define AP_DEFAULT_IP IPAddress(192, 168, 4, 1)
#define AP_GATEWAY IPAddress(192, 168, 4, 1)
#define AP_SUBNET IPAddress(255, 255, 255, 0)
#define AP_CHANNEL 1
#define AP_MAX_CONNECTIONS 4

// NVS keys — Time/NTP
#define NVS_KEY_TIMEZONE "tz"

// NVS keys — Settings
#define NVS_KEY_HOSTNAME "hostname"
#define NVS_KEY_LOG_LEVEL "log_level"
#define NVS_KEY_DEBUG "debug" // TODO: Remove after v0.5 — legacy key, migrated to log_level on first boot
#define LOG_LEVEL_AUTO_OFF_DEBUG_MS 600000 // Auto-revert debug -> off after 10 minutes
#define LOG_LEVEL_AUTO_OFF_INFO_MS 3600000 // Auto-revert info -> off after 1 hour

// Log levels - controls what gets written to SPIFFS. Default off to minimize
// flash wear.
#define LOG_LEVEL_OFF 0 // Nothing written to SPIFFS (default)
#define LOG_LEVEL_INFO 1 // Errors, state transitions, boot, wifi, ota, ntp, cleaning events, notifications
#define LOG_LEVEL_DEBUG 2 // Everything in Info + all serial commands + raw responses
#define NVS_KEY_WIFI_TX_POWER "wifi_tx_pwr"
#define NVS_KEY_UART_TX_PIN "uart_tx_pin"
#define NVS_KEY_UART_RX_PIN "uart_rx_pin"
// NVS keys — Cleaning
#define NVS_KEY_NAV_MODE "nav_mode" // Navigation mode: "Normal", "Gentle", "Deep", "Quick"
// NVS keys — Manual clean
#define NVS_KEY_MC_STALL_THR "mc_stall_thr"
#define NVS_KEY_MC_BRUSH_RPM "mc_brush_rpm"
#define NVS_KEY_MC_VACUUM_PCT "mc_vacuum_pct"
#define NVS_KEY_MC_SBRUSH_MW "mc_sbrush_mw"

// NVS keys — Remote syslog
#define NVS_KEY_SYSLOG_ENABLED "syslog_on"
#define NVS_KEY_SYSLOG_IP "syslog_ip"
#define SYSLOG_DEFAULT_PORT 514
#define SYSLOG_PRI "<14>" // facility=user (1), severity=info (6) -> 1*8+6=14

// NVS keys — Notifications
#define NVS_KEY_NTFY_TOPIC "ntfy_topic"
#define NVS_KEY_NTFY_ENABLED "ntfy_enabled"
#define NVS_KEY_NTFY_ON_DONE "ntfy_on_done"
#define NVS_KEY_NTFY_ON_ERR "ntfy_on_err"
#define NVS_KEY_NTFY_ON_ALERT "ntfy_on_alrt"
#define NVS_KEY_NTFY_ON_DOCK "ntfy_on_dock"

// NVS keys — Schedule (ESP32-managed, not robot serial)
#define NVS_KEY_SCHED_ENABLED "sched_on"
#define NVS_KEY_SKIP_NEXT_CLEAN "skip_next_clean"
#define NVS_KEY_AUTO_RESTART_ENABLED "auto_rst_on"
#define NVS_KEY_AUTO_RESTART_HOUR "auto_rst_h"
#define NVS_KEY_AUTO_RESTART_MIN "auto_rst_m"
#define NVS_KEY_RESTART_BEFORE_CLEAN "rst_b4_clean"
// Per-day keys use suffix: "s0h","s0m","s0on" .. "s6h","s6m","s6on" (Mon=0..Sun=6)
// Built programmatically in SettingsManager — no individual defines needed.
#define SCHEDULE_DAYS 7
#define SCHEDULE_SLOTS_PER_DAY 2 // Two time slots per day (e.g. morning + afternoon)
#define SCHEDULE_CHECK_INTERVAL_MS 30000 // Check schedule against NTP time every 30s
#define SCHEDULE_WINDOW_MINS 5 // Fire if current time is 0..N minutes after scheduled slot
#define RESTART_BOOT_TIMEOUT_MS 120000 // Max wait time for robot to boot after restart (2 min)

// Notification manager — adaptive polling intervals
#define NOTIF_INTERVAL_ACTIVE_MS 3000 // Check state every 3s when robot is active (cleaning/docking)
#define NOTIF_INTERVAL_IDLE_MS 30000 // Check state every 30s when robot is idle

// Cleaning history
#define HISTORY_INTERVAL_IDLE_MS 30000 // Poll state every 30s when idle (detect cleaning start)
#define HISTORY_INTERVAL_ACTIVE_MS 2000 // Poll state/pose every 2s during active cleaning (~0.6m resolution at 300mm/s)
#define HISTORY_FLUSH_INTERVAL_MS 30000 // Flush buffered pose snapshots to disk every 30 seconds
#define HISTORY_COMPRESS_INTERVAL_MS 50 // Fast tick during post-session compression (512B/tick)
#define HISTORY_DIR "/history" // SPIFFS directory for session files
#define HISTORY_MAX_FS_PERCENT 50 // Delete oldest sessions when history dir exceeds this share of filesystem
#define HISTORY_MIN_FS_PERCENT 10 // History always gets at least this % of filesystem
#define HISTORY_MAX_FILES 20 // Maximum number of archived session files to keep
#define HISTORY_AREA_CELL_M 0.5f // Coarse grid cell size in meters for visited-area estimation
#define HISTORY_MIN_SNAPSHOTS 3 // Discard sessions with fewer snapshots (too short to render a useful map)
#define HISTORY_IMPORT_MAX_BYTES 524288 // 512 KB max import file size (mapping runs carry LIDAR scans)
#define HISTORY_SCAN_EVERY 3 // Mapping run: record a LIDAR scan every Nth pose snapshot (~6s)
#define HISTORY_SCAN_BINS 180 // Scans are stored as 2° bins ...
#define HISTORY_SCAN_UNIT_MM 20 // ... of 20 mm steps (1 byte, 0 = no return, max 5.1 m)

// Floor map (pinned cleaning session + rooms + no-go zones) — see floor_map.h
#define FLOORMAP_DIR "/map"
#define FLOORMAP_FILE "/map/floor.txt"
#define FLOORMAP_TMP_FILE "/map/floor.tmp"
#define FLOORMAP_FIELD_FILE "/map/field.bin"
#define FLOORMAP_FIELD_TMP_FILE "/map/field.tmp"
#define FLOORMAP_FIELD_MAX_BYTES 98304 // Likelihood field upload cap (e.g. 300x300 cells @ 5 cm + header)
#define FLOORMAP_MAX_BYTES 65536 // 64 KB upload cap (3000 path points ~ 36 KB as text)
#define FLOORMAP_MAX_PATH_POINTS 3000 // ~1.6h of recorded snapshots at 2s intervals
#define FLOORMAP_MAX_ZONES 16 // Max rooms and max no-go zones (each)
#define FLOORMAP_MAX_VERTICES 24 // Max polygon vertices per zone

// Guided clean — ESP32-driven path replay inside selected rooms (see guided_clean_manager.h).
// Geometry is in millimetres in the map frame of the pinned session (dock = origin).
#define GUIDED_TICK_MS 100 // State machine tick while active
#define GUIDED_POLL_MS 150 // Sensor poll cadence while wheels move (rotates motors/digital/analog)
#define GUIDED_WHEEL_BASE_MM 248.0f // Botvac wheel track (distance between wheel contact points)
#define GUIDED_ROBOT_RADIUS_MM 170 // Body radius + small margin, used for obstacles and shortcuts
#define GUIDED_NOGO_MARGIN_MM 280 // No-go inflation: robot radius + allowance for odometry drift
#define GUIDED_COVER_CELL_MM 100 // Coverage grid cell size for "known free floor" shortcut checks
#define GUIDED_COVER_STAMP_MM 100 // Radius stamped around recorded poses into the coverage grid
#define GUIDED_COVER_MAX_CELLS 250 // Max grid cells per axis (25 m at 100 mm) — ~7.8 KB bitmap
#define GUIDED_SHORTCUT_WINDOW 40 // Max waypoints skipped ahead by one transit shortcut
#define GUIDED_REACHED_MM 120 // Waypoint counts as reached within this distance
#define GUIDED_HEADING_TOL_DEG 12.0f // Turn in place first if heading error exceeds this
#define GUIDED_MAX_STEP_MM 600 // Longest straight drive per segment before re-planning
#define GUIDED_MIN_POINT_SPACING_MM 100 // Drop recorded points closer than this to the previous one
#define GUIDED_SPEED_CLEAN 150 // mm/s while cleaning inside a selected room
#define GUIDED_SPEED_TRANSIT 200 // mm/s while driving between rooms / returning
#define GUIDED_SPEED_TURN 120 // mm/s wheel speed for turns in place
#define GUIDED_SPEED_SLOW 80 // mm/s for backing off obstacles and docking
#define GUIDED_BACKOFF_MM 100 // Reverse distance after a bump / stall / drop
#define GUIDED_SKIP_AFTER_OBSTACLE_MM 350 // Skip waypoints within this distance of an obstructed target
#define GUIDED_MAX_OBSTACLE_STREAK 6 // Consecutive obstructed waypoints before giving up on the phase
#define GUIDED_DROP_MM 50 // Drop (cliff) sensor reading above this is a cliff — floor reads ~20 mm
#define GUIDED_STALL_LOAD_PCT 70 // Wheel load above this for 2 polls counts as stalled
#define GUIDED_SEGMENT_SLACK_MS 2500 // Segment timeout = expected duration + slack
#define GUIDED_MIN_BATTERY_PCT 20 // Return to dock below this charge
#define GUIDED_BATTERY_POLL_MS 60000 // Battery check cadence while running
#define GUIDED_MAX_RUNTIME_MS 7200000UL // Hard limit (2h) — return home after this
#define GUIDED_DOCK_STAGING_MM 900 // Staging point on the dock axis: arrive here, align, scan-match
#define GUIDED_DOCK_PREBACK_MM 300 // Last stop on the dock axis before the final straight back-in
#define GUIDED_DOCK_LATERAL_TOL_MM 50 // Max sideways offset from the dock axis for the final back-in
#define GUIDED_DOCK_ATTEMPTS 2 // Full approach attempts before handing over to the robot's own docking
#define GUIDED_OBSTACLE_RADIUS_MM 150 // Remembered obstacle footprint (bump/stall) placed ahead of the robot
#define GUIDED_CLIFF_RADIUS_MM 250 // Remembered cliff footprint
#define GUIDED_MAX_OBSTACLES 24 // Obstacle memory (oldest dropped first)
// LIDAR wall compass — corrects heading drift by aligning to the dominant wall
// direction (most rooms are rectangular). Reference taken on the dock.
#define GUIDED_LDS_SPINUP_MS 2500 // Wait after SetLDSRotation On before the first scan
#define GUIDED_COMPASS_EVERY_MM 1000 // Re-align after this much travel ...
#define GUIDED_COMPASS_EVERY_DEG 75.0f // ... or this much accumulated turning
#define GUIDED_COMPASS_MIN_CONF 0.35f // Min alignment strength (0..1) of the wall direction histogram
#define GUIDED_COMPASS_MAX_FIX_DEG 20.0f // Ignore corrections larger than this (clutter / non-rectangular room)
// LIDAR localization against the mapping-run field (floor_map.h LocField)
#define GUIDED_LOC_EVERY_MM 400 // Re-localize after this much travel ...
#define GUIDED_LOC_EVERY_DEG 30.0f // ... or this much turning
#define GUIDED_LOC_RADIUS_MM 300 // Search window around the odometry pose
#define GUIDED_LOC_ANGLE_DEG 10 // Heading search window (±)
#define GUIDED_LOC_START_RADIUS_MM 500 // Wider search for the first fix on the dock
#define GUIDED_LOC_START_ANGLE_DEG 25
#define GUIDED_LOC_MIN_SCORE 0.30f // Min normalized match score (0..1) to accept a fix
// Dock homing — ICP scan match against the reference scan taken on the dock
#define GUIDED_ICP_MAX_CORR_MM 900 // Reject matches that move the pose further than this
// Path planning — BFS over a coarse grid of floor the recording covered
#define GUIDED_PLAN_CELL_MM 200 // Planning grid cell (doubled automatically for large homes)
#define GUIDED_PLAN_MAX_CELLS 120 // Max cells per axis (dist map ≤ 29 KB while planning)
#define GUIDED_MAX_REPLANS 4 // Re-plans of one transit leg after obstacles before giving up on it
#define GUIDED_TRACE_FILE "/map/trace.jsonl" // Run trace of the last guided clean (always recorded)
#define GUIDED_TRACE_MAX_BYTES 98304
#define GUIDED_TRACE_FLUSH_BYTES 2048
#define GUIDED_TRACE_EVERY_MM 150 // Pose line after this much travel ...
#define GUIDED_TRACE_EVERY_DEG 20.0f // ... or this much turning
#define GUIDED_RUN_GAP_MM 1200 // Recorded points further apart than this start a new cleaning run

// Task Watchdog Timer (TWDT) — hardware watchdog that resets the ESP32 if
// loop() stops running (deadlock, infinite loop, blocking I/O). The main task
// must call esp_task_wdt_reset() every iteration; if it misses the deadline,
// the TWDT triggers a system reset. This complements the heap watchdog below
// which only catches memory exhaustion (and requires loop() to keep running).
#define TASK_WDT_TIMEOUT_S                                                                                             \
    15 // Seconds before TWDT triggers reset (generous
       // to accommodate slow filesystem operations and
       // LIDAR scans that can take several seconds)

// Heap watchdog — restart if free heap stays below threshold for this duration.
// Prevents the device from becoming unresponsive when memory is exhausted
// (e.g. by runaway async connections after a UART desync cascade).
#define HEAP_WATCHDOG_THRESHOLD 16384 // 16 KB — below this is critical
#define HEAP_WATCHDOG_DURATION_MS 30000 // Must stay low for 30s to trigger

// NTP / time sync
#define NTP_SERVER_1 "pool.ntp.org"
#define NTP_SERVER_2 "time.nist.gov"
#define NTP_DEFAULT_TZ "UTC0" // POSIX TZ string, stored in NVS

// Logging — enabled by default, disable with -DENABLE_LOGGING=0
#ifndef ENABLE_LOGGING
#define ENABLE_LOGGING 1
#endif

#if ENABLE_LOGGING
#define LOG(tag, fmt, ...) Serial.printf("[%s] " fmt "\n", tag, ##__VA_ARGS__)
#else
#define LOG(tag, fmt, ...)
#endif

#endif // CONFIG_H
