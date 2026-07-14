-- Drone Flight Logger schema (SQLite)

CREATE TABLE IF NOT EXISTS flights (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    drone_name TEXT NOT NULL,
    started_at TEXT NOT NULL,          -- ISO 8601 UTC
    ended_at TEXT,                     -- NULL while flight is in progress
    notes TEXT
);

CREATE TABLE IF NOT EXISTS telemetry_points (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    flight_id INTEGER NOT NULL REFERENCES flights(id) ON DELETE CASCADE,
    ts TEXT NOT NULL,                  -- ISO 8601 UTC timestamp of this sample
    lat REAL NOT NULL,                 -- degrees
    lon REAL NOT NULL,                 -- degrees
    alt_rel REAL,                      -- meters, relative to home/takeoff
    alt_msl REAL,                      -- meters, above sea level
    roll REAL,                         -- degrees
    pitch REAL,                        -- degrees
    yaw REAL,                          -- degrees (heading)
    vx REAL,                           -- m/s, north
    vy REAL,                           -- m/s, east
    vz REAL,                           -- m/s, down
    groundspeed REAL,                  -- m/s
    airspeed REAL,                     -- m/s
    battery_voltage REAL,              -- volts
    battery_remaining REAL,            -- percent
    satellites_visible INTEGER,
    fix_type INTEGER,                  -- GPS fix type (0=no fix ... 3=3D fix)
    hdop REAL
);

CREATE INDEX IF NOT EXISTS idx_telemetry_flight_ts ON telemetry_points(flight_id, ts);

-- IMU samples (GY-521 / MPU6050 on the ESP32 logger). Kept separate from
-- telemetry_points because it's sampled ~10x faster (~50Hz vs ~5Hz) and has
-- a completely different schema -- most rows would otherwise be half-empty.
CREATE TABLE IF NOT EXISTS imu_points (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    flight_id INTEGER NOT NULL REFERENCES flights(id) ON DELETE CASCADE,
    ts TEXT NOT NULL,                  -- ISO 8601 UTC timestamp of this sample
    accel_x REAL,                      -- m/s^2
    accel_y REAL,                      -- m/s^2
    accel_z REAL,                      -- m/s^2
    gyro_x REAL,                       -- rad/s
    gyro_y REAL,                       -- rad/s
    gyro_z REAL                        -- rad/s
);

CREATE INDEX IF NOT EXISTS idx_imu_flight_ts ON imu_points(flight_id, ts);
