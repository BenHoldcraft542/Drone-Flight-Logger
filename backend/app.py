"""
FastAPI backend for the Drone Flight Logger.

Serves:
  - REST API for flights and telemetry (read from the SQLite DB the
    mavlink_logger.py script writes to, or that the ESP32 uploads into)
  - The static frontend (2D/3D visualization)

Run:
    pip install -r requirements.txt
    uvicorn app:app --reload --host 0.0.0.0 --port 8000

Then open http://localhost:8000
"""

import os
import secrets
import sqlite3
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import List, Optional

from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, Header, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel

BASE_DIR = Path(__file__).resolve().parent
DB_PATH = BASE_DIR.parent / "database" / "flights.db"
FRONTEND_DIR = BASE_DIR.parent / "frontend"

load_dotenv(BASE_DIR / ".env")  # loads INGEST_API_KEY (and anything else) from backend/.env

# API key for write access (the ESP32's upload endpoint). Set this via an
# environment variable rather than hardcoding it -- see README for setup.
# If INGEST_API_KEY isn't set, the server refuses to start rather than
# silently running unprotected.
INGEST_API_KEY = os.environ.get("INGEST_API_KEY")
if not INGEST_API_KEY:
    raise RuntimeError(
        "INGEST_API_KEY environment variable is not set. "
        "Generate one (e.g. `python3 -c \"import secrets; print(secrets.token_hex(32))\"`) "
        "and set it before starting the server. See README.md."
    )


def verify_api_key(x_api_key: Optional[str] = Header(default=None)):
    if not x_api_key or not secrets.compare_digest(x_api_key, INGEST_API_KEY):
        raise HTTPException(status_code=401, detail="Missing or invalid API key")


app = FastAPI(title="Drone Flight Logger API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


def get_db():
    if not DB_PATH.exists():
        raise HTTPException(status_code=503, detail="No flight database yet. Run the logger first.")
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


@app.get("/api/flights")
def list_flights():
    conn = get_db()
    rows = conn.execute(
        """
        SELECT f.id, f.drone_name, f.started_at, f.ended_at, f.notes,
               COUNT(DISTINCT t.id) AS point_count,
               COUNT(DISTINCT i.id) AS imu_point_count
        FROM flights f
        LEFT JOIN telemetry_points t ON t.flight_id = f.id
        LEFT JOIN imu_points i ON i.flight_id = f.id
        GROUP BY f.id
        ORDER BY f.started_at DESC
        """
    ).fetchall()
    conn.close()
    return [dict(r) for r in rows]


@app.get("/api/flights/{flight_id}")
def get_flight(flight_id: int):
    conn = get_db()
    flight = conn.execute("SELECT * FROM flights WHERE id = ?", (flight_id,)).fetchone()
    if not flight:
        conn.close()
        raise HTTPException(status_code=404, detail="Flight not found")
    conn.close()
    return dict(flight)


@app.get("/api/flights/{flight_id}/telemetry")
def get_telemetry(flight_id: int, limit: int = 20000):
    conn = get_db()
    rows = conn.execute(
        """
        SELECT ts, lat, lon, alt_rel, alt_msl, roll, pitch, yaw,
               vx, vy, vz, groundspeed, airspeed,
               battery_voltage, battery_remaining,
               satellites_visible, fix_type, hdop
        FROM telemetry_points
        WHERE flight_id = ?
        ORDER BY ts ASC
        LIMIT ?
        """,
        (flight_id, limit),
    ).fetchall()
    conn.close()
    if not rows:
        raise HTTPException(status_code=404, detail="No telemetry found for this flight")
    return [dict(r) for r in rows]


@app.get("/api/flights/{flight_id}/imu")
def get_imu(flight_id: int, limit: int = 100000):
    # Default limit is higher than /telemetry's since IMU logs at ~10x the
    # GPS sample rate (~50Hz vs ~5Hz) -- a few minutes of flight produces
    # far more IMU rows than GPS rows.
    conn = get_db()
    rows = conn.execute(
        """
        SELECT ts, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        FROM imu_points
        WHERE flight_id = ?
        ORDER BY ts ASC
        LIMIT ?
        """,
        (flight_id, limit),
    ).fetchall()
    conn.close()
    if not rows:
        raise HTTPException(status_code=404, detail="No IMU data found for this flight")
    return [dict(r) for r in rows]


# --- ESP32 / offline-logger ingestion ---
# The ESP32 logs GPS fixes and IMU samples on independent timers while
# flying (no live clock needed), tagging each JSON line with "type": "gps"
# or "type": "imu", then dumps the whole flight here once it reconnects to
# WiFi. We reconstruct real timestamps by working backward from upload_time
# using each point's ms-since-boot relative to the last point in the batch.

class TelemetryPoint(BaseModel):
    type: str  # "gps" or "imu"
    ms: int
    # GPS fields
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt: Optional[float] = None
    sats: Optional[int] = None
    hdop: Optional[float] = None
    # IMU fields
    ax: Optional[float] = None
    ay: Optional[float] = None
    az: Optional[float] = None
    gx: Optional[float] = None
    gy: Optional[float] = None
    gz: Optional[float] = None


class FlightUpload(BaseModel):
    drone_name: str
    upload_time: str  # ISO 8601 UTC, e.g. "2026-07-12T20:15:00Z"
    points: List[TelemetryPoint]


@app.post("/api/ingest/flight", status_code=201, dependencies=[Depends(verify_api_key)])
def ingest_flight(upload: FlightUpload):
    if not upload.points:
        raise HTTPException(status_code=400, detail="No points in upload")

    upload_dt = datetime.fromisoformat(upload.upload_time.replace("Z", "+00:00"))
    last_ms = max(p.ms for p in upload.points)
    first_ms = min(p.ms for p in upload.points)

    conn = sqlite3.connect(DB_PATH)
    with open(BASE_DIR.parent / "database" / "schema.sql") as f:
        conn.executescript(f.read())

    first_ts = upload_dt - timedelta(milliseconds=(last_ms - first_ms))
    cur = conn.execute(
        "INSERT INTO flights (drone_name, started_at, ended_at, notes) VALUES (?, ?, ?, ?)",
        (upload.drone_name, first_ts.isoformat(), upload_dt.isoformat(), "Uploaded from ESP32 flash log"),
    )
    flight_id = cur.lastrowid

    gps_count = 0
    imu_count = 0
    skipped = 0

    for p in upload.points:
        # Each point's real time = upload time, minus how far before the
        # last recorded point it was captured.
        point_dt = upload_dt - timedelta(milliseconds=(last_ms - p.ms))

        if p.type == "gps":
            if p.lat is None or p.lon is None:
                skipped += 1  # telemetry_points.lat/lon are NOT NULL -- skip rather than crash
                continue
            conn.execute(
                """
                INSERT INTO telemetry_points (
                    flight_id, ts, lat, lon, alt_rel, satellites_visible, hdop
                ) VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                (flight_id, point_dt.isoformat(), p.lat, p.lon, p.alt, p.sats, p.hdop),
            )
            gps_count += 1

        elif p.type == "imu":
            conn.execute(
                """
                INSERT INTO imu_points (
                    flight_id, ts, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (flight_id, point_dt.isoformat(), p.ax, p.ay, p.az, p.gx, p.gy, p.gz),
            )
            imu_count += 1

        else:
            skipped += 1

    conn.commit()
    conn.close()
    return {
        "flight_id": flight_id,
        "gps_points_stored": gps_count,
        "imu_points_stored": imu_count,
        "points_skipped": skipped,
    }


# --- Serve frontend ---
app.mount("/assets", StaticFiles(directory=FRONTEND_DIR), name="assets")


@app.get("/")
def index():
    return FileResponse(FRONTEND_DIR / "index.html")