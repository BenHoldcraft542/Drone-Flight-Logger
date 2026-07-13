"""
FastAPI backend for the Drone Flight Logger.

Serves:
  - REST API for flights and telemetry (read from the SQLite DB the
    mavlink_logger.py script writes to)
  - The static frontend (2D/3D visualization)

Run:
    pip install -r requirements.txt
    uvicorn app:app --reload --host 0.0.0.0 --port 8000

Then open http://localhost:8000
"""

import sqlite3
from datetime import datetime, timedelta, timezone
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel

BASE_DIR = Path(__file__).resolve().parent
DB_PATH = BASE_DIR.parent / "database" / "flights.db"
FRONTEND_DIR = BASE_DIR.parent / "frontend"

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
               COUNT(t.id) AS point_count
        FROM flights f
        LEFT JOIN telemetry_points t ON t.flight_id = f.id
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


# --- ESP32 / offline-logger ingestion ---
# The ESP32 logs {ms, lat, lon, alt, sats, hdop} while flying (no live clock
# needed), then dumps the whole flight here once it reconnects to WiFi. We
# reconstruct real timestamps by working backward from upload_time using
# each point's ms-since-boot relative to the last point in the batch.

class TelemetryPoint(BaseModel):
    ms: int
    lat: float
    lon: float
    alt: float | None = None
    sats: int | None = None
    hdop: float | None = None
    roll: float | None = None
    pitch: float | None = None
    yaw: float | None = None


class FlightUpload(BaseModel):
    drone_name: str
    upload_time: str  # ISO 8601 UTC, e.g. "2026-07-12T20:15:00Z"
    points: list[TelemetryPoint]


@app.post("/api/ingest/flight", status_code=201)
def ingest_flight(upload: FlightUpload):
    if not upload.points:
        raise HTTPException(status_code=400, detail="No points in upload")

    upload_dt = datetime.fromisoformat(upload.upload_time.replace("Z", "+00:00"))
    last_ms = max(p.ms for p in upload.points)

    conn = sqlite3.connect(DB_PATH)
    with open(BASE_DIR.parent / "database" / "schema.sql") as f:
        conn.executescript(f.read())

    first_ts = upload_dt - timedelta(milliseconds=(last_ms - min(p.ms for p in upload.points)))
    cur = conn.execute(
        "INSERT INTO flights (drone_name, started_at, ended_at, notes) VALUES (?, ?, ?, ?)",
        (upload.drone_name, first_ts.isoformat(), upload_dt.isoformat(), "Uploaded from ESP32 flash log"),
    )
    flight_id = cur.lastrowid

    for p in upload.points:
        # Each point's real time = upload time, minus how far before the
        # last recorded point it was captured.
        point_dt = upload_dt - timedelta(milliseconds=(last_ms - p.ms))
        conn.execute(
            """
            INSERT INTO telemetry_points (
                flight_id, ts, lat, lon, alt_rel, satellites_visible, hdop,
                roll, pitch, yaw
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (flight_id, point_dt.isoformat(), p.lat, p.lon, p.alt, p.sats, p.hdop,
             p.roll, p.pitch, p.yaw),
        )

    conn.commit()
    conn.close()
    return {"flight_id": flight_id, "points_stored": len(upload.points)}


# --- Serve frontend ---
app.mount("/assets", StaticFiles(directory=FRONTEND_DIR), name="assets")


@app.get("/")
def index():
    return FileResponse(FRONTEND_DIR / "index.html")
