"""
MAVLink telemetry logger for the Drone Flight Logger.

Connects to a real drone (ArduPilot / PX4 / any MAVLink-speaking flight
controller) over serial, UDP, or TCP, and streams position + attitude
telemetry into a SQLite database for later playback in the web app.

Usage:
    # USB/serial telemetry radio
    python mavlink_logger.py --conn /dev/ttyUSB0 --baud 57600 --drone "Matrice-1"

    # SITL / UDP (useful for testing without real hardware)
    python mavlink_logger.py --conn udpin:127.0.0.1:14550 --drone "SITL-Test"

Requires: pymavlink (pip install pymavlink)
"""

import argparse
import sqlite3
import time
from datetime import datetime, timezone

from pymavlink import mavutil

DB_PATH_DEFAULT = "../database/flights.db"


def utcnow_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def init_db(db_path):
    conn = sqlite3.connect(db_path)
    with open("../database/schema.sql", "r") as f:
        conn.executescript(f.read())
    conn.commit()
    return conn


def start_flight(conn, drone_name):
    cur = conn.execute(
        "INSERT INTO flights (drone_name, started_at) VALUES (?, ?)",
        (drone_name, utcnow_iso()),
    )
    conn.commit()
    return cur.lastrowid


def end_flight(conn, flight_id):
    conn.execute(
        "UPDATE flights SET ended_at = ? WHERE id = ?",
        (utcnow_iso(), flight_id),
    )
    conn.commit()


def insert_point(conn, flight_id, sample):
    conn.execute(
        """
        INSERT INTO telemetry_points (
            flight_id, ts, lat, lon, alt_rel, alt_msl,
            roll, pitch, yaw, vx, vy, vz,
            groundspeed, airspeed, battery_voltage, battery_remaining,
            satellites_visible, fix_type, hdop
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            flight_id, sample["ts"], sample["lat"], sample["lon"],
            sample.get("alt_rel"), sample.get("alt_msl"),
            sample.get("roll"), sample.get("pitch"), sample.get("yaw"),
            sample.get("vx"), sample.get("vy"), sample.get("vz"),
            sample.get("groundspeed"), sample.get("airspeed"),
            sample.get("battery_voltage"), sample.get("battery_remaining"),
            sample.get("satellites_visible"), sample.get("fix_type"),
            sample.get("hdop"),
        ),
    )
    conn.commit()


def run(conn_str, baud, drone_name, db_path, rate_hz):
    print(f"Connecting to {conn_str} ...")
    mav = mavutil.mavlink_connection(conn_str, baud=baud)
    mav.wait_heartbeat()
    print(f"Heartbeat received from system {mav.target_system}, "
          f"component {mav.target_component}")

    db = init_db(db_path)
    flight_id = start_flight(db, drone_name)
    print(f"Logging flight #{flight_id} for '{drone_name}'. Press Ctrl+C to stop.")

    # Running state assembled from multiple MAVLink message types
    state = {}
    min_interval = 1.0 / rate_hz
    last_write = 0.0

    try:
        while True:
            msg = mav.recv_match(blocking=True, timeout=2)
            if msg is None:
                continue
            mtype = msg.get_type()

            if mtype == "GLOBAL_POSITION_INT":
                state["lat"] = msg.lat / 1e7
                state["lon"] = msg.lon / 1e7
                state["alt_msl"] = msg.alt / 1000.0
                state["alt_rel"] = msg.relative_alt / 1000.0
                state["vx"] = msg.vx / 100.0
                state["vy"] = msg.vy / 100.0
                state["vz"] = msg.vz / 100.0
                state["yaw"] = msg.hdg / 100.0 if msg.hdg != 65535 else None

            elif mtype == "ATTITUDE":
                import math
                state["roll"] = math.degrees(msg.roll)
                state["pitch"] = math.degrees(msg.pitch)

            elif mtype == "VFR_HUD":
                state["groundspeed"] = msg.groundspeed
                state["airspeed"] = msg.airspeed

            elif mtype == "SYS_STATUS":
                state["battery_voltage"] = msg.voltage_battery / 1000.0
                state["battery_remaining"] = msg.battery_remaining

            elif mtype == "GPS_RAW_INT":
                state["satellites_visible"] = msg.satellites_visible
                state["fix_type"] = msg.fix_type
                state["hdop"] = msg.eph / 100.0 if msg.eph != 65535 else None

            now = time.time()
            if "lat" in state and "lon" in state and (now - last_write) >= min_interval:
                sample = dict(state)
                sample["ts"] = utcnow_iso()
                insert_point(db, flight_id, sample)
                last_write = now
                print(f"[{sample['ts']}] lat={sample['lat']:.6f} "
                      f"lon={sample['lon']:.6f} alt={sample.get('alt_rel')}")

    except KeyboardInterrupt:
        print("\nStopping logger...")
    finally:
        end_flight(db, flight_id)
        db.close()
        print(f"Flight #{flight_id} ended and saved.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Log live MAVLink telemetry to SQLite")
    parser.add_argument("--conn", required=True,
                         help="Connection string, e.g. /dev/ttyUSB0, udpin:127.0.0.1:14550, tcp:127.0.0.1:5760")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate for serial connections")
    parser.add_argument("--drone", default="Unnamed Drone", help="Name to tag this flight with")
    parser.add_argument("--db", default=DB_PATH_DEFAULT, help="Path to SQLite database file")
    parser.add_argument("--rate", type=float, default=5.0, help="Max samples per second to store")
    args = parser.parse_args()

    run(args.conn, args.baud, args.drone, args.db, args.rate)
