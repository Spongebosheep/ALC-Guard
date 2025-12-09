from flask import Flask, jsonify, request, send_from_directory
from datetime import datetime
import csv
import os
import time

app = Flask(__name__, static_folder='.', static_url_path='')

# ALC-Guard risk model parameters
# ALC_ABV / UNITS_PER_ML are defined for a beer-like scenario
# One UK alcohol unit is approximated as 10 ml of pure ethanol
RISK_WARN_UNITS = 3.0          # ≥ 3 units in last hour is warning state
RISK_DANGER_UNITS = 6.0        # ≥ 6 units in last hour is danger state

#In-memory event list for the “current day”
events = []

# Daily target / limit for beverage volume.
# In the report this is interpreted as a “recommended maximum daily intake”.
DAILY_TARGET_ML = 1000

# For the beer use-case shown in the demo dashboard we override the ABV
# to 5% and recompute UNITS_PER_ML accordingly. This affects unit
# calculation but not the RISK_* thresholds above.
ALC_ABV = 0.05                         # 5% beer
UNITS_PER_ML = ALC_ABV / 10.0          # 1 UK unit = 10 ml pure alcohol


# Load events from events.csv at start
def load_initial_events():
    # Load previously recorded events from events.csv (if present) so that
    # the dashboard has realistic historical data immediately after start.
    # Older CSV files may not contain full metadata (e.g. 'ts').

    global events
    csv_path = "events.csv"
    if not os.path.exists(csv_path):
        print("No events.csv, start with empty events.")
        return

    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        print(f"DEBUG: CSV Headers detected: {reader.fieldnames}")

        events = []
        for row in reader:
            # Clean up possible BOM / whitespace around header names
            clean_row = {k.strip(): v for k, v in row.items() if k}

            # Time field reconstruction
            time_str = ""

            # Prefer an explicit 'time' / 'Time' column if present
            if "time" in clean_row:
                time_str = clean_row["time"]
            elif "Time" in clean_row:
                time_str = clean_row["Time"]

            # Determine date string
            date_str = ""

            # (1) Use 'date' column directly if available
            if "date" in clean_row and clean_row["date"]:
                date_str = clean_row["date"].strip()

            # (2) Otherwise, reconstruct date & time from timestamp (t_ms)
            elif "t_ms" in clean_row and clean_row["t_ms"]:
                try:
                    ts_val = float(clean_row["t_ms"])
                    if ts_val > 1_000_000_000_000:  # 13-digit ms timestamp
                        dt = datetime.fromtimestamp(ts_val / 1000.0)
                    else:
                        dt = datetime.fromtimestamp(ts_val)
                    time_str = dt.strftime("%H:%M")
                    date_str = dt.strftime("%Y-%m-%d")
                except ValueError:
                    time_str = "00:00"

            if not time_str:
                time_str = "00:00"

            ev = {
                "time":   time_str,
                "type":   clean_row.get("event", "drink"),
                "volume": float(clean_row.get("deltaMl", 0.0)),
                "temp":   float(clean_row.get("temp", 0.0)) if "temp" in clean_row else None,
            }
            if date_str:
                ev["date"] = date_str
            # Historical CSV rows do not contain per-event 'ts'; 
            # those events are excluded from the one-hour risk calculation.
            events.append(ev)

    events.sort(key=lambda x: x["time"])
    print(f"Loaded {len(events)} initial events (sorted by time).")
    if len(events) > 0:
        print(f"DEBUG: First event converted: {events[0]}")


# Main dashboard page
@app.route("/")
def index():
    return send_from_directory(".", "index.html")


# JSON data for the daily dashboard
@app.route("/day_data.json")
def day_data():
    
    # Aggregate all recorded events into summary statistics for the
    # front-end: total volume, counts, hourly pattern, gaps between
    # events, refill statistics, and progress towards the daily limit.
    
    # For this project we treat all events in memory as “today”.
    # A date filter can be re-enabled if multi-day analysis is needed.
    USE_TODAY_FILTER = False

    if USE_TODAY_FILTER:
        today_str = datetime.now().strftime("%Y-%m-%d")
        filtered = []
        for e in events:
            if "date" in e and e["date"] == today_str:
                filtered.append(e)
        working_events = filtered
    else:
        working_events = events[:]

    # Sorted list of events
    sorted_events = sorted(working_events, key=lambda x: x["time"])

    # Aggregate statistics (drink and small_sip are both counted as “alcohol intake events” in the dashboard).
    total_today = sum(
        e["volume"]
        for e in sorted_events
        if e["type"] in ("drink", "small_sip")
    )
    num_drink = sum(1 for e in sorted_events if e["type"] == "drink")
    num_sip = sum(1 for e in sorted_events if e["type"] == "small_sip")
    refill_ml = sum(e["volume"] for e in sorted_events if e["type"] == "refill")
    refill_count = sum(1 for e in sorted_events if e["type"] == "refill")

    # Hourly pattern (0–23h)
    def hour_from_time_str(tstr: str) -> int:
        try:
            h = int(tstr.split(":")[0])
            if 0 <= h < 24:
                return h
            return 0
        except Exception:
            return 0

    hourly_volume = [0.0] * 24
    for e in sorted_events:
        if e.get("type") in ("drink", "small_sip"):
            h = hour_from_time_str(e.get("time", "00:00"))
            hourly_volume[h] += float(e.get("volume", 0.0))

    # Progress towards daily target (treated as a safe upper bound)
    progress = total_today / DAILY_TARGET_ML if DAILY_TARGET_ML > 0 else 0.0

    # Gaps between drink events (minutes)
    def time_to_minutes(tstr: str) -> int:
        try:
            h, m = tstr.split(":")
            return int(h) * 60 + int(m)
        except Exception:
            return 0

    drink_events = [e for e in sorted_events if e["type"] == "drink"]
    gaps = []

    if len(drink_events) >= 2:
        last_t = time_to_minutes(drink_events[0]["time"])
        for ev in drink_events[1:]:
            t = time_to_minutes(ev["time"])
            diff = t - last_t
            if diff < 0:
                diff += 24 * 60  # handle wrap-around after midnight
            gaps.append(diff)
            last_t = t

        longest_gap_min = max(gaps)
        last_gap_min = gaps[-1]
    else:
        longest_gap_min = 0
        last_gap_min = 0

    return jsonify({
        "total_today":     total_today,
        "num_drink":       num_drink,
        "num_sip":         num_sip,
        "goal_ml":         DAILY_TARGET_ML,
        "progress":        progress,
        "longest_gap_min": longest_gap_min,
        "last_gap_min":    last_gap_min,
        "hourly_volume":   hourly_volume,
        "events":          sorted_events,
        "refill_ml":       refill_ml,
        "refill_count":    refill_count,
    })


# POST /event from ESP32 cup
@app.route("/event", methods=["POST"])
def event():
    # Receive a single intake/refill event from the ESP32 device.
    # The payload is JSON and includes both raw cup state (weight, volume) and derived volume change for this event.

    data = request.get_json(force=True)
    now_dt   = datetime.now()
    time_str = now_dt.strftime("%H:%M")
    date_str = now_dt.strftime("%Y-%m-%d")
    now_str  = time_str

    start_weight = int(data.get("startWeight", 0))
    end_weight   = int(data.get("endWeight", 0))
    start_ml     = float(data.get("startMl", 0.0))
    end_ml       = float(data.get("endMl", 0.0))
    volume       = float(data.get("volume", 0.0))
    evt_type     = data.get("type", "drink")
    temp_c       = float(data.get("temp", 0.0))

    ev = {
        "time":   now_str,
        "date":   date_str,
        "type":   evt_type,
        "volume": volume,
        "temp":   temp_c,
        "ts":     time.time(),   # timestamp used for one-hour risk calculation
    }
    events.append(ev)
    print(f"RECEIVED: {ev} | Weight: {start_weight} -> {end_weight}")

    # Append to events.csv using the original coursework format so that the raw log can still be inspected or re-analysed later.
    csv_path = "events.csv"
    if os.path.exists(csv_path):
        try:
            with open(csv_path, mode="a", newline="", encoding="utf-8-sig") as f:
                writer = csv.writer(f)
                current_t_ms = int(time.time() * 1000)
                row = [
                    len(events),
                    evt_type,
                    start_weight,
                    end_weight,
                    start_ml,
                    end_ml,
                    volume,
                    temp_c,
                    current_t_ms,
                ]
                writer.writerow(row)
                print(f"Saved to CSV: {row}")
        except Exception as e:
            print(f"Error saving to CSV: {e}")

    return jsonify({"status": "ok"})


@app.route("/api/status")
def api_status():
    # ALC-Guard one-hour risk indicator.

    # 1. Look at all events in the last 60 minutes.
    # 2. Sum their beverage volume (drink + small_sip only).
    # 3. Convert the volume into UK units using UNITS_PER_ML.
    # 4. Classify risk into:
       # 'none'    : no recent intake
       # 'safe'    : some intake but below RISK_WARN_UNITS
       # 'warning' : between RISK_WARN_UNITS and RISK_DANGER_UNITS
       # 'danger'  : above RISK_DANGER_UNITS
    # 5. Return a JSON object consumed by the front-end banner.

    now = time.time()
    one_hour_ago = now - 3600

    recent_ml = 0.0
    for e in events:
        ts = e.get("ts")
        if ts is None:
            # Historical CSV rows without 'ts' are ignored for
            # the rolling one-hour risk calculation.
            continue
        if ts < one_hour_ago:
            continue
        if e.get("type") not in ("drink", "small_sip"):
            continue
        try:
            recent_ml += float(e.get("volume", 0.0))
        except Exception:
            continue

    recent_units = recent_ml * UNITS_PER_ML

    if recent_units >= RISK_DANGER_UNITS:
        level = "danger"
        need_alert = True
    elif recent_units >= RISK_WARN_UNITS:
        level = "warning"
        need_alert = True
    elif recent_units > 0:
        level = "safe"
        need_alert = False
    else:
        level = "none"
        need_alert = False

    return jsonify({
        "need_alert": need_alert,
        "risk_level": level,
        "last_hour_units": round(recent_units, 2),
    })


if __name__ == "__main__":
    load_initial_events()
    app.run(host="0.0.0.0", port=5000, debug=True)
