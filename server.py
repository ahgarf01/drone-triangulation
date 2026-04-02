from flask import Flask, jsonify, send_from_directory, request
import os
import time
import math
import random

app = Flask(__name__, static_url_path="", static_folder="static")

# ---------------- MODE ----------------
# True  = old simulated demo mode
# False = real hardware mode
SIMULATION_MODE = False

# ---------------- SETTINGS ----------------
AOA_STD_DEG = 5.0
STATION_HEADING_DEG = 0.0
RANDOMIZE_EACH_REQUEST = True
SEED = None

if SEED is not None:
    random.seed(SEED)

# Fixed station positions in meters
STATIONS = {
    "station1": {"position": (0.0, 0.0), "heading": 0.0},
}

# Only used in SIMULATION_MODE = True
DRONES = [
    {"id": "f915", "frequency": 915_000_000.0, "true_position": (300.0, 250.0)},
    {"id": "f580", "frequency": 580_000_000.0, "true_position": (-420.0, 180.0)},
]

# Live data storage for hardware mode
LATEST_STATIONS = {
    name: {
        "position": [cfg["position"][0], cfg["position"][1]],
        "heading": cfg["heading"],
        "drones": [],
        "last_update": None
    }
    for name, cfg in STATIONS.items()
}

# ---------------- SIMULATION HELPERS ----------------
def angle_to(target_xy, from_xy):
    tx, ty = target_xy
    sx, sy = from_xy
    return math.atan2(ty - sy, tx - sx)

def wrap_deg(deg):
    d = deg % 360.0
    return d if d >= 0 else d + 360.0

def simulated_station_payload(name):
    s = STATIONS[name]
    sx, sy = s["position"]
    drones = []

    for d in DRONES:
        ang = angle_to(d["true_position"], s["position"]) * 180.0 / math.pi
        if RANDOMIZE_EACH_REQUEST:
            ang += random.gauss(0.0, AOA_STD_DEG)

        drones.append({
            "id": d["id"],
            "frequency": d["frequency"],
            "angle": wrap_deg(ang)
        })

    return {
        "position": [sx, sy],
        "heading": s["heading"],
        "drones": drones,
        "last_update": time.time()
    }

def get_station_payload(name):
    if SIMULATION_MODE:
        return simulated_station_payload(name)
    return LATEST_STATIONS[name]

# ---------------- READ ROUTES USED BY MAP ----------------
@app.route("/station1")
def station1():
    return jsonify(get_station_payload("station1"))

@app.route("/station2")
def station2():
    return jsonify(get_station_payload("station2"))

@app.route("/station3")
def station3():
    return jsonify(get_station_payload("station3"))

# ---------------- LIVE UPLOAD ROUTE ----------------
@app.route("/update_station/<station_id>", methods=["POST"])
def update_station(station_id):
    if station_id not in STATIONS:
        return jsonify({"ok": False, "error": "invalid station id"}), 404

    data = request.get_json(silent=True)
    if not data:
        return jsonify({"ok": False, "error": "missing JSON body"}), 400

    # Keep station positions fixed on server
    px, py = STATIONS[station_id]["position"]
    heading = float(data.get("heading", STATIONS[station_id]["heading"]))
    drones_in = data.get("drones", [])

    cleaned_drones = []
    for d in drones_in:
        if "frequency" not in d or "angle" not in d:
            continue

        cleaned_drones.append({
            "id": d.get("id", f"{station_id}_{int(float(d['frequency']))}"),
            "frequency": float(d["frequency"]),
            "angle": float(d["angle"])
        })

    LATEST_STATIONS[station_id] = {
        "position": [px, py],
        "heading": heading,
        "drones": cleaned_drones,
        "last_update": time.time()
    }

    return jsonify({
        "ok": True,
        "station": station_id,
        "count": len(cleaned_drones)
    })

# ---------------- STATUS ROUTE ----------------
@app.route("/status")
def status():
    return jsonify({
        "simulation_mode": SIMULATION_MODE,
        "stations": {
            name: {
                "last_update": info["last_update"],
                "drone_count": len(info["drones"])
            }
            for name, info in LATEST_STATIONS.items()
        }
    })

# Optional old route for sim mode only
@app.route("/drones")
def drones():
    if SIMULATION_MODE:
        return jsonify(DRONES)
    return jsonify([])

# ---------------- UI ----------------
@app.route("/")
@app.route("/interface")
def interface():
    return send_from_directory(app.static_folder, "index.html")

@app.route("/static/<path:filename>")
def static_files(filename):
    return send_from_directory(app.static_folder, filename)

if __name__ == "__main__":
    port = int(os.environ.get("PORT", "8000"))
    app.run(host="0.0.0.0", port=port, debug=True)
