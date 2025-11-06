from flask import Flask, jsonify, send_from_directory, request
import math, random, os

app = Flask(__name__, static_url_path="", static_folder="static")

# ---------------- Simulation knobs ----------------
AOA_STD_DEG = 5.0           # 1-sigma AoA noise per station (deg)
STATION_HEADING_DEG = 0.0   # Optional: facing direction to report
RANDOMIZE_EACH_REQUEST = True
SEED = None                 # Set to an int to make noise deterministic
# --------------------------------------------------

if SEED is not None:
    random.seed(SEED)

# Coordinate system: simple Cartesian meters (x east, y north)
STATIONS = {
    "station1": {"position": (-50.0,   0.0), "heading": STATION_HEADING_DEG},
    "station2": {"position": ( 50.0,   0.0), "heading": STATION_HEADING_DEG},
    "station3": {"position": (  0.0,  87.0), "heading": STATION_HEADING_DEG},
}

# Ground-truth drones (you can add/remove)
DRONES = [
    {"id": "f915", "frequency": 915_000_000.0, "true_position": (300.0, 250.0)},
    {"id": "f580", "frequency": 580_000_000.0, "true_position": (-420.0, 180.0)},
]

def angle_to(target_xy, from_xy):
    tx, ty = target_xy
    sx, sy = from_xy
    return math.atan2(ty - sy, tx - sx)  # radians

def wrap_deg(deg):
    # Return angle in [0, 360)
    d = deg % 360.0
    return d if d >= 0 else d + 360.0

def station_payload(name):
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
        "position": [sx, sy],           # (x, y) meters
        "heading": STATION_HEADING_DEG, # degrees
        "drones": drones
    }

@app.route("/station1")
def station1():
    return jsonify(station_payload("station1"))

@app.route("/station2")
def station2():
    return jsonify(station_payload("station2"))

@app.route("/station3")
def station3():
    return jsonify(station_payload("station3"))

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