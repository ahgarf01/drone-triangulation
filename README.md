# Drone Triangulation

This web application provides a real-time map interface for drone triangulation. It displays the estimated and actual positions of drones based on data from multiple stations.

## Features

- **Real-time Map:** Uses Leaflet and OpenStreetMap to display a real-time map with drone and station positions.
- **Position Estimation:** Calculates the estimated position of drones using the Iterative Weighted Least Squares (IWLS) algorithm.
- **Simulation Mode:** The application can be run in a simulation mode with configurable noise and drone positions.

## Quick Start

1. **Install dependencies:**

   ```bash
   pip install -r requirements.txt
   ```

2. **Run the server:**

   ```bash
   python server.py
   ```

3. **Open the interface:**

   Open your web browser and navigate to `http://127.0.0.1:8000/`

## Configuration

- **Map Location:** To change the map's origin, edit the `ORIGIN` constant in `static/map.js`.
- **Simulation Parameters:** To change the simulation parameters, such as the number of drones, their positions, and the noise level, edit the `DRONES` and `AOA_STD_DEG` variables in `server.py`.
