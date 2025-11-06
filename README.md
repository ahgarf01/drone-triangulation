# Drone Triangulation — Leaflet Map Version

This version adds a **real basemap** using **Leaflet + OpenStreetMap tiles** while keeping the same Flask JSON
API (`/station1`, `/station2`, `/station3`) and **browser-side triangulation**.

- Stations and drones are still defined in **local meters (x east, y north)** in `server.py`.
- The web client converts those to **latitude/longitude** around a chosen origin (`ORIGIN` in `static/map.js`).
- Bearings, station markers, estimated drone positions, and the **1σ error circle** are rendered on the map.

## Quick start
```bash
pip install Flask
python server.py
# then open http://127.0.0.1:8000/interface
```

## Changing the map location
Edit `static/map.js`:
```js
const ORIGIN = { lat: 40.2783, lon: -111.7173 };
```
This is the reference point for converting meters → lat/lon (equirectangular approximation).
Put it near your master station or the centroid of your layout.

## Notes
- You can keep these endpoints identical when porting to Django.
- To change station geometry or add drones, edit `STATIONS` and `DRONES` in `server.py`.