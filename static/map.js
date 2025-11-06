// Leaflet-based map rendering with the same math as the canvas version.
// We keep the server JSON in meters and convert to lat/lon around a chosen origin.

const ORIGIN = { lat: 40.2783, lon: -111.7173 }; // UVU-ish; change as needed

function metersToLatLng(xEast, yNorth, origin=ORIGIN) {
  const metersPerDegLat = 111320.0;
  const metersPerDegLon = 111320.0 * Math.cos(origin.lat * Math.PI/180.0);
  const lat = origin.lat + (yNorth / metersPerDegLat);
  const lon = origin.lon + (xEast  / metersPerDegLon);
  return [lat, lon];
}

let map, stationLayer, rayLayer, droneLayer, actualDroneLayer;
function initMap() {
  map = L.map('map', { zoomAnimation: false }).setView([ORIGIN.lat, ORIGIN.lon], 14);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; OpenStreetMap contributors'
  }).addTo(map);
  stationLayer = L.layerGroup().addTo(map);
  rayLayer = L.layerGroup().addTo(map);
  droneLayer = L.layerGroup().addTo(map);
  actualDroneLayer = L.layerGroup().addTo(map);
}
initMap();

async function fetchStations() {
  const urls = ["/station1", "/station2", "/station3"];
  const results = await Promise.all(urls.map(u => fetch(u).then(r => r.json())));
  return results;
}

async function fetchDrones() {
  const result = await fetch("/drones").then(r => r.json());
  return result;
}

function deg2rad(d){ return d*Math.PI/180; }

function calculate_standard_weights(stations, p_k) {
  const weights = stations.map(s => {
    const dx = s.pos[0] - p_k[0];
    const dy = s.pos[1] - p_k[1];
    const distSq = dx * dx + dy * dy;
    return 1.0 / (distSq + 1e-9);
  });
  return weights;
}

function solveByFrequency(stations){
  const byFreq = new Map();
  stations.forEach((s, idx) => {
    (s.drones || []).forEach(d => {
      const key = d.frequency.toString();
      if (!byFreq.has(key)) byFreq.set(key, []);
      byFreq.get(key).push({
        name: "S"+(idx+1),
        pos: [s.position[0], s.position[1]],
        angleDeg: d.angle
      });
    });
  });

  const solutions = [];
  for (const [freq, ms] of byFreq.entries()) {
    if (ms.length < 2) continue;

    // Initial guess using unweighted least squares
    let p_k = estimate_position_unweighted(ms);
    if (!p_k) continue;

    // Iteratively improve the estimate using weighted least squares
    for (let i = 0; i < 10; i++) {
      const weights = calculate_standard_weights(ms, p_k);
      const p_next = estimate_position_weighted(ms, weights);
      if (!p_next) break;

      const norm = Math.sqrt(Math.pow(p_next[0] - p_k[0], 2) + Math.pow(p_next[1] - p_k[1], 2));
      if (norm < 1e-4) {
        p_k = p_next;
        break;
      }
      p_k = p_next;
    }

    const x = p_k;
    const sigmaTheta=4.2466*Math.PI/180;
    const w = 1.0 / (sigmaTheta * sigmaTheta);
    let F11 = 0, F12 = 0, F22 = 0;
    ms.forEach(m => {
      const dx = x[0] - m.pos[0];
      const dy = x[1] - m.pos[1];
      const r2 = dx*dx + dy*dy;
      if (r2 < 1e-12) return;
      const hx = -dy / r2;
      const hy = dx / r2;
      F11 += hx*hx; F12 += hx*hy; F22 += hy*hy;
    });
    F11 *= w; F12 *= w; F22 *= w;
    const tr = F11 + F22;
    const det_F = F11*F22 - F12*F12;
    const discr = tr*tr - 4*det_F;
    if (discr < 0) continue;
    const eig_1 = 0.5 * (tr + Math.sqrt(discr));
    const eig_2 = 0.5 * (tr - Math.sqrt(discr));
    if (eig_2 <= 0) continue;
    const drms = Math.sqrt(1/eig_1 + 1/eig_2);
    const err = 1.73 * drms;
    solutions.push({frequency:Number(freq), xy:x, err});
  }
  return solutions;
}

function estimate_position_unweighted(ms) {
  const A = [];
  const b = [];
  ms.forEach(m => {
    const th = deg2rad(m.angleDeg);
    const cos_a = Math.cos(th);
    const sin_a = Math.sin(th);
    A.push([-sin_a, cos_a]);
    b.push(-m.pos[0] * sin_a + m.pos[1] * cos_a);
  });

  return solve_least_squares(A, b);
}

function estimate_position_weighted(ms, weights) {
  const A = [];
  const b = [];
  ms.forEach((m, i) => {
    const th = deg2rad(m.angleDeg);
    const cos_a = Math.cos(th);
    const sin_a = Math.sin(th);
    const w = Math.sqrt(weights[i]);
    A.push([-sin_a * w, cos_a * w]);
    b.push((-m.pos[0] * sin_a + m.pos[1] * cos_a) * w);
  });

  return solve_least_squares(A, b);
}

function solve_least_squares(A, b) {
  const num_rows = A.length;
  if (num_rows < 2) return null;

  const At = [[], []];
  for (let i = 0; i < num_rows; i++) {
    At[0].push(A[i][0]);
    At[1].push(A[i][1]);
  }

  const AtA = [[0, 0], [0, 0]];
  AtA[0][0] = At[0].reduce((sum, val, i) => sum + val * A[i][0], 0);
  AtA[0][1] = At[0].reduce((sum, val, i) => sum + val * A[i][1], 0);
  AtA[1][0] = At[1].reduce((sum, val, i) => sum + val * A[i][0], 0);
  AtA[1][1] = At[1].reduce((sum, val, i) => sum + val * A[i][1], 0);

  const det = AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0];
  if (Math.abs(det) < 1e-9) return null;

  const AtA_inv = [[AtA[1][1] / det, -AtA[0][1] / det], [-AtA[1][0] / det, AtA[0][0] / det]];

  const Atb = [0, 0];
  Atb[0] = At[0].reduce((sum, val, i) => sum + val * b[i], 0);
  Atb[1] = At[1].reduce((sum, val, i) => sum + val * b[i], 0);

  return [AtA_inv[0][0] * Atb[0] + AtA_inv[0][1] * Atb[1], AtA_inv[1][0] * Atb[0] + AtA_inv[1][1] * Atb[1]];
}

function draw(stations, solutions, drones){
  stationLayer.clearLayers(); rayLayer.clearLayers(); droneLayer.clearLayers(); actualDroneLayer.clearLayers();

  // Stations
  stations.forEach((s, idx)=>{
    const [lat, lon] = metersToLatLng(s.position[0], s.position[1]);
    const marker = L.circleMarker([lat, lon], {radius:6, color:"#1f9d55", fill:true, fillOpacity:1});
    marker.bindTooltip("S"+(idx+1), {permanent:true, direction:'right'});
    marker.addTo(stationLayer);
    // Rays for each drone
    (s.drones||[]).forEach(d=>{
      const th = deg2rad(d.angle);
      const len = 2000; // 2 km ray for visualization
      const x2 = s.position[0] + Math.cos(th)*len;
      const y2 = s.position[1] + Math.sin(th)*len;
      const a = metersToLatLng(s.position[0], s.position[1]);
      const b = metersToLatLng(x2,y2);
      L.polyline([a,b], {color:"#2a6fdb", weight:2, opacity:0.9}).addTo(rayLayer);
    });
  });

  // Drone estimates
  solutions.forEach(sol=>{
    const [lat, lon] = metersToLatLng(sol.xy[0], sol.xy[1]);
    L.circleMarker([lat, lon], {radius:5, color:"#d61f1f", fill:true, fillOpacity:1})
      .bindTooltip(`${(sol.frequency/1e6).toFixed(0)} MHz`, {permanent:true, direction:'right'})
      .addTo(droneLayer);
    // error circle in meters
    L.circle([lat, lon], {radius: sol.err, color:"#d61f1f", fill:false, opacity:0.6}).addTo(droneLayer);
  });

  // Drones
  drones.forEach(drone=>{
    const [lat, lon] = metersToLatLng(drone.true_position[0], drone.true_position[1]);
    L.circleMarker([lat, lon], {radius:5, color:"#2a6fdb", fill:true, fillOpacity:1})
      .bindTooltip(`${(drone.frequency/1e6).toFixed(0)} MHz`, {permanent:true, direction:'right'})
      .addTo(actualDroneLayer);
  });

  // Fit bounds
  const latlngs = [];
  stations.forEach(s => latlngs.push(metersToLatLng(s.position[0], s.position[1])));
  solutions.forEach(sol => latlngs.push(metersToLatLng(sol.xy[0], sol.xy[1])));
  if (latlngs.length) map.fitBounds(latlngs, {padding:[30,30]});
}

function updateTable(solutions){
  const tbody = document.querySelector("#estimates tbody");
  tbody.innerHTML = "";
  solutions.sort((a,b)=>a.frequency-b.frequency).forEach(sol=>{
    const tr = document.createElement("tr");
    tr.innerHTML = `<td>${sol.frequency.toFixed(0)}</td>
                    <td>${(sol.frequency/1e6).toFixed(3)}</td>
                    <td>${sol.xy[0].toFixed(1)}</td>
                    <td>${sol.xy[1].toFixed(1)}</td>
                    <td>${sol.err.toFixed(1)}</td>`;
    tbody.appendChild(tr);
  });
}

function updateActualsTable(drones){
  const tbody = document.querySelector("#actuals tbody");
  tbody.innerHTML = "";
  drones.sort((a,b)=>a.frequency-b.frequency).forEach(drone=>{
    const tr = document.createElement("tr");
    tr.innerHTML = `<td>${drone.id}</td>
                    <td>${(drone.frequency/1e6).toFixed(3)}</td>
                    <td>${drone.true_position[0].toFixed(1)}</td>
                    <td>${drone.true_position[1].toFixed(1)}</td>`;
    tbody.appendChild(tr);
  });
}

async function refresh(){
  const stations = await fetchStations();
  const solutions = solveByFrequency(stations);
  const drones = await fetchDrones();
  draw(stations, solutions, drones);
  updateTable(solutions);
  updateActualsTable(drones);
}

document.getElementById("refresh").addEventListener("click", refresh);
window.addEventListener("load", refresh);