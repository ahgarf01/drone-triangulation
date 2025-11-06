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
    let A = Array(ms.length).fill(0).map(() => Array(2).fill(0));
    let b = Array(ms.length).fill(0);
    ms.forEach((m, i) => {
      const th = deg2rad(m.angleDeg);
      const cos_a = Math.cos(th);
      const sin_a = Math.sin(th);
      A[i][0] = -sin_a;
      A[i][1] = cos_a;
      b[i] = -m.pos[0] * sin_a + m.pos[1] * cos_a;
    });

    // Initial guess with unweighted least squares
    let p_k;
    try {
      const A_t = [[A[0][0], A[1][0]], [A[0][1], A[1][1]]];
      const A_t_A = [[A_t[0][0]*A[0][0] + A_t[0][1]*A[1][0], A_t[0][0]*A[0][1] + A_t[0][1]*A[1][1]], [A_t[1][0]*A[0][0] + A_t[1][1]*A[1][0], A_t[1][0]*A[0][1] + A_t[1][1]*A[1][1]]];
      const det = A_t_A[0][0]*A_t_A[1][1] - A_t_A[0][1]*A_t_A[1][0];
      if (Math.abs(det) < 1e-9) continue;
      const A_t_A_inv = [[A_t_A[1][1]/det, -A_t_A[0][1]/det], [-A_t_A[1][0]/det, A_t_A[0][0]/det]];
      const A_t_b = [A_t[0][0]*b[0] + A_t[0][1]*b[1], A_t[1][0]*b[0] + A_t[1][1]*b[1]];
      p_k = [A_t_A_inv[0][0]*A_t_b[0] + A_t_A_inv[0][1]*A_t_b[1], A_t_A_inv[1][0]*A_t_b[0] + A_t_A_inv[1][1]*A_t_b[1]];
    } catch (e) {
      continue;
    }

    for (let i = 0; i < 10; i++) {
      const weights = calculate_standard_weights(ms, p_k);
      const W = Array(ms.length).fill(0).map(() => Array(ms.length).fill(0));
      weights.forEach((w, j) => W[j][j] = w);

      const A_t = [[A[0][0], A[1][0]], [A[0][1], A[1][1]]];
      const A_t_W = [[A_t[0][0]*W[0][0], A_t[0][1]*W[1][1]], [A_t[1][0]*W[0][0], A_t[1][1]*W[1][1]]];
      const A_t_W_A = [[A_t_W[0][0]*A[0][0] + A_t_W[0][1]*A[1][0], A_t_W[0][0]*A[0][1] + A_t_W[0][1]*A[1][1]], [A_t_W[1][0]*A[0][0] + A_t_W[1][1]*A[1][0], A_t_W[1][0]*A[0][1] + A_t_W[1][1]*A[1][1]]];
      const det = A_t_W_A[0][0]*A_t_W_A[1][1] - A_t_W_A[0][1]*A_t_W_A[1][0];
      if (Math.abs(det) < 1e-9) break;

      const A_t_W_A_inv = [[A_t_W_A[1][1]/det, -A_t_W_A[0][1]/det], [-A_t_W_A[1][0]/det, A_t_W_A[0][0]/det]];
      const A_t_W_b = [A_t_W[0][0]*b[0] + A_t_W[0][1]*b[1], A_t_W[1][0]*b[0] + A_t_W[1][1]*b[1]];
      const p_next = [A_t_W_A_inv[0][0]*A_t_W_b[0] + A_t_W_A_inv[0][1]*A_t_W_b[1], A_t_W_A_inv[1][0]*A_t_W_b[0] + A_t_W_A_inv[1][1]*A_t_W_b[1]];

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
    const eig_min = 0.5 * (tr - Math.sqrt(discr));
    if (eig_min <= 0) continue;
    const err = Math.sqrt(1.0/eig_min);
    solutions.push({frequency:Number(freq), xy:x, err});
  }
  return solutions;
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