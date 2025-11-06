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

let map, stationLayer, rayLayer, droneLayer;
function initMap() {
  map = L.map('map').setView([ORIGIN.lat, ORIGIN.lon], 14);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; OpenStreetMap contributors'
  }).addTo(map);
  stationLayer = L.layerGroup().addTo(map);
  rayLayer = L.layerGroup().addTo(map);
  droneLayer = L.layerGroup().addTo(map);
}
initMap();

async function fetchStations() {
  const urls = ["/station1", "/station2", "/station3"];
  const results = await Promise.all(urls.map(u => fetch(u).then(r => r.json())));
  return results;
}

function deg2rad(d){ return d*Math.PI/180; }

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
    let A = [[0,0],[0,0]], c=[0,0];
    ms.forEach(m => {
      const th = deg2rad(m.angleDeg);
      const bx=Math.cos(th), by=Math.sin(th);
      const P = [[1-bx*bx, -bx*by],[-bx*by, 1-by*by]];
      A[0][0]+=P[0][0]; A[0][1]+=P[0][1];
      A[1][0]+=P[1][0]; A[1][1]+=P[1][1];
      c[0]+=P[0][0]*m.pos[0] + P[0][1]*m.pos[1];
      c[1]+=P[1][0]*m.pos[0] + P[1][1]*m.pos[1];
    });
    const det=A[0][0]*A[1][1]-A[0][1]*A[1][0];
    if (Math.abs(det)<1e-9) continue;
    const invA=[[ A[1][1]/det, -A[0][1]/det],[-A[1][0]/det, A[0][0]/det]];
    const x=[invA[0][0]*c[0]+invA[0][1]*c[1], invA[1][0]*c[0]+invA[1][1]*c[1]];
    const sigmaTheta=5*Math.PI/180;
    const covTrace=(invA[0][0]+invA[1][1])*(sigmaTheta*sigmaTheta);
    const err=Math.sqrt(Math.max(0,covTrace));
    solutions.push({frequency:Number(freq), xy:x, err});
  }
  return solutions;
}

function draw(stations, solutions){
  stationLayer.clearLayers(); rayLayer.clearLayers(); droneLayer.clearLayers();

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

async function refresh(){
  const stations = await fetchStations();
  const solutions = solveByFrequency(stations);
  draw(stations, solutions);
  updateTable(solutions);
}

document.getElementById("refresh").addEventListener("click", refresh);
window.addEventListener("load", refresh);