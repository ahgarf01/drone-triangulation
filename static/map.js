const ORIGIN_LAT = 40.2780;
const ORIGIN_LON = -111.7135;

const map = L.map("map").setView([ORIGIN_LAT, ORIGIN_LON], 17);

L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
  maxZoom: 22,
  attribution: '&copy; OpenStreetMap contributors'
}).addTo(map);

const stationLayer = L.layerGroup().addTo(map);
const rayLayer = L.layerGroup().addTo(map);

function deg2rad(d) {
  return d * Math.PI / 180;
}

// x = east meters, y = north meters
function metersToLatLng(x, y) {
  const dLat = y / 111320;
  const dLon = x / (111320 * Math.cos(ORIGIN_LAT * Math.PI / 180));
  return [ORIGIN_LAT + dLat, ORIGIN_LON + dLon];
}

async function fetchStation() {
  const r = await fetch("/station1");
  if (!r.ok) throw new Error("Failed to fetch /station1");
  return await r.json();
}

function drawSingleStation(station) {
  stationLayer.clearLayers();
  rayLayer.clearLayers();

  if (!station || !station.position) return;

  const [sx, sy] = station.position;
  const [slat, slon] = metersToLatLng(sx, sy);

  L.circleMarker([slat, slon], {
    radius: 7,
    color: "#1f9d55",
    fillColor: "#1f9d55",
    fillOpacity: 1
  })
    .bindTooltip("S1", { permanent: true, direction: "right" })
    .addTo(stationLayer);

  const drones = station.drones || [];
  drones.forEach((d) => {
    const angle = Number(d.angle || 0);
    const len = 250; // meters of displayed bearing line
    const th = deg2rad(angle);

    const ex = sx + len * Math.cos(th);
    const ey = sy + len * Math.sin(th);

    const a = metersToLatLng(sx, sy);
    const b = metersToLatLng(ex, ey);

    L.polyline([a, b], {
      color: "#2a6fdb",
      weight: 4,
      opacity: 0.9
    })
      .bindTooltip(
        `${d.id || "signal"} | ${(Number(d.frequency || 0) / 1e6).toFixed(3)} MHz | ${angle.toFixed(1)}°`
      )
      .addTo(rayLayer);
  });

  map.setView([slat, slon], 18);
}

function updateTable(station) {
  const tbody = document.querySelector("#estimates tbody");
  if (!tbody) return;

  tbody.innerHTML = "";

  const drones = (station && station.drones) ? station.drones : [];

  drones.forEach((d) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${d.id || "-"}</td>
      <td>${(Number(d.frequency || 0) / 1e6).toFixed(3)}</td>
      <td>${Number(d.angle || 0).toFixed(1)}°</td>
      <td>${station.last_update ? new Date(station.last_update * 1000).toLocaleTimeString() : "-"}</td>
    `;
    tbody.appendChild(tr);
  });
}

async function refresh() {
  try {
    const station = await fetchStation();
    drawSingleStation(station);
    updateTable(station);
  } catch (err) {
    console.error(err);
  }
}

document.getElementById("refresh").addEventListener("click", refresh);

refresh();
setInterval(refresh, 1000);
