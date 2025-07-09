// Dummy initial location
let lat = -6.2;
let lng = 106.816666;

const map = L.map("map").setView([lat, lng], 13);

L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    attribution: "© OpenStreetMap contributors"
}).addTo(map);

const marker = L.marker([lat, lng])
    .addTo(map)
    .bindPopup("📍 Initial Position")
    .openPopup();

// Update time every second
function updateTime() {
    const now = new Date();
    document.getElementById("time").textContent = now.toLocaleTimeString();
}

// Simulasi GPS updates
function updateGPS() {
    lat += (Math.random() - 0.5) * 0.01;
    lng += (Math.random() - 0.5) * 0.01;

    marker
        .setLatLng([lat, lng])
        .setPopupContent(`📍 New Position<br>Lat: ${lat.toFixed(6)}<br>Lng: ${lng.toFixed(6)}`);
    map.setView([lat, lng]);
}


function loadSensorModules() {
    fetch("server.php")
        .then((res) => res.json())
        .then((data) => {
            // Update jumlah sensor module
            document.getElementById("sensor-count").textContent = data.total_sensor_module;

            // Tampilkan tombol
            const list = document.getElementById("module-list");
            list.innerHTML = "";

            data.sensor_modules.forEach((mod) => {
                
                // const idNumber = mod.module_id_code.replace("MOD-", "");
                const btn = document.createElement("button");
                btn.textContent = `${mod.raspiID}`;
                btn.className = "module-btn";
                btn.onclick = () => {
                    window.location.href = `detail.html?raspiID=${mod.raspiID}`;
                };
                list.appendChild(btn);
            });
        });
}


loadSensorModules();


setInterval(updateTime, 1000);
setInterval(loadSensorModules, 5000);
setInterval(updateGPS, 5000);

updateTime();
updateGPS();