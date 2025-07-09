const params = new URLSearchParams(window.location.search);
const currentId = params.get("id");

function tryParseJSON(value) {
    try {
        return JSON.parse(value);
    } catch {
        return null;
    }
}

async function loadSensorData() {
    console.log("currentId ",currentId);
    console.log("params ",params);
    
    const res = await fetch(`get_sensor_detail.php?${params}`);
    const data = await res.json();

    renderSensorCards(data);
    renderUltrasonicTable(data.ultrasonic);
    renderUltrasonicChart(data.ultrasonic);
}

function renderSensorCards(data) {
  const detail = data.latest_data;
  const orientation = detail?.orientation;

  const container = document.getElementById("sensor-data");
  container.innerHTML = `
      <div class="metric-card">
      <div class="label">🌡️ Distance</div>
      <div class="value">${detail?.ultrasonic ?? "--"} cm</div>
    </div>
    <div class="metric-card">
      <div class="label">🌡️ Temperature</div>
      <div class="value">${detail?.temperature ?? "--"} °C</div>
    </div>
    <div class="metric-card">
      <div class="label">💧 Humidity</div>
      <div class="value">${detail?.humidity ?? "--"} %</div>
    </div>
    <div class="metric-card">
      <div class="label">💧 Noice</div>
      <div class="value">${detail?.sound ?? "--"}</div>
    </div>
    <div class="metric-card">
      <div class="label">⏱️ Last Update</div>
      <div class="value">${detail?.timestamp ?? "--"}</div>
    </div>
  `;
}




function renderUltrasonicTable(ultrasonic) {
    const tableBody = document.querySelector("#ultrasonic-data tbody");
    tableBody.innerHTML = "";

    ultrasonic.forEach(row => {
        const tr = document.createElement("tr");
        tr.innerHTML = `
      <td style="padding: 6px 10px; border-bottom: 1px solid #eee;">${row.timestamp}</td>
      <td style="padding: 6px 10px; border-bottom: 1px solid #eee;">${parseFloat(row.value).toFixed(2)} cm</td>
    `;
        tableBody.appendChild(tr);
    });
}

function renderUltrasonicChart(ultrasonic) {
    const labels = ultrasonic.map(item => item.timestamp);
    const values = ultrasonic.map(item => parseFloat(item.value));

    const ctx = document.getElementById("ultrasonicChart").getContext("2d");

    new Chart(ctx, {
        type: "line",
        data: {
            labels: labels,
            datasets: [{
                label: "Ultrasonic (cm)",
                data: values,
                fill: true,
                borderColor: "#006699",
                backgroundColor: "rgba(0, 102, 153, 0.1)",
                tension: 0.3
            }]
        },
        options: {
            responsive: true,
            plugins: {
                legend: {
                    display: false,
                },
            },
            scales: {
                x: {
                    ticks: {
                        display: false, // HILANGKAN LABEL DI X
                    },
                    grid: {
                        display: false, // Opsional: hilangkan garis grid juga
                    },
                },
            },
        },
    });
}

async function initModuleButtons() {
    const res = await fetch(`get_sensor_detail.php?${params}`);
    const data = await res.json();
    const buttonGroup = document.getElementById("module-button-group");
    buttonGroup.innerHTML = "";

    data.sensor_module_ids.forEach(mod => {
        const idNumber = mod.module_id_code.replace("MOD-", "");
        const btn = document.createElement("button");
        btn.textContent = idNumber;
        btn.className = "module-switch-btn";
        if (mod.id == currentId) btn.classList.add("active");

        btn.onclick = () => {
            if (mod.id != currentId) {
                window.location.href = `detail.html?id=${mod.id}`;
            }
        };
        buttonGroup.appendChild(btn);
    });
}

function toggleView() {
    const table = document.querySelector(".data-table");
    const canvas = document.getElementById("ultrasonicChart");

    if (isTable) {
        table.style.display = "none";
        canvas.style.display = "block";
    } else {
        table.style.display = "block";
        canvas.style.display = "none";
    }

    isTable = !isTable;
}

function exportCSV() {
    const rows = [["Timestamp", "Value (cm)"]];
    const tableRows = document.querySelectorAll("#ultrasonic-data tbody tr");

    tableRows.forEach(row => {
        const cols = row.querySelectorAll("td");
        const timestamp = cols[0]?.innerText;
        const value = cols[1]?.innerText;
        rows.push([timestamp, value]);
    });

    const csvContent = "data:text/csv;charset=utf-8," + rows.map(e => e.join(",")).join("\n");
    const encodedUri = encodeURI(csvContent);
    const link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    link.setAttribute("download", "ultrasonic_data.csv");
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
}

let isModalTable = true;

function toggleModalView() {
    const tableView = document.getElementById("modal-table-view");
    const chartView = document.getElementById("modalUltrasonicChart");

    if (isModalTable) {
        tableView.style.display = "none";
        chartView.style.display = "block";
        renderModalChart(); // Render chart saat pertama kali muncul
    } else {
        tableView.style.display = "block";
        chartView.style.display = "none";
    }

    isModalTable = !isModalTable;
}

function openModal() {
    document.getElementById("modal-overlay").style.display = "flex";

    const modalTableBody = document.querySelector("#modal-ultrasonic-table tbody");
    const mainTableRows = document.querySelectorAll("#ultrasonic-data tbody tr");

    modalTableBody.innerHTML = "";

    mainTableRows.forEach(row => {
        const clone = row.cloneNode(true);
        modalTableBody.appendChild(clone);
    });

    // Reset ke table mode
    document.getElementById("modal-table-view").style.display = "block";
    document.getElementById("modalUltrasonicChart").style.display = "none";
    isModalTable = true;
}

function closeModal() {
    document.getElementById("modal-overlay").style.display = "none";
}

function renderModalChart() {
    const rows = document.querySelectorAll("#ultrasonic-data tbody tr");
    const labels = [];
    const values = [];

    rows.forEach(row => {
        const cells = row.querySelectorAll("td");
        labels.push(cells[0].textContent);
        values.push(parseFloat(cells[1].textContent));
    });

    const ctx = document.getElementById("modalUltrasonicChart").getContext("2d");

    // Hapus chart sebelumnya (jika ada)
    if (window.modalChartInstance) {
        window.modalChartInstance.destroy();
    }

    window.modalChartInstance = new Chart(ctx, {
        type: "line",
        data: {
            labels: labels,
            datasets: [{
                label: "Ultrasonic (cm)",
                data: values,
                fill: true,
                borderColor: "#006699",
                backgroundColor: "rgba(0, 102, 153, 0.1)",
                tension: 0.3
            }]
        },
        options: {
            responsive: true,
            plugins: {
                legend: { display: false }
            },
            scales: {
                x: {
                    ticks: {
                        display: true, // ✅ tampilkan ticks
                        callback: function (value, index, ticks) {
                            // Format: HH:MM dari timestamp
                            const label = this.getLabelForValue(value);
                            const date = new Date(label);
                            return `${date.getHours()}:${String(date.getMinutes()).padStart(2, "0")}`;
                        }
                    },
                    grid: {
                        display: true // ✅ tampilkan grid jika diinginkan
                    }
                },

                y: {
                    title: {
                        display: true,
                        text: "Jarak (cm)"
                    },
                    beginAtZero: true
                }
            }
        }
    });
}

// Init
let isTable = true;
loadSensorData();
initModuleButtons();

setInterval(loadSensorData, 100);
// setInterval(loadSensorModules, 5000);
// setInterval(updateGPS, 5000);