<?php
header('Content-Type: application/json');

// Koneksi database
$servername = "localhost";
$username = "root";
$password = "";
$dbname = "ciren";

$conn = new mysqli($servername, $username, $password, $dbname);
if ($conn->connect_error) {
    http_response_code(500);
    echo json_encode(["status" => "error", "message" => "Koneksi gagal"]);
    exit;
}

// Pastikan timezone konsisten
date_default_timezone_set("Asia/Tokyo");

$current_time = time();
$active_threshold = 120; // 2 menit = 120 detik

// Ambil semua raspiID unik dan waktu terakhir
$sql = "
    SELECT raspiID, MAX(timestamp) as last_seen
    FROM sensor_logs
    GROUP BY raspiID
    ORDER BY raspiID ASC
";

$result = $conn->query($sql);

$sensor_modules = [];
$total_active = 0;

if ($result) {
    while ($row = $result->fetch_assoc()) {
        $raspiID = (int)$row['raspiID'];
        $last_seen = $row['last_seen'];

        if ($last_seen === null || strtotime($last_seen) === false) {
            $status = "inactive";
            $last_seen_formatted = null;
        } else {
            $last_seen_unix = strtotime($last_seen);
            $last_seen_formatted = date("Y-m-d H:i:s", $last_seen_unix);

            $status = ($current_time - $last_seen_unix <= $active_threshold) ? "active" : "inactive";
        }

        if ($status === "active") {
            $total_active++;

            // Ambil data detail terbaru untuk device ini
            $detail_sql = "
                SELECT 
                    temperature,
                    humidity,
                    distance_cm,
                    sound,
                    rotary_value,
                    timestamp
                FROM sensor_logs
                WHERE raspiID = $raspiID
                ORDER BY timestamp DESC
                LIMIT 1
            ";

            $detail_result = $conn->query($detail_sql);
            $details = null;

            if ($detail_result && $detail_result->num_rows > 0) {
                $details_row = $detail_result->fetch_assoc();
                $details = [
                    "temperature" => $details_row["temperature"],
                    "humidity" => $details_row["humidity"],
                    "ultrasonic" => $details_row["distance_cm"],
                    "rotary_value" => $details_row["rotary_value"],
                    "sound" => $details_row["sound"],
                    "timestamp" => $details_row["timestamp"]
                ];
            }

            $sensor_modules[] = [
                "raspiID" => $raspiID,
                "last_seen" => $last_seen_formatted,
                "status" => $status,
                "latest_data" => $details
            ];
        }
    }
}

echo json_encode([
    "status" => "success",
    "total_sensor_module" => $total_active,
    "sensor_modules" => $sensor_modules
], JSON_PRETTY_PRINT);

$conn->close();
?>



