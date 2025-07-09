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

// Validasi parameter
if (!isset($_GET['raspiID'])) {
    http_response_code(400);
    echo json_encode(["status" => "error", "message" => "Parameter raspiID wajib diisi."]);
    exit;
}

$raspiID = (int)$_GET['raspiID'];

if ($raspiID <= 0) {
    http_response_code(400);
    echo json_encode(["status" => "error", "message" => "Parameter raspiID tidak valid."]);
    exit;
}

// Query detail terbaru untuk raspiID
$sql = "
    SELECT temperature, humidity, sound, distance_cm, rotary_value, timestamp
    FROM sensor_logs
    WHERE raspiID = $raspiID
    ORDER BY timestamp DESC
    LIMIT 1
";


$result = $conn->query($sql);

if ($result && $result->num_rows > 0) {
    $row = $result->fetch_assoc();
    
    $latest_data = [
        "temperature" => $row["temperature"],
        "humidity" => $row["humidity"],
        "sound" => $row["sound"],
        "ultrasonic" => $row["distance_cm"],
        "rotary_value" => $row["rotary_value"],
        "timestamp" => $row["timestamp"]
    ];

    echo json_encode([
        "status" => "success",
        "raspiID" => $raspiID,
        "latest_data" => $latest_data
    ], JSON_PRETTY_PRINT);

} else {
    echo json_encode([
        "status" => "error",
        "message" => "Data tidak ditemukan untuk raspiID $raspiID"
    ]);
}

$conn->close();
?>



