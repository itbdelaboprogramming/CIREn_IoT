<?php
header("Content-Type: application/json");
date_default_timezone_set("Asia/Tokyo");

// Validasi metode
if ($_SERVER['REQUEST_METHOD'] !== 'POST') {
    http_response_code(405);
    echo json_encode(["status" => "error", "message" => "Only POST allowed"]);
    exit;
}

// Ambil dan decode input JSON
$input = json_decode(file_get_contents("php://input"), true);

if (!$input) {
    http_response_code(400);
    echo json_encode(["status" => "error", "message" => "Invalid JSON"]);
    exit;
}

// Ambil data dari JSON
$raspiId = $input["raspiID"] ?? 1;
$temperature = $input["temperature"] ?? 0;
$humidity = $input["humidity"] ?? 0;
$sound = $input["sound"] ?? 0;
$distance_cm = $input["distance_cm"] ?? 0;
$rotary_value = $input["rotary_value"] ?? 0;
$created_at = date("Y-m-d H:i:s");

// Konfigurasi database
$host = "localhost";
$user = "root";
$pass = ""; // default password MySQL (ubah jika tidak kosong)
$dbname = "ciren";

// Koneksi database
$conn = new mysqli($host, $user, $pass, $dbname);
if ($conn->connect_error) {
    http_response_code(500);
    echo json_encode(["status" => "error", "message" => "Database connection failed"]);
    exit;
}

// Persiapan dan eksekusi query
$stmt = $conn->prepare("INSERT INTO sensor_logs 
    (raspiID, temperature, humidity, sound, distance_cm, rotary_value, timestamp) 
    VALUES (?, ?, ?, ?, ?, ?, ?)");

$stmt->bind_param("ddiiiis",
    $raspiId, $temperature, $humidity, $sound, $distance_cm, $rotary_value, $created_at);

if ($stmt->execute()) {
    echo json_encode(["status" => "success", "message" => "Data saved"]);
} else {
    http_response_code(500);
    echo json_encode(["status" => "error", "message" => "Insert failed"]);
}

// Tutup koneksi
$stmt->close();
$conn->close();
?>





