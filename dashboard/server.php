<?php
$servername = "localhost";
$username = "root";
$password = "";
$dbname = "new_ciren";

// Buat koneksi
$conn = new mysqli($servername, $username, $password, $dbname);

// Cek koneksi
if ($conn->connect_error) {
    die("Koneksi gagal: " . $conn->connect_error);
}

$data = [];

// Ambil jumlah sensor module
$sql = "SELECT COUNT(*) AS total FROM sensor_module";
$result = $conn->query($sql);
$row = $result->fetch_assoc();
$data['total_sensor_module'] = (int)$row['total'];

// Ambil semua ID sensor module
$sql = "SELECT id, module_id_code, status FROM sensor_module";
$result = $conn->query($sql);
$data['sensor_module_ids'] = $result->fetch_all(MYSQLI_ASSOC);

// Jika ingin ambil data sensor untuk module tertentu
$sensor_module_id = isset($_GET['id']) ? intval($_GET['id']) : 1;

// Ambil data ultrasonic
$sql = "SELECT value, status, timestamp FROM ultrasonic WHERE sensor_module_id = $sensor_module_id";
$result = $conn->query($sql);
$data['ultrasonic'] = $result->fetch_all(MYSQLI_ASSOC);

// Ambil data IMU
$sql = "SELECT value, status, timestamp FROM imu WHERE sensor_module_id = $sensor_module_id";
$result = $conn->query($sql);
$data['imu'] = $result->fetch_all(MYSQLI_ASSOC);

// Ambil data Temperature & Humidity
$sql = "SELECT value, status, timestamp FROM temp_humid WHERE sensor_module_id = $sensor_module_id";
$result = $conn->query($sql);
$data['temp_humid'] = $result->fetch_all(MYSQLI_ASSOC);

// Ambil data Proximity
$sql = "SELECT value, status, timestamp FROM prox WHERE sensor_module_id = $sensor_module_id";
$result = $conn->query($sql);
$data['prox'] = $result->fetch_all(MYSQLI_ASSOC);

// Ambil data Rotary
$sql = "SELECT value, status, timestamp FROM rotary WHERE sensor_module_id = $sensor_module_id";
$result = $conn->query($sql);
$data['rotary'] = $result->fetch_all(MYSQLI_ASSOC);

// Tampilkan hasil dalam format JSON
header('Content-Type: application/json');
echo json_encode($data, JSON_PRETTY_PRINT);

// Tutup koneksi
$conn->close();
?>
