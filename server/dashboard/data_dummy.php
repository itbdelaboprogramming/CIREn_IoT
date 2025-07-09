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

$sensor_module_id = 1;  // Ubah sesuai kebutuhan
$status = "active";

// Hapus data ultrasonic lama untuk sensor_module_id tertentu (opsional)
$sqlDelete = "DELETE FROM ultrasonic WHERE sensor_module_id = $sensor_module_id";
$conn->query($sqlDelete);

// Masukkan 30 data dummy
for ($i = 30; $i >= 1; $i--) {
    // Membuat nilai acak antara 80.0 dan 90.0 cm
    $value = rand(800, 900) / 10;
    // Timestamp: sekarang dikurangi $i menit
    $timestamp = date("Y-m-d H:i:s", strtotime("-$i minutes"));
    
    $sqlInsert = "INSERT INTO ultrasonic (sensor_module_id, value, status, timestamp)
                  VALUES ($sensor_module_id, '$value', '$status', '$timestamp')";
    
    if (!$conn->query($sqlInsert)) {
        echo "Error: " . $conn->error;
    }
}

echo "30 dummy data ultrasonic berhasil dimasukkan.";

$conn->close();
?>
