import serial
import json
import requests
import threading
from queue import Queue
from datetime import datetime
import time


# SERIAL CONFIG
SERIAL_PORT = "/dev/ttyUSB0"  # Atur sesuai port kamu
BAUD_RATE = 115200

# VPS CONFIG
VPS_API_URL = "http://103.189.235.163/ciren/api/receive-data.php"
REQUEST_TIMEOUT = 5
MAX_RETRY = 3


# QUEUE
data_queue = Queue(maxsize=1000)


# Kirim data ke VPS
def send_to_vps_worker():
   while True:
       data = data_queue.get()
       try:
           success = False
           for _ in range(MAX_RETRY):
               try:
                   response = requests.post(VPS_API_URL, json=data, timeout=REQUEST_TIMEOUT)
                   print(f"[VPS RESPONSE] {response.status_code}: {response.text}")
                   success = True
                   break
               except requests.exceptions.RequestException as e:
                   print(f"[RETRY] VPS ERROR: {e}")
                   time.sleep(1)
           if not success:
               print("[FAILED] Gagal kirim setelah retry")
       finally:
           data_queue.task_done()

# Baca data dari serial
def read_serial_loop():
   ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
   buffer = ""
   while True:
       try:
           if ser.in_waiting:
               buffer += ser.read(ser.in_waiting).decode('utf-8')
               while "\n" in buffer:
                   line, buffer = buffer.split("\n", 1)
                   print(f"[SERIAL] {line.strip()}")
                   try:
                       parsed_data = json.loads(line.strip())
                       data_queue.put(parsed_data, timeout=1)
                   except json.JSONDecodeError as e:
                       print(f"[JSON ERROR] {e}")
                   except:
                       print("[QUEUE FULL] Skipping data")
       except Exception as e:
           print(f"[SERIAL ERROR] {e}")
           time.sleep(2)

if __name__ == "__main__":
   print("[STARTING] Serial → VPS logger running")

   # Jalankan thread pengiriman ke VPS
   for _ in range(5):
       threading.Thread(target=send_to_vps_worker, daemon=True).start()

   # Jalankan pembaca serial
   read_serial_loop()















