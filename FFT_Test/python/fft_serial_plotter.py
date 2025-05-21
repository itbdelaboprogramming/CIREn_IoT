import serial
import serial.tools.list_ports
import json
import csv
import os
import time
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import RadioButtons

# === CONFIGURATION ===
def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Arduino" in p.description or "CH340" in p.description or "USB Serial" in p.description or "USB to UART" in p.description:
            return p.device
    return None

PORT = find_arduino_port()
if not PORT:
    print("❌ Could not find Arduino port automatically.")
    exit(1)

print(f"✅ Arduino found on port: {PORT}")

# PORT = "COM3"  # Update this with your ESP32 serial port
BAUDRATE = 115200
SAVE_FOLDER = "fft_data"
MAX_POINTS = 512  # max FFT bins per sensor
SAVEDATA = False

# === INIT ===
os.makedirs(SAVE_FOLDER, exist_ok=True)
fft_buffers = {}  # key: sensor label → value: deque of (freq, amp)
fft_lock = threading.Lock()  # NEW
current_sensor = ["All Sensors"]  # mutable for widget callback

serial_port = serial.Serial(PORT, BAUDRATE, timeout=1)


# === FILE SAVE HELPERS ===
def save_csv(label, data):
    filename = os.path.join(SAVE_FOLDER, f"{label}_{int(time.time())}.csv")
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Frequency", "Amplitude"])
        writer.writerows(data)

def save_json(label, data):
    filename = os.path.join(SAVE_FOLDER, f"{label}_{int(time.time())}.json")
    json_data = [{"freq": f, "amp": a} for f, a in data]
    with open(filename, "w") as f:
        json.dump({"label": label, "fft": json_data}, f)

# === SERIAL DATA HANDLER ===
def handle_fft_csv(label):
    data = []
    #print(f"[CSV] Starting FFT read for: {label}")  # Add this line for debug
    while True:
        line = serial_port.readline().decode(errors="ignore").strip()
        if line.startswith("$SENSOR"):  # Prevent sensor triggers inside stream
            print(f"[⚠️ Skipped] Unexpected sensor trigger in data stream: {line}")
            break
        if line.startswith("#") or line == "":
            break
        # print(f"[RAW CSV LINE] {line}")  # 👈 ADD THIS LINE
        try:
            freq, amp = map(float, line.split(","))
            data.append((freq, amp))
        except Exception as e:
            print(f"[CSV PARSE ERROR] Line: '{line}' Error: {e}")
            break
    if data:
        #print(f"[FFT] Received data for {label}")
        with fft_lock:
            fft_buffers[label] = deque(data, maxlen=MAX_POINTS)
        if SAVEDATA:
            save_csv(label, data)

def handle_fft_json(line):
    try:
        obj = json.loads(line)
        label = obj.get("label", "unknown")
        label = label.strip()
        fft = [(pt[0], pt[1]) for pt in obj["fft"]]
        with fft_lock:
            fft_buffers[label] = deque(fft, maxlen=MAX_POINTS)
        if SAVEDATA:
            save_json(label, fft)
    except Exception as e:
        print("JSON parse error:", e)

def serial_reader():
    label = None
    try:
        while serial_port.is_open:
            line = serial_port.readline().decode(errors="ignore").strip()
            if line.startswith("$SENSOR,"):
                label = line.split(",")[1]
                #print(f"[SENSOR] Switching to label: {label}")
                handle_fft_csv(label.strip())  # Do NOT read the same line again inside this
                continue  # 🔁 Skip the rest of this iteration
            elif line.startswith("{") and line.endswith("}"):
                handle_fft_json(line)
    except serial.serialutil.PortNotOpenError:
        print("Serial port not open.")
    except Exception as e:
        print("Serial Error:", e)
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        serial_port.close()
        print("Serial port closed.")

# === PLOTTING ===
fig, ax = plt.subplots(figsize=(8, 5))
plt.subplots_adjust(left=0.25, right=0.95)
ax.set_title("Real-Time FFT Spectrum")
ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("Amplitude")
ax.grid(True)

sensor_list = ["All Sensors"]
radio_ax = plt.axes([0.02, 0.3, 0.2, 0.6], frameon=True)
radio_buttons = RadioButtons(radio_ax, sensor_list)

def update_sensor_list():
    # Dynamically update radio buttons if new sensor arrives
    with fft_lock:
        current_labels = set(sensor_list[1:])  # exclude "All Sensors"
        new_labels = set(fft_buffers.keys())
    if new_labels != current_labels:
        radio_buttons.labels.clear()
        radio_buttons.ax.clear()
        sensor_list[:] = ["All Sensors"] + sorted(new_labels)
        radio_buttons.__init__(radio_ax, sensor_list)
        radio_buttons.on_clicked(on_sensor_selected)

def on_sensor_selected(label):
    current_sensor[0] = label

#radio_buttons.on_clicked(on_sensor_selected)

def update_plot(frame):
    update_sensor_list()
    ax.clear()
    ax.set_title("Real-Time FFT Spectrum")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Amplitude")
    ax.grid(True)

    selected = current_sensor[0]
    with fft_lock:
        if selected == "All Sensors":
            for label, data in fft_buffers.items():
                if len(data) > 0:
                    freqs, amps = zip(*data)
                    ax.plot(freqs, amps, label=label)
            ax.legend(loc='upper right')
        else:
            data = fft_buffers.get(selected, [])
            if len(data) > 0:
                freqs, amps = zip(*data)
                ax.plot(freqs, amps, label=selected)
                ax.legend(loc='upper right')

# === MAIN ===
if __name__ == "__main__":
    print("Starting serial listener...")
    threading.Thread(target=serial_reader, daemon=True).start()
    ani = FuncAnimation(fig, update_plot, interval=1000, cache_frame_data=False)
    plt.show()
