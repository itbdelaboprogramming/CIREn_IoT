import serial
import re
import numpy as np
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from collections import deque
import threading

# ==== Serial Config ====
SERIAL_PORT = "COM13"   # change this to match your ESP32 receiver COM port
BAUD_RATE = 115200
SAMPLE_RATE = 1000       # Hz (adjust to match sender rate)
BUFFER_SIZE = 1024      # samples for FFT

# ==== Data Buffers ====
x_data = deque(maxlen=BUFFER_SIZE)
y_data = deque(maxlen=BUFFER_SIZE)
z_data = deque(maxlen=BUFFER_SIZE)

# ==== Regex for parsing "X:.. Y:.. Z:.." ====
pattern = re.compile(r"X:(-?\d+)\s*mg\s*\|\s*Y:(-?\d+)\s*mg\s*\|\s*Z:(-?\d+)\s*mg")

# ==== Tkinter GUI ====
root = tk.Tk()
root.title("ESP32 + LIS331HH Real-time Acceleration & FFT")

fig, (ax_time, ax_fft_x, ax_fft_y, ax_fft_z) = plt.subplots(4, 1, figsize=(8, 8))
plt.subplots_adjust(hspace=0.6)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Time series lines
line_x, = ax_time.plot([], [], label="X")
line_y, = ax_time.plot([], [], label="Y")
line_z, = ax_time.plot([], [], label="Z")
ax_time.set_title("Acceleration (Time Series)")
ax_time.set_xlabel("Samples")
ax_time.set_ylabel("g")
ax_time.legend()

# FFT subplots titles
ax_fft_x.set_title("FFT Spectrum - X")
ax_fft_y.set_title("FFT Spectrum - Y")
ax_fft_z.set_title("FFT Spectrum - Z")

for ax in (ax_fft_x, ax_fft_y, ax_fft_z):
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("g")

# ==== Serial Thread ====
def serial_reader():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            match = pattern.search(line)
            if match:
                ax_val, ay_val, az_val = map(int, match.groups())

                # convert mg -> g
                x_data.append(ax_val / 1000.0)
                y_data.append(ay_val / 1000.0)
                z_data.append(az_val / 1000.0)

threading.Thread(target=serial_reader, daemon=True).start()

# ==== Update Plot ====
def update_plot():
    if len(z_data) > 10:
        # --- Time series ---
        line_x.set_data(range(len(x_data)), list(x_data))
        line_y.set_data(range(len(y_data)), list(y_data))
        line_z.set_data(range(len(z_data)), list(z_data))
        ax_time.set_xlim(0, len(x_data))
        ymin = min(min(x_data), min(y_data), min(z_data)) - 0.1
        ymax = max(max(x_data), max(y_data), max(z_data)) + 0.1
        ax_time.set_ylim(ymin, ymax)

        # --- FFT helper ---
        def compute_fft(data):
            arr = np.array(data)
            fft_vals = np.fft.rfft(arr - np.mean(arr))
            fft_freqs = np.fft.rfftfreq(len(arr), d=1.0/SAMPLE_RATE)
            return fft_freqs, np.abs(fft_vals) / len(arr)

        # FFT X
        fx, fft_x = compute_fft(x_data)
        ax_fft_x.clear()
        ax_fft_x.plot(fx, fft_x, "b")
        ax_fft_x.set_xlim(0, SAMPLE_RATE/2)
        ax_fft_x.set_ylim(0, 3.0)   # fixed to 2 g
        ax_fft_x.set_title("FFT Spectrum - X")
        ax_fft_x.set_xlabel("Frequency [Hz]")
        ax_fft_x.set_ylabel("g")

        # FFT Y
        fy, fft_y = compute_fft(y_data)
        ax_fft_y.clear()
        ax_fft_y.plot(fy, fft_y, "g")
        ax_fft_y.set_xlim(0, SAMPLE_RATE/2)
        ax_fft_y.set_ylim(0, 3.0)   # fixed to 2 g
        ax_fft_y.set_title("FFT Spectrum - Y")
        ax_fft_y.set_xlabel("Frequency [Hz]")
        ax_fft_y.set_ylabel("g")

        # FFT Z
        fz, fft_z = compute_fft(z_data)
        ax_fft_z.clear()
        ax_fft_z.plot(fz, fft_z, "r")
        ax_fft_z.set_xlim(0, SAMPLE_RATE/2)
        ax_fft_z.set_ylim(0, 3.0)   # fixed to 2 g
        ax_fft_z.set_title("FFT Spectrum - Z")
        ax_fft_z.set_xlabel("Frequency [Hz]")
        ax_fft_z.set_ylabel("g")

    canvas.draw()
    root.after(50, update_plot)

update_plot()
root.mainloop()
