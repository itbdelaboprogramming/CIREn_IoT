import minimalmodbus
import time
import datetime

# Configure instrument (port name, slave address)
instrument = minimalmodbus.Instrument("COM12", 1)  # COM12, slave ID = 1
instrument.serial.baudrate = 4800
instrument.serial.bytesize = 8
instrument.serial.parity   = minimalmodbus.serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout  = 1  # seconds

print("✅ MinimalModbus configured. Starting continuous read loop...\n")

try:
    while True:
        try:
            # Read registers (0x0000 to 0x0003) -> holding registers
            temperature_raw   = instrument.read_register(0x0000, 0, functioncode=3, signed=True)
            velocity_raw      = instrument.read_register(0x0001, 0, functioncode=3, signed=True)
            displacement_raw  = instrument.read_register(0x0002, 0, functioncode=3, signed=True)
            acceleration_raw  = instrument.read_register(0x0003, 0, functioncode=3, signed=True)

            # Scale values (manual says ×10)
            temperature  = temperature_raw / 10.0
            velocity     = velocity_raw / 10.0
            displacement = displacement_raw / 10.0
            acceleration = acceleration_raw / 10.0

            # Timestamp with milliseconds
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            print(f"[{timestamp}] "
                  f"🌡 {temperature:.1f} °C | "
                  f"📈 {velocity:.1f} mm/s | "
                  f"📏 {displacement:.1f} µm | "
                  f"⚡ {acceleration:.1f} m/s²")

        except Exception as e:
            print(f"❌ Read error: {e}")

        time.sleep(1)  # wait 1 second

except KeyboardInterrupt:
    print("\n🛑 Loop stopped by user.")
