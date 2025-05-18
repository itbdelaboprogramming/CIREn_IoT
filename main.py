import can
import time
import logging
import os
import crcmod

# Logging setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("CAN_ID_SERVER")

# Database path
DB_FILE = "mac_id_database.txt"

# CRC function (CRC-16-CCITT)
crc16 = crcmod.predefined.mkPredefinedCrcFun('crc-ccitt-false')

def calculate_crc(data):
    crc = 0xFFFF  # Initial value (MODBUS standard)
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:  # LSB is set
                crc >>= 1
                crc ^= 0xA001  # MODBUS polynomial (reversed 0x8005)
            else:
                crc >>= 1
    return crc  # No final XOR needed (MODBUS)

def load_database():
    db = {}
    if not os.path.exists(DB_FILE):
        return db

    with open(DB_FILE, 'rw') as f:
        for line in f:
            mac_str, id_str = line.strip().split(',')
            db[mac_str] = int(id_str)
    return db


def save_to_database(mac, assigned_id):
    with open(DB_FILE, 'a') as f:
        f.write(f"{mac},{assigned_id}\n")


def format_mac(data):
    return ':'.join(f"{b:02X}" for b in data)


def main():
    # logger.info("Initializing CAN interface on COM3 using SLCAN...")
    bus = can.interface.Bus(channel="can0", interface="socketcan")

    logger.info("Loading MAC database...")
    database = load_database()
    next_id = max(database.values(), default=0) + 1

    logger.info("Listening for ID requests...")
    while True:
        msg = bus.recv()
        if msg is None:
            continue

        logger.info(f"Received: ID=0x{msg.arbitration_id:X}, DLC={msg.dlc}, Data={msg.data.hex()}")

        if (msg.arbitration_id & 0x7) != 5 or msg.dlc != 6:
            continue

        extracted_crc = (msg.arbitration_id >> 3) & 0xFFFF
        mac_bytes = list(msg.data[:6])
        calculated_crc = calculate_crc(mac_bytes)

        logger.info(f"Extracted CRC: 0x{extracted_crc:04X}, Calculated CRC: 0x{calculated_crc:04X}")

        if extracted_crc != calculated_crc:
            logger.warning("CRC mismatch, ignoring packet")
            continue

        mac_str = format_mac(mac_bytes)
        logger.info(f"Valid MAC: {mac_str}")

        if mac_str not in database:
            logger.info(f"MAC not registered. Assigning new ID: {next_id}")
            database[mac_str] = next_id
            save_to_database(mac_str, next_id)
            assigned_id = next_id
            next_id += 1
        else:
            assigned_id = database[mac_str]
            logger.info(f"MAC already registered with ID: {assigned_id}")

        # Create response message
        response_data = [
            assigned_id & 0xFF,        # lower byte
            (assigned_id >> 8) & 0xFF  # higher byte
        ]

        response_id = msg.arbitration_id  # Same ID as request
        response_msg = can.Message(
            arbitration_id=response_id,
            data=bytearray(response_data),
            is_extended_id=True
        )

        try:
            bus.send(response_msg)
            logger.info(f"Responded with ID={assigned_id} to MAC {mac_str}")
        except can.CanError as e:
            logger.error(f"Failed to send response: {e}")

        time.sleep(0.01)  # avoid overloading bus


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("Exiting...")
