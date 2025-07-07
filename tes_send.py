import can

def send_can_message():
    # Buat interface CAN (socketcan_native)
    bus = can.interface.Bus(channel='can0', bustype='socketcan')

    # Buat pesan CAN ID 0x123, data 01 02 03 04
    message = can.Message(
        arbitration_id=0x123,
        data=[0x01, 0x02, 0x03, 0x04],
        is_extended_id=False
    )

    try:
        bus.send(message)
        print("Pesan CAN berhasil dikirim.")
    except can.CanError:
        print("Gagal mengirim pesan CAN.")

if __name__ == "__main__":
    send_can_message()
