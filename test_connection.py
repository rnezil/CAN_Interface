import binascii
import can
from can.interfaces.serial.serial_can import SerialBus
from time import sleep

bus = SerialBus("COM4")
msg = can.Message(
    arbitration_id = 0x06900001,
    data = [0x3C, 0x4B, 0x5A, 0x69, 0x78, 0x87, 0x96, 0xA5],
    is_extended_id=True
)

i = 0
while i < 10:
    try:
        bus.send(msg)
        print(f"Message sent on {bus.channel_info}")
        reply = bus.recv(0.1)
        if reply is not None:
            print(f"Received {binascii.hexlify(reply.data)} in response.")
    except can.CanError:
        print("Message NOT sent")
    
    sleep(1)
    i += 1

i = 0
while i < 10:
    reply = bus.recv(0.1)
    if reply is not None:
        print(f"Received {binascii.hexlify(reply.data)} in response.")
    sleep(1)
    i += 1

bus.shutdown()
