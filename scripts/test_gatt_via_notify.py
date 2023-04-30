import asyncio
import time
import struct
from bleak import BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic

'''
    Script to stream quaternion data to console. Data is transfered by notification
'''

# Define MAC address and characteristic UUID of ESP32
esp_mac_address = "10:91:a8:00:4c:b2"
ble_char_uuid = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# Name of IMUs in order
sensor_names = ["BNO08x", "BNO055", "LSM6DSO32"]

# Dictionary to hold quaternions for IMU
q_dict = {}
for sensor in sensor_names:
    q_dict[sensor] = [0]*4

# Callback function
def onBLENotification(characteristic: BleakGATTCharacteristic, data: bytearray):
    for idx, sensor in enumerate(q_dict.keys()):
        q_dict[sensor] = [struct.unpack('<f', data[((i+idx)*4):(i+idx+1)*4])[0] for i in range(0, 4)]
    # Print last sensor data
    print(f"{q_dict[sensor_names[-1]]}")
    return

print(f"Sensors: {q_dict.keys()}")

async def main(esp_mac_address):
    print(f"Connecting to {esp_mac_address}")
    async with BleakClient(esp_mac_address) as device:
        # Register callback for notification
        await device.start_notify(ble_char_uuid, onBLENotification)
        # Dummy loop to keep it running
        while True:
            await asyncio.sleep(10)
            
asyncio.run(main(esp_mac_address))