import asyncio
import time
import struct
from bleak import BleakClient

'''
    Script to send command to ESP. Data is transfered by notifying command characteristic
'''

# Define MAC address and characteristic UUID of ESP32
esp_mac_address = "10:91:a8:00:4c:b2"
ble_cmd_char_uuid = "e64415c6-7354-497f-b3c4-1ac4b3cdf756"

print("------------------ Command Test Script ------------------")

async def main(esp_mac_address):
    while True:
        await asyncio.sleep(1)
        try:
            print(f"Trying to connect to {esp_mac_address}")
            async with BleakClient(esp_mac_address) as device:
                while True:
                    print("Sending shutdown command")
                    await device.write_gatt_char(ble_cmd_char_uuid, bytearray(b'\x00'))
                    await asyncio.sleep(30)
        except:
            print("ESP not available or Bluetooth disabled.")



asyncio.run(main(esp_mac_address))