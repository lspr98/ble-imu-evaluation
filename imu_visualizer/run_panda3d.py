from src.app import IMUVis
import asyncio
import time
from bleak import BleakClient

esp_mac_address = '10:91:A8:00:4C:B2'
esp_ble_char_uuid = "beb5483e-36e1-4688-b7f5-ea07361b26a8"


async def gameLoop(app):
    while True:
        app.taskMgr.step()
        await asyncio.sleep(0.005)
    return 0


async def BLEread(app):
    async with BleakClient(esp_mac_address) as device:
        while True:
            data = await device.read_gatt_char(esp_ble_char_uuid)
            app.setQuartFromByteArray(data)
            await asyncio.sleep(0.05)

async def BLEListen(app):
    while True:
        # repeadetly try to connect if not connected
        await asyncio.sleep(1)
        try:
            print(f"Trying to connect to ESP with MAC {esp_mac_address}")
            async with BleakClient(esp_mac_address, timeout=5.0) as device:
                if device.is_connected:
                    print("Connected.")
                    await device.start_notify(esp_ble_char_uuid, app.onBLENotification)
                else:
                    print("Device not found.")
                
                # Poll connection status every two seconds
                while True:
                    app.onDeviceConnectionUpdate(device)
                    if not device.is_connected:
                        print("Device disconnected.")
                        break
                    await asyncio.sleep(2)
        except:
            print("Device not found.")

    return 0

async def main():
    app = IMUVis()
    app.gui_elements["esp_mac"]["value"] = esp_mac_address
    app.updateGui()

    f_async1 = loop.create_task(gameLoop(app))
    f_async2 = loop.create_task(BLEListen(app))
    await asyncio.wait([f_async1, f_async2])
    

if __name__=="__main__":
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(main())
        loop.close()
    except KeyboardInterrupt:
        loop.close()
    