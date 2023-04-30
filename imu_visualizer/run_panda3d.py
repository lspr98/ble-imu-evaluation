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
    async with BleakClient(esp_mac_address) as device:
        # TODO: repeadetly try to connect if not connected
        await device.start_notify(esp_ble_char_uuid, app.onBLENotification)
        app.gui_elements["esp_connected"]["value"] = True
        app.updateGui()
        # TODO: Is there a more elegant way?
        while True:
            await asyncio.sleep(2)

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
    