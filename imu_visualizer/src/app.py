from direct.showbase.ShowBase import ShowBase
from direct.actor.Actor import Actor
from direct.task import Task
from direct.gui.DirectFrame import DirectFrame
from direct.gui.DirectLabel import DirectLabel
from direct.gui.DirectButton import DirectButton
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import Quat, WindowProperties, TextNode, LVecBase3f
from bleak import BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic

import simplepbr

import asyncio
import struct
import time
import copy
import numpy as np

class IMUVis(ShowBase):

    def __init__(self):
        ShowBase.__init__(self)

        # Set windows size
        wp = WindowProperties()
        wp.setSize(1920, 1080)
        self.win.requestProperties(wp)


        simplepbr.init()

        # Current orientation of each sensor expressed as quaternions
        self.q_bno08x = Quat()
        self.q_bno055 = Quat()
        self.q_lsm6ds = Quat()

        # Default Rotation offset quaternion
        self.q_offset = Quat()
        self.q_offset.setHpr(LVecBase3f(-90, 0, 0))
        
        # Individual rotation offsets
        self.q_offset_bno08x = self.q_offset
        self.q_offset_bno055 = self.q_offset
        self.q_offset_lsm6ds = self.q_offset


        self.gui = DirectFrame(
            frameColor=(0, 0, 0, 0.1),
            pos=(-1.75, 0, 0.9),
            frameSize=(-0.05, 0.8, -0.2, 0.1))


        # Set GUI text params
        self.gui_text_label_params = {
            "parent": self.gui,
            "text_scale": 0.05,
            "text_bg": (0, 0, 0, 0),
            "text_fg": (0, 0, 0, 1),
            "frameColor": (0, 0, 0, 0),
            "text_align": TextNode.ALeft
        }
        self.gui_text_value_params = {
            "parent": self.gui,
            "text_scale": 0.05,
            "text_bg": (0, 0, 0, 0),
            "text_fg": (0, 0, 0, 1),
            "frameColor": (0, 0, 0, 0),
            "text_align": TextNode.ARight
        }

        self.gui_elements = {}
        self.gui_elements["esp_mac"] = {
            "label": "MAC:",
            "value": "-",
            "unit": "",
        }
        self.gui_elements["esp_connected"] = {
            "label": "Connection Status:",
            "value": False,
            "unit": ""
        }
        self.gui_elements["esp_notify_rate"] = {
            "label": "Notification Rate:",
            "value": 0,
            "unit": "Hz"
        }

        for idx, gui_element_name in enumerate(self.gui_elements.keys()):
            self.gui_elements[gui_element_name]["label_node"] = DirectLabel(
                                                                    text=self.gui_elements[gui_element_name]["label"], 
                                                                    text_pos=(0, -0.075*idx),
                                                                    **self.gui_text_label_params)
            self.gui_elements[gui_element_name]["value_node"] = DirectLabel(
                                                                    text=self.gui_elements[gui_element_name]["unit"],
                                                                    text_pos=(0.7, -0.075*idx),
                                                                    **self.gui_text_value_params)

        # Create command buttons
        self.zero_button = DirectButton(
            parent=self.gui,
            frameColor=(0, 0, 0, 0),
            text_fg=(1, 1, 1, 1),
            text_bg=(1, 0, 0, 0.3),
            text=("Zero Orientation"), 
            scale=0.075, 
            pos=(3.215, 0, -0.1), 
            command=self.onZeroOrientationButtonPressed)
        
        # Create sensor labels
        self.label_bno055 = DirectLabel(
            parent=self.gui, 
            text="BNO055", 
            pos=(1.1, 0, -0.8), 
            scale=0.1, 
            frameColor=(0, 0, 0, 0))
        self.label_bno08x = DirectLabel(
            parent=self.gui, 
            text="BNO08X", 
            pos=(1.8, 0, -0.8), 
            scale=0.1, 
            frameColor=(0, 0, 0, 0))
        self.label_lsm6dso = DirectLabel(
            parent=self.gui, 
            text="LSM6DSO", 
            pos=(2.5, 0, -0.8), 
            scale=0.1, 
            frameColor=(0, 0, 0, 0))

        # Track amount of recieved IMU dataframes and time delta to estimate rate
        self.n_recv = 0
        self.last_rate_update = time.time_ns()

        self.updateGui()
        
        # Show framerate
        self.setFrameRateMeter(True)
        
        self.setBackgroundColor(1, 1, 1, 1)

        self.disable_mouse()

        # Initialize 3D models of coordinate frames
        self.cs_bno055 = self.loader.loadModel("assets/3D_Coordinate_System.gltf")
        self.cs_bno055.setScale(0.01, 0.01, 0.01)
        self.cs_bno055.setPos(-3, 15, 0)

        self.cs_bno08x = self.loader.loadModel("assets/3D_Coordinate_System.gltf")
        self.cs_bno08x.setScale(0.01, 0.01, 0.01)
        self.cs_bno08x.setPos(0, 15, 0)
        
        self.cs_lsm6ds = self.loader.loadModel("assets/3D_Coordinate_System.gltf")
        self.cs_lsm6ds.setScale(0.01, 0.01, 0.01)
        self.cs_lsm6ds.setPos(3, 15, 0)

        # Position camera
        self.camera.setPos(0, 0, 5)
        self.camera.setHpr(0, -10, 0)

        # Render coordinate frames
        self.cs_bno08x.reparentTo(self.render)
        self.cs_bno055.reparentTo(self.render)
        self.cs_lsm6ds.reparentTo(self.render)

        # Start task to periodically update rotation positions
        self.taskMgr.add(self.updateRotation, "rotTest")

        
    # Callback function that is called by bleak once new notifications are recieved
    def onBLENotification(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        self.q_bno08x = self.byteArrayToQuart(data, 0)
        self.q_bno055 = self.byteArrayToQuart(data, 4)
        self.q_lsm6ds = self.byteArrayToQuart(data, 8)

        self.n_recv += 1
        if self.n_recv % 100 == 0:
            self.n_recv = 0
            t_now = time.time_ns()
            t_diff = t_now - self.last_rate_update
            # Prevent division by 0, rates above 1MHz will not be detectable (but are also very unlikely)
            if t_diff < 1e-6: t_diff += 1e-6
            self.gui_elements["esp_notify_rate"]["value"] = round(100/(t_diff*1e-9))
            self.last_rate_update = t_now
            self.updateGui()
        return

    # Convert a raw byte array to a quaternion
    def byteArrayToQuart(self, b_arr: bytearray, f_offset: int) -> Quat:
        # f_offset is the FLOAT offset, has to be multiples of 4 as a quaternion consists of 4 float values
        assert f_offset % 4 == 0, "Invalid byte offset"
        # Convert 4*sizeof(float) bytes of bytearray starting at f_offset to an array of 4 floats
        q_f = [struct.unpack('<f', b_arr[((i+f_offset)*4):(i+f_offset+1)*4])[0] for i in range(0, 4)]
        # Create a quaternion from that array
        return Quat(q_f[0], q_f[1], q_f[2], q_f[3])

    # Convert a raw byte array to a Heading-Pitch-Roll orientation representation
    def byteArrayToHpr(self, b_arr: bytearray, f_offset: int):
        hpr_f = [struct.unpack('<f', b_arr[((i+f_offset)*4):(i+f_offset+1)*4])[0] for i in range(0, 3)]
        return hpr_f
    
    # Update orientation of 3D-objects
    def updateRotation(self, task):
        self.cs_bno08x.setQuat(self.q_bno08x*self.q_offset_bno08x)
        self.cs_bno055.setQuat(self.q_bno055*self.q_offset_bno055)
        self.cs_lsm6ds.setQuat(self.q_lsm6ds*self.q_offset_lsm6ds)

        return task.cont
    
    # Updates GUI elements such as buttons or text fields
    def updateGui(self):
        for gui_element_name in self.gui_elements.keys():
            new_value = str(self.gui_elements[gui_element_name]["value"]) + " " + self.gui_elements[gui_element_name]["unit"]
            self.gui_elements[gui_element_name]["value_node"].setText(new_value)
    
    # Apply current orientation as zero offset
    def onZeroOrientationButtonPressed(self):
        self.q_offset_bno08x = self.q_bno08x.conjugate()*self.q_offset
        self.q_offset_bno055 = self.q_bno055.conjugate()*self.q_offset
        self.q_offset_lsm6ds = self.q_lsm6ds.conjugate()*self.q_offset
        return
    
    # Callback function that is called once connection status changes.
    def onDeviceConnectionUpdate(self, device):
        self.gui_elements["esp_connected"]["value"] = device.is_connected
        if not device.is_connected:
            self.gui_elements["esp_notify_rate"]["value"] = 0
        self.updateGui()
        return

    
