#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from evdev import InputDevice, list_devices, ecodes
import select
import time

DEVICE_NAME_SUBSTRING = "S18"  # Change this to a unique part of your device name
RECONNECT_INTERVAL = 2.0  # Seconds between reconnect attempts

class BluetoothButtonNode(Node):
    def __init__(self):
        super().__init__('bluetooth_button_node')
        self.publisher_ = self.create_publisher(String, '/bluetooth_button', 10)

        self.device = None
        self.pressing = False
        self.start_x = self.start_y = None
        self.end_x = self.end_y = None
        self.last_reconnect_attempt = 0

        # Timer to run the event loop
        self.create_timer(0.01, self.read_loop)

    def read_loop(self):
        # Try to connect if device is missing
        if self.device is None:
            now = time.time()
            if now - self.last_reconnect_attempt > RECONNECT_INTERVAL:
                self.device = self.find_device_by_name(DEVICE_NAME_SUBSTRING)
                if self.device:
                    self.device.grab()  # Optional: exclusive access
                    self.get_logger().info(f"Connected to {self.device.name}")
                else:
                    self.get_logger().warn("Bluetooth button not found, retrying...")
                self.last_reconnect_attempt = now
            return

        try:
            r, _, _ = select.select([self.device.fd], [], [], 0)
            if r:
                for event in self.device.read():
                    self.process_event(event)
        except (OSError, IOError):
            self.get_logger().warn("Device disconnected. Waiting for reconnection...")
            self.device = None

    def process_event(self, event):
        if event.type == ecodes.EV_KEY and event.code == ecodes.BTN_TOUCH:
            if event.value == 1:
                self.pressing = True
                self.start_x = self.start_y = None
                self.end_x = self.end_y = None
            elif event.value == 0 and self.pressing:
                self.pressing = False
                dx = (self.end_x or 0) - (self.start_x or 0)
                dy = (self.end_y or 0) - (self.start_y or 0)
                button = self.classify_button(dx, dy)
                msg = String()
                msg.data = button
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published: {button}")
        elif event.type == ecodes.EV_ABS and self.pressing:
            if event.code == ecodes.ABS_X:
                if self.start_x is None:
                    self.start_x = event.value
                self.end_x = event.value
            elif event.code == ecodes.ABS_Y:
                if self.start_y is None:
                    self.start_y = event.value
                self.end_y = event.value

    def classify_button(self, dx, dy):
        if dx == 0 and dy == 0:
            return "button1"
        elif dx == 0 and dy > 500:
            return "button2"
        elif dx < -500 and dy == 0:
            return "button3"
        elif dx == 0 and dy < -500:
            return "button4"
        elif dx > 500 and dy == 0:
            return "button5"
        else:
            return "button_unknown"

    def find_device_by_name(self, name_substring):
        for path in list_devices():
            try:
                dev = InputDevice(path)
                if name_substring in dev.name:
                    return dev
            except Exception:
                continue
        return None

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothButtonNode()
    try:
        rclpy.spin(node)
    finally:
        if node.device:
            node.device.ungrab()
        node.destroy_node()
        rclpy.shutdown()

