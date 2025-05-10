#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from evdev import InputDevice, ecodes

class BluetoothButtonNode(Node):
    def __init__(self):
        super().__init__('bluetooth_button_node')

        self.publisher_ = self.create_publisher(String, '/bluetooth_button', 10)
        self.device_path = '/dev/input/event9'  # Change if needed
        self.device = InputDevice(self.device_path)

        self.get_logger().info(f"Listening on {self.device.name} ({self.device_path})")

        self.pressing = False
        self.start_x = self.start_y = None
        self.end_x = self.end_y = None

        # Use a timer to poll the input loop
        self.create_timer(0.01, self.read_events)

    def read_events(self):
        try:
            for event in self.device.read():
                if event.type == ecodes.EV_KEY and event.code == ecodes.BTN_TOUCH:
                    if event.value == 1:
                        self.pressing = True
                        self.start_x = self.start_y = None
                        self.end_x = self.end_y = None
                    elif event.value == 0 and self.pressing:
                        self.pressing = False
                        delta_x = (self.end_x or 0) - (self.start_x or 0)
                        delta_y = (self.end_y or 0) - (self.start_y or 0)
                        button_name = self.classify_button(delta_x, delta_y)
                        msg = String()
                        msg.data = button_name
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Published: {button_name}")
                elif event.type == ecodes.EV_ABS and self.pressing:
                    if event.code == ecodes.ABS_X:
                        if self.start_x is None:
                            self.start_x = event.value
                        self.end_x = event.value
                    elif event.code == ecodes.ABS_Y:
                        if self.start_y is None:
                            self.start_y = event.value
                        self.end_y = event.value
        except BlockingIOError:
            # No events to read at this time
            pass

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

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothButtonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

