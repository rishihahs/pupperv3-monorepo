#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gpiozero import Servo
import math
import time
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup

class EarWaveService(Node):
    def __init__(self):
        super().__init__("ear_wave_service")
        
        # Create a callback group for services
        self.callback_group = ReentrantCallbackGroup()
        
        # GPIO pins for the servos controlling the ears
        self.servo_l_pin = 16
        self.servo_r_pin = 26

        # Flag to make sure we properly detach servos when not in use
        self.servos_initialized = False
        self.servo_l = None
        self.servo_r = None
        
        # Initialize the servo objects but don't attach them yet
        self._init_servos(attach=False)

        # Wave parameters - adjust these to change the movement pattern
        self.amplitude = 0.2  # Max deflection (0-1)
        self.frequency = 2.2  # Waves per second
        self.phase_difference = math.pi  # Phase difference between ears (radians)
        
        # Create services
        self.start_service = self.create_service(
            Trigger, 
            'start_ear_dance', 
            self.start_callback,
            callback_group=self.callback_group
        )
        
        self.stop_service = self.create_service(
            Trigger, 
            'stop_ear_dance', 
            self.stop_callback,
            callback_group=self.callback_group
        )
        
        # Timer for the wave movement (initially not set)
        self.timer = None
        self.active = False
        self.start_time = 0
        
        self.get_logger().info("Ear wave service started. Use:")
        self.get_logger().info("ros2 service call /start_ear_dance std_srvs/srv/Trigger")
        self.get_logger().info("ros2 service call /stop_ear_dance std_srvs/srv/Trigger")
        self.get_logger().info(
            "If getting GPIO errors, install requirements with:\n"
            "pip uninstall rpi.gpio\n"
            "sudo apt install python3-rpi-lgpio\n"
            "sudo rm /usr/lib/python3.*/EXTERNALLY-MANAGED\n"
            "pip install gpiozero"
        )

    def _init_servos(self, attach=True):
        """Initialize or detach servos"""
        if attach and not self.servos_initialized:
            # Initialize Servos with gpiozero
            self.servo_l = Servo(self.servo_l_pin, min_pulse_width=0.000500, max_pulse_width=0.002500, frame_width=0.020, initial_value=0)
            self.servo_r = Servo(self.servo_r_pin, min_pulse_width=0.000500, max_pulse_width=0.002500, frame_width=0.020, initial_value=0)
            self.servos_initialized = True
            self.get_logger().info("Servos initialized and attached")
        elif not attach and self.servos_initialized:
            # Detach servos by cleaning up their resources
            if self.servo_l:
                self.servo_l.detach()
                self.servo_l.close()
            if self.servo_r:
                self.servo_r.detach()
                self.servo_r.close()
            self.servo_l = None
            self.servo_r = None
            self.servos_initialized = False
            self.get_logger().info("Servos detached and closed")

    def start_callback(self, request, response):
        if not self.active:
            # Initialize servos if not already done
            self._init_servos(attach=True)

            self.active = True
            self.start_time = time.time()
            # Create timer to update ear positions at 20Hz
            self.timer = self.create_timer(0.01, self.wave_ears)
            response.success = True
            response.message = "Ear dance started"
            self.get_logger().info("Ear dance started")
        else:
            response.success = True
            response.message = "Ear dance was already running"
            self.get_logger().info("Ear dance was already running")
        return response

    def stop_callback(self, request, response):
        if self.active:
            self.active = False
            # Cancel the timer if it exists
            if self.timer:
                self.timer.cancel()
                self.timer = None

            # Return to center position before detaching
            if self.servos_initialized:
                self.servo_l.value = 0
                self.servo_r.value = 0
                # Small delay to allow servos to reach position
                time.sleep(0.5)
            
            # Detach servos to prevent jittering and save power
            self._init_servos(attach=False)
            
            response.success = True
            response.message = "Ear dance stopped"
            self.get_logger().info("Ear dance stopped, ears centered")
        else:
            response.success = True
            response.message = "Ear dance was not running"
            self.get_logger().info("Ear dance was not running")
        return response

    def wave_ears(self):
        if not self.active or not self.servos_initialized:
            return
            
        # Calculate the current time in seconds since start
        current_time = time.time() - self.start_time
        
        # Calculate the sine wave value for each ear
        l_position = self.amplitude * math.sin(2 * math.pi * self.frequency * current_time)
        r_position = self.amplitude * math.sin(2 * math.pi * self.frequency * current_time + self.phase_difference)
        
        # Set servo positions
        self.servo_l.value = l_position
        self.servo_r.value = r_position
        
        # Log position occasionally (once per second) to avoid flooding logs
        if int(current_time * 10) % 60 == 0:
            self.get_logger().info(f"Ear positions: L: {l_position:.2f}, R: {r_position:.2f}")

    def destroy_node(self):
        # Ensure we clean up servos on shutdown
        if self.active:
            self.active = False
            if self.timer:
                self.timer.cancel()
        
        if self.servos_initialized:
            # Try to center servos before shutting down
            try:
                self.servo_l.value = 0
                self.servo_r.value = 0
                time.sleep(0.5)  # Short delay to reach position
            except:
                pass
            finally:
                self._init_servos(attach=False)  # Detach servos
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    ear_service = EarWaveService()
    
    try:
        rclpy.spin(ear_service)
    except KeyboardInterrupt:
        # Handle clean shutdown
        ear_service.servo_l.value = 0
        ear_service.servo_r.value = 0
        ear_service.get_logger().info("Shutting down and centering ears")
    finally:
        # Clean up
        ear_service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
