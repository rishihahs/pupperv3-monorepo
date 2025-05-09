#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
import threading
import time


class DanceController(Node):
    def __init__(self):
        super().__init__('dance_node')
        
        # Velocity publisher (from RobotMover)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_move_thread = None
        self.move_stop_event = threading.Event()
        
        # Button subscriber
        self.button_subscription = self.create_subscription(
            String,
            'bluetooth_button',  # Replace with actual topic
            self.button_callback,
            10
        )
        
        # Service clients
        self.start_headnod_client = self.create_client(Trigger, '/start_replay')
        self.stop_headnod_client = self.create_client(Trigger, '/stop_replay')
        
        self.start_eardance_client = self.create_client(Trigger, '/start_ear_dance')
        self.stop_eardance_client = self.create_client(Trigger, '/stop_ear_dance')
        
        self.switch_controller_client = self.create_client(
            SwitchController, 
            '/controller_manager/switch_controller'
        )
        
        # Wait for services to be available
        self.wait_for_services()

        self.switch_controller('neural_controller_dance', 'neural_controller')
        
        # Motion control
        self.current_motion = None
        self.motion_thread = None
        self.motion_stop_event = threading.Event()
        
        self.get_logger().info('Dance Controller initialized. Ready for button presses!')
    
    # --- RobotMover functionality ---
    
    def move(self, x, y, z, ax, ay, az, duration, rate=60):
        """
        Move the robot with specified linear and angular velocities for a duration.
        
        Args:
            x, y, z: Linear velocities in m/s
            ax, ay, az: Angular velocities in rad/s
            duration: Duration of movement in seconds
            rate: Publishing frequency in Hz
        """
        # Cancel any ongoing movement
        if self.current_move_thread and self.current_move_thread.is_alive():
            self.move_stop_event.set()
            self.current_move_thread.join()
        
        self.move_stop_event.clear()
        
        # Create and start a new movement thread
        self.current_move_thread = threading.Thread(
            target=self._move_thread,
            args=(x, y, z, ax, ay, az, duration, rate)
        )
        self.current_move_thread.start()
    
    def _move_thread(self, x, y, z, ax, ay, az, duration, rate):
        """Thread function that publishes velocity commands at the specified rate."""
        twist = Twist()
        twist.linear.x = float(x)
        twist.linear.y = float(y)
        twist.linear.z = float(z)
        twist.angular.x = float(ax)
        twist.angular.y = float(ay)
        twist.angular.z = float(az)
        
        # Calculate how many messages to publish
        period = 1.0 / rate
        count = int(duration * rate)
        
        self.get_logger().info(f'Starting movement: linear({x},{y},{z}), angular({ax},{ay},{az}) for {duration}s')
        
        for _ in range(count):
            if self.move_stop_event.is_set():
                break
            
            self.vel_publisher.publish(twist)
            time.sleep(period)
        
        # Send a zero velocity command at the end to stop the robot
        self._publish_zero_velocity()
        self.get_logger().info('Movement completed')
    
    def _publish_zero_velocity(self):
        """Publish a zero velocity command to stop the robot."""
        twist = Twist()
        self.vel_publisher.publish(twist)

    def stop_all_movement(self):
        """Stop any ongoing movement."""
        if self.current_move_thread and self.current_move_thread.is_alive():
            self.move_stop_event.set()
            self.current_move_thread.join()
            self._publish_zero_velocity()
    
    # --- Motion Controller functionality ---
    
    def wait_for_services(self):
        """Wait for all required services to be available."""
        services = [
            (self.start_headnod_client, '/start_replay'),
            (self.stop_headnod_client, '/stop_replay'),
            # (self.start_eardance_client, '/start_ear_dance'),
            # (self.stop_eardance_client, '/stop_ear_dance'),
            (self.switch_controller_client, '/controller_manager/switch_controller')
        ]
        
        for client, name in services:
            self.get_logger().info(f'Waiting for service: {name}...')
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    self.get_logger().error('ROS interrupted while waiting for service')
                    return False
                self.get_logger().info(f'Service {name} not available, waiting...')
            
            self.get_logger().info(f'Service {name} is available')
        
        return True
    
    def button_callback(self, msg):
        """Handle button press events."""
        button = msg.data
        self.get_logger().info(f'Button pressed: {button}')
        
        # Map buttons to actions
        if button == 'button1':
            self.execute_motion('headbang')
        elif button == 'button2':
            self.execute_motion('headnod')
        elif button == 'button3':
            self.execute_motion('eardance')
        elif button == 'button4':
            self.execute_motion('circledance')
        else:
            self.get_logger().info(f'No action mapped to button: {button}')
    
    def execute_motion(self, motion_name):
        """Execute the specified motion, stopping any current motion first."""
        # Stop current motion if any
        self.stop_current_motion()
        
        # Start new motion
        self.current_motion = motion_name
        self.get_logger().info(f'Starting motion: {motion_name}')
        
        if motion_name == 'headbang':
            self.motion_thread = threading.Thread(target=self.run_headbang)
            self.motion_thread.start()
        
        elif motion_name == 'headnod':
            self.call_service(self.start_headnod_client)
        
        elif motion_name == 'eardance':
            self.call_service(self.start_eardance_client)
        
        elif motion_name == 'circledance':
            self.motion_thread = threading.Thread(target=self.run_circledance)
            self.motion_thread.start()
    
    def stop_current_motion(self):
        """Stop the currently running motion if any."""
        if self.current_motion is None:
            return
        
        self.get_logger().info(f'Stopping current motion: {self.current_motion}')
        
        # Stop any velocity commands
        self.stop_all_movement()
        
        if self.current_motion == 'headbang':
            self.motion_stop_event.set()
            if self.motion_thread and self.motion_thread.is_alive():
                self.motion_thread.join(timeout=1.0)
        
        elif self.current_motion == 'headnod':
            self.call_service(self.stop_headnod_client)
        
        elif self.current_motion == 'eardance':
            self.call_service(self.stop_eardance_client)
        
        elif self.current_motion == 'circledance':
            self.motion_stop_event.set()
            if self.motion_thread and self.motion_thread.is_alive():
                self.motion_thread.join(timeout=1.0)
        
        self.current_motion = None
        self.motion_stop_event.clear()
    
    # --- Motion implementations ---
    
    def headbang(self):
        """Example headbang motion implementation using move."""
        duration_a = 0.24
        duration_b = 0.12
        self.move(1.1, 0.0, 0.0, 0.0, 0.0, 0.0, duration_a)  # Move forward and rotate for 2 seconds
        time.sleep(duration_a)
        self.move(-1.1, 0.0, 0.0, 0.0, 0.0, 0.0, duration_b)  # Move forward and rotate for 2 seconds
        time.sleep(duration_b + 0.1)

    def circledance(self):
        """Example circledance motion implementation using move."""
        self.move(0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 4.0)  # Turn right
        time.sleep(4.0)
    
    def run_headbang(self):
        """Run the headbang motion until stopped."""
        try:
            while not self.motion_stop_event.is_set() and rclpy.ok():
                self.headbang()  # Use internal method
                
                # Check if we should stop
                if self.motion_stop_event.is_set():
                    break
                
                # Small sleep to prevent CPU hogging between iterations
                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f'Error in headbang: {str(e)}')
    
    def run_circledance(self):
        """Run the circledance motion with the necessary controller switches."""
        try:
            # Switch controller before
            self.switch_controller('neural_controller', 'neural_controller_dance')  # Assuming controller name, modify as needed
            
            # Run the motion if switch was successful
            if not self.motion_stop_event.is_set():
                self.circledance()  # Use internal method
            
            # Switch controller after (only if we haven't been asked to stop)
            if not self.motion_stop_event.is_set():
                self.switch_controller('neural_controller_dance', 'neural_controller')  # Assuming controller name, modify as needed
                
        except Exception as e:
            self.get_logger().error(f'Error in circledance: {str(e)}')
            # Attempt to switch back to default controller in case of error
            self.switch_controller('neural_controller_dance', 'neural_controller')
    
    def switch_controller(self, controller_name, deactivate_controller_name):
        """Call the switch_controller service."""
        request = SwitchController.Request()
        
        request.activate_controllers = [controller_name]
        request.deactivate_controllers = [deactivate_controller_name]
        
        request.strictness = 1  # BEST_EFFORT
        request.start_asap = True
        request.timeout = 1.0
        
        self.get_logger().info(f'Switching controller: {"starting" if start_controller else "stopping"} {controller_name}')
        
        future = self.switch_controller_client.call_async(request)
        # Note: In a real application, you'd want to add a callback to handle the response
    
    def call_service(self, client):
        """Call a Trigger service."""
        request = Trigger.Request()
        future = client.call_async(request)
        # Note: In a real application, you'd want to add a callback to handle the response


def main(args=None):
    rclpy.init(args=args)
    
    # Create the controller node
    controller = DanceController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure any ongoing motion is stopped
        controller.stop_current_motion()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
