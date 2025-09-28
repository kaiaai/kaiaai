import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, BatteryState, CompressedImage
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from kaiaai_msgs.msg import WifiState
from builtin_interfaces.msg import Time
from std_msgs.msg import String
import math
import numpy as np
import random
import io
import json
from PIL import Image, ImageDraw, ImageFont

class FakeROS2Robot(Node):
    def __init__(self):
        super().__init__('fake_ros2_robot')
        self.scan_publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.battery_publisher_ = self.create_publisher(BatteryState, 'battery_status', 10)
        self.wifi_publisher_ = self.create_publisher(WifiState, 'wifi_state', 10)
        self.map_publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.camera_publisher_ = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)

        # Create QoS profile for latched subscription
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Subscribe to remote control status for testing/monitoring
        self.remote_control_status_subscription = self.create_subscription(
            String,
            '/remote_control_status',
            self.remote_control_status_callback,
            latched_qos
        )

        # Timer for laser scan (0.5 seconds)
        self.scan_timer = self.create_timer(0.5, self.publish_fake_scan)

        # Timer for battery status (2 seconds)
        self.battery_timer = self.create_timer(2.0, self.publish_fake_battery)

        # Timer for WiFi status (1.5 seconds)
        self.wifi_timer = self.create_timer(1.5, self.publish_fake_wifi)

        # Timer for map (5 seconds - maps don't change often)
        self.map_timer = self.create_timer(5.0, self.publish_fake_map)

        # Timer for camera (0.3 seconds - ~3 FPS)
        self.camera_timer = self.create_timer(0.3, self.publish_fake_camera)

        # Initialize battery simulation variables
        self.battery_percentage = 85.0  # Start at 85%
        self.battery_drain_rate = 0.02  # Drain 0.02% every 2 seconds

        # Initialize WiFi simulation variables
        self.base_rssi = -45.0  # Base RSSI in dBm (pretty good signal)
        self.rssi_noise_range = 15.0  # +/- dBm variation

        # Initialize map simulation variables
        self.map_width = 100  # 100 cells wide
        self.map_height = 100  # 100 cells tall
        self.map_resolution = 0.05  # 5cm per cell
        self.map_data = None
        self.map_update_counter = 0

        # Initialize camera simulation variables
        self.camera_frame_counter = 0
        self.camera_width = 640
        self.camera_height = 480

        # Initialize remote control status tracking
        self.remote_control_active = False  # Start with remote control off

        self.get_logger().info('Fake ROS2 Robot Node started (LaserScan + BatteryState + WifiState + Map + Camera + Remote Control Monitoring).')
        self.get_logger().info('Monitoring /remote_control_status topic for testing')

    def publish_fake_scan(self):
        # Only publish if remote control is active
        if not self.remote_control_active:
            return

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'  # Or your desired frame ID

        # Define scan parameters
        msg.angle_min = -math.pi  # -180 degrees
        msg.angle_max = math.pi   # +180 degrees
        msg.angle_increment = 2* math.pi / 360  # 1 degree increment
        msg.time_increment = 0.0  # Not used for fixed scans
        msg.scan_time = 0.1 # Time it takes to complete one scan
        msg.range_min = 0.1  # Minimum range value
        msg.range_max = 10.0 # Maximum range value

        # Create fake ranges
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = np.random.uniform(low=0.5, high=8.0, size=(360,))
        msg.intensities = [] # Optional, can be left empty

        self.scan_publisher_.publish(msg)
        self.get_logger().info('Publishing fake LaserScan message.')

    def publish_fake_battery(self):
        """Publish fake battery status data"""
        # Only publish if remote control is active
        if not self.remote_control_active:
            return

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Simulate battery drain with some randomness
        self.battery_percentage = max(0.0, self.battery_percentage - random.uniform(0.01, self.battery_drain_rate * 2))

        # If battery gets too low, simulate recharging
        if self.battery_percentage < 5.0:
            self.battery_percentage = 90.0  # Simulate battery swap/recharge
            self.get_logger().info('🔋 Battery recharged to 90%!')

        # Fill in battery state message
        msg.voltage = 12.0 + (self.battery_percentage / 100.0) * 2.0  # 12V to 14V range
        msg.temperature = 25.0 + random.uniform(-2.0, 5.0)  # 23-30°C range
        msg.current = -2.5 + random.uniform(-0.5, 0.5)  # Simulated current draw
        msg.charge = float('nan')  # Not available
        msg.capacity = float('nan')  # Not available
        msg.design_capacity = 100.0  # 100Ah design capacity
        msg.percentage = self.battery_percentage
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True

        # Add some location info (optional)
        msg.location = 'main_battery'
        msg.serial_number = 'DUMMY_BAT_001'

        self.battery_publisher_.publish(msg)
        self.get_logger().info(f'Publishing battery status: {self.battery_percentage:.1f}%')

    def publish_fake_wifi(self):
        """Publish fake WiFi status data"""
        # Only publish if remote control is active
        if not self.remote_control_active:
            return

        msg = WifiState()
        msg.stamp = self.get_clock().now().to_msg()

        # Simulate WiFi signal strength variation
        # Occasionally simulate signal drops or improvements
        if random.random() < 0.1:  # 10% chance of significant change
            if random.random() < 0.5:
                # Signal degradation (moving away from router, interference)
                self.base_rssi -= random.uniform(5.0, 20.0)
                self.base_rssi = max(-90.0, self.base_rssi)  # Don't go below -90 dBm
            else:
                # Signal improvement (moving closer to router)
                self.base_rssi += random.uniform(5.0, 15.0)
                self.base_rssi = min(-20.0, self.base_rssi)  # Don't go above -20 dBm

        # Add some random noise to simulate normal variation
        current_rssi = self.base_rssi + random.uniform(-3.0, 3.0)
        msg.rssi_dbm = current_rssi

        self.wifi_publisher_.publish(msg)
        self.get_logger().info(f'Publishing WiFi status: {msg.rssi_dbm:.1f} dBm')

    def publish_fake_map(self):
        """Publish fake occupancy grid map data"""
        # Only publish if remote control is active
        if not self.remote_control_active:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Set up map metadata
        msg.info.map_load_time = self.get_clock().now().to_msg()
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height

        # Set origin (bottom-left corner of the map in world coordinates)
        msg.info.origin.position.x = -self.map_width * self.map_resolution / 2.0
        msg.info.origin.position.y = -self.map_height * self.map_resolution / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Generate or update map data
        if self.map_data is None:
            # Create initial map with some interesting features
            self.map_data = self.generate_fake_map()

        # Occasionally modify the map to simulate dynamic updates
        self.map_update_counter += 1
        if self.map_update_counter % 10 == 0:  # Every 50 seconds (10 * 5 second timer)
            self.modify_map()

        msg.data = self.map_data.tolist()

        self.map_publisher_.publish(msg)
        self.get_logger().info(f'Publishing map: {self.map_width}x{self.map_height}, {len(msg.data)} cells')

    def generate_fake_map(self):
        """Generate a fake occupancy grid with some interesting features"""
        # Initialize with free space (0)
        map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)

        # Add some walls around the perimeter
        map_data[0, :] = 100  # Top wall
        map_data[-1, :] = 100  # Bottom wall
        map_data[:, 0] = 100  # Left wall
        map_data[:, -1] = 100  # Right wall

        # Add some internal walls and obstacles
        # Vertical wall in the middle
        map_data[20:80, 50] = 100

        # Horizontal walls
        map_data[30, 10:40] = 100
        map_data[70, 60:90] = 100

        # Some scattered obstacles
        for _ in range(15):
            x = random.randint(10, self.map_width - 10)
            y = random.randint(10, self.map_height - 10)
            size = random.randint(2, 5)
            map_data[y:y+size, x:x+size] = 100

        # Add some unknown areas (-1)
        for _ in range(5):
            x = random.randint(5, self.map_width - 15)
            y = random.randint(5, self.map_height - 15)
            size = random.randint(3, 8)
            map_data[y:y+size, x:x+size] = -1

        # Flatten to 1D array (row-major order)
        return map_data.flatten()

    def modify_map(self):
        """Slightly modify the map to simulate dynamic updates"""
        # Convert back to 2D for easier manipulation
        map_2d = self.map_data.reshape((self.map_height, self.map_width))

        # Add a few random obstacles
        for _ in range(2):
            x = random.randint(5, self.map_width - 5)
            y = random.randint(5, self.map_height - 5)
            if map_2d[y, x] == 0:  # Only modify free space
                map_2d[y, x] = 100

        # Remove a few random obstacles (simulate clearing)
        for _ in range(1):
            x = random.randint(5, self.map_width - 5)
            y = random.randint(5, self.map_height - 5)
            if map_2d[y, x] == 100 and not self.is_perimeter_wall(x, y):
                map_2d[y, x] = 0

        # Convert back to 1D
        self.map_data = map_2d.flatten()

    def publish_fake_camera(self):
        """Publish fake compressed image data"""
        # Only publish if remote control is active
        if not self.remote_control_active:
            return

        try:
            # Create a fake image with some dynamic content
            img = Image.new('RGB', (self.camera_width, self.camera_height), color='lightblue')
            draw = ImageDraw.Draw(img)

            # Add some dynamic elements
            self.camera_frame_counter += 1

            # Draw a moving circle
            circle_x = int(50 + 100 * math.sin(self.camera_frame_counter * 0.1))
            circle_y = int(50 + 50 * math.cos(self.camera_frame_counter * 0.1))
            draw.ellipse([circle_x, circle_y, circle_x + 50, circle_y + 50], fill='red')

            # Draw some static elements (simulate robot's view)
            # Floor
            draw.rectangle([0, self.camera_height//2, self.camera_width, self.camera_height], fill='gray')

            # Some obstacles/objects
            draw.rectangle([200, 300, 250, 400], fill='brown')  # Box
            draw.rectangle([400, 250, 450, 350], fill='green')  # Another object

            # Add frame counter text
            try:
                # Try to use default font, fallback to basic if not available
                font = ImageFont.load_default()
            except:
                font = None

            frame_text = f"Frame: {self.camera_frame_counter}"
            time_text = f"Time: {self.get_clock().now().nanoseconds // 1000000}"

            draw.text((10, 10), frame_text, fill='black', font=font)
            draw.text((10, 30), time_text, fill='black', font=font)
            draw.text((10, 50), "Fake Camera Feed", fill='black', font=font)

            # Add some random noise/variation
            if self.camera_frame_counter % 10 == 0:
                # Add random rectangles occasionally
                for _ in range(3):
                    x1 = random.randint(0, self.camera_width - 50)
                    y1 = random.randint(0, self.camera_height - 50)
                    x2 = x1 + random.randint(20, 50)
                    y2 = y1 + random.randint(20, 50)
                    color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                    draw.rectangle([x1, y1, x2, y2], fill=color)

            # Convert PIL image to JPEG bytes
            img_byte_arr = io.BytesIO()
            img.save(img_byte_arr, format='JPEG', quality=85)
            img_byte_arr.seek(0)

            # Create CompressedImage message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            msg.format = 'jpeg'
            msg.data = img_byte_arr.getvalue()

            # Publish the message
            self.camera_publisher_.publish(msg)
            self.get_logger().info(f'Publishing fake camera image: frame {self.camera_frame_counter}, size {len(msg.data)} bytes')

        except Exception as e:
            self.get_logger().error(f'Error generating fake camera image: {e}')

    def remote_control_status_callback(self, msg):
        """Handle String messages from /remote_control_status topic (for testing)"""
        try:
            self.get_logger().info(f'Received remote control status: {msg.data}')

            # Try to parse as JSON for prettier display and status tracking
            try:
                status_data = json.loads(msg.data)
                remote_control_status = status_data.get('remote_control', False)

                # Update our tracking variable
                previous_status = self.remote_control_active
                self.remote_control_active = bool(remote_control_status)

            except (json.JSONDecodeError, AttributeError):
                # If not valid JSON, just print the raw string
                self.get_logger().warn(f"📡 [RAW] {msg.data}")

        except Exception as e:
            self.get_logger().error(f'Error processing remote control status message: {e}')

    def is_perimeter_wall(self, x, y):
        """Check if a cell is part of the perimeter wall (don't remove these)"""
        return x == 0 or x == self.map_width - 1 or y == 0 or y == self.map_height - 1

def main(args=None):
    rclpy.init(args=args)
    fake_ros2_robot = FakeROS2Robot()
    rclpy.spin(fake_ros2_robot)
    fake_ros2_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
