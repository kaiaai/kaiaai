import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, BatteryState, CompressedImage
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseWithCovarianceStamped, PoseStamped, TransformStamped
from kaiaai_msgs.msg import WifiState
from builtin_interfaces.msg import Time
from std_msgs.msg import String, Header
from tf2_ros import TransformBroadcaster
import math
import numpy as np
import random
import io
import json
from PIL import Image, ImageDraw, ImageFont

class FakeROS2Robot(Node):
    def __init__(self):
        super().__init__('fake_ros2_robot')

        # Create QoS profile for latched subscription/publication
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.scan_publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.battery_publisher_ = self.create_publisher(BatteryState, 'battery_status', 10)
        self.wifi_publisher_ = self.create_publisher(WifiState, 'wifi_state', 10)
        self.map_publisher_ = self.create_publisher(OccupancyGrid, 'map', latched_qos)
        self.camera_publisher_ = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)

        # TF2 broadcaster for publishing robot pose
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to remote control status for testing/monitoring
        self.remote_control_status_subscription = self.create_subscription(
            String,
            '/remote_control_status',
            self.remote_control_status_callback,
            latched_qos
        )

        # Subscribe to cmd_vel to simulate robot movement
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
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

        # Timer for robot pose (0.2 seconds - 5 Hz)
        self.pose_timer = self.create_timer(0.2, self.publish_robot_pose)

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

        # Initialize robot pose simulation
        self.robot_x = 0.0  # Robot position in meters
        self.robot_y = 0.0
        self.robot_yaw = 0.0  # Robot orientation in radians
        self.last_cmd_vel = None
        self.last_pose_update = self.get_clock().now()

        # Initialize static objects for more realistic laser scans
        self.static_objects = self.generate_static_objects()

        self.get_logger().info('Fake ROS2 Robot Node started (LaserScan + BatteryState + WifiState + Map + Camera + TF + Remote Control Monitoring).')
        self.get_logger().info('Monitoring /remote_control_status topic for testing')
        self.get_logger().info('Subscribing to /cmd_vel for robot movement simulation')
        self.get_logger().info('Publishing robot pose to /tf (map -> base_footprint)')

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

        # Generate realistic ranges based on robot position and map
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = self.generate_realistic_laser_scan(msg.angle_min, msg.angle_increment, num_readings)
        msg.intensities = [] # Optional, can be left empty

        self.scan_publisher_.publish(msg)
        self.get_logger().info(f'Publishing realistic LaserScan from pose ({self.robot_x:.2f}, {self.robot_y:.2f}, {math.degrees(self.robot_yaw):.1f}°)')

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

    def cmd_vel_callback(self, msg):
        """Handle Twist messages from cmd_vel topic to simulate robot movement"""
        if not self.remote_control_active:
            return

        try:
            self.last_cmd_vel = msg
            self.get_logger().info(f'Received cmd_vel: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')

            # Update robot pose based on cmd_vel
            self.update_robot_pose_from_cmd_vel(msg)

        except Exception as e:
            self.get_logger().error(f'Error processing cmd_vel: {e}')

    def update_robot_pose_from_cmd_vel(self, cmd_vel):
        """Update robot pose based on received cmd_vel command"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_pose_update).nanoseconds / 1e9  # Convert to seconds
        self.last_pose_update = current_time

        if dt <= 0 or dt > 1.0:  # Sanity check
            return

        # Extract velocities
        linear_x = cmd_vel.linear.x
        angular_z = cmd_vel.angular.z

        # Simple kinematic model for differential drive robot
        # Update position
        dx = linear_x * math.cos(self.robot_yaw) * dt
        dy = linear_x * math.sin(self.robot_yaw) * dt
        dyaw = angular_z * dt

        self.robot_x += dx
        self.robot_y += dy
        self.robot_yaw += dyaw

        # Normalize yaw to [-pi, pi]
        while self.robot_yaw > math.pi:
            self.robot_yaw -= 2 * math.pi
        while self.robot_yaw < -math.pi:
            self.robot_yaw += 2 * math.pi

        # Add some noise to make it more realistic
        self.robot_x += random.uniform(-0.001, 0.001)
        self.robot_y += random.uniform(-0.001, 0.001)
        self.robot_yaw += random.uniform(-0.001, 0.001)

    def publish_robot_pose(self):
        """Publish robot pose data via TF"""
        if not self.remote_control_active:
            return

        try:
            # Create transform message
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'base_footprint'

            # Set translation
            t.transform.translation.x = self.robot_x
            t.transform.translation.y = self.robot_y
            t.transform.translation.z = 0.0

            # Convert yaw to quaternion
            qz = math.sin(self.robot_yaw / 2.0)
            qw = math.cos(self.robot_yaw / 2.0)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            # Broadcast the transform
            self.tf_broadcaster.sendTransform(t)
            self.get_logger().info(f'Publishing robot pose via TF: ({self.robot_x:.2f}, {self.robot_y:.2f}, {math.degrees(self.robot_yaw):.1f}\u00b0)')

        except Exception as e:
            self.get_logger().error(f'Error publishing robot pose: {e}')

    def generate_static_objects(self):
        """Generate static objects for realistic laser scan simulation"""
        objects = []

        # Add some furniture-like objects
        # Round table
        objects.append({
            'type': 'circle',
            'x': 1.5, 'y': 1.0, 'radius': 0.4
        })

        # Chair legs (small circles)
        objects.append({
            'type': 'circle',
            'x': 2.0, 'y': 2.0, 'radius': 0.05
        })
        objects.append({
            'type': 'circle',
            'x': 2.3, 'y': 2.0, 'radius': 0.05
        })
        objects.append({
            'type': 'circle',
            'x': 2.0, 'y': 2.3, 'radius': 0.05
        })
        objects.append({
            'type': 'circle',
            'x': 2.3, 'y': 2.3, 'radius': 0.05
        })

        # Rectangular objects (boxes, furniture)
        objects.append({
            'type': 'rectangle',
            'x1': -1.0, 'y1': 0.5, 'x2': -0.5, 'y2': 1.0
        })
        objects.append({
            'type': 'rectangle',
            'x1': 0.0, 'y1': -2.0, 'x2': 0.5, 'y2': -1.5
        })

        # Wall segments
        objects.append({
            'type': 'line',
            'x1': -3.0, 'y1': 3.0, 'x2': 3.0, 'y2': 3.0  # North wall
        })
        objects.append({
            'type': 'line',
            'x1': -3.0, 'y1': -3.0, 'x2': 3.0, 'y2': -3.0  # South wall
        })
        objects.append({
            'type': 'line',
            'x1': -3.0, 'y1': -3.0, 'x2': -3.0, 'y2': 3.0  # West wall
        })
        objects.append({
            'type': 'line',
            'x1': 3.0, 'y1': -3.0, 'x2': 3.0, 'y2': 3.0  # East wall
        })

        # Interior walls with gaps (doors)
        objects.append({
            'type': 'line',
            'x1': -1.0, 'y1': -1.0, 'x2': 1.0, 'y2': -1.0  # Interior wall with gap
        })

        return objects

    def generate_realistic_laser_scan(self, angle_min, angle_increment, num_readings):
        """Generate realistic laser scan ranges based on robot position and objects"""
        ranges = []
        max_range = 8.0

        for i in range(num_readings):
            angle = angle_min + i * angle_increment

            # Calculate the absolute angle in world coordinates
            world_angle = self.robot_yaw + angle

            # Ray casting to find closest obstacle
            min_distance = max_range

            # Check against static objects
            for obj in self.static_objects:
                distance = self.ray_cast_object(obj, world_angle)
                if distance < min_distance:
                    min_distance = distance

            # Add some noise
            noise = random.uniform(-0.02, 0.02)
            min_distance += noise

            # Clamp to valid range
            min_distance = max(0.1, min(max_range, min_distance))

            ranges.append(min_distance)

        return ranges

    def ray_cast_object(self, obj, angle):
        """Cast a ray from robot position at given angle and find intersection with object"""
        max_distance = 8.0

        # Ray direction
        dx = math.cos(angle)
        dy = math.sin(angle)

        if obj['type'] == 'circle':
            return self.ray_circle_intersection(self.robot_x, self.robot_y, dx, dy,
                                              obj['x'], obj['y'], obj['radius'])
        elif obj['type'] == 'line':
            return self.ray_line_intersection(self.robot_x, self.robot_y, dx, dy,
                                            obj['x1'], obj['y1'], obj['x2'], obj['y2'])
        elif obj['type'] == 'rectangle':
            return self.ray_rectangle_intersection(self.robot_x, self.robot_y, dx, dy,
                                                 obj['x1'], obj['y1'], obj['x2'], obj['y2'])

        return max_distance

    def ray_circle_intersection(self, rx, ry, dx, dy, cx, cy, radius):
        """Find intersection of ray with circle"""
        # Vector from ray origin to circle center
        fx = cx - rx
        fy = cy - ry

        # Quadratic equation coefficients for ray-circle intersection
        a = dx * dx + dy * dy
        b = 2 * (dx * (-fx) + dy * (-fy))
        c = fx * fx + fy * fy - radius * radius

        discriminant = b * b - 4 * a * c

        if discriminant < 0:
            return 8.0  # No intersection

        # Find closest positive intersection
        sqrt_discriminant = math.sqrt(discriminant)
        t1 = (-b - sqrt_discriminant) / (2 * a)
        t2 = (-b + sqrt_discriminant) / (2 * a)

        if t1 > 0.01:  # Small threshold to avoid self-intersection
            return t1
        elif t2 > 0.01:
            return t2
        else:
            return 8.0

    def ray_line_intersection(self, rx, ry, dx, dy, x1, y1, x2, y2):
        """Find intersection of ray with line segment"""
        # Line segment vector
        lx = x2 - x1
        ly = y2 - y1

        # Solve: ray_origin + t * ray_dir = line_start + s * line_dir
        denominator = dx * ly - dy * lx

        if abs(denominator) < 1e-10:
            return 8.0  # Parallel lines

        # Calculate parameters
        t = ((x1 - rx) * ly - (y1 - ry) * lx) / denominator
        s = ((x1 - rx) * dy - (y1 - ry) * dx) / denominator

        # Check if intersection is valid
        if t > 0.01 and 0.0 <= s <= 1.0:
            return t
        else:
            return 8.0

    def ray_rectangle_intersection(self, rx, ry, dx, dy, x1, y1, x2, y2):
        """Find intersection of ray with rectangle (axis-aligned)"""
        # Ensure x1 < x2 and y1 < y2
        if x1 > x2:
            x1, x2 = x2, x1
        if y1 > y2:
            y1, y2 = y2, y1

        # Check intersection with each edge of rectangle
        min_distance = 8.0

        # Top edge
        dist = self.ray_line_intersection(rx, ry, dx, dy, x1, y2, x2, y2)
        min_distance = min(min_distance, dist)

        # Bottom edge
        dist = self.ray_line_intersection(rx, ry, dx, dy, x1, y1, x2, y1)
        min_distance = min(min_distance, dist)

        # Left edge
        dist = self.ray_line_intersection(rx, ry, dx, dy, x1, y1, x1, y2)
        min_distance = min(min_distance, dist)

        # Right edge
        dist = self.ray_line_intersection(rx, ry, dx, dy, x2, y1, x2, y2)
        min_distance = min(min_distance, dist)

        return min_distance

def main(args=None):
    rclpy.init(args=args)
    fake_ros2_robot = FakeROS2Robot()
    rclpy.spin(fake_ros2_robot)
    fake_ros2_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
