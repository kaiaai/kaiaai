import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class FakeLaserScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_laser_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_fake_scan)
        self.get_logger().info('Fake LaserScan Publisher Node started.')

    def publish_fake_scan(self):
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

        # Create fake ranges (e.g., a constant distance)
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = np.random.uniform(low=0.5, high=8.0, size=(360,))
        msg.intensities = [] # Optional, can be left empty

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing fake LaserScan message.')

def main(args=None):
    rclpy.init(args=args)
    fake_laser_scan_publisher = FakeLaserScanPublisher()
    rclpy.spin(fake_laser_scan_publisher)
    fake_laser_scan_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
