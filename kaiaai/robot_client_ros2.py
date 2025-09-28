import asyncio
import random
import base64
import io
from PIL import Image, ImageDraw, ImageFont
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, BatteryState, CompressedImage
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from kaiaai_msgs.msg import WifiState
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import json
from robot_client import RobotClient

class RobotClientROS2(RobotClient, Node):
    """ROS2 implementation of the RobotClient"""

    def __init__(self, specification=None):
        # Initialize both parent classes
        RobotClient.__init__(self, specification)
        Node.__init__(self, 'robot_controller')

        # ROS2-specific data storage
        self.position = {"x": 0.0, "y": 0.0}
        self.angle = 0.0  # Robot's facing direction in degrees
        self.battery_data = None  # Will store latest BatteryState message
        self.wifi_data = None  # Will store latest WifiState message
        self.cmd_vel_data = None  # Will store latest Twist message from /cmd_vel
        self.map_data = None  # Will store latest OccupancyGrid message from /map
        self.camera_data = None  # Will store latest CompressedImage message from /camera/image/compressed
        self.pose_data = None  # Will store latest PoseWithCovarianceStamped message from /amcl_pose
        self.temperature = 25.0

        # Navigation state
        self.current_goal_handle = None
        self.navigation_status = "idle"  # idle, navigating, succeeded, failed, cancelled

        # TF2 setup for pose monitoring
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.global_frame_id = 'map'
        self.base_frame_id = 'base_footprint'

        # Create QoS profile similar to nav2::qos::LatchedPublisherQoS
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Initialize ROS2 publishers and subscribers
        self.command_publisher = self.create_publisher(String, '/command_received', 10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.remote_control_status_publisher = self.create_publisher(String, '/remote_control_status', latched_qos)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', latched_qos)

        # Subscriptions
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.battery_subscription = self.create_subscription(
            BatteryState,
            '/battery_status',
            self.battery_callback,
            10
        )
        self.wifi_subscription = self.create_subscription(
            WifiState,
            '/wifi_state',
            self.wifi_callback,
            10
        )
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.camera_callback,
            10
        )

        # Timer for pose monitoring using TF2 (every 0.5 seconds)
        self.pose_timer = self.create_timer(0.5, self.pose_timer_callback)

        # Create action client for navigation
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('ROS2 robot client node initialized')

        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)

    def set_remote_control_status(self, connected: bool):
        """Set remote control status by publishing to /remote_control_status topic"""
        try:
            # Prevent duplicate disconnect messages
            if not connected and self.disconnect_status_set:
                self.get_logger().debug('Disconnect status already set, skipping duplicate')
                return

            status_data = {
                "remote_control": connected,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "robot_id": "robot_controller"
            }

            msg = String()
            msg.data = json.dumps(status_data)

            self.remote_control_status_publisher.publish(msg)
            self.get_logger().info(f'Published remote control status: {connected}')
            print(f"📡 Remote control status: {'CONNECTED' if connected else 'DISCONNECTED'}")

            # Set flag when setting disconnect status
            if not connected:
                self.disconnect_status_set = True
            else:
                # Reset flag when setting connect status
                self.disconnect_status_set = False

        except Exception as e:
            self.get_logger().error(f'Error publishing remote control status: {e}')
            print(f"❌ Error publishing remote control status: {e}")

    async def set_velocity(self, command):
        """Set robot velocity by publishing Twist command to /cmd_vel topic"""
        try:
            # Extract linear_x and angular_z from command
            linear_x = command.get('linear_x', 0.0)
            angular_z = command.get('angular_z', 0.0)

            twist_msg = Twist()
            twist_msg.linear.x = float(linear_x)
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = float(angular_z)

            self.twist_publisher.publish(twist_msg)
            self.get_logger().info(f'Published Twist command: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}')
            print(f"🚀 Published Twist: linear.x={linear_x:.2f} m/s, angular.z={angular_z:.2f} rad/s")

        except Exception as e:
            self.get_logger().error(f'Error publishing Twist command: {e}')
            print(f"❌ Error publishing Twist command: {e}")

    async def set_occupancy_grid(self, grid_data):
        """Set occupancy grid by publishing OccupancyGrid to /map topic with latched QoS"""
        try:
            if not grid_data:
                print("❌ No grid data provided")
                return

            # Create OccupancyGrid message
            grid_msg = OccupancyGrid()

            # Set header
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            grid_msg.header.frame_id = grid_data.get("frame_id", "map")

            # Set map metadata
            grid_msg.info.resolution = float(grid_data.get("resolution", 0.05))
            grid_msg.info.width = int(grid_data.get("width", 384))
            grid_msg.info.height = int(grid_data.get("height", 384))

            # Set origin
            origin_data = grid_data.get("origin", {})
            grid_msg.info.origin.position.x = float(origin_data.get("x", -9.6))
            grid_msg.info.origin.position.y = float(origin_data.get("y", -9.6))
            grid_msg.info.origin.position.z = float(origin_data.get("z", 0.0))
            grid_msg.info.origin.orientation.x = float(origin_data.get("qx", 0.0))
            grid_msg.info.origin.orientation.y = float(origin_data.get("qy", 0.0))
            grid_msg.info.origin.orientation.z = float(origin_data.get("qz", 0.0))
            grid_msg.info.origin.orientation.w = float(origin_data.get("qw", 1.0))

            # Set map load time
            grid_msg.info.map_load_time = self.get_clock().now().to_msg()

            # Set occupancy data (convert from list to int8 array)
            occupancy_data = grid_data.get("data", [])
            if isinstance(occupancy_data, list):
                grid_msg.data = [int(cell) for cell in occupancy_data]
            else:
                print("❌ Invalid occupancy data format")
                return

            # Publish the message
            self.map_publisher.publish(grid_msg)
            self.get_logger().info(f'Published OccupancyGrid: {grid_msg.info.width}x{grid_msg.info.height}, resolution={grid_msg.info.resolution:.3f}m/cell')
            print(f"🗺️ Published OccupancyGrid to /map: {grid_msg.info.width}x{grid_msg.info.height} cells")

        except Exception as e:
            self.get_logger().error(f'Error publishing OccupancyGrid: {e}')
            print(f"❌ Error publishing OccupancyGrid: {e}")

    async def navigate_to_pose(self, pose_data, relative=False):
        """Send navigation goal to navigate_to_pose action server"""
        try:
            if not pose_data:
                print("❌ No pose data provided")
                return

            # Wait for action server
            if not self.navigate_to_pose_client.wait_for_server(timeout_sec=5.0):
                print("❌ Navigation action server not available")
                await self.send_navigation_status("failed", "Action server not available")
                return

            # Create goal message
            goal_msg = NavigateToPose.Goal()

            # Set pose
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.header.frame_id = pose_data.get("frame_id", "map")

            # Calculate target pose (absolute or relative)
            target_x = float(pose_data.get("x", 0.0))
            target_y = float(pose_data.get("y", 0.0))
            target_z = float(pose_data.get("z", 0.0))
            target_qx = float(pose_data.get("qx", 0.0))
            target_qy = float(pose_data.get("qy", 0.0))
            target_qz = float(pose_data.get("qz", 0.0))
            target_qw = float(pose_data.get("qw", 1.0))

            if relative:
                # Get current robot pose for relative navigation
                current_pose = self.get_map_pos_2d()
                if current_pose is None:
                    print("❌ Cannot get current robot pose for relative navigation")
                    await self.send_navigation_status("failed", "Cannot get current robot pose")
                    return

                # Calculate absolute target pose from relative offset
                current_x = current_pose["x"]
                current_y = current_pose["y"]
                current_yaw = current_pose["yaw"]

                # Transform relative position by current robot orientation
                cos_yaw = math.cos(current_yaw)
                sin_yaw = math.sin(current_yaw)

                # Rotate relative offset by current yaw
                rotated_x = target_x * cos_yaw - target_y * sin_yaw
                rotated_y = target_x * sin_yaw + target_y * cos_yaw

                # Add to current position
                target_x = current_x + rotated_x
                target_y = current_y + rotated_y
                target_z = target_z  # Z remains as offset

                # For orientation, add relative yaw to current yaw
                relative_yaw = math.atan2(2.0 * (target_qw * target_qz + target_qx * target_qy),
                                        1.0 - 2.0 * (target_qy * target_qy + target_qz * target_qz))
                final_yaw = current_yaw + relative_yaw

                # Convert back to quaternion
                target_qx = 0.0
                target_qy = 0.0
                target_qz = math.sin(final_yaw / 2.0)
                target_qw = math.cos(final_yaw / 2.0)

                print(f"🎯 Relative navigation: offset ({pose_data.get('x', 0.0):.2f}, {pose_data.get('y', 0.0):.2f}) -> absolute ({target_x:.2f}, {target_y:.2f})")
            else:
                print(f"🎯 Absolute navigation to ({target_x:.2f}, {target_y:.2f})")

            # Set final target pose
            goal_msg.pose.pose.position.x = target_x
            goal_msg.pose.pose.position.y = target_y
            goal_msg.pose.pose.position.z = target_z
            goal_msg.pose.pose.orientation.x = target_qx
            goal_msg.pose.pose.orientation.y = target_qy
            goal_msg.pose.pose.orientation.z = target_qz
            goal_msg.pose.pose.orientation.w = target_qw

            # Send goal
            self.navigation_status = "navigating"
            await self.send_navigation_status("navigating", "Navigation goal sent")

            send_goal_future = self.navigate_to_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=self.navigation_feedback_callback
            )

            # Wait for goal to be accepted
            goal_handle = await asyncio.wrap_future(send_goal_future)

            if not goal_handle.accepted:
                print("❌ Navigation goal rejected")
                self.navigation_status = "failed"
                await self.send_navigation_status("failed", "Goal rejected")
                return

            print("✅ Navigation goal accepted")
            self.current_goal_handle = goal_handle

            # Wait for result
            result_future = goal_handle.get_result_async()
            result = await asyncio.wrap_future(result_future)

            # Handle result
            if result.status == 4:  # SUCCEEDED
                print("✅ Navigation succeeded")
                self.navigation_status = "succeeded"
                await self.send_navigation_status("succeeded", "Navigation completed successfully")
            elif result.status == 5:  # CANCELED
                print("🛑 Navigation cancelled")
                self.navigation_status = "cancelled"
                await self.send_navigation_status("cancelled", "Navigation was cancelled")
            else:
                print(f"❌ Navigation failed with status: {result.status}")
                self.navigation_status = "failed"
                await self.send_navigation_status("failed", f"Navigation failed with status {result.status}")

            self.current_goal_handle = None

        except Exception as e:
            print(f"❌ Error in navigation: {e}")
            self.navigation_status = "failed"
            await self.send_navigation_status("failed", f"Navigation error: {str(e)}")
            self.current_goal_handle = None

    async def cancel_navigation(self):
        """Cancel current navigation goal"""
        try:
            if self.current_goal_handle is None:
                print("ℹ️ No active navigation goal to cancel")
                await self.send_navigation_status("idle", "No active goal")
                return

            print("🛑 Cancelling navigation goal")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_result = await asyncio.wrap_future(cancel_future)

            if len(cancel_result.goals_canceling) > 0:
                print("✅ Navigation goal cancelled")
                self.navigation_status = "cancelled"
                await self.send_navigation_status("cancelled", "Navigation cancelled by user")
            else:
                print("⚠️ Could not cancel navigation goal")
                await self.send_navigation_status("failed", "Could not cancel goal")

        except Exception as e:
            print(f"❌ Error cancelling navigation: {e}")
            await self.send_navigation_status("failed", f"Cancel error: {str(e)}")

    def cleanup_and_exit(self):
        """Cleanup method to publish remote control status as false before exiting"""
        try:
            self.get_logger().info('Robot client shutting down - publishing remote control status as false')

            # Set remote control status as disconnected
            self.set_remote_control_status(False)

            # Call parent cleanup method
            super().cleanup_and_exit()

        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}')
            print(f"❌ Error during cleanup: {e}")

    # ROS2 callback methods
    def scan_callback(self, msg):
        """Handle LaserScan messages from /scan topic"""
        try:
            # Convert LaserScan message to JSON structure
            scan_data = {
                "type": "laser_scan",
                "header": {
                    "stamp": {
                        "sec": msg.header.stamp.sec,
                        "nanosec": msg.header.stamp.nanosec
                    },
                    "frame_id": msg.header.frame_id
                },
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
                "angle_increment": msg.angle_increment,
                "time_increment": msg.time_increment,
                "scan_time": msg.scan_time,
                "range_min": msg.range_min,
                "range_max": msg.range_max,
                "ranges": list(msg.ranges),
                "intensities": list(msg.intensities),
                "timestamp": time.strftime("%H:%M:%S")
            }

            self.get_logger().info(f'Received LaserScan with {len(msg.ranges)} points')

            # Forward the scan data to cloud controller
            if self.is_connected():
                asyncio.create_task(self.forward_data(scan_data))
        except Exception as e:
            self.get_logger().error(f'Error processing LaserScan data: {e}')

    def battery_callback(self, msg):
        """Handle BatteryState messages from /battery_status topic"""
        try:
            self.battery_data = msg
            self.get_logger().info(f'Received battery status: {msg.percentage:.1f}%')

            # Send battery data immediately to cloud controller
            if self.is_connected():
                battery_data = {
                    "type": "battery",
                    "level": round(msg.percentage, 1),
                    "voltage": round(msg.voltage, 2),
                    "current": round(msg.current, 2),
                    "temperature": round(msg.temperature, 1),
                    "status": msg.power_supply_status,
                    "health": msg.power_supply_health,
                    "timestamp": time.strftime("%H:%M:%S")
                }
                asyncio.create_task(self.forward_data(battery_data))
        except Exception as e:
            self.get_logger().error(f'Error processing battery data: {e}')

    def wifi_callback(self, msg):
        """Handle WifiState messages from /wifi_state topic"""
        try:
            self.wifi_data = msg
            self.get_logger().info(f'Received WiFi status: {msg.rssi_dbm:.1f} dBm')

            # Send wifi data immediately to cloud controller
            if self.is_connected():
                wifi_data = {
                    "type": "wifi",
                    "rssi_dbm": round(msg.rssi_dbm, 1),
                    "signal_strength": self.get_signal_strength_description(msg.rssi_dbm),
                    "timestamp": time.strftime("%H:%M:%S")
                }
                asyncio.create_task(self.forward_data(wifi_data))
        except Exception as e:
            self.get_logger().error(f'Error processing WiFi data: {e}')

    def cmd_vel_callback(self, msg):
        """Handle Twist messages from /cmd_vel topic"""
        try:
            self.cmd_vel_data = msg
            self.get_logger().info(f'Received cmd_vel: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')

            # Send cmd_vel data immediately to cloud controller
            if self.is_connected():
                cmd_vel_data = {
                    "type": "cmd_vel",
                    "linear": {
                        "x": round(msg.linear.x, 3),
                        "y": round(msg.linear.y, 3),
                        "z": round(msg.linear.z, 3)
                    },
                    "angular": {
                        "x": round(msg.angular.x, 3),
                        "y": round(msg.angular.y, 3),
                        "z": round(msg.angular.z, 3)
                    },
                    "timestamp": time.strftime("%H:%M:%S")
                }
                asyncio.create_task(self.forward_data(cmd_vel_data))
        except Exception as e:
            self.get_logger().error(f'Error processing cmd_vel data: {e}')

    def map_callback(self, msg):
        """Handle OccupancyGrid messages from /map topic"""
        try:
            self.map_data = msg
            self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}, resolution={msg.info.resolution:.3f}m/cell')

            # Send map data immediately to cloud controller
            if self.is_connected():
                map_data = {
                    "type": "map",
                    "header": {
                        "stamp": {
                            "sec": msg.header.stamp.sec,
                            "nanosec": msg.header.stamp.nanosec
                        },
                        "frame_id": msg.header.frame_id
                    },
                    "info": {
                        "map_load_time": {
                            "sec": msg.info.map_load_time.sec,
                            "nanosec": msg.info.map_load_time.nanosec
                        },
                        "resolution": msg.info.resolution,
                        "width": msg.info.width,
                        "height": msg.info.height,
                        "origin": {
                            "position": {
                                "x": msg.info.origin.position.x,
                                "y": msg.info.origin.position.y,
                                "z": msg.info.origin.position.z
                            },
                            "orientation": {
                                "x": msg.info.origin.orientation.x,
                                "y": msg.info.origin.orientation.y,
                                "z": msg.info.origin.orientation.z,
                                "w": msg.info.origin.orientation.w
                            }
                        }
                    },
                    "data": list(msg.data),
                    "timestamp": time.strftime("%H:%M:%S")
                }
                asyncio.create_task(self.forward_data(map_data))
        except Exception as e:
            self.get_logger().error(f'Error processing map data: {e}')

    def camera_callback(self, msg):
        """Handle CompressedImage messages from /camera/image/compressed topic"""
        try:
            self.camera_data = msg
            self.get_logger().info(f'Received compressed image: {msg.format}, size={len(msg.data)} bytes')

            # Convert compressed image data to base64 for JSON transmission
            image_base64 = base64.b64encode(msg.data).decode('utf-8')

            # Send camera data immediately to cloud controller
            if self.is_connected():
                camera_data = {
                    "type": "camera",
                    "header": {
                        "stamp": {
                            "sec": msg.header.stamp.sec,
                            "nanosec": msg.header.stamp.nanosec
                        },
                        "frame_id": msg.header.frame_id
                    },
                    "format": msg.format,
                    "data": image_base64,
                    "timestamp": time.strftime("%H:%M:%S")
                }
                asyncio.create_task(self.forward_data(camera_data))
        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {e}')

    def pose_timer_callback(self):
        """Timer callback to get robot pose using TF2"""
        try:
            pose = self.get_map_pos_2d()
            if pose is not None:
                self.get_logger().info(f'TF robot pose: ({pose["x"]:.2f}, {pose["y"]:.2f}, {math.degrees(pose["yaw"]):.1f}°)')

                # Send simplified pose data to cloud controller
                if self.is_connected():
                    pose_data = {
                        "type": "robot_pose",
                        "x": round(pose["x"], 3),
                        "y": round(pose["y"], 3),
                        "yaw": round(math.degrees(pose["yaw"]), 1),  # Convert to degrees
                        "frame_id": self.global_frame_id,
                        "timestamp": time.strftime("%H:%M:%S")
                    }
                    asyncio.create_task(self.forward_data(pose_data))
        except Exception as e:
            self.get_logger().error(f'Error in pose timer callback: {e}')

    # Helper methods
    def get_map_pos_2d(self):
        """Get robot pose in map frame using TF2 - based on nav_util.py getMapPos2d()"""
        tf = self.try_get_map_pos()
        if tf is None:
            return None

        pos = dict()
        pos['x'] = tf.transform.translation.x
        pos['y'] = tf.transform.translation.y
        roll, pitch, yaw = self.euler_from_quaternion(tf.transform.rotation)
        pos['yaw'] = yaw

        return pos

    def try_get_map_pos(self):
        """Try to get transform from map to base_footprint - based on nav_util.py tryGetMapPos()"""
        try:
            now = rclpy.time.Time()
            tf = self.tf_buffer.lookup_transform(self.global_frame_id, self.base_frame_id, now)
            return tf
        except TransformException as ex:
            self.get_logger().debug(f'Could not transform {self.base_frame_id} to {self.global_frame_id}: {ex}')
            return None

    @staticmethod
    def euler_from_quaternion(r):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw) - from nav_util.py
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (r.w * r.x + r.y * r.z)
        t1 = +1.0 - 2.0 * (r.x * r.x + r.y * r.y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (r.w * r.y - r.z * r.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (r.w * r.z + r.x * r.y)
        t4 = +1.0 - 2.0 * (r.y * r.y + r.z * r.z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def get_signal_strength_description(self, rssi_dbm):
        """Convert RSSI dBm to human-readable signal strength"""
        if rssi_dbm >= -30:
            return "excellent"
        elif rssi_dbm >= -50:
            return "very_good"
        elif rssi_dbm >= -60:
            return "good"
        elif rssi_dbm >= -70:
            return "fair"
        elif rssi_dbm >= -80:
            return "weak"
        else:
            return "very_weak"

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        try:
            feedback = feedback_msg.feedback
            current_pose = feedback.current_pose.pose
            distance_remaining = feedback.distance_remaining
            estimated_time_remaining = feedback.estimated_time_remaining

            print(f"📍 Navigation feedback: {distance_remaining:.2f}m remaining, ETA: {estimated_time_remaining.sec}s")

            # Send feedback to cloud controller
            if self.is_connected():
                feedback_data = {
                    "type": "navigation_feedback",
                    "distance_remaining": round(distance_remaining, 2),
                    "estimated_time_remaining": estimated_time_remaining.sec,
                    "current_pose": {
                        "x": round(current_pose.position.x, 3),
                        "y": round(current_pose.position.y, 3),
                        "z": round(current_pose.position.z, 3),
                        "qx": round(current_pose.orientation.x, 3),
                        "qy": round(current_pose.orientation.y, 3),
                        "qz": round(current_pose.orientation.z, 3),
                        "qw": round(current_pose.orientation.w, 3)
                    },
                    "timestamp": time.strftime("%H:%M:%S")
                }
                asyncio.create_task(self.forward_data(feedback_data))

        except Exception as e:
            self.get_logger().error(f'Error processing navigation feedback: {e}')

    # Navigation status methods

    async def send_navigation_status(self, status, message):
        """Send navigation status update to cloud controller"""
        try:
            status_data = {
                "type": "navigation_status",
                "status": status,
                "message": message,
                "timestamp": time.strftime("%H:%M:%S")
            }
            await self.forward_data(status_data)
            self.get_logger().info(f'Sent navigation status: {status} - {message}')
        except Exception as e:
            self.get_logger().error(f'Error sending navigation status: {e}')

    async def execute_command(self, command):
        """Override to publish commands to ROS2 topic"""
        # Publish command to ROS2 topic
        msg = String()
        msg.data = json.dumps(command)  # Send the original JSON string
        self.command_publisher.publish(msg)
        self.get_logger().info(f'Published command to /command_received: {json.dumps(command)}')

        # Call parent implementation
        await super().execute_command(command)
