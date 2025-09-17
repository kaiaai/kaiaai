import asyncio
import websockets
import json
import time
import random
import base64
import io
from PIL import Image, ImageDraw, ImageFont
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class DemoRobot(Node):
    def __init__(self):
        super().__init__('demo_robot')
        self.position = {"x": 0.0, "y": 0.0}
        self.angle = 0.0  # Robot's facing direction in degrees
        self.battery = 100.0
        self.temperature = 25.0
        self.websocket = None
        self.running = False

        # Initialize ROS2 publishers and subscribers
        self.command_publisher = self.create_publisher(String, '/command_received', 10)
        self.sensor_subscription = self.create_subscription(
            String,
            '/sensor_data',
            self.sensor_data_callback,
            10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('Demo robot node initialized')
        
    async def connect_to_cloud(self, uri="ws://localhost:8000/robot"):
        """Connect to the cloud controller"""
        try:
            self.websocket = await websockets.connect(uri)
            print(f"✅ Connected to cloud controller at {uri}")
            self.running = True
            
            # Start background tasks
            await asyncio.gather(
                self.listen_for_commands(),
                self.send_sensor_data(),
                self.send_camera_data()
            )
        except Exception as e:
            print(f"❌ Failed to connect to cloud: {e}")
            self.running = False
    
    async def listen_for_commands(self):
        """Listen for commands from the cloud controller"""
        try:
            async for message in self.websocket:
                command = json.loads(message)

                # Publish command to ROS2 topic
                msg = String()
                msg.data = message  # Send the original JSON string
                self.command_publisher.publish(msg)
                self.get_logger().info(f'Published command to /command_received: {message}')

                await self.execute_command(command)
        except websockets.exceptions.ConnectionClosed:
            print("🔌 Connection to cloud lost")
            self.running = False
        except Exception as e:
            print(f"❌ Error listening for commands: {e}")
    
    async def execute_command(self, command):
        """Execute received commands"""
        command_type = command.get("type")

        if command_type == "move":
            await self.move_to_position(command.get("x", 0), command.get("y", 0))
        elif command_type == "turn":
            await self.turn(command.get("angle", 0))
        else:
            print(f"❓ Unknown command type: {command_type}")

    def sensor_data_callback(self, msg):
        """Handle sensor data received from ROS2 topic"""
        try:
            sensor_data_json = msg.data
            self.get_logger().info(f'Received sensor data from /sensor_data: {sensor_data_json}')

            # Forward the sensor data to websocket
            if self.websocket and self.running:
                asyncio.create_task(self.forward_sensor_data(sensor_data_json))
        except Exception as e:
            self.get_logger().error(f'Error processing sensor data: {e}')

    async def forward_sensor_data(self, sensor_data_json):
        """Forward sensor data from ROS2 to websocket connection"""
        try:
            if self.websocket and self.running:
                await self.websocket.send(sensor_data_json)
                self.get_logger().info(f'Forwarded sensor data to websocket: {sensor_data_json}')
        except Exception as e:
            self.get_logger().error(f'Error forwarding sensor data to websocket: {e}')

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

            # Forward the scan data to websocket
            if self.websocket and self.running:
                asyncio.create_task(self.forward_scan_data(scan_data))
        except Exception as e:
            self.get_logger().error(f'Error processing LaserScan data: {e}')

    async def forward_scan_data(self, scan_data):
        """Forward LaserScan data to websocket connection"""
        try:
            if self.websocket and self.running:
                await self.websocket.send(json.dumps(scan_data))
                self.get_logger().info(f'Forwarded LaserScan data to websocket')
        except Exception as e:
            self.get_logger().error(f'Error forwarding LaserScan data to websocket: {e}')
    
    async def move_to_position(self, target_x, target_y):
        """Simulate moving to a target position"""
        print(f"🚀 Moving from ({self.position['x']:.1f}, {self.position['y']:.1f}) to ({target_x}, {target_y})")
        
        # Calculate distance and simulate movement time
        distance = math.sqrt((target_x - self.position["x"])**2 + (target_y - self.position["y"])**2)
        movement_time = distance * 0.1  # Simulate 10 units per second movement speed
        
        # Simulate gradual movement
        steps = max(int(movement_time * 10), 1)  # 10 steps per second
        for step in range(steps):
            progress = (step + 1) / steps
            self.position["x"] = self.position["x"] + (target_x - self.position["x"]) * progress / steps * steps
            self.position["y"] = self.position["y"] + (target_y - self.position["y"]) * progress / steps * steps
            await asyncio.sleep(0.1)
        
        # Set final position
        self.position["x"] = target_x
        self.position["y"] = target_y
        print(f"✅ Arrived at position ({target_x}, {target_y})")
    
    async def turn(self, degrees):
        """Simulate turning by specified degrees"""
        print(f"🔄 Turning {degrees} degrees (current angle: {self.angle}°)")
        
        # Simulate turning time (1 second per 90 degrees)
        turn_time = abs(degrees) / 90.0
        steps = max(int(turn_time * 10), 1)
        
        start_angle = self.angle
        for step in range(steps):
            progress = (step + 1) / steps
            self.angle = start_angle + degrees * progress
            await asyncio.sleep(0.1)
        
        # Normalize angle to 0-360 range
        self.angle = self.angle % 360
        print(f"✅ Turned to angle {self.angle}°")
    
    async def send_sensor_data(self):
        """Send periodic sensor data to the cloud"""
        while self.running and self.websocket:
            try:
                # Simulate some sensor noise and battery drain
                self.battery = max(0, self.battery - random.uniform(0.01, 0.05))
                self.temperature = 25.0 + random.uniform(-2, 3)
                
                sensor_data = {
                    "type": "sensors",
                    "position": {
                        "x": round(self.position["x"], 2),
                        "y": round(self.position["y"], 2)
                    },
                    "angle": round(self.angle, 1),
                    "battery": round(self.battery, 1),
                    "temperature": round(self.temperature, 1),
                    "timestamp": time.strftime("%H:%M:%S")
                }
                
                await self.websocket.send(json.dumps(sensor_data))
                await asyncio.sleep(2)  # Send sensor data every 2 seconds
                
            except Exception as e:
                print(f"❌ Error sending sensor data: {e}")
                break
    
    def generate_camera_image(self):
        """Generate a simple demo camera image"""
        # Create a 400x300 image
        img = Image.new('RGB', (400, 300), color='lightblue')
        draw = ImageDraw.Draw(img)
        
        # Draw some simple graphics to simulate a camera view
        # Draw floor grid
        for i in range(0, 400, 40):
            draw.line([(i, 0), (i, 300)], fill='lightgray', width=1)
        for i in range(0, 300, 30):
            draw.line([(0, i), (400, i)], fill='lightgray', width=1)
        
        # Draw robot position indicator
        robot_screen_x = int(200 + self.position["x"] * 10)  # Center at 200, scale by 10
        robot_screen_y = int(150 + self.position["y"] * 10)  # Center at 150, scale by 10
        
        # Clamp to image bounds
        robot_screen_x = max(10, min(390, robot_screen_x))
        robot_screen_y = max(10, min(290, robot_screen_y))
        
        draw.ellipse([robot_screen_x-10, robot_screen_y-10, robot_screen_x+10, robot_screen_y+10], 
                    fill='red', outline='darkred', width=2)
        
        # Draw direction indicator
        angle_rad = math.radians(self.angle)
        end_x = robot_screen_x + int(20 * math.cos(angle_rad))
        end_y = robot_screen_y + int(20 * math.sin(angle_rad))
        draw.line([(robot_screen_x, robot_screen_y), (end_x, end_y)], fill='darkred', width=3)
        
        # Add some text
        try:
            draw.text((10, 10), f"Robot Camera View", fill='black')
            draw.text((10, 30), f"Pos: ({self.position['x']:.1f}, {self.position['y']:.1f})", fill='black')
            draw.text((10, 50), f"Angle: {self.angle:.1f}°", fill='black')
            draw.text((10, 70), f"Battery: {self.battery:.1f}%", fill='black')
        except:
            pass  # Font might not be available
        
        # Add some random "obstacles"
        for _ in range(3):
            x = random.randint(50, 350)
            y = random.randint(100, 250)
            size = random.randint(15, 30)
            draw.rectangle([x, y, x+size, y+size], fill='brown', outline='black')
        
        # Convert to base64
        buffer = io.BytesIO()
        img.save(buffer, format='JPEG', quality=85)
        buffer.seek(0)
        return base64.b64encode(buffer.getvalue()).decode()
    
    async def send_camera_data(self):
        """Send periodic camera images to the cloud"""
        while self.running and self.websocket:
            try:
                image_data = self.generate_camera_image()
                
                camera_data = {
                    "type": "camera",
                    "image": image_data,
                    "timestamp": time.strftime("%H:%M:%S")
                }
                
                await self.websocket.send(json.dumps(camera_data))
                await asyncio.sleep(3)  # Send camera data every 3 seconds
                
            except Exception as e:
                print(f"❌ Error sending camera data: {e}")
                break

async def spin(robot):
    """Run ROS2 spinning asynchronously"""
    try:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.01)
            await asyncio.sleep(0.001)
    except asyncio.CancelledError:
        pass

async def main():
    """Main function to run the demo robot"""
    print("🤖 Starting Demo Robot Client...")
    print("📡 This robot will connect to the cloud controller and simulate:")
    print("   • Receiving movement and turn commands")
    print("   • Sending sensor data (position, battery, temperature)")
    print("   • Sending simulated camera images")
    print("   • Publishing commands to ROS2 topic /command_received")
    print("   • Subscribing to ROS2 topic /sensor_data")
    print()

    # Initialize ROS2
    rclpy.init()

    try:
        robot = DemoRobot()

        # Create tasks for both ROS2 spinning and cloud connection
        async def connect_with_retries():
            max_retries = 5
            retry_delay = 3

            for attempt in range(max_retries):
                try:
                    await robot.connect_to_cloud("ws://host.docker.internal:8000/robot")
                    break
                except Exception as e:
                    print(f"⏳ Connection attempt {attempt + 1}/{max_retries} failed: {e}")
                    if attempt < max_retries - 1:
                        print(f"🔄 Retrying in {retry_delay} seconds...")
                        await asyncio.sleep(retry_delay)
                    else:
                        print("❌ Max retries reached. Make sure the backend server is running on localhost:8000")

        # Run both ROS2 spinning and cloud connection concurrently
        await asyncio.gather(
            spin(robot),
            connect_with_retries()
        )

    except KeyboardInterrupt:
        print("\n🛑 Robot client stopped by user")
    finally:
        # Cleanup ROS2
        try:
            robot.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except:
        pass
