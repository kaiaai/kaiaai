import asyncio
import signal
import sys
import time
import rclpy
from robot_client_ros2 import RobotClientROS2

# Robot specification configuration
ROBOT_SPECIFICATION = {
    "api_version": 0,
    "model_name": "CleanBot Pro 3000",
    "manufacturer": "RoboVac Industries",
    "firmware_version": "2.4.1",
    "hardware_version": "v3.1",
    "sensors": [
        "lidar_2d",
        "cliff_sensors",
        "bump_sensors",
        "imu",
        "wheel_encoders",
        "camera_rgb",
        "wifi"
    ],
    "capabilities": [
        "vacuum_cleaning",
        "mopping",
        "edge_cleaning",
        "spot_cleaning",
        "auto_docking",
        "mapping",
        "navigation",
        "remote_control"
    ],
    "battery_capacity_mah": 5200,
    "max_speed_ms": 0.35,
    "shape": {
        "type": "circular",
        "diameter_m": 0.35
    },
    "height_m": 0.092,
    "lidar_sensor": {
        "position": {
            "x_m": 0.0,
            "y_m": 0.0,
            "z_m": 0.082
        },
        "orientation": {
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": 0.0
        },
        "min_range_m": 0.12,
        "max_range_m": 8.0
    },
    "drive_type": "differential",
    "wheel_track_distance_m": 0.235,
    "docking_capability": True,
    "max_angular_speed_rad_s": 2.0,
    "weight_kg": 3.2
}


async def spin(robot, should_exit_func):
    """Run ROS2 spinning asynchronously"""
    try:
        while rclpy.ok() and not should_exit_func():
            rclpy.spin_once(robot, timeout_sec=0.01)
            await asyncio.sleep(0.001)
    except asyncio.CancelledError:
        pass

async def main():
    """Main function to run the robot controller"""
    print("🤖 Starting Robot Controller Client...")
    print("📡 This robot controller will connect to the cloud controller and simulate:")
    print("   • Receiving movement and turn commands")
    print("   • Sending sensor data (position, battery, temperature)")
    print("   • Publishing commands to ROS2 topic /command_received")
    print("   • Subscribing to ROS2 topic /sensor_data")
    print()

    # Initialize ROS2
    rclpy.init()

    robot = None
    should_exit = False

    try:
        robot = RobotClientROS2(specification=ROBOT_SPECIFICATION)

        # Set up disconnect callback to stop reconnection and exit
        def disconnect_callback(command):
            nonlocal should_exit
            print("🛑 Disconnect command received - stopping reconnection and exiting")
            should_exit = True

        robot.add_message_callback('end_session', disconnect_callback)

        # Set up signal handlers for graceful shutdown
        def signal_handler(signum, frame):
            print(f"\n🛑 Received signal {signum}")
            if robot:
                robot.cleanup_and_exit()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # Create tasks for both ROS2 spinning and cloud connection
        async def connect_with_retries():
            retry_delay = 3
            attempt = 0

            while not should_exit:
                attempt += 1
                try:
                    print(f"🔗 Connection attempt {attempt}...")
                    await robot.connect_to_cloud("ws://host.docker.internal:8000/robot")
                    print("🔌 Connection lost. Will retry...")
                    # If we reach here, connection was lost, so we'll retry
                    if should_exit:
                        print("🛑 Stopping reconnection due to disconnect command")
                        break
                except KeyboardInterrupt:
                    print("\n🛑 Connection attempts stopped by user")
                    break
                except Exception as e:
                    print(f"❌ Connection attempt {attempt} failed: {e}")
                    if should_exit:
                        print("🛑 Stopping reconnection due to disconnect command")
                        break
                    print(f"🔄 Retrying in {retry_delay} seconds... (Press Ctrl+C to stop)")
                    try:
                        await asyncio.sleep(retry_delay)
                    except KeyboardInterrupt:
                        print("\n🛑 Connection attempts stopped by user")
                        break

            print("🔌 Reconnection loop stopped")

        # Run both ROS2 spinning and cloud connection concurrently
        await asyncio.gather(
            spin(robot, lambda: should_exit),
            connect_with_retries()
        )

    except KeyboardInterrupt:
        print("\n🛑 Robot controller stopped by user")
        if robot:
            robot.cleanup_and_exit()
    finally:
        # Cleanup ROS2
        try:
            if robot:
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
    except KeyboardInterrupt:
        print("\n🛑 Program interrupted")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        print("👋 Robot controller client terminated")
