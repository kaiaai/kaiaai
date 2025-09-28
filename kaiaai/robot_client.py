import asyncio
import websockets
import json
import time
import sys
import os
from abc import ABC, abstractmethod

# Add the backend directory to Python path to import MessageCallbackMixin
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'backend'))
from message_callback_mixin import MessageCallbackMixin

class RobotClient(ABC, MessageCallbackMixin):
    """Abstract base class for robot clients"""

    def __init__(self, specification=None):
        # Initialize MessageCallbackMixin
        MessageCallbackMixin.__init__(self)

        # Store robot specification
        self.specification = specification or {}
        self.websocket = None
        self.running = False
        self.disconnect_status_set = False  # Flag to prevent duplicate disconnect status setting


        # Register known message types and their callback handlers
        self.register_message_type('twist')
        self.register_message_type('navigate_to_pose')
        self.register_message_type('cancel_navigation')
        self.register_message_type('get_robot_specification')
        self.register_message_type('robot_disconnect')
        self.register_message_type('occupancy_grid')

        # Set up message type callbacks
        self.add_message_callback('twist', self.set_velocity)
        self.add_message_callback('navigate_to_pose', self.navigate_to_pose)
        self.add_message_callback('cancel_navigation', self.cancel_navigation)
        self.add_message_callback('get_robot_specification', self.send_robot_specification)
        self.add_message_callback('robot_disconnect', self.handle_disconnect_command)
        self.add_message_callback('occupancy_grid', self.set_occupancy_grid)

        print("Robot client initialized")


    # Abstract methods that must be implemented by subclasses
    @abstractmethod
    def set_remote_control_status(self, connected: bool):
        """Set remote control status"""
        pass

    @abstractmethod
    async def set_velocity(self, command):
        """Set robot velocity"""
        pass

    @abstractmethod
    async def set_occupancy_grid(self, command):
        """Set occupancy grid"""
        pass

    @abstractmethod
    async def navigate_to_pose(self, command):
        """Send navigation goal"""
        pass

    @abstractmethod
    async def cancel_navigation(self, command=None):
        """Cancel current navigation goal"""
        pass

    def cleanup_and_exit(self):
        """Cleanup method for graceful shutdown"""
        try:
            print(f"\n🛑 Shutting down robot client...")

            # Close websocket connection if it exists
            self.running = False
            if self.websocket:
                # We can't use await here since this might be called from signal handler
                # The websocket will be closed in the finally block of listen_for_commands
                pass

            print(f"✅ Cleanup completed")

        except Exception as e:
            print(f"❌ Error during cleanup: {e}")

    async def connect_to_cloud(self, uri="ws://localhost:8000/robot"):
        """Connect to the cloud controller"""
        self.websocket = await websockets.connect(uri)
        print(f"✅ Connected to cloud controller at {uri}")
        self.running = True

        # Set remote control status as connected
        self.set_remote_control_status(True)

        # Start background tasks
        await asyncio.gather(
            self.listen_for_commands()
        )

    async def listen_for_commands(self):
        """Listen for commands from the cloud controller"""
        try:
            async for message in self.websocket:
                command = json.loads(message)
                await self.execute_command(command)
        except websockets.exceptions.ConnectionClosed:
            print("🔌 Connection to cloud lost")
            self.running = False
        except Exception as e:
            print(f"❌ Error listening for commands: {e}")
            self.running = False
        finally:
            # Set remote control status as disconnected
            self.set_remote_control_status(False)

            # Clean up websocket connection
            if self.websocket:
                try:
                    await self.websocket.close()
                except:
                    pass
                self.websocket = None

    async def execute_command(self, command):
        """Execute received commands using callback system"""
        # Use the MessageCallbackMixin to handle the command
        self.inject_message(command)

    async def send_robot_specification(self, command=None):
        """Send robot specification data to cloud controller via websocket"""
        try:
            # Use the stored robot specification with timestamp
            robot_spec = self.specification.copy()
            robot_spec["timestamp"] = time.strftime("%H:%M:%S")
            robot_spec["type"] = "robot_specification"

            print(f"📋 Sending robot specification")

            # Send robot specification via forward_data
            await self.forward_data(robot_spec)

        except Exception as e:
            print(f"❌ Error sending robot specification: {e}")

    async def handle_disconnect_command(self, command=None):
        """Handle robot_disconnect command from backend"""
        try:
            print(f"🔌 Received disconnect command from backend")

            # Set remote control status as disconnected
            self.set_remote_control_status(False)

            # Close websocket connection
            self.running = False
            if self.websocket:
                await self.websocket.close()
                self.websocket = None
                print(f"✅ Websocket connection closed by disconnect command")

        except Exception as e:
            print(f"❌ Error handling disconnect command: {e}")

    def is_connected(self):
        """Check if the robot client is connected and ready to communicate"""
        return self.websocket is not None and self.running

    async def forward_data(self, data):
        """Forward data to websocket connection"""
        try:
            if self.is_connected():
                await self.websocket.send(json.dumps(data))
                print(f"📡 Forwarded {data.get('type', 'unknown')} data to websocket")
        except Exception as e:
            print(f"❌ Error forwarding data to websocket: {e}")
