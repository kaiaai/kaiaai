import asyncio
import logging
from typing import Dict, List, Callable, Any

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MessageCallbackMixin:
    """Base class providing message callback functionality"""

    def __init__(self):
        # Callbacks for each message type
        self.message_callbacks: Dict[str, List[Callable[[Dict[str, Any]], None]]] = {}
        # Latest message storage for each message type
        self.latest_messages: Dict[str, Dict[str, Any]] = {}

    def add_message_callback(self, message_type: str, callback: Callable[[Dict[str, Any]], None]):
        """Add a callback for a specific message type"""
        if message_type not in self.message_callbacks:
            self.message_callbacks[message_type] = []
        self.message_callbacks[message_type].append(callback)

    def remove_message_callback(self, message_type: str, callback: Callable[[Dict[str, Any]], None]):
        """Remove a callback for a specific message type"""
        if message_type in self.message_callbacks:
            try:
                self.message_callbacks[message_type].remove(callback)
            except ValueError:
                logger.warning(f"Callback not found for message type {message_type}")

    def inject_message(self, message: Dict[str, Any]):
        """Invoke all callbacks for a message and store as latest message"""
        message_type = message.get('type')
        if message_type:
            # Store as latest message
            self.latest_messages[message_type] = message

            # Invoke callbacks if any are registered
            if message_type in self.message_callbacks:
                for callback in self.message_callbacks[message_type]:
                    try:
                        result = callback(message)
                        # If the callback is async, schedule it to run
                        if asyncio.iscoroutine(result):
                            try:
                                # Try to get the current event loop and schedule the coroutine
                                loop = asyncio.get_event_loop()
                                if loop.is_running():
                                    loop.create_task(result)
                                else:
                                    # If no loop is running, run it in a new event loop
                                    asyncio.run(result)
                            except RuntimeError:
                                # If we can't get a loop, create a new one
                                asyncio.run(result)
                    except Exception as e:
                        logger.error(f"Error in message callback for {message_type}: {e}")

    def register_message_type(self, message_type: str):
        """Register a new message type for callbacks"""
        if message_type not in self.message_callbacks:
            self.message_callbacks[message_type] = []
        # Also initialize latest message storage
        if message_type not in self.latest_messages:
            self.latest_messages[message_type] = None

    def get_message_types(self):
        """Get list of all available message types that can have callbacks"""
        return list(self.message_callbacks.keys())

    def get_latest_message(self, message_type: str):
        """Get the latest message for a specific message type"""
        return self.latest_messages.get(message_type)

    def get_all_latest_messages(self):
        """Get all latest messages as a dictionary"""
        return self.latest_messages.copy()

    async def wait_for_message(self, message_type: str, timeout_seconds: float = None):
        """Wait for a message of the specified type

        Args:
            message_type: The type of message to wait for
            timeout_seconds: Optional maximum time to wait in seconds. If None, waits indefinitely.

        Returns:
            The received message if successful, None if timeout occurred
        """
        # Create event and storage for the message
        message_received = asyncio.Event()
        received_message = None

        def message_callback(message):
            nonlocal received_message
            received_message = message
            message_received.set()

        # Add callback for the specified message type
        self.add_message_callback(message_type, message_callback)

        try:
            if timeout_seconds is not None:
                # Wait for the message with timeout
                await asyncio.wait_for(message_received.wait(), timeout=timeout_seconds)
            else:
                # Wait for the message indefinitely
                await message_received.wait()
            logger.debug(f"Received {message_type} message")
            return received_message
        except asyncio.TimeoutError:
            logger.warning(f"Timeout waiting for {message_type} message after {timeout_seconds}s")
            return None
        finally:
            # Remove the callback
            self.remove_message_callback(message_type, message_callback)
