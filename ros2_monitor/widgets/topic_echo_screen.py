#!/usr/bin/env python3
import yaml
import collections.abc
from textual.screen import Screen
from textual.app import ComposeResult
from textual.widgets import Header, Footer, Log

def ros_message_to_dict(msg):
    """
    Recursively convert a ROS message object into a Python dictionary.
    This allows it to be properly serialized by libraries like PyYAML.
    """
    result = {}
    # Iterate through all fields in the message
    for field, field_type in msg.get_fields_and_field_types().items():
        value = getattr(msg, field)

        # Recursively convert nested messages
        if hasattr(value, 'get_fields_and_field_types'):
            result[field] = ros_message_to_dict(value)
        # Recursively convert sequences (arrays) of messages
        elif isinstance(value, collections.abc.Sequence) and not isinstance(value, str):
            result[field] = [ros_message_to_dict(item) for item in value]
        # Handle primitive types
        else:
            result[field] = value
    return result


class TopicEchoScreen(Screen):
    """A screen that subscribes to a topic and displays its messages."""

    BINDINGS = [("escape", "app.pop_screen", "Back")]

    def __init__(self, topic_name: str, topic_type: str, ros_manager, **kwargs):
        """Initialize the screen."""
        super().__init__(**kwargs)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.ros_manager = ros_manager

    def compose(self) -> ComposeResult:
        """Compose the layout of the topic echo screen."""
        yield Header()
        yield Log(highlight=True, id="topic-log")
        yield Footer()

    def on_mount(self) -> None:
        """Called when the screen is mounted."""
        self.query_one(Header).tall = False
        self.sub_title = f"Echoing topic: {self.topic_name}"
        log = self.query_one("#topic-log", Log)
        log.write_line(f"Subscribing to {self.topic_name} ({self.topic_type})...")

        self.ros_manager.subscribe_to_topic(
            self.topic_name,
            self.topic_type,
            self.on_new_message
        )

    def on_unmount(self) -> None:
        """Called when the screen is unmounted."""
        self.ros_manager.unsubscribe_from_topic(self.topic_name)

    def on_new_message(self, msg) -> None:
        """
        Callback that receives messages from the ROS thread and displays them.
        """
        log = self.query_one("#topic-log", Log)
        
        try:
            # FIX: First, convert the ROS message to a dictionary
            message_dict = ros_message_to_dict(msg)
            # Then, dump the dictionary to a nicely formatted YAML string
            pretty_msg = yaml.dump(message_dict, indent=2, sort_keys=False, Dumper=yaml.Dumper)
        except Exception:
            # Fallback to the standard string representation if conversion fails
            pretty_msg = str(msg)

        log.write_line("---")
        log.write_line("\n")
        log.write(pretty_msg)