#!/usr/bin/env python3
import subprocess
import threading
from textual.screen import Screen
from textual.app import ComposeResult
from textual.widgets import Header, Footer, Log
from textual.message import Message

class TopicEchoScreen(Screen):
    """
    A screen that runs 'ros2 topic echo' as a subprocess and displays its output
    using a background thread.
    """

    BINDINGS = [("escape", "app.pop_screen", "Back")]

    class NewLine(Message):
        """A message carrying a new line of output from the subprocess."""
        def __init__(self, line: str) -> None:
            self.line = line
            super().__init__()

    def __init__(self, topic_name: str, topic_type: str, ros_manager, **kwargs):
        super().__init__(**kwargs)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.ros_manager = ros_manager
        self.process = None

    def compose(self) -> ComposeResult:
        yield Header()
        yield Log(highlight=True, id="topic-log")
        yield Footer()

    def on_mount(self) -> None:
        """Called when the screen is mounted. Writes status and schedules the process to start."""
        self.sub_title = f"Echoing topic: {self.topic_name}"
        log = self.query_one("#topic-log", Log)
        log.write(f"Starting 'ros2 topic echo {self.topic_name}'...\n")

        # Use call_later to start the subprocess on the next event loop tick.
        # This guarantees the "Starting..." message is rendered first.
        self.call_later(self._start_subprocess)

    def _start_subprocess(self) -> None:
        """Starts the subprocess and the threads to read its output."""
        # It's safer to pass the command as a list and avoid shell=True
        command = ["ros2", "topic", "echo", self.topic_name]

        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # Start background threads to read stdout and stderr
        threading.Thread(target=self._read_stream, args=(self.process.stdout,), daemon=True).start()

    def _read_stream(self, stream):
        """Reads lines from a stream and posts them as messages."""
        for line in iter(stream.readline, b''):
            try:
                decoded_line = line.decode('utf-8')
                self.post_message(self.NewLine(decoded_line))
            except UnicodeDecodeError:
                self.post_message(self.NewLine("[Decode Error]\n"))

    def on_topic_echo_screen_new_line(self, message: NewLine) -> None:
        """Handles a NewLine message by writing its content to the log."""
        log = self.query_one("#topic-log", Log)
        log.write(message.line)

    def on_unmount(self) -> None:
        """Called when the screen is unmounted. Kills the subprocess."""
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.app.log(f"Killed 'ros2 topic echo' process for {self.topic_name}")