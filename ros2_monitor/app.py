#!/usr/bin/env python3
import rclpy
from textual.app import App
from textual.widgets import Header, Footer

from ros_utils import ROS2DataManager
from widgets.main_view import MainView
from widgets.node_detail_screen import NodeDetailScreen
from widgets.topic_echo_screen import TopicEchoScreen

class ROS2MonitorApp(App):
    """A Textual application to monitor a ROS2 graph."""

    CSS_PATH = "styles.css"
    BINDINGS = [
        ("r", "refresh", "Refresh Data"),
        ("q", "quit", "Quit"),
        ("d", "toggle_dark", "Toggle Dark Mode"),
    ]

    def __init__(self):
        """Initialize the app and the ROS2 data manager."""
        super().__init__()
        self.ros_manager = ROS2DataManager(self)
        self.main_view = MainView()

    def compose(self) -> "ComposeResult":
        """Compose the widget layout."""
        yield Header()
        yield self.main_view
        yield Footer()

    def on_mount(self) -> None:
        """Called when the app is mounted."""
        # FIX: Replaced 'call_later' with the more compatible 'set_timer'.
        # This will call the function once after the specified delay.
        self.set_timer(0.5, self.update_ros_info)

    def on_unmount(self) -> None:
        """Called when the app is about to quit."""
        # self.ros_manager.shutdown() # Shutdown is handled by rclpy.shutdown now
        if rclpy.ok():
            rclpy.shutdown()

    def on_main_view_node_selected(self, message: MainView.NodeSelected) -> None:
        node_detail_screen = NodeDetailScreen(
            node_name=message.name,
            node_namespace=message.namespace,
            ros_manager=self.ros_manager
        )
        self.push_screen(node_detail_screen)

    def on_main_view_topic_selected(self, message: MainView.TopicSelected) -> None:
        topic_echo_screen = TopicEchoScreen(
            topic_name=message.name,
            topic_type=message.type,
            ros_manager=self.ros_manager
        )
        self.push_screen(topic_echo_screen)

    def on_main_view_tab_switched(self, message: MainView.TabSwitched) -> None:
        self.update_ros_info()

    def action_refresh(self) -> None:
        """Refresh the data for the current view when 'r' is pressed."""
        self.notify("Refreshing data...")
        if isinstance(self.screen, NodeDetailScreen):
            self.screen.update_details()
        else:
            self.update_ros_info()

    def update_ros_info(self) -> None:
        """Fetch and display the latest ROS2 graph information for the main view."""
        active_tab_id = self.main_view.get_active_tab()

        if active_tab_id == "nodes":
            nodes = self.ros_manager.get_nodes()
            self.main_view.update_nodes_table(nodes)
        elif active_tab_id == "topics":
            topics = self.ros_manager.get_topics()
            self.main_view.update_topics_table(topics)
        elif active_tab_id == "services":
            services, _ = self.ros_manager.get_services_and_actions()
            self.main_view.update_services_table(services)
        elif active_tab_id == "actions":
            _, actions = self.ros_manager.get_services_and_actions()
            self.main_view.update_actions_table(actions)