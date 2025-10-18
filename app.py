#!/usr/bin/env python3
import rclpy
import psutil
from textual.app import App
from textual.widgets import Header, Footer

from ros_utils import ROS2DataManager
from widgets.main_view import MainView
from widgets.node_detail_screen import NodeDetailScreen
from widgets.topic_echo_screen import TopicEchoScreen

class ROS2MonitorApp(App):
    CSS_PATH = "styles.css"
    BINDINGS = [
        ("r", "refresh", "Refresh Data"),
        ("q", "quit", "Quit"),
        ("d", "toggle_dark", "Toggle Dark Mode"),
    ]

    def __init__(self):
        super().__init__()
        self.ros_manager = ROS2DataManager(self)
        self.main_view = MainView()

    def compose(self) -> "ComposeResult":
        yield Header()
        yield self.main_view
        yield Footer()

    def on_mount(self) -> None:
        self.set_timer(0.5, self.update_ros_info)
        self.set_interval(1.0, self.update_system_info)

    def on_unmount(self) -> None:
        # FIX: rclpy.shutdown() is removed from here. main.py now manages it.
        # We still want to shut down our manager's resources if any.
        self.ros_manager.shutdown()

    def update_system_info(self) -> None:
        """Fetches system usage stats, but only if the nodes tab is visible."""
        if self.main_view.get_active_tab() == "nodes":
            cpu_percent = psutil.cpu_percent(interval=None)
            ram_stats = psutil.virtual_memory()
            self.main_view.update_system_display(cpu_percent, ram_stats)

    # ... (rest of the message handlers and actions are unchanged) ...
    def on_main_view_node_selected(self, message: MainView.NodeSelected):
        self.push_screen(NodeDetailScreen(node_name=message.name, node_namespace=message.namespace, ros_manager=self.ros_manager))
    def on_main_view_topic_selected(self, message: MainView.TopicSelected):
        self.push_screen(TopicEchoScreen(topic_name=message.name, topic_type=message.type, ros_manager=self.ros_manager))
    def on_main_view_tab_switched(self, message: MainView.TabSwitched):
        self.update_ros_info()
    def action_refresh(self) -> None:
        self.notify("Refreshing data...")
        if isinstance(self.screen, NodeDetailScreen):
            self.screen.update_details()
        else:
            self.update_ros_info()
    def update_ros_info(self) -> None:
        active_tab_id = self.main_view.get_active_tab()
        if active_tab_id == "nodes":
            self.main_view.update_nodes_table(self.ros_manager.get_nodes())
        elif active_tab_id == "topics":
            self.main_view.update_topics_table(self.ros_manager.get_topics())
        elif active_tab_id == "services":
            services, _ = self.ros_manager.get_services_and_actions()
            self.main_view.update_services_table(services)
        elif active_tab_id == "actions":
            _, actions = self.ros_manager.get_services_and_actions()
            self.main_view.update_actions_table(actions)
        elif active_tab_id == "controllers":
            self.main_view.update_controllers_table(self.ros_manager.get_controllers())