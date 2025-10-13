#!/usr/bin/env python3
import rclpy
import importlib
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Note: Parameter-related imports are currently unused but kept for future implementation
from rclpy.parameter import Parameter
from rcl_interfaces.srv import ListParameters, GetParameters
import rcl_interfaces.msg

class ROS2DataManager:
    """Handles all ROS2 graph data fetching and subscriptions in a background thread."""

    def __init__(self, app_node):
        """Initializes rclpy, the node, and starts the background spin thread."""
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("textual_ros_monitor_data_node",
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.app = app_node
        self._active_subscriptions = {}

        self.spin_thread = threading.Thread(target=lambda: rclpy.spin(self.node), daemon=True)
        self.spin_thread.start()

    def shutdown(self):
        """Cleans up the ROS2 node."""
        pass # rclpy.shutdown() in the main app will handle stopping the thread and node.

    def _get_message_class(self, topic_type_str: str):
        try:
            parts = topic_type_str.split('/')
            package_name, module_name, class_name = parts[0], parts[1], parts[2]
            module = importlib.import_module(f"{package_name}.{module_name}")
            return getattr(module, class_name)
        except Exception as e:
            self.node.get_logger().error(f"Failed to import message type '{topic_type_str}': {e}")
            return None

    def subscribe_to_topic(self, topic_name: str, topic_type: str, ui_callback):
        if topic_name in self._active_subscriptions:
            return
        msg_class = self._get_message_class(topic_type)
        if msg_class is None:
            return

        def ros_callback(msg):
            self.app.call_from_thread(ui_callback, msg)
        
        # FIX: Use a more compatible QoS profile to connect to more publishers.
        # This profile prioritizes receiving the latest message over guaranteed delivery.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE  # For transient_local publishers
        )
        
        try:
            subscription = self.node.create_subscription(
                msg_class, 
                topic_name, 
                ros_callback, 
                qos_profile  # Use the compatible QoS profile
            )
            self._active_subscriptions[topic_name] = subscription
            self.node.get_logger().info(f"Subscribed to '{topic_name}'")
        except Exception as e:
            self.node.get_logger().error(f"Failed to create subscription to '{topic_name}': {e}")
            
    def unsubscribe_from_topic(self, topic_name: str):
        if topic_name in self._active_subscriptions:
            subscription = self._active_subscriptions.pop(topic_name)
            self.node.destroy_subscription(subscription)
            self.node.get_logger().info(f"Unsubscribed from '{topic_name}'")

    def get_nodes(self):
        """Fetches a list of node names, filtering out internal/CLI nodes."""
        node_list = self.node.get_node_names_and_namespaces()
        # FIX: Add filtering for extraneous ros2cli nodes.
        filtered_list = [
            (name, namespace) for name, namespace in node_list
            if name != "textual_ros_monitor_data_node" and "_ros2cli_" not in name
        ]
        return sorted(filtered_list)

    def get_topics(self):
        return sorted(self.node.get_topic_names_and_types())

    def get_services_and_actions(self):
        service_info = self.node.get_service_names_and_types()
        services, action_names = [], set()
        for name, types in service_info:
            if "/_action/" in name:
                action_names.add(name.split('/_action/')[0])
            else:
                services.append((name, ", ".join(types)))
        actions = [(name, "N/A") for name in sorted(list(action_names))]
        return sorted(services), actions

    def get_node_details(self, node_name: str, node_namespace: str):
        publishers = sorted(self.node.get_publisher_names_and_types_by_node(node_name, node_namespace))
        subscribers = sorted(self.node.get_subscriber_names_and_types_by_node(node_name, node_namespace))
        parameters = [("Info", "Parameter listing is currently disabled.")]
        return publishers, subscribers, parameters