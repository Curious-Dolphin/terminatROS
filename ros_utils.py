#!/usr/-bin/env python3
import rclpy
import importlib
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import ListParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterType

class ROS2DataManager:
    """Handles all ROS2 graph data fetching and subscriptions in a background thread."""

    def __init__(self, app_node):
        if not rclpy.ok(): rclpy.init()
        self.node = Node("textual_ros_monitor_data_node", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.app = app_node
        self._active_subscriptions = {}
        self._controller_cache = []
        self._is_fetching_controllers = False

        self.spin_thread = threading.Thread(target=lambda: rclpy.spin(self.node), daemon=True)
        self.spin_thread.start()

    def shutdown(self):
        pass 

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
        """Creates a subscription with a QoS profile that matches the publisher."""
        if topic_name in self._active_subscriptions: return
        msg_class = self._get_message_class(topic_type)
        if msg_class is None: return
        
        def ros_callback(msg): self.app.call_from_thread(ui_callback, msg)
        
        try:
            publishers_info = self.node.get_publishers_info_by_topic(topic_name)
            if not publishers_info:
                # FIX: If no publisher is found yet, fall back to a compatible SENSOR profile, not a generic RELIABLE one.
                self.node.get_logger().warn(f"No publisher found for '{topic_name}'. Falling back to a compatible 'sensor data' QoS.")
                qos_profile = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=5
                )
            else:
                qos_profile = publishers_info[0].qos_profile
                self.node.get_logger().info(f"Matching QoS profile for '{topic_name}'")
        except Exception as e:
            self.node.get_logger().error(f"Could not determine QoS for '{topic_name}': {e}. Falling back to default.")
            qos_profile = QoSProfile(depth=10)

        try:
            subscription = self.node.create_subscription(msg_class, topic_name, ros_callback, qos_profile)
            self._active_subscriptions[topic_name] = subscription
        except Exception as e:
            self.node.get_logger().error(f"Failed to create subscription to '{topic_name}': {e}")
            
    def unsubscribe_from_topic(self, topic_name: str):
        if topic_name in self._active_subscriptions:
            self.node.destroy_subscription(self._active_subscriptions.pop(topic_name))

    def get_nodes(self):
        filtered_list = [(name, namespace) for name, namespace in self.node.get_node_names_and_namespaces() if name != "textual_ros_monitor_data_node" and "_ros2cli_" not in name]
        return sorted(filtered_list)

    def get_topics(self):
        return sorted(self.node.get_topic_names_and_types())

    def get_services_and_actions(self):
        service_info = self.node.get_service_names_and_types()
        services, action_names = [], set()
        for name, types in service_info:
            if "/_action/" in name: action_names.add(name.split('/_action/')[0])
            else: services.append((name, ", ".join(types)))
        actions = [(name, "N/A") for name in sorted(list(action_names))]
        return sorted(services), actions

    def get_node_details(self, node_name: str, node_namespace: str, param_ui_callback):
        publishers = sorted(self.node.get_publisher_names_and_types_by_node(node_name, node_namespace))
        subscribers = sorted(self.node.get_subscriber_names_and_types_by_node(node_name, node_namespace))
        self._async_fetch_parameters(node_name, node_namespace, param_ui_callback)
        return publishers, subscribers, [("Status", "Loading...")]

    def _async_fetch_parameters(self, node_name: str, node_namespace: str, ui_callback):
        base_path = f"{node_namespace.rstrip('/')}/{node_name}"
        list_client = self.node.create_client(ListParameters, f"{base_path}/list_parameters")
        if not list_client.wait_for_service(timeout_sec=0.5):
            self.app.call_from_thread(ui_callback, [("ERROR", "Parameter services not available")])
            return
        future = list_client.call_async(ListParameters.Request())
        future.add_done_callback(lambda f: self._on_list_parameters_done(f, list_client, base_path, ui_callback))

    def _on_list_parameters_done(self, future, list_client, base_path, ui_callback):
        try:
            param_names = future.result().result.names
            if not param_names:
                self.app.call_from_thread(ui_callback, [("Info", "No parameters found")]); return
            get_client = self.node.create_client(GetParameters, f"{base_path}/get_parameters")
            get_future = get_client.call_async(GetParameters.Request(names=param_names))
            get_future.add_done_callback(lambda f: self._on_get_parameters_done(f, get_client, param_names, ui_callback))
        except Exception as e:
            self.app.call_from_thread(ui_callback, [("ERROR", f"Failed to list parameters: {e}")])
        finally:
            self.node.destroy_client(list_client)

    def _on_get_parameters_done(self, future, get_client, param_names, ui_callback):
        try:
            response = future.result()
            params_data = []
            for name, value_msg in zip(param_names, response.values):
                param_type, value = value_msg.type, None
                if param_type == ParameterType.PARAMETER_DOUBLE: value = value_msg.double_value
                elif param_type == ParameterType.PARAMETER_INTEGER: value = value_msg.integer_value
                elif param_type == ParameterType.PARAMETER_STRING: value = value_msg.string_value
                elif param_type == ParameterType.PARAMETER_BOOL: value = value_msg.bool_value
                elif param_type in [ParameterType.PARAMETER_BYTE_ARRAY, ParameterType.PARAMETER_DOUBLE_ARRAY, ParameterType.PARAMETER_INTEGER_ARRAY, ParameterType.PARAMETER_STRING_ARRAY]:
                    value = str(getattr(value_msg, f"{value_msg.get_type_name()}_value"))
                else: value = "Not set"
                params_data.append((name, str(value)))
            self.app.call_from_thread(ui_callback, sorted(params_data))
        except Exception as e:
            self.app.call_from_thread(ui_callback, [("ERROR", f"Failed to get parameters: {e}")])
        finally:
            self.node.destroy_client(get_client)

    def get_controllers(self):
        if not self._is_fetching_controllers: self._async_fetch_controllers()
        return self._controller_cache

    def _async_fetch_controllers(self):
        self._is_fetching_controllers = True
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        future = client.call_async(ListControllers.Request())
        future.add_done_callback(lambda f: self._on_fetch_controllers_done(f, client))

    def _on_fetch_controllers_done(self, future, client):
        try:
            self._controller_cache = sorted([(c.name, c.state, c.type) for c in future.result().controller])
        except Exception as e:
            self._controller_cache = [("N/A", "ERROR", "Service not available")]
        self.app.call_from_thread(self.app.main_view.update_controllers_table, self._controller_cache)
        self.node.destroy_client(client)
        self._is_fetching_controllers = False