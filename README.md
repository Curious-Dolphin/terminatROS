# terminatROS

A powerful, lightweight, and user-friendly Text User Interface (TUI) for monitoring and debugging with your ROS 2 applications directly from the terminal.

## Overview

terminatROS was born out of the need for a fast, resource-efficient, and accessible tool for ROS 2 developers. terminatROS provides a "headless" alternative that gives you a clear overview of your ROS 2 system, allows you to inspect **topics, nodes, services, actions, and parameters**, and even echo topic data in real-time, all without leaving your terminal.

## Features

* **Comprehensive ROS 2 Graph Analysis:**
    * **Topic Discovery:** Automatically discovers and lists all active topics on the ROS 2 graph. For each topic, you can view its name and message type.
    * **Node Discovery:** Automatically discovers and lists all active nodes, providing a clear overview of your running system.
    * **Service and Action Discovery:** Automatically discovers and lists all available services and actions.
* **In-depth Inspection:**
    * **Topic Details:** Select a topic to see a detailed view of its publishers and subscribers, including their node names and QoS profiles.
    * **Node Details:** Select a node to inspect its publications, subscriptions, services, and actions.
    * **Parameter Inspection:** View the parameters of any selected node.
    * **Real-time Topic Echo:** Subscribe to any topic and see its messages streamed in real-time within the TUI. This is invaluable for debugging message contents.
* **Intuitive and Responsive Interface:**
    * **Keyboard-driven:** Designed for efficient, mouse-free operation.
    * **Responsive Layout:** The UI adapts to different terminal sizes.
    * **Asynchronous Operations:** The UI remains responsive while fetching data from the ROS 2 graph in the background.

## Installation

The project includes a convenient installation script that sets up a local Python environment to avoid conflicts with system packages.

```bash
bash install.sh
```

This script performs the following steps:

1.  **Creates a Virtual Environment:** A Python virtual environment is created in the `venv` directory.
2.  **Activates the Environment:** The script activates the newly created environment.
3.  **Installs Dependencies:** It installs the required Python packages (`textual` and `textual-dev`) into the virtual environment.

**Prerequisites:**

* A working installation of ROS 2 (e.g., Jazzy, Humble, Iron).
* Python 3.6+

## How to Use

1.  **Source Your ROS 2 Workspace:** Before running the application, make sure to source your ROS 2 environment. This is a crucial step, as it allows the application to find the necessary ROS 2 packages.

    ```bash
    # Example for ROS 2 Humble
    source /opt/ros/humble/setup.bash
    
    # If you have a custom workspace, source it as well
    # source /path/to/your/ros2_ws/install/setup.bash
    ```

2.  **Run terminatROS:**

    ```bash
    terminatros
    ```

### Navigating the UI

* **Main Screen:** The initial screen shows a list of all discovered ROS 2 topics.
* **Navigation:** Use the `Up` and `Down` arrow keys to navigate the lists.
* **Selection:** Press `Enter` to select an item and view its details.
* **Go Back:** Press `Esc` to return to the previous screen.

## Dependencies

* **ROS 2:** Foxy, Humble, Iron, or later.
* **Python:** 3.6+
* **textual:**
* **textual-dev:**

## Contributing

Contributions are welcome! If you have an idea for a new feature, a bug fix, or an improvement to the documentation, please feel free to open an issue or submit a pull request.

## Roadmap

* [x] **Services and Actions:** List available services and actions.
* [x] **Parameter Management:** View node parameters.
* [ ] **Service/Action Interaction:** Add support for calling ROS 2 services and actions.
* [ ] **Parameter Modification:** Add support for modifying node parameters.
* [ ] **Customizable Layouts:** Allow users to customize the layout of the TUI.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
