#!/usr/bin/env python3
import rclpy
from app import ROS2MonitorApp

def main():
    """Initializes ROS 2, runs the app, and handles shutdown."""
    # FIX: Initialize rclpy at the highest level, before the app is created.
    try:
        rclpy.init()
        app = ROS2MonitorApp()
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        # FIX: Ensure rclpy is shut down cleanly when the app exits.
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()