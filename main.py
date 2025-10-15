#!/usr/bin/env python3

from app import ROS2MonitorApp

if __name__ == "__main__":
    """Entry point for the ROS2 Monitor TUI."""
    app = ROS2MonitorApp()
    app.run()
