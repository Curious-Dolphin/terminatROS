#!/usr/bin/env python3
from textual.screen import Screen
from textual.app import ComposeResult
from textual.widgets import Header, Footer, DataTable, Label
from textual.containers import VerticalScroll, Container

class NodeDetailScreen(Screen):
    """A screen to display detailed information about a single ROS2 node."""

    BINDINGS = [("escape", "app.pop_screen", "Back")]

    def __init__(self, node_name: str, node_namespace: str, ros_manager, **kwargs):
        """Initialize the screen."""
        super().__init__(**kwargs)
        self.node_name = node_name
        self.node_namespace = node_namespace
        self.ros_manager = ros_manager

    def compose(self) -> ComposeResult:
        """Compose the layout of the node detail screen."""
        yield Header()
        yield Container(
            Label(f"Details for: {self.node_namespace}{self.node_name}", id="node-title"),
            
            Label("Publishers", classes="table-title"),
            VerticalScroll(DataTable(id="publishers-table")),
            
            Label("Subscribers", classes="table-title"),
            VerticalScroll(DataTable(id="subscribers-table")),
            
            Label("Parameters", classes="table-title"),
            VerticalScroll(DataTable(id="parameters-table")),
            
            id="details-container"
        )
        yield Footer()

    def on_mount(self) -> None:
        """Set up tables and populate them with node data."""
        self._setup_tables()
        self._update_details()

    def _setup_tables(self) -> None:
        """Add columns to the data tables."""
        pub_table = self.query_one("#publishers-table", DataTable)
        pub_table.add_columns("Topic", "Type")
        
        sub_table = self.query_one("#subscribers-table", DataTable)
        sub_table.add_columns("Topic", "Type")
        
        param_table = self.query_one("#parameters-table", DataTable)
        param_table.add_columns("Name", "Value")

    def _update_details(self) -> None:
        """Fetch details from the ROS manager and update the tables."""
        publishers, subscribers, parameters = self.ros_manager.get_node_details(
            self.node_name, self.node_namespace
        )
        
        pub_table = self.query_one("#publishers-table", DataTable)
        pub_table.clear()
        for topic, types in publishers:
            pub_table.add_row(topic, ", ".join(types))

        sub_table = self.query_one("#subscribers-table", DataTable)
        sub_table.clear()
        for topic, types in subscribers:
            sub_table.add_row(topic, ", ".join(types))

        param_table = self.query_one("#parameters-table", DataTable)
        param_table.clear()
        for name, value in parameters:
            param_table.add_row(name, value)
