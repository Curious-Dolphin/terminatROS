#!/usr/bin/env python3
from textual.widget import Widget
from textual.widgets import DataTable, Tabs, Tab, ContentSwitcher
from textual.containers import VerticalScroll
from textual.message import Message
from textual.app import ComposeResult

TAB_NAMES = ["Nodes", "Topics", "Services", "Actions", "Controllers"]

class MainView(Widget):
    """The main view with tabs for different ROS2 introspection points."""

    class NodeSelected(Message):
        def __init__(self, name: str, namespace: str) -> None:
            self.name, self.namespace = name, namespace
            super().__init__()
            
    class TopicSelected(Message):
        def __init__(self, name: str, type: str) -> None:
            self.name, self.type = name, type
            super().__init__()

    class TabSwitched(Message):
        def __init__(self, tab_id: str) -> None:
            self.tab_id = tab_id
            super().__init__()

    def compose(self) -> ComposeResult:
        tabs = [Tab(name, id=name.lower()) for name in TAB_NAMES]
        yield Tabs(*tabs)
        
        with ContentSwitcher(initial="nodes"):
            with VerticalScroll(id="nodes"):
                yield DataTable(id="table-nodes")
            with VerticalScroll(id="topics"):
                yield DataTable(id="table-topics")
            with VerticalScroll(id="services"):
                yield DataTable(id="table-services")
            with VerticalScroll(id="actions"):
                yield DataTable(id="table-actions")
            with VerticalScroll(id="controllers"):
                yield DataTable(id="table-controllers")

    def on_mount(self) -> None:
        self._setup_tables()

    def on_tabs_tab_activated(self, event: Tabs.TabActivated) -> None:
        self.query_one(ContentSwitcher).current = event.tab.id
        self.post_message(self.TabSwitched(event.tab.id))
    
    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        """Handle row selection for both nodes and topics tables."""
        table_id = event.control.id
        if event.row_key.value is None:
            return

        # FIX: Split the variable assignment and usage into two lines.
        row_key = event.row_key.value
        row_data = event.control.get_row(row_key)

        if table_id == "table-nodes":
            self.post_message(self.NodeSelected(name=row_data[0], namespace=row_data[1]))
        elif table_id == "table-topics":
            actual_type = row_data[1].split(',')[0].strip()
            self.post_message(self.TopicSelected(name=row_data[0], type=actual_type))

    def get_active_tab(self) -> str:
        active_tab = self.query_one(Tabs).active_tab
        return active_tab.id if active_tab else "nodes"

    def _setup_tables(self) -> None:
        for table_id in ["#table-nodes", "#table-topics", "#table-services", "#table-actions", "#table-controllers"]:
            table = self.query_one(table_id, DataTable)
            table.cursor_type = "row"
        
        self.query_one("#table-nodes", DataTable).add_columns("Node Name", "Namespace")
        self.query_one("#table-topics", DataTable).add_columns("Topic Name", "Type")
        self.query_one("#table-services", DataTable).add_columns("Service Name", "Type")
        self.query_one("#table-actions", DataTable).add_columns("Action Name", "Type")
        self.query_one("#table-controllers", DataTable).add_columns("Controller Name", "State", "Type")

    def _non_destructive_update(self, table: DataTable, new_rows: list, row_generator):
        new_items_map = {item[0]: item for item in new_rows}
        new_keys, current_keys = set(new_items_map.keys()), set()

        if table.row_count > 0:
            try:
                current_keys.update(table.get_row_key(i) for i in range(table.row_count))
            except Exception:
                table.clear()
                for item in new_rows: table.add_row(*row_generator(item), key=item[0])
                return

        keys_to_remove, keys_to_add = current_keys - new_keys, new_keys - current_keys
        for key in keys_to_remove:
            try: table.remove_row(key)
            except Exception: continue
        if keys_to_add:
            for key in sorted(list(keys_to_add)):
                table.add_row(*row_generator(new_items_map[key]), key=key)

    def update_nodes_table(self, nodes: list) -> None:
        self._non_destructive_update(self.query_one("#table-nodes", DataTable), nodes, lambda item: (item[0], item[1]))

    def update_topics_table(self, topics: list) -> None:
        self._non_destructive_update(self.query_one("#table-topics", DataTable), topics, lambda item: (item[0], ", ".join(item[1])))

    def update_services_table(self, services: list) -> None:
        self._non_destructive_update(self.query_one("#table-services", DataTable), services, lambda item: (item[0], item[1]))

    def update_actions_table(self, actions: list) -> None:
        self._non_destructive_update(self.query_one("#table-actions", DataTable), actions, lambda item: (item[0], item[1]))

    def update_controllers_table(self, controllers: list) -> None:
        self._non_destructive_update(self.query_one("#table-controllers", DataTable), controllers, lambda item: (item[0], item[1], item[2]))