#!/usr/bin/env python3
from collections import deque
from textual.widget import Widget
from textual.widgets import DataTable, Tabs, Tab, ContentSwitcher, Input, Static, ProgressBar, Sparkline
# Import Horizontal and Vertical containers for the new layout
from textual.containers import Container, Horizontal, Vertical
from textual.message import Message
from textual.app import ComposeResult

TAB_NAMES = ["Nodes", "Topics", "Services", "Actions", "Controllers"]

LOGO_ART = """
                                                                                                              
                                            ██                                   ▄▄▄▄▄▄      ▄▄▄▄      ▄▄▄▄   
   ██                                       ▀▀                           ██      ██▀▀▀▀██   ██▀▀██   ▄█▀▀▀▀█  
 ███████    ▄████▄    ██▄████  ████▄██▄   ████     ██▄████▄   ▄█████▄  ███████   ██    ██  ██    ██  ██▄      
   ██      ██▄▄▄▄██   ██▀      ██ ██ ██     ██     ██▀   ██   ▀ ▄▄▄██    ██      ███████   ██    ██   ▀████▄  
   ██      ██▀▀▀▀▀▀   ██       ██ ██ ██     ██     ██    ██  ▄██▀▀▀██    ██      ██  ▀██▄  ██    ██       ▀██ 
   ██▄▄▄   ▀██▄▄▄▄█   ██       ██ ██ ██  ▄▄▄██▄▄▄  ██    ██  ██▄▄▄███    ██▄▄▄   ██    ██   ██▄▄██   █▄▄▄▄▄█▀ 
    ▀▀▀▀     ▀▀▀▀▀    ▀▀       ▀▀ ▀▀ ▀▀  ▀▀▀▀▀▀▀▀  ▀▀    ▀▀   ▀▀▀▀ ▀▀     ▀▀▀▀   ▀▀    ▀▀▀   ▀▀▀▀     ▀▀▀▀▀ 
                                                                                                              
"""

class MainView(Widget):
    class NodeSelected(Message):
        def __init__(self, name: str, namespace: str) -> None: self.name, self.namespace = name, namespace; super().__init__()
    class TopicSelected(Message):
        def __init__(self, name: str, type: str) -> None: self.name, self.type = name, type; super().__init__()
    class TabSwitched(Message):
        def __init__(self, tab_id: str) -> None: self.tab_id = tab_id; super().__init__()

    def compose(self) -> ComposeResult:
        tabs = [Tab(name, id=name.lower()) for name in TAB_NAMES]
        yield Tabs(*tabs)
        with ContentSwitcher(initial="nodes"):
            
            # --- START of Updated Nodes Tab Layout ---
            with Container(id="nodes"):
                # Top section for the logo and name, side-by-side
                with Horizontal(id="logo-container"):
                    yield Static(LOGO_ART, id="logo-art")

                # Main section with nodes list on the left and system stats on the right
                with Horizontal(id="main-split"):
                    # Left column
                    with Vertical(id="nodes-list-container"):
                        yield Input(placeholder="Search nodes...", id="node-filter-input")
                        yield DataTable(id="table-nodes")
                    # Right column
                    with Vertical(id="system-stats-container"):
                        yield Static("CPU Usage", classes="system-title")
                        yield ProgressBar(total=100, show_eta=False, id="cpu-progress")
                        yield Sparkline(id="cpu-sparkline")
                        yield Static("Memory (RAM) Usage", classes="system-title")
                        yield ProgressBar(total=100, show_eta=False, id="ram-progress")
            # --- END of Updated Nodes Tab Layout ---

            with Container(id="topics"):
                yield Input(placeholder="Search topics...", id="topic-filter-input")
                yield DataTable(id="table-topics")
            with Container(id="services"):
                yield Input(placeholder="Search services...", id="service-filter-input")
                yield DataTable(id="table-services")
            with Container(id="actions"):
                yield Input(placeholder="Search actions...", id="action-filter-input")
                yield DataTable(id="table-actions")
            with Container(id="controllers"):
                yield Input(placeholder="Search controllers...", id="controller-filter-input")
                yield DataTable(id="table-controllers")

    def on_mount(self) -> None:
        self._setup_tables()
        self.node_filter_text, self.full_node_list = "", []
        self.topic_filter_text, self.full_topic_list = "", []
        self.service_filter_text, self.full_service_list = "", []
        self.action_filter_text, self.full_action_list = "", []
        self.controller_filter_text, self.full_controller_list = "", []
        self.cpu_history = deque([0.0] * 60, maxlen=60)

    def update_system_display(self, cpu_percent, ram_stats):
        self.query_one("#cpu-progress", ProgressBar).progress = cpu_percent
        self.query_one("#cpu-progress").sub_title = f"{cpu_percent:.1f}%"
        self.cpu_history.append(cpu_percent)
        self.query_one("#cpu-sparkline", Sparkline).data = list(self.cpu_history)
        self.query_one("#ram-progress", ProgressBar).progress = ram_stats.percent
        self.query_one("#ram-progress").sub_title = f"{ram_stats.percent:.1f}% ({ram_stats.used/1e9:.1f}/{ram_stats.total/1e9:.1f} GB)"

    def on_tabs_tab_activated(self, event: Tabs.TabActivated) -> None:
        self.query_one(ContentSwitcher).current = event.tab.id
        self.post_message(self.TabSwitched(event.tab.id))
    
    async def on_input_changed(self, event: Input.Changed) -> None:
        input_id = event.input.id
        if input_id == "node-filter-input": self.node_filter_text = event.value; self.update_nodes_table(self.full_node_list)
        elif input_id == "topic-filter-input": self.topic_filter_text = event.value; self.update_topics_table(self.full_topic_list)
        elif input_id == "service-filter-input": self.service_filter_text = event.value; self.update_services_table(self.full_service_list)
        elif input_id == "controller-filter-input": self.controller_filter_text = event.value; self.update_controllers_table(self.full_controller_list)
        elif input_id == "action-filter-input": self.action_filter_text = event.value; self.update_actions_table(self.full_action_list)

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        if event.row_key.value is None: return
        table_id, row_key = event.control.id, event.row_key.value
        row_data = event.control.get_row(row_key)
        if table_id == "table-nodes": self.post_message(self.NodeSelected(name=row_data[0], namespace=row_data[1]))
        elif table_id == "table-topics":
            actual_type = row_data[1].split(',')[0].strip()
            self.post_message(self.TopicSelected(name=row_data[0], type=actual_type))

    def get_active_tab(self) -> str:
        active_tab = self.query_one(Tabs).active_tab
        return active_tab.id if active_tab else "nodes"

    def _setup_tables(self) -> None:
        for tid in ["#table-nodes", "#table-topics", "#table-services", "#table-actions", "#table-controllers"]:
            self.query_one(tid, DataTable).cursor_type = "row"
        self.query_one("#table-nodes", DataTable).add_columns("Node Name", "Namespace")
        self.query_one("#table-topics", DataTable).add_columns("Topic Name", "Type")
        self.query_one("#table-services", DataTable).add_columns("Service Name", "Type")
        self.query_one("#table-actions", DataTable).add_columns("Action Name", "Type")
        self.query_one("#table-controllers", DataTable).add_columns("Controller Name", "State", "Type")

    def _non_destructive_update(self, table: DataTable, new_rows: list, row_generator, key_generator=lambda item: item[0]):
        new_items_map = {key_generator(item): item for item in new_rows}
        new_keys, current_keys = set(new_items_map.keys()), set()
        if table.row_count > 0:
            try: current_keys.update(table.get_row_key(i) for i in range(table.row_count))
            except Exception:
                table.clear()
                for key, item in new_items_map.items(): table.add_row(*row_generator(item), key=key)
                return
        keys_to_remove, keys_to_add = current_keys - new_keys, new_keys - current_keys
        for key in keys_to_remove:
            try: table.remove_row(key)
            except Exception: continue
        if keys_to_add:
            for key in sorted(list(keys_to_add)): table.add_row(*row_generator(new_items_map[key]), key=key)

    def update_nodes_table(self, nodes: list) -> None:
        self.full_node_list = nodes
        search_text = self.node_filter_text.lower()
        filtered_list = [n for n in nodes if search_text in n[0].lower()] if search_text else nodes
        def node_key_generator(item): return f"{item[1].rstrip('/')}/{item[0]}"
        self._non_destructive_update(self.query_one("#table-nodes", DataTable), filtered_list, lambda item: (item[0], item[1]), node_key_generator)

    def update_topics_table(self, topics: list) -> None:
        self.full_topic_list = topics
        search_text = self.topic_filter_text.lower()
        filtered_list = [t for t in topics if search_text in t[0].lower()] if search_text else topics
        self._non_destructive_update(self.query_one("#table-topics", DataTable), filtered_list, lambda item: (item[0], ", ".join(item[1])))

    def update_services_table(self, services: list) -> None:
        self.full_service_list = services
        search_text = self.service_filter_text.lower()
        filtered_list = [s for s in services if search_text in s[0].lower()] if search_text else services
        self._non_destructive_update(self.query_one("#table-services", DataTable), filtered_list, lambda item: (item[0], item[1]))

    def update_actions_table(self, actions: list) -> None:
        self.full_action_list = actions
        search_text = self.action_filter_text.lower()
        filtered_list = [a for a in actions if search_text in a[0].lower()] if search_text else actions
        self._non_destructive_update(self.query_one("#table-actions", DataTable), filtered_list, lambda item: (item[0], item[1]))

    def update_controllers_table(self, controllers: list) -> None:
        self.full_controller_list = controllers
        search_text = self.controller_filter_text.lower()
        filtered_list = [c for c in controllers if search_text in c[0].lower()] if search_text else controllers
        self._non_destructive_update(self.query_one("#table-controllers", DataTable), filtered_list, lambda item: (item[0], item[1], item[2]))