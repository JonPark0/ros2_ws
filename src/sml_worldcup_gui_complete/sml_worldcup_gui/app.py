#!/usr/bin/env python3

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime
import json
import math
import random
import time
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Callable, Dict, Iterable, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log, ParameterEvent
from rclpy.parameter import parameter_value_to_python
from sml_messages.msg import Order, Station, Task
from std_msgs.msg import String


PACKAGE_NAME = "sml_worldcup_gui"
DEFAULT_TOPIC = "/eai/task"
PLANNER_STATE_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)
WORKSPACE_LAYOUT_PATH = Path(
    "/home/st02/ros2_ws/src/sml_worldcup_gui_complete/config/sml_worldcup_2026_layout.json"
)
SOURCE_CONFIG_DIRECTORY = Path(__file__).resolve().parents[1] / "config"

SIDE_ALIASES = {
    "a": "side_a",
    "side_a": "side_a",
    "b": "side_b",
    "side_b": "side_b",
    "all": "all",
    "both": "all",
}

# Fallback crops expressed in the coordinate system of the World Cup layout JSON.
# The actual viewport is now computed from visible zones when possible, so the
# opposite side is not drawn in side-specific mode.
SIDE_VIEWPORTS = {
    "side_a": (120.0, 45.0, 175.0, 235.0),
    "side_b": (255.0, 45.0, 180.0, 235.0),
}

BACKGROUND = "#111827"
PANEL = "#182235"
PANEL_ALT = "#202c40"
TEXT = "#e5edf8"
MUTED = "#94a3b8"
ACCENT = "#38bdf8"
SUCCESS = "#34d399"
WARNING = "#fbbf24"
ERROR = "#f87171"
RETURN_REQUIRED_COLOR = "#a855f7"

STATION_COLORS = {
    "customer_counter": "#f59e0b",
    "workbench": "#a855f7",
    "hybrid_work_shelf": "#c084fc",
    "storage_shelf": "#22c55e",
    "shared_storage": "#10b981",
}

ZONE_COLORS = {
    "side_a": "#172554",
    "side_b": "#3b172b",
    "center": "#1f2937",
    "customers": "#3b2a15",
    "robot_fleets": "#163047",
    "wait_a": "#1d3557",
    "wait_b": "#4a1d32",
    "warehouse": "#193425",
    "workbench_area_b": "#332047",
}

OBJECT_NAMES = {
    1: "2x2 red",
    2: "2x2 green",
    3: "2x2 blue",
    4: "2x2 yellow",
    5: "4x2 red",
    6: "4x2 green",
    7: "4x2 blue",
    8: "4x2 yellow",
    10: "2x2 red batch",
    20: "2x2 green batch",
    30: "2x2 blue batch",
    40: "2x2 yellow batch",
    50: "4x2 red batch",
    60: "4x2 green batch",
    70: "4x2 blue batch",
    80: "4x2 yellow batch",
    90: "mixed batch",
    13: "Magnet",
    34: "Battery",
    81: "E-Stop",
    241: "Traffic Light",
    442: "Carrot",
    462: "Small Tree",
    711: "Hammer",
    4482: "Big Carrot",
    8518: "Burger",
    48132: "Ice Cream",
    46262: "Big Tree",
}


RAW_MATERIAL_SIZES = {
    1: (2, 2),
    2: (2, 2),
    3: (2, 2),
    4: (2, 2),
    5: (4, 2),
    6: (4, 2),
    7: (4, 2),
    8: (4, 2),
}

RAW_MATERIAL_COLORS = {
    1: "#d00000",
    2: "#548235",
    3: "#0070c0",
    4: "#ffc000",
    5: "#d00000",
    6: "#548235",
    7: "#0070c0",
    8: "#ffc000",
}

RAW_MATERIAL_TEXT_COLORS = {
    1: "#ffffff",
    2: "#ffffff",
    3: "#ffffff",
    4: "#111827",
    5: "#ffffff",
    6: "#ffffff",
    7: "#ffffff",
    8: "#111827",
}

BATCH_IDS = {10, 20, 30, 40, 50, 60, 70, 80, 90}
STATION_MATERIAL_IDS = set(range(1, 9)).union(BATCH_IDS)
PRODUCT_IDS = {13, 34, 81, 241, 442, 462, 711, 4482, 8518, 48132, 46262}
PRODUCT_MATERIALS = {
    81: [8, 1],
    34: [3, 4],
    13: [1, 3],
    442: [4, 4, 2],
    241: [2, 4, 1],
    462: [4, 6, 2],
    4482: [4, 4, 8, 2],
    711: [1, 1, 7],
    8518: [8, 5, 1, 8],
    46262: [4, 6, 2, 6, 2],
    48132: [4, 8, 1, 3, 2],
}

AUTO_TIER_PRODUCTS = {
    "entry": [34, 13, 81, 442, 241, 462, 711],
    "beginner": [34, 13, 81, 442, 241, 462, 711],
    "advanced": sorted(PRODUCT_IDS),
    "expert": sorted(PRODUCT_IDS),
}

AUTO_ORDER_COUNTS = {
    ("entry", "production"): (1, 0),
    ("entry", "recycling"): (0, 1),
    ("entry", "lifecycle"): (2, 1),
    ("beginner", "production"): (2, 0),
    ("beginner", "recycling"): (0, 2),
    ("beginner", "lifecycle"): (3, 2),
    ("advanced", "production"): (5, 0),
    ("advanced", "recycling"): (0, 5),
    ("advanced", "lifecycle"): (5, 5),
    ("expert", "production"): (20, 0),
    ("expert", "recycling"): (0, 20),
    ("expert", "lifecycle"): (30, 20),
}

SIDE_MONITOR_STATIONS = {
    "side_a": [1, 2, 3, 4, 5, 6, 7],
    "side_b": [7, 8, 9, 10, 11, 12, 13],
}


@dataclass(frozen=True)
class OrderState:
    name: str
    order_type: int
    product_id: int

    @property
    def type_name(self) -> str:
        if self.order_type == Order.OT_PRODUCE:
            return "PRODUCE"
        if self.order_type == Order.OT_RECYCLE:
            return "RECYCLE"
        return f"UNKNOWN({self.order_type})"


@dataclass(frozen=True)
class StationState:
    station_id: int
    name: str
    station_type: int
    material_ids: Tuple[int, ...]


@dataclass
class TaskState:
    orders: List[OrderState] = field(default_factory=list)
    stations: Dict[int, StationState] = field(default_factory=dict)
    received_at: Optional[datetime] = None
    message_count: int = 0
    source: str = "UNKNOWN"

    def update(self, msg: Task) -> None:
        self.orders = [
            OrderState(
                name=order.name,
                order_type=order.order_type,
                product_id=order.product_id,
            )
            for order in msg.order_list
        ]
        if self.orders and all(
            order.name.startswith("manual_") for order in self.orders
        ):
            self.source = "MANUAL"
        elif self.orders and all(
            order.name.startswith("semi_") for order in self.orders
        ):
            self.source = "SEMI-AUTO"
        else:
            self.source = "AUTOMATIC"
        self.stations = {
            station.station_id: StationState(
                station_id=station.station_id,
                name=station.name,
                station_type=station.station_type,
                material_ids=tuple(station.material_ids),
            )
            for station in msg.arena_layout
        }
        self.received_at = datetime.now()
        self.message_count += 1


@dataclass(frozen=True)
class CargoRawItem:
    raw_id: int
    slide_id: int
    position: int
    return_required: bool = False
    return_station: Optional[int] = None


@dataclass
class CargoState:
    """Local visualization state for AMR cargo.

    The current /eai/task message contains station layout and orders only.
    Therefore cargo is initialized as empty.  When a cargo/status topic is added,
    update this dataclass and call redraw_storage_monitor().
    """

    finished_product: Optional[int] = None
    raw_slides: Dict[int, List[CargoRawItem]] = field(
        default_factory=lambda: {1: [], 2: [], 3: [], 4: [], 5: []}
    )
    assembly_spaces: Dict[int, Optional[int]] = field(
        default_factory=lambda: {1: None, 2: None}
    )


class LayoutModel:
    def __init__(self, path: Path) -> None:
        self.path = path
        with path.open(encoding="utf-8") as stream:
            self.data = json.load(stream)
        self._validate()

        map_size = self.data["coordinate_system"]["map_size_px"]
        self.width = float(map_size["width"])
        self.height = float(map_size["height"])
        self.stations_by_id = {
            int(station["station_id"]): station
            for station in self.data["stations"]
        }

    def _validate(self) -> None:
        required = {
            "coordinate_system",
            "zones",
            "stations",
            "start_areas",
            "walls",
        }
        missing = sorted(required.difference(self.data))
        if missing:
            raise ValueError(f"Layout JSON missing keys: {', '.join(missing)}")

        station_ids = [int(station["station_id"]) for station in self.data["stations"]]
        if len(station_ids) != len(set(station_ids)):
            raise ValueError("Layout JSON contains duplicate station_id values")


class ObjectAssetModel:
    """Scalable raw-material and assembled-product drawings loaded from JSON."""

    def __init__(self, path: Path) -> None:
        self.path = path
        with path.open(encoding="utf-8") as stream:
            data = json.load(stream)

        self.raw_materials = {
            int(raw_id): material
            for raw_id, material in data.get("raw_materials", {}).items()
        }
        self.products = {
            int(product_id): product
            for product_id, product in data.get("products", {}).items()
        }


class TaskListenerNode(Node):
    def __init__(self) -> None:
        super().__init__("sml_worldcup_gui")
        self.declare_parameter("topic_name", DEFAULT_TOPIC)
        self.declare_parameter("layout_file", "")
        self.declare_parameter("object_asset_file", "")
        self.declare_parameter("refresh_ms", 50)
        self.declare_parameter("side", "all")

        # Initial GUI layout parameters.
        # These change only the startup size; the user can still resize/drag panes.
        # Default panel ratio: station : arena : cargo : info = 4.5 : 8 : 4 : 4.5
        # Pixel scale uses 80 px per ratio unit -> 360 : 640 : 320 : 360.
        self.declare_parameter("window_geometry", "1850x980")
        self.declare_parameter("window_min_width", 1700)
        self.declare_parameter("window_min_height", 820)
        self.declare_parameter("main_panel_min_width", 1320)
        self.declare_parameter("info_panel_width", 360)
        self.declare_parameter("info_panel_min_width", 360)
        self.declare_parameter("station_panel_min_width", 360)
        self.declare_parameter("arena_panel_min_width", 640)
        self.declare_parameter("cargo_panel_min_width", 320)

        # Optional monitor topics.
        # Planner plan / integrated execution state are std_msgs/String(JSON).
        # Location is nav_msgs/Odometry because the navigation side will provide odometry.
        self.declare_parameter("planner_state_topic", "/sml/test/planner_plan")
        self.declare_parameter("manager_status_topic", "/sml/test/manager_status")
        self.declare_parameter("location_topic", "/sml/test/odom")
        self.declare_parameter("parameter_events_topic", "/parameter_events")
        self.declare_parameter("rosout_topic", "/rosout")
        self.declare_parameter("enable_planner_monitor", True)
        self.declare_parameter("enable_manager_monitor", True)
        self.declare_parameter("enable_location_monitor", True)
        self.declare_parameter("enable_parameter_monitor", True)
        self.declare_parameter("enable_rosout_monitor", True)
        self.declare_parameter("manual_order_mode", False)
        self.declare_parameter("task_template_topic", "/eai/task_template")
        self.declare_parameter("auto_tier", "beginner")

        # How to convert Odometry pose.position into the layout coordinate system.
        # Test odometry publishes layout pixel coordinates by default.
        # For real odometry in meters, run with:
        #   -p odom_coordinate_mode:=meters
        #   -p odom_px_per_meter:=<calibrated scale>
        #   -p odom_origin_x_px:=<layout origin x>
        #   -p odom_origin_y_px:=<layout origin y>
        self.declare_parameter("odom_coordinate_mode", "layout_px")
        self.declare_parameter("odom_px_per_meter", 100.0)
        self.declare_parameter("odom_origin_x_px", 0.0)
        self.declare_parameter("odom_origin_y_px", 0.0)
        self.declare_parameter("odom_invert_y", False)

        self._callback: Optional[Callable[[Task], None]] = None
        self._planner_callback: Optional[Callable[[str], None]] = None
        self._manager_callback: Optional[Callable[[str], None]] = None
        self._location_callback: Optional[Callable[[str], None]] = None
        self._parameter_callback: Optional[Callable[[str], None]] = None
        self._rosout_callback: Optional[Callable[[str], None]] = None
        self._template_callback: Optional[Callable[[Task], None]] = None
        self._connection_callback: Optional[
            Callable[[Dict[str, Optional[int]]], None]
        ] = None
        self._connection_topics: Dict[str, Optional[str]] = {}

        topic = self.get_parameter("topic_name").value
        self._connection_topics["TASK"] = str(topic)
        self._subscription = self.create_subscription(
            Task,
            topic,
            self._on_task,
            10,
        )
        self.get_logger().info(f"GUI listening for tasks on {topic}")

        self._task_publisher = None
        self._template_subscription = None
        if bool(self.get_parameter("manual_order_mode").value):
            template_topic = str(
                self.get_parameter("task_template_topic").value
            )
            self._connection_topics["TEMPLATE"] = template_topic
            self._template_subscription = self.create_subscription(
                Task,
                template_topic,
                self._on_task_template,
                PLANNER_STATE_QOS,
            )
            self._task_publisher = self.create_publisher(
                Task,
                topic,
                PLANNER_STATE_QOS,
            )
            self.get_logger().info(
                f"Manual order mode: template={template_topic}, publish={topic}"
            )
        else:
            self._connection_topics["TEMPLATE"] = None

        self._planner_subscription = None
        self._manager_subscription = None
        self._location_subscription = None
        self._parameter_subscription = None
        self._rosout_subscription = None

        if bool(self.get_parameter("enable_planner_monitor").value):
            planner_topic = str(self.get_parameter("planner_state_topic").value)
            self._connection_topics["PLANNER"] = planner_topic
            self._planner_subscription = self.create_subscription(
                String,
                planner_topic,
                self._on_planner_state,
                PLANNER_STATE_QOS,
            )
            self.get_logger().info(f"GUI listening for planner state on {planner_topic}")
        else:
            self._connection_topics["PLANNER"] = None

        if bool(self.get_parameter("enable_manager_monitor").value):
            manager_topic = str(self.get_parameter("manager_status_topic").value)
            self._connection_topics["EXECUTION"] = manager_topic
            self._manager_subscription = self.create_subscription(
                String,
                manager_topic,
                self._on_manager_status,
                10,
            )
            self.get_logger().info(
                f"GUI listening for execution status on {manager_topic}"
            )
        else:
            self._connection_topics["EXECUTION"] = None

        if bool(self.get_parameter("enable_location_monitor").value):
            location_topic = str(self.get_parameter("location_topic").value)
            self._connection_topics["ODOM"] = location_topic
            self._location_subscription = self.create_subscription(
                Odometry,
                location_topic,
                self._on_location_odom,
                10,
            )
            self.get_logger().info(f"GUI listening for current odometry on {location_topic}")
        else:
            self._connection_topics["ODOM"] = None

        if bool(self.get_parameter("enable_parameter_monitor").value):
            parameter_topic = str(self.get_parameter("parameter_events_topic").value)
            self._connection_topics["PARAM"] = parameter_topic
            self._parameter_subscription = self.create_subscription(
                ParameterEvent,
                parameter_topic,
                self._on_parameter_event,
                10,
            )
            self.get_logger().info(f"GUI listening for parameter events on {parameter_topic}")
        else:
            self._connection_topics["PARAM"] = None

        if bool(self.get_parameter("enable_rosout_monitor").value):
            rosout_topic = str(self.get_parameter("rosout_topic").value)
            self._connection_topics["ROSOUT"] = rosout_topic
            self._rosout_subscription = self.create_subscription(
                Log,
                rosout_topic,
                self._on_rosout,
                50,
            )
            self.get_logger().info(f"GUI listening for ROS logs on {rosout_topic}")
        else:
            self._connection_topics["ROSOUT"] = None

        self._connection_timer = self.create_timer(
            0.5,
            self._emit_connection_status,
        )

    def set_task_callback(self, callback: Callable[[Task], None]) -> None:
        self._callback = callback

    def set_template_callback(
        self, callback: Callable[[Task], None]
    ) -> None:
        self._template_callback = callback

    def publish_manual_task(self, task: Task) -> None:
        if self._task_publisher is None:
            raise RuntimeError("Manual order publisher is disabled")
        self._task_publisher.publish(task)
        self.get_logger().info(
            f"GUI published manual task with {len(task.order_list)} orders"
        )

    def set_monitor_callbacks(
        self,
        planner_callback: Callable[[str], None],
        manager_callback: Callable[[str], None],
        location_callback: Callable[[str], None],
    ) -> None:
        self._planner_callback = planner_callback
        self._manager_callback = manager_callback
        self._location_callback = location_callback

    def set_system_callbacks(
        self,
        parameter_callback: Callable[[str], None],
        rosout_callback: Callable[[str], None],
    ) -> None:
        self._parameter_callback = parameter_callback
        self._rosout_callback = rosout_callback

    def set_connection_callback(
        self,
        callback: Callable[[Dict[str, Optional[int]]], None],
    ) -> None:
        self._connection_callback = callback
        self._emit_connection_status()

    def _emit_connection_status(self) -> None:
        if self._connection_callback is None:
            return
        status: Dict[str, Optional[int]] = {}
        for source, topic in self._connection_topics.items():
            status[source] = None if topic is None else self.count_publishers(topic)
        self._connection_callback(status)

    def _on_task(self, msg: Task) -> None:
        if self._callback is not None:
            self._callback(msg)

    def _on_task_template(self, msg: Task) -> None:
        if self._template_callback is not None:
            self._template_callback(msg)

    def _on_planner_state(self, msg: String) -> None:
        if self._planner_callback is not None:
            self._planner_callback(msg.data)

    def _on_manager_status(self, msg: String) -> None:
        if self._manager_callback is not None:
            self._manager_callback(msg.data)

    def _on_location_odom(self, msg: Odometry) -> None:
        if self._location_callback is not None:
            payload = self._odom_to_payload(msg)
            self._location_callback(json.dumps(payload, ensure_ascii=False))

    def _on_parameter_event(self, msg: ParameterEvent) -> None:
        if self._parameter_callback is None:
            return
        changes: List[str] = []
        for action, parameters in (
            ("new", msg.new_parameters),
            ("changed", msg.changed_parameters),
            ("deleted", msg.deleted_parameters),
        ):
            for parameter in parameters:
                if action == "deleted":
                    changes.append(f"{action}:{parameter.name}")
                    continue
                try:
                    value = parameter_value_to_python(parameter.value)
                except (TypeError, ValueError):
                    value = "<unsupported>"
                value_text = repr(value)
                if len(value_text) > 120:
                    value_text = value_text[:117] + "..."
                changes.append(f"{action}:{parameter.name}={value_text}")
        if changes:
            node_name = msg.node or "<unknown node>"
            self._parameter_callback(f"{node_name} | " + ", ".join(changes))

    def _on_rosout(self, msg: Log) -> None:
        if self._rosout_callback is None:
            return
        level_name = {
            10: "DEBUG",
            20: "INFO",
            30: "WARN",
            40: "ERROR",
            50: "FATAL",
        }.get(msg.level, str(msg.level))
        location = f" ({msg.file}:{msg.line})" if msg.file else ""
        self._rosout_callback(f"{level_name} {msg.name}: {msg.msg}{location}")

    def _odom_to_payload(self, msg: Odometry) -> dict:
        raw_x = float(msg.pose.pose.position.x)
        raw_y = float(msg.pose.pose.position.y)

        mode = str(self.get_parameter("odom_coordinate_mode").value).strip().lower()
        if mode in {"meter", "meters", "m"}:
            scale = float(self.get_parameter("odom_px_per_meter").value)
            origin_x = float(self.get_parameter("odom_origin_x_px").value)
            origin_y = float(self.get_parameter("odom_origin_y_px").value)
            invert_y = bool(self.get_parameter("odom_invert_y").value)
            layout_x = origin_x + raw_x * scale
            layout_y = origin_y - raw_y * scale if invert_y else origin_y + raw_y * scale
        else:
            layout_x = raw_x
            layout_y = raw_y

        quat = msg.pose.pose.orientation
        heading_deg = self._yaw_deg_from_quaternion(quat.x, quat.y, quat.z, quat.w)
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        speed = math.hypot(vx, vy)
        state = "MOVING" if speed > 0.02 else "IDLE"

        stamp = msg.header.stamp
        return {
            "source": "odometry",
            "robot_id": msg.child_frame_id or "AMR",
            "frame_id": msg.header.frame_id or "",
            "stamp_sec": int(stamp.sec),
            "stamp_nanosec": int(stamp.nanosec),
            "x": round(layout_x, 3),
            "y": round(layout_y, 3),
            "odom_x": round(raw_x, 3),
            "odom_y": round(raw_y, 3),
            "odom_z": round(float(msg.pose.pose.position.z), 3),
            "heading_deg": round(heading_deg, 2),
            "speed": round(speed, 3),
            "state": state,
        }

    @staticmethod
    def _yaw_deg_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))


@dataclass(frozen=True)
class WindowLayoutConfig:
    """Initial window and panel sizing.

    These values only control the starting layout. Users can still resize the
    window and drag PanedWindow sashes at runtime.
    """

    geometry: str = "1850x980"
    min_width: int = 1700
    min_height: int = 820
    main_panel_min_width: int = 1320
    info_panel_width: int = 360
    info_panel_min_width: int = 360
    station_panel_min_width: int = 360
    arena_panel_min_width: int = 640
    cargo_panel_min_width: int = 320


class WorldCupGui:
    def __init__(
        self,
        root: tk.Tk,
        layout: LayoutModel,
        object_assets: ObjectAssetModel,
        topic_name: str,
        selected_side: str,
        layout_config: WindowLayoutConfig,
        manual_order_mode: bool = False,
        auto_tier: str = "beginner",
    ) -> None:
        self.root = root
        self.layout = layout
        self.object_assets = object_assets
        self.topic_name = topic_name
        self.selected_side = selected_side
        self.layout_config = layout_config
        self.manual_order_mode = bool(manual_order_mode)
        self.auto_tier = str(auto_tier).strip().lower()
        if self.auto_tier not in AUTO_TIER_PRODUCTS:
            self.auto_tier = "beginner"
        self.task = TaskState()
        self.task_template: Optional[Task] = None
        self._manual_task_publisher: Optional[Callable[[Task], None]] = None
        self.order_dialog: Optional[tk.Toplevel] = None
        self.order_publish_button: Optional[tk.Button] = None
        self.order_mode_var = tk.StringVar(
            master=self.root, value="manual"
        )
        self.auto_stage_var = tk.StringVar(
            master=self.root, value="lifecycle"
        )
        self.order_mode_label: Optional[tk.Label] = None
        self.manual_station_frame: Optional[tk.LabelFrame] = None
        self.manual_station_entries: Dict[int, tk.Entry] = {}
        self._manual_station_widgets: List[tk.Entry] = []
        self.cargo = CargoState()
        self.cargo_state_received = False
        self.planner_station_materials: Dict[int, Tuple[int, ...]] = {}
        self.expected_path: List[int] = []
        self.active_segment: Optional[int] = None
        self.solid_until_segment: int = -1
        self.hide_completed_route_var = tk.BooleanVar(
            master=self.root,
            value=False,
        )
        self.execution_active = False
        self.estimated_duration_sec: Optional[float] = None
        self.estimated_duration_min_sec: Optional[float] = None
        self.estimated_duration_max_sec: Optional[float] = None
        self.execution_started_monotonic: Optional[float] = None
        self.execution_elapsed_sec = 0.0
        self.execution_outcome = "WAITING"
        self.robot_pose: Optional[dict] = None
        self._last_location_log_bucket: Optional[Tuple[str, int, int]] = None
        self.selected_station_id: Optional[int] = None
        self._transform = (1.0, 0.0, 0.0)
        self.text_info_window: Optional[tk.Toplevel] = None
        self.text_info_text: Optional[tk.Text] = None
        self.text_info_summary: Optional[tk.Label] = None
        self.text_info_toggle_var = tk.BooleanVar(master=self.root, value=False)
        self.text_info_alpha_var = tk.DoubleVar(master=self.root, value=78.0)
        self._text_info_history: List[Tuple[str, str]] = []
        self.monitor_source_var = tk.StringVar(master=self.root, value="ALL")
        self._monitor_history: List[Tuple[str, str, str]] = []
        self.visible_station_ids = {
            station_id
            for station_id, station in self.layout.stations_by_id.items()
            if self._is_visible_side(station.get("side", "shared"))
        }

        self._configure_window()
        self._configure_styles()
        self._build_widgets()
        self._refresh_timing_display()
        self.root.after(200, self._tick_execution_clock)
        self.root.after_idle(self.redraw_map)
        self.root.after_idle(self.redraw_storage_monitor)

    def _configure_window(self) -> None:
        self.root.title("SML World Cup 2026 — Match Visualizer")
        self.root.geometry(self.layout_config.geometry)
        self.root.minsize(self.layout_config.min_width, self.layout_config.min_height)
        self.root.configure(bg=BACKGROUND)

    def _configure_styles(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(
            "Treeview",
            background=PANEL_ALT,
            fieldbackground=PANEL_ALT,
            foreground=TEXT,
            rowheight=28,
            borderwidth=0,
        )
        style.configure(
            "Treeview.Heading",
            background="#273449",
            foreground=TEXT,
            relief="flat",
            font=("TkDefaultFont", 10, "bold"),
        )
        style.map("Treeview", background=[("selected", "#075985")])
        style.configure("TSeparator", background="#334155")
        style.configure("TNotebook", background=PANEL, borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            background="#273449",
            foreground=TEXT,
            padding=(12, 6),
            font=("TkDefaultFont", 9, "bold"),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", PANEL_ALT)],
            foreground=[("selected", TEXT)],
        )

    def _build_widgets(self) -> None:
        header = tk.Frame(self.root, bg=BACKGROUND, padx=18, pady=12)
        header.pack(fill=tk.X)

        tk.Checkbutton(
            header,
            text="TEXT INFO",
            variable=self.text_info_toggle_var,
            command=self.toggle_text_info_window,
            indicatoron=False,
            bg=PANEL_ALT,
            fg=ACCENT,
            activebackground="#075985",
            activeforeground="#ffffff",
            selectcolor="#075985",
            relief=tk.FLAT,
            padx=10,
            pady=5,
            font=("TkDefaultFont", 9, "bold"),
            cursor="hand2",
        ).pack(side=tk.LEFT, padx=(0, 12))

        if self.manual_order_mode:
            tk.Button(
                header,
                text="ENTER ORDERS",
                command=self.open_order_dialog,
                bg="#166534",
                fg="#ffffff",
                activebackground="#15803d",
                activeforeground="#ffffff",
                relief=tk.FLAT,
                padx=12,
                pady=5,
                font=("TkDefaultFont", 9, "bold"),
                cursor="hand2",
            ).pack(side=tk.LEFT, padx=(0, 12))
            tk.Button(
                header,
                text="RESET",
                command=self.reset_gui,
                bg="#7f1d1d",
                fg="#ffffff",
                activebackground="#991b1b",
                activeforeground="#ffffff",
                relief=tk.FLAT,
                padx=12,
                pady=5,
                font=("TkDefaultFont", 9, "bold"),
                cursor="hand2",
            ).pack(side=tk.LEFT, padx=(0, 12))
            self.order_mode_label = tk.Label(
                header,
                text="ORDER MODE: MANUAL",
                bg=PANEL_ALT,
                fg=WARNING,
                padx=8,
                pady=4,
                font=("TkDefaultFont", 8, "bold"),
            )
            self.order_mode_label.pack(side=tk.LEFT, padx=(0, 12))

        tk.Checkbutton(
            header,
            text="HIDE COMPLETED ROUTE",
            variable=self.hide_completed_route_var,
            command=self._toggle_completed_route_visibility,
            indicatoron=False,
            bg=BACKGROUND,
            fg=MUTED,
            activebackground="#166534",
            activeforeground="#ffffff",
            selectcolor="#166534",
            relief=tk.FLAT,
            padx=10,
            pady=5,
            font=("TkDefaultFont", 9, "bold"),
            cursor="hand2",
        ).pack(side=tk.LEFT, padx=(0, 12))

        indicator_frame = tk.Frame(header, bg=BACKGROUND)
        indicator_frame.pack(side=tk.LEFT, padx=(18, 0))
        self.connection_indicator_labels: Dict[str, tk.Label] = {}
        for source in (
            "TASK",
            "TEMPLATE",
            "PLANNER",
            "EXECUTION",
            "ODOM",
            "PARAM",
            "ROSOUT",
        ):
            label = tk.Label(
                indicator_frame,
                text=f"● {source} 0",
                bg=PANEL_ALT,
                fg=ERROR,
                padx=7,
                pady=3,
                font=("TkDefaultFont", 8, "bold"),
            )
            label.pack(side=tk.LEFT, padx=(0, 5))
            self.connection_indicator_labels[source] = label

        self.connection_label = tk.Label(
            header,
            text=f"● WAITING  {self.topic_name}",
            bg=BACKGROUND,
            fg=WARNING,
            font=("TkDefaultFont", 10, "bold"),
        )
        self.connection_label.pack(side=tk.RIGHT)

        body = tk.PanedWindow(
            self.root,
            orient=tk.HORIZONTAL,
            bg=BACKGROUND,
            sashwidth=6,
            sashrelief=tk.FLAT,
            bd=0,
        )
        body.pack(fill=tk.BOTH, expand=True, padx=14, pady=(0, 14))

        main_panel = tk.Frame(body, bg=PANEL, highlightthickness=1, highlightbackground="#334155")
        side_panel = tk.Frame(body, bg=PANEL, width=self.layout_config.info_panel_width)
        body.add(main_panel, stretch="always", minsize=self.layout_config.main_panel_min_width)
        body.add(side_panel, stretch="never", minsize=self.layout_config.info_panel_min_width)

        main_header = tk.Frame(main_panel, bg=PANEL, padx=12, pady=9)
        main_header.pack(fill=tk.X)
        tk.Label(
            main_header,
            text=f"{self.layout.data.get('name', 'Arena Layout')} — {self._side_title()}",
            bg=PANEL,
            fg=TEXT,
            font=("TkDefaultFont", 11, "bold"),
        ).pack(side=tk.LEFT)
        tk.Label(
            main_header,
            text="Station storage  |  Arena map  |  AMR cargo",
            bg=PANEL,
            fg=MUTED,
        ).pack(side=tk.RIGHT)

        monitor_area = tk.PanedWindow(
            main_panel,
            orient=tk.HORIZONTAL,
            bg=PANEL,
            sashwidth=5,
            sashrelief=tk.FLAT,
            bd=0,
        )
        monitor_area.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        station_panel = tk.Frame(
            monitor_area,
            bg="#0b1220",
            width=self.layout_config.station_panel_min_width,
            highlightthickness=1,
            highlightbackground="#334155",
        )
        arena_panel = tk.Frame(
            monitor_area,
            bg="#0b1220",
            width=self.layout_config.arena_panel_min_width,
            highlightthickness=1,
            highlightbackground="#334155",
        )
        cargo_panel = tk.Frame(
            monitor_area,
            bg="#0b1220",
            width=self.layout_config.cargo_panel_min_width,
            highlightthickness=1,
            highlightbackground="#334155",
        )

        monitor_area.add(
            station_panel,
            stretch="never",
            minsize=self.layout_config.station_panel_min_width,
        )
        monitor_area.add(
            arena_panel,
            stretch="always",
            minsize=self.layout_config.arena_panel_min_width,
        )
        monitor_area.add(
            cargo_panel,
            stretch="never",
            minsize=self.layout_config.cargo_panel_min_width,
        )

        station_header = tk.Frame(station_panel, bg="#0b1220", padx=10, pady=7)
        station_header.pack(fill=tk.X)
        tk.Label(
            station_header,
            text="STATION STORAGE",
            bg="#0b1220",
            fg=ACCENT,
            font=("TkDefaultFont", 10, "bold"),
        ).pack(side=tk.LEFT)

        arena_header = tk.Frame(arena_panel, bg="#0b1220", padx=10, pady=7)
        arena_header.pack(fill=tk.X)
        tk.Label(
            arena_header,
            text="ARENA MAP",
            bg="#0b1220",
            fg=ACCENT,
            font=("TkDefaultFont", 10, "bold"),
        ).pack(side=tk.LEFT)
        tk.Label(
            arena_header,
            text="Click a station for details",
            bg="#0b1220",
            fg=MUTED,
            font=("TkDefaultFont", 8),
        ).pack(side=tk.RIGHT)

        cargo_header = tk.Frame(cargo_panel, bg="#0b1220", padx=10, pady=7)
        cargo_header.pack(fill=tk.X)
        tk.Label(
            cargo_header,
            text="AMR CARGO",
            bg="#0b1220",
            fg=ACCENT,
            font=("TkDefaultFont", 10, "bold"),
        ).pack(side=tk.LEFT)

        self.station_canvas = tk.Canvas(
            station_panel,
            bg="#0b1220",
            highlightthickness=0,
        )
        self.station_canvas.pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))
        self.station_canvas.bind("<Configure>", lambda _event: self.redraw_storage_monitor())

        self.canvas = tk.Canvas(
            arena_panel,
            bg="#0b1220",
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))
        self.canvas.bind("<Configure>", lambda _event: self.redraw_map())

        self.cargo_canvas = tk.Canvas(
            cargo_panel,
            bg="#0b1220",
            highlightthickness=0,
        )
        self.cargo_canvas.pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))
        self.cargo_canvas.bind("<Configure>", lambda _event: self.redraw_storage_monitor())

        # Compatibility alias for drawing helper methods that draw to self.monitor_canvas.
        self.monitor_canvas = self.station_canvas

        self._build_side_panel(side_panel)

    def _build_side_panel(self, panel: tk.Frame) -> None:
        summary = tk.Frame(panel, bg=PANEL, padx=14, pady=12)
        summary.pack(fill=tk.X)
        self.summary_label = tk.Label(
            summary,
            text="No task received",
            bg=PANEL,
            fg=TEXT,
            anchor="w",
            font=("TkDefaultFont", 12, "bold"),
        )
        self.summary_label.pack(fill=tk.X)
        self.time_label = tk.Label(
            summary,
            text="Waiting for /eai/task …",
            bg=PANEL,
            fg=MUTED,
            anchor="w",
        )
        self.time_label.pack(fill=tk.X, pady=(4, 0))
        self.timing_label = tk.Label(
            summary,
            text="EST --:--  •  ACT 00:00  [WAITING]",
            bg=PANEL,
            fg=ACCENT,
            anchor="w",
            font=("TkDefaultFont", 10, "bold"),
        )
        self.timing_label.pack(fill=tk.X, pady=(4, 0))

        ttk.Separator(panel).pack(fill=tk.X, padx=12)
        tk.Label(
            panel,
            text="ORDERS  (max 10)",
            bg=PANEL,
            fg=ACCENT,
            anchor="w",
            font=("TkDefaultFont", 10, "bold"),
            padx=14,
            pady=8,
        ).pack(fill=tk.X)

        # Orders are limited to at most 10 in the current task generator, so keep
        # this table fixed-size.  The freed vertical space is used by the monitor
        # log below.
        order_frame = tk.Frame(panel, bg=PANEL)
        order_frame.pack(fill=tk.X, padx=12)
        self.order_tree = ttk.Treeview(
            order_frame,
            columns=("type", "object_id", "order"),
            show="headings",
            height=10,
        )
        self.order_tree.heading("type", text="Type")
        self.order_tree.heading("object_id", text="OB_ID")
        self.order_tree.heading("order", text="Order")
        self.order_tree.column("type", width=52, anchor=tk.CENTER, stretch=False)
        self.order_tree.column("object_id", width=72, anchor=tk.CENTER, stretch=False)
        self.order_tree.column("order", width=190, anchor=tk.W, stretch=True)
        order_scroll = ttk.Scrollbar(
            order_frame,
            orient=tk.VERTICAL,
            command=self.order_tree.yview,
        )
        self.order_tree.configure(yscrollcommand=order_scroll.set)
        self.order_tree.pack(side=tk.LEFT, fill=tk.X, expand=True)
        order_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        ttk.Separator(panel).pack(fill=tk.X, padx=12, pady=(10, 0))
        tk.Label(
            panel,
            text="STATION DETAILS",
            bg=PANEL,
            fg=ACCENT,
            anchor="w",
            font=("TkDefaultFont", 10, "bold"),
            padx=14,
            pady=8,
        ).pack(fill=tk.X)

        self.station_detail = tk.Label(
            panel,
            text="Select a station on the map.",
            bg=PANEL_ALT,
            fg=TEXT,
            justify=tk.LEFT,
            anchor="nw",
            padx=12,
            pady=10,
            wraplength=330,
            height=6,
        )
        self.station_detail.pack(fill=tk.X, padx=12, pady=(0, 10))

        ttk.Separator(panel).pack(fill=tk.X, padx=12, pady=(0, 0))
        tk.Label(
            panel,
            text="MONITOR LOG",
            bg=PANEL,
            fg=ACCENT,
            anchor="w",
            font=("TkDefaultFont", 10, "bold"),
            padx=14,
            pady=8,
        ).pack(fill=tk.X)

        source_frame = tk.Frame(panel, bg=PANEL, padx=12)
        source_frame.pack(fill=tk.X, pady=(0, 6))
        for index, source in enumerate(
            (
                "ALL",
                "TASK",
                "PLANNER",
                "EXECUTION",
                "LOCATION",
                "PARAM",
                "ROSOUT",
                "STATION",
                "SYSTEM",
                "DEBUG",
            )
        ):
            radio = tk.Radiobutton(
                source_frame,
                text=source.title(),
                variable=self.monitor_source_var,
                value=source,
                command=self._refresh_monitor_log_filter,
                bg=PANEL,
                fg=MUTED,
                activebackground=PANEL,
                activeforeground=TEXT,
                selectcolor=PANEL_ALT,
                font=("TkDefaultFont", 8),
                relief=tk.FLAT,
                padx=3,
                anchor=tk.W,
            )
            radio.grid(
                row=index // 5,
                column=index % 5,
                sticky=tk.W,
                padx=(0, 5),
                pady=1,
            )
        for column in range(5):
            source_frame.grid_columnconfigure(column, weight=1)

        log_frame = tk.Frame(panel, bg=PANEL)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 8))
        self.monitor_log = tk.Text(
            log_frame,
            bg=PANEL_ALT,
            fg=TEXT,
            insertbackground=TEXT,
            height=7,
            wrap=tk.WORD,
            relief=tk.FLAT,
            padx=8,
            pady=8,
            state=tk.DISABLED,
            font=("TkDefaultFont", 9),
        )
        log_scroll = ttk.Scrollbar(
            log_frame,
            orient=tk.VERTICAL,
            command=self.monitor_log.yview,
        )
        self.monitor_log.configure(yscrollcommand=log_scroll.set)
        self.monitor_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.monitor_log.tag_configure("ERROR", foreground=ERROR, font=("TkDefaultFont", 9, "bold"))
        self.monitor_log.tag_configure("WARN", foreground=WARNING, font=("TkDefaultFont", 9, "bold"))
        self.monitor_log.tag_configure("INFO", foreground=TEXT)
        self.monitor_log.tag_configure("SYSTEM", foreground=MUTED)
        self._append_monitor_log("SYSTEM", "Monitor log ready. Waiting for incoming task/external values.")

        legend = tk.Frame(panel, bg=PANEL, padx=12, pady=4)
        legend.pack(fill=tk.X)
        for label, color in (
            ("Storage", STATION_COLORS["storage_shelf"]),
            ("Workbench", STATION_COLORS["workbench"]),
            ("Hybrid", STATION_COLORS["hybrid_work_shelf"]),
            ("Customer", STATION_COLORS["customer_counter"]),
            ("Return raw", RETURN_REQUIRED_COLOR),
        ):
            item = tk.Frame(legend, bg=PANEL)
            item.pack(side=tk.LEFT, padx=(0, 10))
            tk.Label(item, text="■", bg=PANEL, fg=color).pack(side=tk.LEFT)
            tk.Label(item, text=label, bg=PANEL, fg=MUTED).pack(side=tk.LEFT)

    def set_manual_task_publisher(
        self, callback: Callable[[Task], None]
    ) -> None:
        self._manual_task_publisher = callback

    def reset_gui(self) -> None:
        if self.execution_active:
            messagebox.showwarning(
                "Reset unavailable",
                "작업 실행 중에는 리셋할 수 없습니다.",
                parent=self.root,
            )
            return
        if not messagebox.askyesno(
            "Reset GUI",
            "주문, 예상 경로, cargo 표시를 초기화할까요?",
            parent=self.root,
        ):
            return

        self._close_order_dialog()
        self.task = TaskState()
        self.cargo = CargoState()
        self.cargo_state_received = False
        self.planner_station_materials = {}
        self.expected_path = []
        self.active_segment = None
        self.solid_until_segment = -1
        self.robot_pose = None
        self._last_location_log_bucket = None
        self.selected_station_id = None
        self.execution_active = False
        self.estimated_duration_sec = None
        self.estimated_duration_min_sec = None
        self.estimated_duration_max_sec = None
        self.execution_started_monotonic = None
        self.execution_elapsed_sec = 0.0
        self.execution_outcome = "WAITING"

        self.summary_label.configure(text="No task received")
        self.time_label.configure(text="Waiting for manual/auto task input …")
        self._refresh_timing_display()
        self.connection_label.configure(
            text=f"● READY  {self.topic_name}",
            fg=WARNING,
        )
        self.station_detail.configure(
            text="Select a station on the map."
        )
        self._refresh_order_tree()

        self._monitor_history = []
        self._text_info_history = []
        self._refresh_monitor_log_filter()
        if (
            self.text_info_text is not None
            and self.text_info_text.winfo_exists()
        ):
            self.text_info_text.configure(state=tk.NORMAL)
            self.text_info_text.delete("1.0", tk.END)
            self.text_info_text.configure(state=tk.DISABLED)

        self._append_monitor_log(
            "SYSTEM",
            "GUI reset complete. Waiting for the next order.",
        )
        self._refresh_text_info_summary()
        self.redraw_map()
        self.redraw_storage_monitor()
        self.root.after(150, self.open_order_dialog)

    def update_task_template(self, msg: Task) -> None:
        self.task_template = msg
        self._append_monitor_log(
            "TASK",
            f"Arena template received: stations={len(msg.arena_layout)}. "
            "Enter orders and press PUBLISH TASK.",
        )
        if self.order_publish_button is not None:
            self.order_publish_button.configure(
                state=tk.NORMAL,
            )
            self._populate_manual_station_toggles()
            self._refresh_order_dialog_mode()

    def open_order_dialog(self) -> None:
        if self.order_dialog is not None and self.order_dialog.winfo_exists():
            self.order_dialog.deiconify()
            self.order_dialog.lift()
            return

        dialog = tk.Toplevel(self.root)
        self.order_dialog = dialog
        dialog.title("Enter RoboCup Orders")
        dialog.configure(bg=BACKGROUND)
        dialog.transient(self.root)
        dialog.resizable(False, False)
        dialog.protocol("WM_DELETE_WINDOW", self._close_order_dialog)

        frame = tk.Frame(dialog, bg=BACKGROUND, padx=18, pady=16)
        frame.pack(fill=tk.BOTH, expand=True)
        tk.Label(
            frame,
            text="ROBOCUP ORDER INPUT",
            bg=BACKGROUND,
            fg=ACCENT,
            font=("TkDefaultFont", 13, "bold"),
        ).grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 12))

        mode_frame = tk.Frame(frame, bg=PANEL, padx=10, pady=8)
        mode_frame.grid(row=1, column=0, columnspan=2, sticky=tk.EW)
        tk.Label(
            mode_frame,
            text="ORDER MODE",
            bg=PANEL,
            fg=TEXT,
            font=("TkDefaultFont", 9, "bold"),
        ).pack(side=tk.LEFT, padx=(0, 14))
        for mode, label in (
            ("manual", "MANUAL"),
            ("semi", "SEMI-AUTO"),
            ("auto", "AUTO"),
        ):
            button = tk.Radiobutton(
                mode_frame,
                text=label,
                variable=self.order_mode_var,
                value=mode,
                bg=PANEL,
                fg=TEXT,
                activebackground=PANEL,
                activeforeground=TEXT,
                selectcolor="#075985",
                command=self._refresh_order_dialog_mode,
            )
            button.pack(side=tk.LEFT, padx=(0, 10))

        manual_frame = tk.LabelFrame(
            frame,
            text=" MANUAL ORDERS ",
            bg=BACKGROUND,
            fg=WARNING,
            padx=10,
            pady=8,
        )
        manual_frame.grid(
            row=2,
            column=0,
            columnspan=2,
            sticky=tk.EW,
            pady=(10, 0),
        )
        tk.Label(
            manual_frame,
            text="PRODUCE IDs",
            bg=BACKGROUND,
            fg=TEXT,
            anchor=tk.W,
        ).grid(row=0, column=0, sticky=tk.W, padx=(0, 12), pady=5)
        produce_entry = tk.Entry(
            manual_frame,
            width=42,
            bg=PANEL_ALT,
            fg=TEXT,
            insertbackground=TEXT,
            relief=tk.FLAT,
        )
        produce_entry.grid(row=0, column=1, sticky=tk.EW, pady=5)

        tk.Label(
            manual_frame,
            text="RECYCLE IDs",
            bg=BACKGROUND,
            fg=TEXT,
            anchor=tk.W,
        ).grid(row=1, column=0, sticky=tk.W, padx=(0, 12), pady=5)
        recycle_entry = tk.Entry(
            manual_frame,
            width=42,
            bg=PANEL_ALT,
            fg=TEXT,
            insertbackground=TEXT,
            relief=tk.FLAT,
        )
        recycle_entry.grid(row=1, column=1, sticky=tk.EW, pady=5)

        tk.Label(
            manual_frame,
            text="쉼표 또는 공백으로 구분하세요. 같은 ID를 반복할 수 있습니다.",
            bg=BACKGROUND,
            fg=MUTED,
            justify=tk.LEFT,
        ).grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=(4, 2))

        station_frame = tk.LabelFrame(
            frame,
            text=" MANUAL STATION MATERIAL LAYOUT ",
            bg=BACKGROUND,
            fg=WARNING,
            padx=10,
            pady=8,
        )
        station_frame.grid(
            row=3,
            column=0,
            columnspan=2,
            sticky=tk.EW,
            pady=(10, 0),
        )
        self.manual_station_frame = station_frame
        self._populate_manual_station_toggles()

        auto_frame = tk.LabelFrame(
            frame,
            text=f" AUTO MATCH  /  TIER: {self.auto_tier.upper()} ",
            bg=BACKGROUND,
            fg=SUCCESS,
            padx=10,
            pady=8,
        )
        auto_frame.grid(
            row=4,
            column=0,
            columnspan=2,
            sticky=tk.EW,
            pady=(10, 0),
        )
        auto_buttons = []
        for stage, label in (
            ("production", "PRODUCTION"),
            ("recycling", "RECYCLING"),
            ("lifecycle", "LIFECYCLE"),
        ):
            button = tk.Radiobutton(
                auto_frame,
                text=label,
                variable=self.auto_stage_var,
                value=stage,
                bg=BACKGROUND,
                fg=TEXT,
                activebackground=BACKGROUND,
                activeforeground=TEXT,
                selectcolor="#166534",
                command=self._refresh_order_dialog_mode,
            )
            button.pack(side=tk.LEFT, padx=(0, 12))
            auto_buttons.append(button)

        product_lines = [
            f"{product_id}: {OBJECT_NAMES.get(product_id, 'product')}"
            for product_id in sorted(PRODUCT_IDS)
        ]
        tk.Label(
            frame,
            text="Available products\n" + "   ".join(product_lines),
            bg=PANEL,
            fg=MUTED,
            justify=tk.LEFT,
            anchor=tk.W,
            wraplength=560,
            padx=10,
            pady=9,
        ).grid(
            row=5,
            column=0,
            columnspan=2,
            sticky=tk.EW,
            pady=(10, 0),
        )

        publish_button = tk.Button(
            frame,
            text=(
                "PUBLISH TASK"
                if self.task_template is not None
                else "WAITING FOR ARENA TEMPLATE"
            ),
            state=(
                tk.NORMAL
                if self.task_template is not None
                else tk.DISABLED
            ),
            command=lambda: self._publish_selected_orders(
                produce_entry.get(), recycle_entry.get()
            ),
            bg="#166534",
            fg="#ffffff",
            disabledforeground="#94a3b8",
            activebackground="#15803d",
            activeforeground="#ffffff",
            relief=tk.FLAT,
            padx=14,
            pady=7,
            font=("TkDefaultFont", 10, "bold"),
        )
        publish_button.grid(
            row=6,
            column=0,
            columnspan=2,
            sticky=tk.EW,
            pady=(14, 0),
        )
        self.order_publish_button = publish_button
        self._order_manual_widgets = [
            produce_entry,
            recycle_entry,
        ]
        self._order_auto_widgets = auto_buttons
        self._refresh_order_dialog_mode()
        if self.order_mode_var.get() == "manual":
            produce_entry.focus_set()

    def _close_order_dialog(self) -> None:
        if self.order_dialog is not None and self.order_dialog.winfo_exists():
            self.order_dialog.destroy()
        self.order_dialog = None
        self.order_publish_button = None
        self.manual_station_frame = None
        self.manual_station_entries = {}
        self._manual_station_widgets = []

    def _populate_manual_station_toggles(self) -> None:
        frame = self.manual_station_frame
        if frame is None or not frame.winfo_exists():
            return
        for child in frame.winfo_children():
            child.destroy()
        self.manual_station_entries = {}
        self._manual_station_widgets = []

        if self.task_template is None:
            tk.Label(
                frame,
                text="Waiting for arena template...",
                bg=BACKGROUND,
                fg=MUTED,
            ).pack(anchor=tk.W)
            return

        stations = [
            station
            for station in self.task_template.arena_layout
            if station.station_type in (
                Station.ST_STORAGE,
                Station.ST_CUSTOMER,
                Station.ST_HYBRID,
            )
        ]
        tk.Label(
            frame,
            text=(
                "Storage/Hybrid에는 재료 ID, Customer에는 "
                "제품 ID를 입력하세요."
            ),
            bg=BACKGROUND,
            fg=MUTED,
        ).grid(
            row=0,
            column=0,
            columnspan=2,
            sticky=tk.W,
            pady=(0, 5),
        )
        for row, station in enumerate(stations, start=1):
            station_id = int(station.station_id)
            tk.Label(
                frame,
                text=f"S{station_id}  {station.name}",
                bg=BACKGROUND,
                fg=TEXT,
                anchor=tk.W,
            ).grid(row=row, column=0, sticky=tk.W, padx=(0, 10), pady=3)
            entry = tk.Entry(
                frame,
                width=34,
                bg=PANEL_ALT,
                fg=TEXT,
                insertbackground=TEXT,
                relief=tk.FLAT,
            )
            entry.grid(row=row, column=1, sticky=tk.EW, pady=3)
            self.manual_station_entries[station_id] = entry
            self._manual_station_widgets.append(entry)

    def _refresh_order_dialog_mode(self) -> None:
        mode = self.order_mode_var.get()
        order_state = (
            tk.NORMAL if mode in {"manual", "semi"} else tk.DISABLED
        )
        station_state = tk.NORMAL if mode == "manual" else tk.DISABLED
        auto_state = tk.NORMAL if mode == "auto" else tk.DISABLED
        for widget in getattr(self, "_order_manual_widgets", []):
            widget.configure(state=order_state)
        for widget in self._manual_station_widgets:
            widget.configure(state=station_state)
        for widget in getattr(self, "_order_auto_widgets", []):
            widget.configure(state=auto_state)

        label = f"ORDER MODE: {mode.upper()}"
        if mode == "auto":
            label += f" / {self.auto_stage_var.get().upper()}"
        if self.order_mode_label is not None:
            mode_color = {
                "manual": WARNING,
                "semi": ACCENT,
                "auto": SUCCESS,
            }.get(mode, TEXT)
            self.order_mode_label.configure(
                text=label,
                fg=mode_color,
            )
        if (
            self.order_publish_button is not None
            and self.task_template is not None
        ):
            self.order_publish_button.configure(
                text=f"PUBLISH {mode.upper()} TASK"
            )

    @staticmethod
    def _parse_product_ids(text: str) -> List[int]:
        values = []
        for token in text.replace(",", " ").split():
            try:
                product_id = int(token)
            except ValueError as exc:
                raise ValueError(f"Invalid product ID: {token}") from exc
            if product_id not in PRODUCT_IDS:
                raise ValueError(f"Unsupported product ID: {product_id}")
            values.append(product_id)
        return values

    @staticmethod
    def _parse_station_material_ids(
        station_id: int, text: str
    ) -> List[int]:
        values = []
        for token in text.replace(",", " ").split():
            try:
                material_id = int(token)
            except ValueError as exc:
                raise ValueError(
                    f"S{station_id}: invalid material ID {token}"
                ) from exc
            if material_id not in STATION_MATERIAL_IDS:
                raise ValueError(
                    f"S{station_id}: unsupported material ID {material_id}"
                )
            values.append(material_id)
        return values

    @staticmethod
    def _parse_customer_product_ids(
        station_id: int, text: str
    ) -> List[int]:
        values = []
        for token in text.replace(",", " ").split():
            try:
                product_id = int(token)
            except ValueError as exc:
                raise ValueError(
                    f"S{station_id}: invalid product ID {token}"
                ) from exc
            if product_id not in PRODUCT_IDS:
                raise ValueError(
                    f"S{station_id}: unsupported product ID {product_id}"
                )
            values.append(product_id)
        return values

    def _publish_selected_orders(
        self, produce_text: str, recycle_text: str
    ) -> None:
        mode = self.order_mode_var.get()
        try:
            if mode == "auto":
                produce_ids, recycle_ids = self._generate_auto_orders(
                    self.auto_stage_var.get()
                )
                material_station_ids = (
                    self._automatic_material_station_ids()
                )
                manual_station_materials = None
            else:
                produce_ids = self._parse_product_ids(produce_text)
                recycle_ids = self._parse_product_ids(recycle_text)
                if mode == "semi":
                    material_station_ids = (
                        self._automatic_material_station_ids()
                    )
                    manual_station_materials = None
                else:
                    material_station_ids = []
                    station_types = {
                        int(station.station_id): int(station.station_type)
                        for station in self.task_template.arena_layout
                    }
                    manual_station_materials = {
                        station_id: (
                            self._parse_customer_product_ids(
                                station_id, entry.get()
                            )
                            if station_types.get(station_id)
                            == Station.ST_CUSTOMER
                            else self._parse_station_material_ids(
                                station_id, entry.get()
                            )
                        )
                        for station_id, entry
                        in self.manual_station_entries.items()
                    }
            if not produce_ids and not recycle_ids:
                raise ValueError("Enter at least one PRODUCE or RECYCLE ID")
            if (
                mode in {"manual", "semi"}
                and len(produce_ids) + len(recycle_ids) > 10
            ):
                raise ValueError("The GUI supports at most 10 orders")
            task = self._build_order_task(
                produce_ids,
                recycle_ids,
                source_mode=mode,
                material_station_ids=material_station_ids,
                manual_station_materials=manual_station_materials,
            )
            if self._manual_task_publisher is None:
                raise RuntimeError("Manual task publisher is not connected")
            self._manual_task_publisher(task)
        except (RuntimeError, ValueError) as error:
            messagebox.showerror(
                "Invalid orders",
                str(error),
                parent=self.order_dialog or self.root,
            )
            return

        self._append_monitor_log(
            "TASK",
            f"{mode.upper()} task published: produce={produce_ids}, "
            f"recycle={recycle_ids}, "
            + (
                f"station_layout={manual_station_materials}"
                if manual_station_materials is not None
                else f"material_stations={material_station_ids}"
            ),
        )
        self._close_order_dialog()

    def _generate_auto_orders(
        self, stage: str
    ) -> Tuple[List[int], List[int]]:
        stage = str(stage).strip().lower()
        key = (self.auto_tier, stage)
        counts = AUTO_ORDER_COUNTS.get(key)
        if counts is None:
            raise ValueError(
                f"Unsupported automatic match: tier={self.auto_tier}, "
                f"stage={stage}"
            )
        candidates = AUTO_TIER_PRODUCTS[self.auto_tier]
        random_source = random.SystemRandom()
        produce_count, recycle_count = counts
        produce_ids = [
            random_source.choice(candidates)
            for _ in range(produce_count)
        ]
        recycle_ids = []
        if produce_ids and recycle_count:
            production_materials = {
                material_id
                for product_id in produce_ids
                for material_id in PRODUCT_MATERIALS[product_id]
            }
            reusable_candidates = [
                product_id
                for product_id in candidates
                if production_materials.intersection(
                    PRODUCT_MATERIALS[product_id]
                )
            ]
            recycle_ids.append(
                random_source.choice(reusable_candidates or candidates)
            )
        recycle_ids.extend(
            random_source.choice(candidates)
            for _ in range(recycle_count - len(recycle_ids))
        )
        return produce_ids, recycle_ids

    def _automatic_material_station_ids(self) -> List[int]:
        if self.task_template is None:
            raise RuntimeError("Arena template has not arrived yet")
        # Station 11 is the B-side mirror of A-side hybrid station 3.
        excluded_station_ids = {3, 11}
        station_ids = [
            int(station.station_id)
            for station in self.task_template.arena_layout
            if station.station_type in (
                Station.ST_STORAGE,
                Station.ST_HYBRID,
            )
            and int(station.station_id) not in excluded_station_ids
        ]
        if not station_ids:
            raise RuntimeError(
                "No automatic material station remains after excluding "
                "station 3/11"
            )
        return station_ids

    def _build_order_task(
        self,
        produce_ids: List[int],
        recycle_ids: List[int],
        source_mode: str,
        material_station_ids: List[int],
        manual_station_materials: Optional[Dict[int, List[int]]],
    ) -> Task:
        if self.task_template is None:
            raise RuntimeError("Arena template has not arrived yet")

        task = Task()
        orders = []
        for order_type, product_ids, prefix in (
            (Order.OT_PRODUCE, produce_ids, "produce"),
            (Order.OT_RECYCLE, recycle_ids, "recycle"),
        ):
            for index, product_id in enumerate(product_ids, start=1):
                order = Order()
                order.order_type = order_type
                order.product_id = product_id
                product_name = OBJECT_NAMES.get(product_id, str(product_id))
                order.name = (
                    f"{source_mode}_{prefix}_{index}_{product_name}"
                )
                orders.append(order)
        task.order_list = orders

        produce_materials = Counter(
            material_id
            for product_id in produce_ids
            for material_id in PRODUCT_MATERIALS[product_id]
        )
        recycle_materials = Counter(
            material_id
            for product_id in recycle_ids
            for material_id in PRODUCT_MATERIALS[product_id]
        )
        required_supply = list(
            (produce_materials - recycle_materials).elements()
        )
        # Every recycled material needs a designated return storage even when
        # production consumes none of it.
        for material_id in recycle_materials:
            if material_id not in required_supply:
                required_supply.append(material_id)

        all_storage_indices = [
            index
            for index, station in enumerate(
                self.task_template.arena_layout
            )
            if station.station_type in (
                Station.ST_STORAGE,
                Station.ST_HYBRID,
            )
        ]
        if not all_storage_indices:
            raise RuntimeError("Arena template has no storage station")

        if manual_station_materials is not None:
            buckets = {
                index: list(
                    manual_station_materials.get(
                        int(
                            self.task_template
                            .arena_layout[index]
                            .station_id
                        ),
                        [],
                    )
                )
                for index in all_storage_indices
            }
        else:
            storage_indices = [
                index
                for index in all_storage_indices
                if int(
                    self.task_template.arena_layout[index].station_id
                ) in material_station_ids
            ]
            if not storage_indices:
                raise RuntimeError(
                    "No material station selected for automatic layout"
                )
            buckets = {index: [] for index in storage_indices}
            for offset, material_id in enumerate(required_supply):
                target_index = (
                    storage_indices[offset % len(storage_indices)]
                )
                buckets[target_index].append(int(material_id))

        arena_layout = []
        for index, source in enumerate(self.task_template.arena_layout):
            station = Station()
            station.station_type = int(source.station_type)
            station.name = str(source.name)
            station.station_id = int(source.station_id)
            if source.station_type == Station.ST_CUSTOMER:
                if manual_station_materials is not None:
                    station.material_ids = list(
                        manual_station_materials.get(
                            int(source.station_id), []
                        )
                    )
                else:
                    station.material_ids = list(recycle_ids)
            elif source.station_type in (
                Station.ST_STORAGE,
                Station.ST_HYBRID,
            ):
                # Unselected manual stations and auto-excluded station 3/11
                # must be empty rather than retaining template objects.
                station.material_ids = list(buckets.get(index, []))
            else:
                station.material_ids = list(source.material_ids)
            arena_layout.append(station)
        task.arena_layout = arena_layout
        return task

    def update_task(self, msg: Task) -> None:
        self.task.update(msg)
        self.planner_station_materials = {}
        self.estimated_duration_sec = None
        self.estimated_duration_min_sec = None
        self.estimated_duration_max_sec = None
        self.execution_started_monotonic = None
        self.execution_elapsed_sec = 0.0
        self.execution_outcome = "WAITING"
        self.execution_active = False
        received = self.task.received_at
        received_text = received.strftime("%H:%M:%S") if received else "-"
        self.connection_label.configure(
            text=f"● LIVE [{self.task.source}]  {self.topic_name}",
            fg=SUCCESS,
        )
        self.summary_label.configure(
            text=(
                f"{self.task.source} TASK  •  "
                f"{len(self.task.orders)} orders  •  "
                f"{sum(station_id in self.visible_station_ids for station_id in self.task.stations)} "
                "visible stations"
            )
        )
        self.time_label.configure(
            text=f"Last update {received_text}  •  message #{self.task.message_count}"
        )
        self._refresh_timing_display()
        visible_count = sum(
            station_id in self.visible_station_ids for station_id in self.task.stations
        )
        self._append_monitor_log(
            "TASK",
            f"{self.task.source} task #{self.task.message_count} received: "
            f"orders={len(self.task.orders)}, visible_stations={visible_count}"
        )
        for station_id in sorted(self.task.stations):
            if station_id not in self.visible_station_ids:
                continue
            materials = list(
                self.task.stations[station_id].material_ids
            )
            self._append_monitor_log(
                "STATION",
                f"S{station_id} initial placement={materials or 'empty'}",
            )
        self._refresh_order_tree()
        self.redraw_map()
        self.redraw_storage_monitor()
        if self.selected_station_id is not None:
            self._show_station_details(self.selected_station_id)
        self._refresh_text_info_summary()

    @staticmethod
    def _format_duration(seconds: Optional[float]) -> str:
        if seconds is None:
            return "--:--"
        seconds = max(0, int(round(float(seconds))))
        hours, remainder = divmod(seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        if hours:
            return f"{hours:02d}:{minutes:02d}:{seconds:02d}"
        return f"{minutes:02d}:{seconds:02d}"

    def _refresh_timing_display(self) -> None:
        if not hasattr(self, "timing_label"):
            return
        estimated = self._format_estimated_duration()
        actual = self._format_duration(self.execution_elapsed_sec)
        color = (
            SUCCESS
            if self.execution_outcome == "DONE"
            else ERROR
            if self.execution_outcome == "FAILED"
            else ACCENT
        )
        self.timing_label.configure(
            text=(
                f"EST {estimated}  •  ACT {actual}  "
                f"[{self.execution_outcome}]"
            ),
            fg=color,
        )

    def _format_estimated_duration(self) -> str:
        minimum = self.estimated_duration_min_sec
        maximum = self.estimated_duration_max_sec
        if minimum is not None and maximum is not None:
            return (
                f"{self._format_duration(minimum)}"
                f"~{self._format_duration(maximum)}"
            )
        return self._format_duration(self.estimated_duration_sec)

    def _tick_execution_clock(self) -> None:
        if (
            self.execution_active
            and self.execution_started_monotonic is not None
        ):
            self.execution_elapsed_sec = (
                time.monotonic() - self.execution_started_monotonic
            )
            self._refresh_timing_display()
        try:
            self.root.after(200, self._tick_execution_clock)
        except tk.TclError:
            pass

    def update_connection_indicators(
        self,
        status: Dict[str, Optional[int]],
    ) -> None:
        for source, label in self.connection_indicator_labels.items():
            count = status.get(source)
            if count is None:
                text = f"○ {source} OFF"
                color = MUTED
            elif count > 0:
                text = f"● {source} {count}"
                color = SUCCESS
            else:
                text = f"● {source} 0"
                color = ERROR
            label.configure(text=text, fg=color)

    def _refresh_order_tree(self) -> None:
        self.order_tree.delete(*self.order_tree.get_children())
        for order in self.task.orders:
            if order.order_type == Order.OT_PRODUCE:
                short_type = "P"
            elif order.order_type == Order.OT_RECYCLE:
                short_type = "R"
            else:
                short_type = "?"
            self.order_tree.insert(
                "",
                tk.END,
                values=(short_type, str(order.product_id), order.name),
            )

    def _monitor_source_enabled(self, source: str) -> bool:
        if not hasattr(self, "monitor_source_var"):
            return True
        selected = self.monitor_source_var.get().upper()
        return selected == "ALL" or selected == source.upper()

    def _refresh_monitor_log_filter(self) -> None:
        if not hasattr(self, "monitor_log"):
            return
        self.monitor_log.configure(state=tk.NORMAL)
        self.monitor_log.delete("1.0", tk.END)
        for source, line, tag in self._monitor_history:
            if self._monitor_source_enabled(source):
                self.monitor_log.insert(tk.END, line, tag)
        self.monitor_log.see(tk.END)
        self.monitor_log.configure(state=tk.DISABLED)

    def _append_monitor_log(self, source: str, message: str) -> None:
        """Append one line to the right-side monitor log.

        Source is used by the radio buttons above the log.  All messages remain
        in history so changing the selected source immediately rebuilds the log.
        Error and warning messages are visually highlighted.
        """
        source = source.upper()

        upper_message = message.upper()
        if "ERROR" in upper_message or "FAIL" in upper_message or "FAILED" in upper_message:
            tag = "ERROR"
        elif "WARN" in upper_message or "TIMEOUT" in upper_message or "RETRY" in upper_message:
            tag = "WARN"
        elif source == "SYSTEM":
            tag = "SYSTEM"
        else:
            tag = "INFO"

        timestamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{timestamp}][{source}] {message}\n"
        self._monitor_history.append((source, line, tag))
        if len(self._monitor_history) > 500:
            self._monitor_history = self._monitor_history[-500:]
        self._text_info_history.append((line, tag))
        if len(self._text_info_history) > 500:
            self._text_info_history = self._text_info_history[-500:]
        self._append_text_info_line(line, tag)

        if not hasattr(self, "monitor_log") or not self._monitor_source_enabled(source):
            return
        self.monitor_log.configure(state=tk.NORMAL)
        self.monitor_log.insert(tk.END, line, tag)
        self.monitor_log.see(tk.END)
        self.monitor_log.configure(state=tk.DISABLED)

    def toggle_text_info_window(self) -> None:
        if self.text_info_toggle_var.get():
            self._show_text_info_window()
        else:
            self._hide_text_info_window()

    def _show_text_info_window(self) -> None:
        if self.text_info_window is None or not self.text_info_window.winfo_exists():
            self._create_text_info_window()
        else:
            self.text_info_window.deiconify()
            self.text_info_window.lift()
        self._set_text_info_alpha(self.text_info_alpha_var.get())
        self._refresh_text_info_summary()

    def _hide_text_info_window(self) -> None:
        self.text_info_toggle_var.set(False)
        if self.text_info_window is not None and self.text_info_window.winfo_exists():
            self.text_info_window.withdraw()

    def _create_text_info_window(self) -> None:
        window = tk.Toplevel(self.root)
        self.text_info_window = window
        window.title("SML Live Text Information")
        window.configure(bg=BACKGROUND)
        window.transient(self.root)
        window.attributes("-topmost", True)
        window.protocol("WM_DELETE_WINDOW", self._hide_text_info_window)

        self.root.update_idletasks()
        x = self.root.winfo_rootx() + 36
        y = self.root.winfo_rooty() + 82
        window.geometry(f"680x480+{x}+{y}")
        window.minsize(480, 300)

        header = tk.Frame(window, bg=BACKGROUND, padx=12, pady=10)
        header.pack(fill=tk.X)
        tk.Label(
            header,
            text="LIVE TEXT INFORMATION",
            bg=BACKGROUND,
            fg=TEXT,
            font=("TkDefaultFont", 12, "bold"),
        ).pack(side=tk.LEFT)

        alpha_frame = tk.Frame(header, bg=BACKGROUND)
        alpha_frame.pack(side=tk.RIGHT)
        self.text_info_alpha_label = tk.Label(
            alpha_frame,
            text="Alpha 78%",
            bg=BACKGROUND,
            fg=MUTED,
            width=10,
        )
        self.text_info_alpha_label.pack(side=tk.LEFT)
        tk.Scale(
            alpha_frame,
            from_=30,
            to=100,
            orient=tk.HORIZONTAL,
            variable=self.text_info_alpha_var,
            command=self._set_text_info_alpha,
            showvalue=False,
            length=150,
            resolution=1,
            bg=BACKGROUND,
            fg=TEXT,
            troughcolor=PANEL_ALT,
            activebackground=ACCENT,
            highlightthickness=0,
            bd=0,
        ).pack(side=tk.LEFT)

        self.text_info_summary = tk.Label(
            window,
            text="",
            bg=PANEL,
            fg=TEXT,
            justify=tk.LEFT,
            anchor=tk.NW,
            padx=12,
            pady=9,
            font=("TkFixedFont", 10),
        )
        self.text_info_summary.pack(fill=tk.X, padx=12, pady=(0, 8))

        log_frame = tk.Frame(window, bg=BACKGROUND)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))
        text = tk.Text(
            log_frame,
            bg="#0b1220",
            fg=TEXT,
            insertbackground=TEXT,
            wrap=tk.WORD,
            relief=tk.FLAT,
            padx=10,
            pady=10,
            state=tk.DISABLED,
            font=("TkFixedFont", 9),
        )
        self.text_info_text = text
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=text.yview)
        text.configure(yscrollcommand=scrollbar.set)
        text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        text.tag_configure("ERROR", foreground=ERROR, font=("TkFixedFont", 9, "bold"))
        text.tag_configure("WARN", foreground=WARNING, font=("TkFixedFont", 9, "bold"))
        text.tag_configure("INFO", foreground=TEXT)
        text.tag_configure("SYSTEM", foreground=MUTED)

        text.configure(state=tk.NORMAL)
        for line, tag in self._text_info_history:
            text.insert(tk.END, line, tag)
        text.see(tk.END)
        text.configure(state=tk.DISABLED)
        self._set_text_info_alpha(self.text_info_alpha_var.get())

    def _append_text_info_line(self, line: str, tag: str) -> None:
        text = self.text_info_text
        if text is None or not text.winfo_exists():
            return
        text.configure(state=tk.NORMAL)
        text.insert(tk.END, line, tag)
        text.see(tk.END)
        text.configure(state=tk.DISABLED)

    def _set_text_info_alpha(self, value) -> None:
        try:
            percent = max(30.0, min(100.0, float(value)))
        except (TypeError, ValueError):
            percent = 78.0
        self.text_info_alpha_var.set(percent)
        if hasattr(self, "text_info_alpha_label"):
            self.text_info_alpha_label.configure(text=f"Alpha {percent:.0f}%")
        if self.text_info_window is not None and self.text_info_window.winfo_exists():
            self.text_info_window.attributes("-alpha", percent / 100.0)

    def _refresh_text_info_summary(self) -> None:
        label = self.text_info_summary
        if label is None or not label.winfo_exists():
            return

        if self.expected_path:
            route = " → ".join(str(station_id) for station_id in self.expected_path)
        else:
            route = "waiting"
        segment_total = max(0, len(self.expected_path) - 1)
        active = "-" if self.active_segment is None else str(self.active_segment + 1)
        completed = max(0, self.solid_until_segment + 1)

        if self.robot_pose:
            robot_state = str(
                self.robot_pose.get("state", self.robot_pose.get("status", "UNKNOWN"))
            )
            robot_x = self.robot_pose.get("x", "-")
            robot_y = self.robot_pose.get("y", "-")
            robot_text = f"{robot_state}  position=({robot_x}, {robot_y})"
        else:
            robot_text = "waiting for odometry"

        received = (
            self.task.received_at.strftime("%H:%M:%S")
            if self.task.received_at is not None
            else "-"
        )
        label.configure(
            text=(
                f"SIDE     {self._side_title()}\n"
                f"TASK     {self.task.source}  #{self.task.message_count}  "
                f"orders={len(self.task.orders)}  "
                f"last={received}\n"
                f"ROUTE    {route}\n"
                f"PROGRESS active={active}/{segment_total}  completed={completed}/{segment_total}\n"
                f"AMR      {robot_text}\n"
                f"TIME     estimated={self._format_estimated_duration()}  "
                f"actual={self._format_duration(self.execution_elapsed_sec)}  "
                f"[{self.execution_outcome}]"
            )
        )

    def update_planner_state(self, raw_text: str) -> None:
        payload = self._load_json_payload(raw_text)
        if payload is None:
            self._append_monitor_log("PLANNER", raw_text)
            return

        path_supplied = "expected_path" in payload or "path" in payload
        path = payload.get("expected_path", payload.get("path"))
        if path_supplied and isinstance(path, list):
            self.expected_path = [
                int(value)
                for value in path
                if self._safe_int(value) is not None
            ]
            self.active_segment = None
            self.solid_until_segment = -1

        self._apply_station_payload(payload, source="PLANNER")
        self._apply_cargo_payload(payload)
        estimate = self._safe_float(payload.get("estimated_duration_sec"))
        estimate_min = self._safe_float(
            payload.get("estimated_duration_min_sec")
        )
        estimate_max = self._safe_float(
            payload.get("estimated_duration_max_sec")
        )
        if (
            estimate_min is not None
            and estimate_max is not None
            and estimate_min >= 0.0
            and estimate_max >= estimate_min
        ):
            self.estimated_duration_min_sec = estimate_min
            self.estimated_duration_max_sec = estimate_max
        if estimate is not None and estimate >= 0.0:
            self.estimated_duration_sec = estimate
        self._refresh_timing_display()

        plan_id = payload.get("plan_id", "-")
        step_count = len(payload.get("steps", [])) if isinstance(payload.get("steps"), list) else "-"
        self._append_monitor_log(
            "PLANNER",
            f"plan={plan_id} expected_path={self.expected_path} steps={step_count}"
        )
        self._refresh_text_info_summary()
        self.redraw_map()
        self.redraw_storage_monitor()

    def update_manager_status(self, raw_text: str) -> None:
        payload = self._load_json_payload(raw_text)
        if payload is not None:
            action = str(payload.get("action", "")).upper()
            status = str(
                payload.get("status", payload.get("state", ""))
            ).upper()
            if action == "PLAN" and status == "START":
                self.execution_active = True
                self.execution_started_monotonic = time.monotonic()
                self.execution_elapsed_sec = 0.0
                self.execution_outcome = "RUNNING"
            elif action == "PLAN" and status in {
                "DONE",
                "FAILED",
                "READY",
            }:
                self.execution_active = False
                if status in {"DONE", "FAILED"}:
                    if self.execution_started_monotonic is not None:
                        self.execution_elapsed_sec = (
                            time.monotonic()
                            - self.execution_started_monotonic
                        )
                    self.execution_started_monotonic = None
                    self.execution_outcome = status
            self._refresh_timing_display()
            active = self._safe_int(payload.get("active_segment", payload.get("route_segment_index")))
            solid = self._safe_int(payload.get("solid_until_segment"))
            if active is not None:
                self.active_segment = active
            elif "active_segment" in payload:
                self.active_segment = None
            if solid is not None:
                self.solid_until_segment = solid
            current_station = self._safe_int(
                payload.get("current_station", payload.get("station"))
            )
            if current_station is not None:
                self.robot_pose = {
                    "current_station": current_station,
                    "state": payload.get(
                        "status", payload.get("state", "IDLE")
                    ),
                }
            self._apply_station_payload(payload, source="EXECUTION")
            self._apply_cargo_payload(payload)
        self._append_monitor_log(
            "EXECUTION", self._format_manager_status(raw_text)
        )
        self._refresh_text_info_summary()
        self.redraw_map()
        self.redraw_storage_monitor()

    def update_location_status(self, raw_text: str) -> None:
        payload = self._load_json_payload(raw_text)
        if payload is not None:
            self.robot_pose = payload
            active = self._safe_int(payload.get("active_segment", payload.get("route_segment_index")))
            if active is not None:
                self.active_segment = active
            progress = self._safe_float(payload.get("progress", 0.0)) or 0.0
            state = str(payload.get("state", payload.get("status", ""))).upper()
            current = self._safe_int(payload.get("current_station", payload.get("station"))) or -1
            target = self._safe_int(payload.get("target_station", payload.get("target"))) or -1
            bucket = (state, current * 1000 + target, int(progress * 10.0))
            # Do not spam the log at location publish rate.  Show meaningful
            # state/progress changes while still updating the AMR marker every message.
            if bucket != self._last_location_log_bucket or state in {"ERROR", "ARRIVED"}:
                self._append_monitor_log("LOCATION", self._format_location_status(raw_text))
                self._last_location_log_bucket = bucket
            self._refresh_text_info_summary()
            self.redraw_map()
            return

        self._append_monitor_log("LOCATION", self._format_location_status(raw_text))

    def update_parameter_event(self, text: str) -> None:
        self._append_monitor_log("PARAM", text)

    def update_rosout(self, text: str) -> None:
        self._append_monitor_log("ROSOUT", text)

    def _load_json_payload(self, raw_text: str) -> Optional[dict]:
        try:
            value = json.loads(raw_text)
        except json.JSONDecodeError:
            return None
        return value if isinstance(value, dict) else None

    @staticmethod
    def _safe_int(value) -> Optional[int]:
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _safe_float(value) -> Optional[float]:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _apply_station_payload(
        self, payload: dict, source: str = "PLANNER"
    ) -> None:
        stations = payload.get("stations")
        if not isinstance(stations, dict):
            return
        converted: Dict[int, Tuple[int, ...]] = {}
        for key, values in stations.items():
            station_id = self._safe_int(key)
            if station_id is None:
                continue
            if isinstance(values, list):
                converted[station_id] = tuple(
                    int(value) for value in values if self._safe_int(value) is not None
                )
        previous = {
            station_id: tuple(state.material_ids)
            for station_id, state in self.task.stations.items()
        }
        previous.update(self.planner_station_materials)
        for station_id, materials in sorted(converted.items()):
            if station_id not in self.visible_station_ids:
                continue
            before = previous.get(station_id, ())
            if before == materials:
                continue
            self._append_monitor_log(
                "STATION",
                f"S{station_id} placement {list(before) or 'empty'}"
                f" -> {list(materials) or 'empty'} ({source})",
            )
        self.planner_station_materials = converted

    def _apply_cargo_payload(self, payload: dict) -> None:
        cargo = payload.get("cargo")
        if not isinstance(cargo, dict):
            return

        finished = self._safe_int(cargo.get("finished_product"))
        raw_slides_payload = cargo.get("raw_slides", {})
        raw_slides: Dict[int, List[CargoRawItem]] = {1: [], 2: [], 3: [], 4: [], 5: []}
        if isinstance(raw_slides_payload, dict):
            for slide_key, items in raw_slides_payload.items():
                slide_id = self._safe_int(slide_key)
                if slide_id is None or slide_id not in raw_slides:
                    continue
                if not isinstance(items, list):
                    continue
                for item in items:
                    if isinstance(item, dict):
                        raw_id = self._safe_int(item.get("raw_id", item.get("object_id")))
                        position = self._safe_int(item.get("position", item.get("pick_position")))
                        return_required = bool(
                            item.get("return_required", False)
                        )
                        return_station = self._safe_int(
                            item.get("return_station")
                        )
                    else:
                        raw_id = self._safe_int(item)
                        position = None
                        return_required = False
                        return_station = None
                    if raw_id is None:
                        continue
                    if position is None:
                        position = self._default_slide_position(raw_id, raw_slides[slide_id])
                    raw_slides[slide_id].append(
                        CargoRawItem(
                            raw_id=raw_id,
                            slide_id=slide_id,
                            position=position,
                            return_required=return_required,
                            return_station=return_station,
                        )
                    )

        assembly_payload = cargo.get("assembly_spaces", {})
        assembly_spaces: Dict[int, Optional[int]] = {1: None, 2: None}
        if isinstance(assembly_payload, dict):
            for key, value in assembly_payload.items():
                assembly_id = self._safe_int(key)
                object_id = self._safe_int(value)
                if assembly_id in assembly_spaces:
                    assembly_spaces[assembly_id] = object_id

        self.cargo = CargoState(
            finished_product=finished,
            raw_slides=raw_slides,
            assembly_spaces=assembly_spaces,
        )
        self.cargo_state_received = True

    def _default_slide_position(self, raw_id: int, existing: List[CargoRawItem]) -> int:
        used = {item.position for item in existing}
        candidates = [0, 2, 4] if raw_id in {1, 2, 3, 4} else [1, 3]
        for position in candidates:
            if position not in used:
                return position
        return candidates[-1]

    def _format_manager_status(self, raw_text: str) -> str:
        payload = self._load_json_payload(raw_text)
        if payload is None:
            return raw_text
        level = payload.get("level", "INFO")
        step = payload.get("step", "-")
        action = payload.get("action", "-")
        station = payload.get("station", payload.get("target_station", payload.get("to_station", "-")))
        status = payload.get("status", payload.get("state", "-"))
        obj = payload.get("object_id", payload.get("raw_id", ""))
        segment = payload.get("active_segment", payload.get("route_segment_index", "-"))
        message = payload.get("message", "")
        object_text = f" object={obj}" if obj not in {None, ""} else ""
        return f"{level} step={step} seg={segment} action={action} station={station} status={status}{object_text} {message}".strip()

    def _format_location_status(self, raw_text: str) -> str:
        payload = self._load_json_payload(raw_text)
        if payload is None:
            return raw_text
        robot_id = payload.get("robot_id", payload.get("amr_id", "amr"))
        if payload.get("source") == "odometry":
            frame = payload.get("frame_id", "")
            x = payload.get("x", "-")
            y = payload.get("y", "-")
            odom_x = payload.get("odom_x", "-")
            odom_y = payload.get("odom_y", "-")
            heading = payload.get("heading_deg", "-")
            speed = payload.get("speed", "-")
            state = payload.get("state", "-")
            return (
                f"{robot_id} odom=({odom_x},{odom_y}) layout=({x},{y}) "
                f"yaw={heading}deg speed={speed} state={state} frame={frame}"
            ).strip()
        current = payload.get("current_station", payload.get("station", "-"))
        target = payload.get("target_station", payload.get("target", "-"))
        state = payload.get("state", payload.get("status", "-"))
        progress = payload.get("progress", "")
        message = payload.get("message", "")
        progress_text = f" progress={float(progress):.2f}" if self._safe_float(progress) is not None else ""
        return f"{robot_id} current={current} target={target} state={state}{progress_text} {message}".strip()

    def redraw_map(self) -> None:
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        if width < 20 or height < 20:
            return

        self.canvas.delete("all")
        padding = 24.0
        view_x, view_y, view_width, view_height = self._viewport()
        scale = min(
            (width - 2 * padding) / view_width,
            (height - 2 * padding) / view_height,
        )
        offset_x = (width - view_width * scale) / 2 - view_x * scale
        offset_y = (height - view_height * scale) / 2 - view_y * scale
        self._transform = (scale, offset_x, offset_y)

        self._draw_zones()
        self._draw_walls()
        self._draw_start_areas()
        self._draw_expected_route()
        self._draw_stations()
        self._draw_amr_marker()

    def _toggle_completed_route_visibility(self) -> None:
        """Hide/show only completed route items without rebuilding the map."""
        if not hasattr(self, "canvas"):
            return
        state = (
            tk.HIDDEN
            if self.hide_completed_route_var.get()
            else tk.NORMAL
        )
        self.canvas.itemconfigure("completed_route", state=state)

    def _xy(self, x: float, y: float) -> Tuple[float, float]:
        scale, offset_x, offset_y = self._transform
        return offset_x + x * scale, offset_y + y * scale

    def redraw_storage_monitor(self) -> None:
        """Draw station storage and AMR cargo in separate always-visible panels."""
        if not hasattr(self, "station_canvas") or not hasattr(self, "cargo_canvas"):
            return

        # Left panel: station storage status.
        station_w = max(self.station_canvas.winfo_width(), 1)
        station_h = max(self.station_canvas.winfo_height(), 1)
        if station_w >= 20 and station_h >= 20:
            self.station_canvas.delete("all")
            old_canvas = getattr(self, "monitor_canvas", None)
            self.monitor_canvas = self.station_canvas
            padding = 12.0
            self._draw_station_storage_monitor(
                padding,
                padding,
                max(1.0, station_w - 2 * padding),
                max(1.0, station_h - 2 * padding),
            )
            if old_canvas is not None:
                self.monitor_canvas = old_canvas

        # Right panel: AMR cargo status.
        cargo_w = max(self.cargo_canvas.winfo_width(), 1)
        cargo_h = max(self.cargo_canvas.winfo_height(), 1)
        if cargo_w >= 20 and cargo_h >= 20:
            self.cargo_canvas.delete("all")
            old_canvas = getattr(self, "monitor_canvas", None)
            self.monitor_canvas = self.cargo_canvas
            padding = 12.0
            self._draw_cargo_monitor(
                padding,
                padding,
                max(1.0, cargo_w - 2 * padding),
                max(1.0, cargo_h - 2 * padding),
            )
            if old_canvas is not None:
                self.monitor_canvas = old_canvas

    def _monitor_station_ids(self) -> List[int]:
        if self.selected_side in SIDE_MONITOR_STATIONS:
            return SIDE_MONITOR_STATIONS[self.selected_side]
        return sorted(self.visible_station_ids)

    def _draw_station_storage_monitor(
        self,
        x: float,
        y: float,
        width: float,
        height: float,
    ) -> None:
        station_ids = self._monitor_station_ids()
        if not station_ids:
            return

        title_w = min(112.0, max(86.0, width * 0.38))
        grid_units_w = 12.0  # 3 cells * 4 units
        grid_units_h = 4.0   # 2 cells * 2 units

        # Scale up in large windows.  The previous version capped the station
        # unit at 16 px, which left a lot of unused space when the window was
        # maximized.  The new cap keeps readability but uses available height.
        available_row_h = height / max(len(station_ids), 1)
        unit_from_width = max(6.0, (width - title_w - 18.0) / grid_units_w)
        unit_from_height = max(6.0, (available_row_h - 14.0) / grid_units_h)
        unit = min(22.0, unit_from_width, unit_from_height)

        cell_w = 4.0 * unit
        cell_h = 2.0 * unit
        grid_h = 2.0 * cell_h
        row_h = height / max(len(station_ids), 1)

        for row, station_id in enumerate(station_ids):
            row_y = y + row * row_h
            station = self.layout.stations_by_id.get(station_id)
            state = self.task.stations.get(station_id)
            materials = self._station_materials(station_id)
            station_name = station["name"] if station else f"station_{station_id}"
            label_color = TEXT if materials or state is not None else MUTED

            # Subtle row background to make the larger layout easier to scan.
            self.monitor_canvas.create_rectangle(
                x,
                row_y + 2,
                x + width,
                row_y + row_h - 3,
                fill="#0b1220",
                outline="",
            )

            self.monitor_canvas.create_text(
                x,
                row_y + row_h * 0.42,
                text=f"#{station_id:02d}",
                fill=label_color,
                anchor=tk.W,
                font=("TkDefaultFont", 9, "bold"),
            )
            self.monitor_canvas.create_text(
                x + 42,
                row_y + row_h * 0.42,
                text=self._compact_station_name(station_name),
                fill=label_color,
                anchor=tk.W,
                font=("TkDefaultFont", 8),
            )

            grid_x = x + title_w
            grid_y = row_y + max(0.0, (row_h - grid_h) / 2.0)
            self._draw_raw_grid(
                grid_x,
                grid_y,
                cell_w,
                cell_h,
                materials,
                max_items=6,
            )

    @staticmethod
    def _compact_station_name(name: str) -> str:
        return (
            name.replace("side_a_", "A ")
            .replace("side_b_", "B ")
            .replace("shared_", "Shared ")
            .replace("_", " ")
        )

    def _draw_raw_grid(
        self,
        x: float,
        y: float,
        cell_w: float,
        cell_h: float,
        object_ids: List[int],
        max_items: int = 6,
    ) -> None:
        for slot in range(max_items):
            col = slot % 3
            row = slot // 3
            cell_x = x + col * cell_w
            cell_y = y + row * cell_h
            self.monitor_canvas.create_rectangle(
                cell_x,
                cell_y,
                cell_x + cell_w,
                cell_y + cell_h,
                fill="#0f172a",
                outline="#334155",
                width=1,
            )
            self.monitor_canvas.create_text(
                cell_x + 4,
                cell_y + 3,
                text=str(slot),
                fill="#475569",
                anchor=tk.NW,
                font=("TkDefaultFont", 6),
            )

        for slot, object_id in enumerate(object_ids[:max_items]):
            col = slot % 3
            row = slot // 3
            cell_x = x + col * cell_w
            cell_y = y + row * cell_h
            self._draw_object_in_cell(object_id, cell_x, cell_y, cell_w, cell_h)

        if len(object_ids) > max_items:
            self.monitor_canvas.create_text(
                x + 3 * cell_w + 8,
                y + cell_h,
                text=f"+{len(object_ids) - max_items}",
                fill=WARNING,
                anchor=tk.W,
                font=("TkDefaultFont", 9, "bold"),
            )

    def _draw_object_in_cell(
        self,
        object_id: int,
        cell_x: float,
        cell_y: float,
        cell_w: float,
        cell_h: float,
    ) -> None:
        if object_id in RAW_MATERIAL_SIZES:
            raw_w, raw_h = RAW_MATERIAL_SIZES[object_id]
            # The cell is 4x2.  Therefore unit can be derived from both axes.
            unit = min(cell_w / 4.0, cell_h / 2.0)
            block_w = raw_w * unit
            block_h = raw_h * unit
            x = cell_x + (cell_w - block_w) / 2.0
            y = cell_y + (cell_h - block_h) / 2.0
            self._draw_labeled_rect(
                x,
                y,
                block_w,
                block_h,
                RAW_MATERIAL_COLORS[object_id],
                RAW_MATERIAL_TEXT_COLORS[object_id],
                str(object_id),
                font_size=9,
            )
            return

        if object_id in BATCH_IDS:
            self._draw_labeled_rect(
                cell_x + 2,
                cell_y + 2,
                cell_w - 4,
                cell_h - 4,
                "#64748b",
                "#ffffff",
                f"B{object_id}",
                font_size=8,
            )
            return

        # Product or unknown object.
        color = "#f59e0b" if object_id in PRODUCT_IDS else "#94a3b8"
        label = str(object_id)
        self._draw_labeled_rect(
            cell_x + 2,
            cell_y + 2,
            cell_w - 4,
            cell_h - 4,
            color,
            "#111827",
            label,
            font_size=8,
        )

    def _draw_labeled_rect(
        self,
        x: float,
        y: float,
        w: float,
        h: float,
        fill: str,
        text_color: str,
        label: str,
        font_size: int = 9,
        outline: str = "#020617",
    ) -> None:
        self.monitor_canvas.create_rectangle(
            x,
            y,
            x + w,
            y + h,
            fill=fill,
            outline=outline,
            width=2,
        )
        self.monitor_canvas.create_text(
            x + w / 2,
            y + h / 2,
            text=label,
            fill=text_color,
            font=("TkDefaultFont", font_size, "bold"),
        )

    def _draw_cargo_monitor(
        self,
        x: float,
        y: float,
        width: float,
        height: float,
    ) -> None:
        # Logical height: 6x8 + five 6x2 slides + two 6x8 assembly spaces = 34 units.
        section_defs = [
            ("finished_product", "Cargo 1 / Recycle Product", 6, 8),
            ("raw_slide_1", "Raw Slide 1", 6, 2),
            ("raw_slide_2", "Raw Slide 2", 6, 2),
            ("raw_slide_3", "Raw Slide 3", 6, 2),
            ("raw_slide_4", "Raw Slide 4", 6, 2),
            ("raw_slide_5", "Raw Slide 5", 6, 2),
            ("assembly_1", "Assembly Space / Cargo 7", 6, 8),
            ("assembly_2", "Assembly Space / Cargo 8", 6, 8),
        ]

        label_w = min(128.0, max(96.0, width * 0.36))
        box_area_w = max(80.0, width - label_w - 12.0)
        total_units_h = sum(item[3] for item in section_defs)
        gap_px = 8.0
        total_gap_h = gap_px * (len(section_defs) - 1)

        # The previous cargo renderer spent 18 px above every section for labels,
        # which made the actual cargo spaces small.  Labels are now placed to the
        # left, so the 6x8 / 6x2 boxes can use almost the full vertical space.
        unit = min(
            22.0,
            max(5.0, box_area_w / 6.0),
            max(6.0, (height - total_gap_h - 18.0) / total_units_h),
        )

        cargo_w = 6.0 * unit
        total_draw_h = total_units_h * unit + total_gap_h
        start_y = y + max(0.0, (height - total_draw_h) / 2.0)
        current_y = start_y
        box_x = x + label_w + max(0.0, (box_area_w - cargo_w) / 2.0)

        for key, label, logical_w, logical_h in section_defs:
            section_h = logical_h * unit
            label_y = current_y + section_h / 2.0
            self.monitor_canvas.create_text(
                x,
                label_y,
                text=label,
                fill=MUTED,
                anchor=tk.W,
                font=("TkDefaultFont", 8, "bold"),
            )

            self.monitor_canvas.create_rectangle(
                box_x,
                current_y,
                box_x + logical_w * unit,
                current_y + section_h,
                fill="#0f172a",
                outline="#475569",
                width=1,
            )

            if key == "finished_product":
                self._draw_product_space_content(
                    self.cargo.finished_product,
                    box_x,
                    current_y,
                    logical_w * unit,
                    section_h,
                )
            elif key.startswith("raw_slide_"):
                slide_id = int(key.rsplit("_", 1)[1])
                self._draw_raw_slide_content(
                    slide_id,
                    box_x,
                    current_y,
                    unit,
                )
            elif key.startswith("assembly_"):
                assembly_id = int(key.rsplit("_", 1)[1])
                self._draw_product_space_content(
                    self.cargo.assembly_spaces.get(assembly_id),
                    box_x,
                    current_y,
                    logical_w * unit,
                    section_h,
                )

            current_y += section_h + gap_px

        self.monitor_canvas.create_text(
            x,
            y + height - 4,
            text=(
                "Live planner cargo state"
                if self.cargo_state_received
                else "Waiting for planner cargo state."
            ),
            fill=SUCCESS if self.cargo_state_received else "#64748b",
            anchor=tk.SW,
            font=("TkDefaultFont", 7),
        )

    def _draw_product_space_content(
        self,
        product_id: Optional[int],
        x: float,
        y: float,
        w: float,
        h: float,
    ) -> None:
        if product_id is None:
            self.monitor_canvas.create_text(
                x + w / 2,
                y + h / 2,
                text="EMPTY",
                fill="#334155",
                font=("TkDefaultFont", 9, "bold"),
            )
            return

        product = self.object_assets.products.get(product_id)
        if product is None:
            label = f"{product_id}\n{OBJECT_NAMES.get(product_id, 'product')}"
            self.monitor_canvas.create_text(
                x + w / 2,
                y + h / 2,
                text=label,
                fill="#fbbf24",
                justify=tk.CENTER,
                font=("TkDefaultFont", 9, "bold"),
            )
            return

        area = product.get("placement_area", {"w": 6, "h": 8})
        area_w = max(1.0, float(area.get("w", 6)))
        area_h = max(1.0, float(area.get("h", 8)))
        unit = min(w / area_w, h / area_h)
        origin_x = x + (w - area_w * unit) / 2.0
        origin_y = y + (h - area_h * unit) / 2.0

        for block in product.get("blocks", []):
            raw_id = int(block["raw_id"])
            material = self.object_assets.raw_materials.get(raw_id, {})
            block_x = origin_x + float(block["x"]) * unit
            # Asset JSON uses a bottom-left origin; Tk Canvas uses top-left.
            block_y = origin_y + (
                area_h - float(block["y"]) - float(block["h"])
            ) * unit
            self._draw_labeled_rect(
                block_x,
                block_y,
                float(block["w"]) * unit,
                float(block["h"]) * unit,
                str(
                    material.get(
                        "hex",
                        RAW_MATERIAL_COLORS.get(raw_id, "#94a3b8"),
                    )
                ),
                RAW_MATERIAL_TEXT_COLORS.get(raw_id, "#ffffff"),
                str(block.get("label", raw_id)),
                font_size=max(7, min(11, int(unit * 0.65))),
            )

    def _draw_raw_slide_content(
        self,
        slide_id: int,
        x: float,
        y: float,
        unit: float,
    ) -> None:
        # Draw valid pick-position ticks.
        for position in (0, 2, 4):
            tick_x = x + position * unit
            self.monitor_canvas.create_line(
                tick_x,
                y,
                tick_x,
                y + 2 * unit,
                fill="#1e293b",
            )
            self.monitor_canvas.create_text(
                tick_x + 3,
                y + 2,
                text=str(position),
                fill="#475569",
                anchor=tk.NW,
                font=("TkDefaultFont", 6),
            )

        for item in self.cargo.raw_slides.get(slide_id, []):
            raw_id = item.raw_id
            if raw_id not in RAW_MATERIAL_SIZES:
                continue
            raw_w, raw_h = RAW_MATERIAL_SIZES[raw_id]
            if raw_w == 2:
                x_unit = item.position
            else:
                x_unit = item.position - 1
            block_x = x + x_unit * unit
            block_y = y
            self._draw_labeled_rect(
                block_x,
                block_y,
                raw_w * unit,
                raw_h * unit,
                (
                    RETURN_REQUIRED_COLOR
                    if item.return_required
                    else RAW_MATERIAL_COLORS[raw_id]
                ),
                (
                    "#ffffff"
                    if item.return_required
                    else RAW_MATERIAL_TEXT_COLORS[raw_id]
                ),
                str(raw_id),
                font_size=9,
                outline=(
                    "#e9d5ff"
                    if item.return_required
                    else "#020617"
                ),
            )

    def _item_side(self, item: dict) -> str:
        """Infer side for layout items that may not explicitly contain side.

        Stations and start areas usually have a `side` field, but zones and walls
        often only have an id such as `side_a`, `center`, `wall_a_*`, or
        `wall_b_*`.  Side-specific rendering uses this function so the opposite
        side is not drawn when `-p side:=a` or `-p side:=b` is selected.
        """
        side = item.get("side")
        if side in {"side_a", "side_b", "shared"}:
            return side

        item_id = str(item.get("id", "")).lower()
        name = str(item.get("name", "")).lower()

        if (
            item_id in {"center", "shared"}
            or item_id.startswith("shared")
            or item_id.startswith("wall_center")
            or "shared" in name
            or int(item.get("station_id", -1)) == 7
        ):
            return "shared"
        if item_id.startswith("side_a") or item_id.startswith("wall_a") or item_id.endswith("_a"):
            return "side_a"
        if item_id.startswith("side_b") or item_id.startswith("wall_b") or item_id.endswith("_b"):
            return "side_b"
        if "side a" in name or name.endswith(" a"):
            return "side_a"
        if "side b" in name or name.endswith(" b"):
            return "side_b"
        return "shared"

    def _is_visible_layout_item(self, item: dict) -> bool:
        return self._is_visible_side(self._item_side(item))

    def _rect_intersects_viewport(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        margin: float = 2.0,
    ) -> bool:
        view_x, view_y, view_width, view_height = self._viewport()
        view_x2 = view_x + view_width
        view_y2 = view_y + view_height
        min_x, max_x = min(x1, x2), max(x1, x2)
        min_y, max_y = min(y1, y2), max(y1, y2)
        return not (
            max_x < view_x - margin
            or min_x > view_x2 + margin
            or max_y < view_y - margin
            or min_y > view_y2 + margin
        )

    def _line_intersects_viewport(self, start: dict, end: dict, margin: float = 2.0) -> bool:
        return self._rect_intersects_viewport(
            float(start["x"]),
            float(start["y"]),
            float(end["x"]),
            float(end["y"]),
            margin=margin,
        )

    def _station_materials(self, station_id: int) -> List[int]:
        lookup_id = int(station_id)
        if lookup_id == 7:
            side_shared_id = 72 if self.selected_side == "side_b" else 71
            if side_shared_id in self.planner_station_materials:
                return list(
                    self.planner_station_materials[side_shared_id]
                )
            if side_shared_id in self.task.stations:
                return list(
                    self.task.stations[side_shared_id].material_ids
                )
        if lookup_id in self.planner_station_materials:
            return list(self.planner_station_materials[lookup_id])
        state = self.task.stations.get(lookup_id)
        return list(state.material_ids) if state is not None else []

    def _layout_point_for_station(self, station_id: int) -> Optional[Tuple[float, float]]:
        if station_id in (71, 72):
            station_id = 7
        if station_id in (0, 14):
            preferred_id = (
                "start_goal_b"
                if station_id == 14 or self.selected_side == "side_b"
                else "start_goal_a"
            )
            for area in self.layout.data.get("start_areas", []):
                if area.get("id") == preferred_id:
                    center = area["center"]
                    return float(center["x"]), float(center["y"])
            for area in self.layout.data.get("start_areas", []):
                if self._is_visible_layout_item(area):
                    center = area["center"]
                    return float(center["x"]), float(center["y"])
            return None
        station = self.layout.stations_by_id.get(int(station_id))
        if station is None:
            return None
        if int(station_id) not in self.visible_station_ids:
            return None
        center = station["center"]
        return float(center["x"]), float(center["y"])

    def _draw_expected_route(self) -> None:
        if len(self.expected_path) < 2:
            return
        points = [
            self._layout_point_for_station(int(station_id))
            for station_id in self.expected_path
        ]
        next_segment = (
            self.active_segment + 1
            if self.active_segment is not None
            else self.solid_until_segment + 1
        )

        for segment_index, (start, end) in enumerate(zip(points, points[1:])):
            # A station outside the selected arena side must not erase every
            # other visible segment of the expected route.
            if start is None or end is None:
                continue
            sx, sy = self._xy(start[0], start[1])
            ex, ey = self._xy(end[0], end[1])
            if segment_index <= self.solid_until_segment:
                fill = "#22c55e"
                dash = None
                width = 5
                route_tag = "completed_route"
                item_state = (
                    tk.HIDDEN
                    if self.hide_completed_route_var.get()
                    else tk.NORMAL
                )
            elif self.active_segment == segment_index:
                fill = "#fbbf24"
                dash = (8, 4)
                width = 5
                route_tag = "active_route"
                item_state = tk.NORMAL
            elif segment_index == next_segment:
                fill = "#ef4444"
                dash = (8, 4)
                width = 5
                route_tag = "next_route"
                item_state = tk.NORMAL
            else:
                fill = "#94a3b8"
                dash = (5, 5)
                width = 3
                route_tag = "future_route"
                item_state = tk.NORMAL
            self.canvas.create_line(
                sx,
                sy,
                ex,
                ey,
                fill=fill,
                width=width,
                dash=dash,
                arrow=tk.LAST if segment_index == self.active_segment else None,
                capstyle=tk.ROUND,
                state=item_state,
                tags=("expected_route", route_tag),
            )
            mid_x = (sx + ex) / 2.0
            mid_y = (sy + ey) / 2.0
            self.canvas.create_text(
                mid_x,
                mid_y - 8,
                text=str(segment_index + 1),
                fill=fill,
                font=("TkDefaultFont", 7, "bold"),
                state=item_state,
                tags=("expected_route", route_tag),
            )

    def _draw_amr_marker(self) -> None:
        if not isinstance(self.robot_pose, dict):
            return
        x = self._safe_float(self.robot_pose.get("x"))
        y = self._safe_float(self.robot_pose.get("y"))
        if x is None or y is None:
            station_id = self._safe_int(self.robot_pose.get("current_station", self.robot_pose.get("station")))
            if station_id is None:
                return
            point = self._layout_point_for_station(station_id)
            if point is None:
                return
            x, y = point
        cx, cy = self._xy(x, y)
        state = str(self.robot_pose.get("state", self.robot_pose.get("status", ""))).upper()
        fill = ERROR if "ERROR" in state else (SUCCESS if "ARRIVED" in state else ACCENT)
        r = 8
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill=fill, outline="#ffffff", width=2)
        heading = self._safe_float(self.robot_pose.get("heading_deg"))
        if heading is not None:
            rad = math.radians(heading)
            arrow_len = 22
            end_x = cx + math.cos(rad) * arrow_len
            end_y = cy + math.sin(rad) * arrow_len
            self.canvas.create_line(cx, cy, end_x, end_y, fill="#ffffff", width=3, arrow=tk.LAST)
        self.canvas.create_text(
            cx,
            cy - 18,
            text=str(self.robot_pose.get("robot_id", "AMR")),
            fill="#ffffff",
            font=("TkDefaultFont", 8, "bold"),
        )

    def _draw_zones(self) -> None:
        for zone in self.layout.data["zones"]:
            if not self._is_visible_layout_item(zone):
                continue
            bounds = zone["bounds"]
            zone_x1 = float(bounds["x"])
            zone_y1 = float(bounds["y"])
            zone_x2 = zone_x1 + float(bounds["width"])
            zone_y2 = zone_y1 + float(bounds["height"])
            if not self._rect_intersects_viewport(zone_x1, zone_y1, zone_x2, zone_y2):
                continue

            x1, y1 = self._xy(zone_x1, zone_y1)
            x2, y2 = self._xy(zone_x2, zone_y2)
            color = ZONE_COLORS.get(zone["id"], "#1f2937")
            self.canvas.create_rectangle(
                x1,
                y1,
                x2,
                y2,
                fill=color,
                outline="#475569",
                width=1,
            )
            self.canvas.create_text(
                x1 + 5,
                y1 + 5,
                text=zone["name"],
                fill="#7f93ad",
                anchor=tk.NW,
                font=("TkDefaultFont", 8),
            )

    def _draw_walls(self) -> None:
        scale, _, _ = self._transform
        for wall in self.layout.data["walls"]:
            if not self._is_visible_layout_item(wall):
                continue
            if not self._line_intersects_viewport(wall["start"], wall["end"]):
                continue
            start_x, start_y = self._xy(wall["start"]["x"], wall["start"]["y"])
            end_x, end_y = self._xy(wall["end"]["x"], wall["end"]["y"])
            color = "#22d3ee" if wall["type"] == "wall_100cm" else "#c4b5fd"
            self.canvas.create_line(
                start_x,
                start_y,
                end_x,
                end_y,
                fill=color,
                width=max(2, wall.get("thickness", 3) * scale),
                capstyle=tk.ROUND,
            )

    def _draw_start_areas(self) -> None:
        for area in self.layout.data["start_areas"]:
            if not self._is_visible_layout_item(area):
                continue
            center = area["center"]
            size = area["size"]
            data_x1 = center["x"] - size["width"] / 2
            data_y1 = center["y"] - size["height"] / 2
            data_x2 = center["x"] + size["width"] / 2
            data_y2 = center["y"] + size["height"] / 2
            if not self._rect_intersects_viewport(data_x1, data_y1, data_x2, data_y2):
                continue
            x1, y1 = self._xy(data_x1, data_y1)
            x2, y2 = self._xy(data_x2, data_y2)
            self.canvas.create_rectangle(
                x1,
                y1,
                x2,
                y2,
                outline="#ef4444",
                dash=(5, 3),
                width=2,
            )
            self.canvas.create_text(
                (x1 + x2) / 2,
                (y1 + y2) / 2,
                text=area["name"],
                fill="#fca5a5",
                font=("TkDefaultFont", 7, "bold"),
                width=max(30, x2 - x1 - 4),
            )

    def _draw_stations(self) -> None:
        for station in self.layout.data["stations"]:
            station_id = int(station["station_id"])
            if station_id not in self.visible_station_ids:
                continue
            received = self.task.stations.get(station_id)
            color = STATION_COLORS.get(station["type"], "#64748b")
            outline = (
                "#ffffff"
                if station_id == self.selected_station_id
                else SUCCESS if received is not None else "#cbd5e1"
            )
            polygon = self._rotated_rectangle(station)
            tag = f"station_{station_id}"
            self.canvas.create_polygon(
                *polygon,
                fill=color,
                outline=outline,
                width=3 if station_id == self.selected_station_id else 2,
                tags=(tag, "station"),
            )

            center_x, center_y = self._xy(
                station["center"]["x"],
                station["center"]["y"],
            )
            self.canvas.create_text(
                center_x,
                center_y,
                text=str(station_id),
                fill="#ffffff",
                font=("TkDefaultFont", 9, "bold"),
                tags=(tag, "station"),
            )

            label = station["name"].replace("side_a_", "A ").replace("side_b_", "B ")
            label = label.replace("shared_", "Shared ")
            label_x, label_y = self._xy(
                station["label_position"]["x"],
                station["label_position"]["y"],
            )
            self.canvas.create_text(
                label_x,
                label_y,
                text=label,
                fill=TEXT,
                font=("TkDefaultFont", 7),
                tags=(tag, "station"),
            )

            if received is not None and received.material_ids:
                material_text = ",".join(str(value) for value in received.material_ids)
                badge_y = center_y + 15
                badge = self.canvas.create_text(
                    center_x,
                    badge_y,
                    text=f"[{material_text}]",
                    fill="#04111d",
                    font=("TkDefaultFont", 7, "bold"),
                    tags=(tag, "station"),
                )
                bounds = self.canvas.bbox(badge)
                if bounds:
                    background = self.canvas.create_rectangle(
                        bounds[0] - 3,
                        bounds[1] - 1,
                        bounds[2] + 3,
                        bounds[3] + 1,
                        fill="#bae6fd",
                        outline="",
                        tags=(tag, "station"),
                    )
                    self.canvas.tag_lower(background, badge)

            self.canvas.tag_bind(
                tag,
                "<Button-1>",
                lambda _event, value=station_id: self.select_station(value),
            )
            self.canvas.tag_bind(tag, "<Enter>", lambda _event: self.canvas.configure(cursor="hand2"))
            self.canvas.tag_bind(tag, "<Leave>", lambda _event: self.canvas.configure(cursor=""))

    def _rotated_rectangle(self, station: dict) -> List[float]:
        center = station["center"]
        size = station["size"]
        angle = math.radians(float(station.get("rotation_deg", 0)))
        half_width = size["width"] / 2
        half_height = size["height"] / 2
        points: List[float] = []
        for local_x, local_y in (
            (-half_width, -half_height),
            (half_width, -half_height),
            (half_width, half_height),
            (-half_width, half_height),
        ):
            rotated_x = local_x * math.cos(angle) - local_y * math.sin(angle)
            rotated_y = local_x * math.sin(angle) + local_y * math.cos(angle)
            canvas_x, canvas_y = self._xy(
                center["x"] + rotated_x,
                center["y"] + rotated_y,
            )
            points.extend((canvas_x, canvas_y))
        return points

    def select_station(self, station_id: int) -> None:
        if station_id not in self.visible_station_ids:
            return
        self.selected_station_id = station_id
        self._show_station_details(station_id)
        materials = self._station_materials(station_id)
        self._append_monitor_log("STATION", f"Station #{station_id} selected: materials={materials or 'empty'}")
        self.redraw_map()

    def _show_station_details(self, station_id: int) -> None:
        if station_id not in self.visible_station_ids:
            return
        layout_station = self.layout.stations_by_id.get(station_id)
        if layout_station is None:
            return
        state = self.task.stations.get(station_id)
        materials = tuple(self._station_materials(station_id))
        material_lines = self._format_materials(materials)
        live_status = "Planner/Test data received" if station_id in self.planner_station_materials else ("Task data received" if state is not None else "No task data")
        text = (
            f"#{station_id}  {layout_station['name']}\n"
            f"Type: {layout_station['type']}\n"
            f"Side: {layout_station['side']}\n"
            f"Status: {live_status}\n"
            f"Materials: {material_lines}"
        )
        self.station_detail.configure(text=text)

    @staticmethod
    def _format_materials(materials: Iterable[int]) -> str:
        values = list(materials)
        if not values:
            return "empty"
        return ", ".join(
            f"{value} ({OBJECT_NAMES.get(value, 'unknown')})"
            for value in values
        )

    def _viewport(self) -> Tuple[float, float, float, float]:
        if self.selected_side == "all":
            return (0.0, 0.0, self.layout.width, self.layout.height)

        # Prefer zone-based crop so only the selected side + shared 7 column is shown.
        visible_zones = [
            zone
            for zone in self.layout.data.get("zones", [])
            if self._is_visible_layout_item(zone)
        ]
        if visible_zones:
            min_x = min(float(zone["bounds"]["x"]) for zone in visible_zones)
            min_y = min(float(zone["bounds"]["y"]) for zone in visible_zones)
            max_x = max(
                float(zone["bounds"]["x"]) + float(zone["bounds"]["width"])
                for zone in visible_zones
            )
            max_y = max(
                float(zone["bounds"]["y"]) + float(zone["bounds"]["height"])
                for zone in visible_zones
            )
            padding = 8.0
            view_x = max(0.0, min_x - padding)
            view_y = max(0.0, min_y - padding)
            view_x2 = min(self.layout.width, max_x + padding)
            view_y2 = min(self.layout.height, max_y + padding)
            return (view_x, view_y, view_x2 - view_x, view_y2 - view_y)

        if self.selected_side in SIDE_VIEWPORTS:
            return SIDE_VIEWPORTS[self.selected_side]
        return (0.0, 0.0, self.layout.width, self.layout.height)

    def _is_visible_side(self, item_side: str) -> bool:
        if self.selected_side == "all":
            return True
        return item_side in {self.selected_side, "shared"}

    def _side_title(self) -> str:
        return {
            "side_a": "SIDE A",
            "side_b": "SIDE B",
            "all": "FULL ARENA",
        }[self.selected_side]


def default_layout_path() -> Path:
    if WORKSPACE_LAYOUT_PATH.is_file():
        return WORKSPACE_LAYOUT_PATH
    share = Path(get_package_share_directory(PACKAGE_NAME))
    return share / "config" / "sml_worldcup_2026_layout.json"


def default_object_asset_path() -> Path:
    source_path = SOURCE_CONFIG_DIRECTORY / "sml_object_id_gui_assets_6x8.json"
    if source_path.is_file():
        return source_path
    share = Path(get_package_share_directory(PACKAGE_NAME))
    return share / "config" / "sml_object_id_gui_assets_6x8.json"


def normalize_side(value: str) -> str:
    normalized = SIDE_ALIASES.get(value.strip().lower())
    if normalized is None:
        valid = ", ".join(("a", "b", "all"))
        raise ValueError(f"Unsupported side '{value}'. Valid values: {valid}")
    return normalized


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskListenerNode()
    root: Optional[tk.Tk] = None

    try:
        configured_path = str(node.get_parameter("layout_file").value).strip()
        layout_path = (
            Path(configured_path).expanduser()
            if configured_path
            else default_layout_path()
        )
        layout = LayoutModel(layout_path)
        configured_asset_path = str(
            node.get_parameter("object_asset_file").value
        ).strip()
        object_asset_path = (
            Path(configured_asset_path).expanduser()
            if configured_asset_path
            else default_object_asset_path()
        )
        object_assets = ObjectAssetModel(object_asset_path)

        root = tk.Tk()
        topic_name = str(node.get_parameter("topic_name").value)
        selected_side = normalize_side(str(node.get_parameter("side").value))
        refresh_ms = max(10, int(node.get_parameter("refresh_ms").value))
        manual_order_mode = bool(
            node.get_parameter("manual_order_mode").value
        )
        auto_tier = str(node.get_parameter("auto_tier").value)
        layout_config = WindowLayoutConfig(
            geometry=str(node.get_parameter("window_geometry").value),
            min_width=int(node.get_parameter("window_min_width").value),
            min_height=int(node.get_parameter("window_min_height").value),
            main_panel_min_width=int(node.get_parameter("main_panel_min_width").value),
            info_panel_width=int(node.get_parameter("info_panel_width").value),
            info_panel_min_width=int(node.get_parameter("info_panel_min_width").value),
            station_panel_min_width=int(node.get_parameter("station_panel_min_width").value),
            arena_panel_min_width=int(node.get_parameter("arena_panel_min_width").value),
            cargo_panel_min_width=int(node.get_parameter("cargo_panel_min_width").value),
        )
        gui = WorldCupGui(
            root,
            layout,
            object_assets,
            topic_name,
            selected_side,
            layout_config,
            manual_order_mode=manual_order_mode,
            auto_tier=auto_tier,
        )
        node.set_task_callback(gui.update_task)
        if manual_order_mode:
            node.set_template_callback(gui.update_task_template)
            gui.set_manual_task_publisher(node.publish_manual_task)
            root.after(350, gui.open_order_dialog)
        node.set_monitor_callbacks(gui.update_planner_state, gui.update_manager_status, gui.update_location_status)
        node.set_system_callbacks(gui.update_parameter_event, gui.update_rosout)
        node.set_connection_callback(gui.update_connection_indicators)

        closed = False

        def close() -> None:
            nonlocal closed
            if closed:
                return
            closed = True
            root.destroy()

        def pump_ros() -> None:
            if closed:
                return
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except Exception as error:  # Keep Tk alive long enough to show the error.
                messagebox.showerror("ROS 2 error", str(error), parent=root)
                close()
                return
            root.after(refresh_ms, pump_ros)

        root.protocol("WM_DELETE_WINDOW", close)
        root.after(refresh_ms, pump_ros)
        root.mainloop()
    except Exception as error:
        if root is not None:
            messagebox.showerror("SML GUI error", str(error), parent=root)
        else:
            node.get_logger().error(str(error))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
