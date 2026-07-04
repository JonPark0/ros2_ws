"""Task time estimator for the integrated RoboCup planner.

The current robocup_planner node combines the old planning and manager roles:
it receives /sml/task, builds an internal Plan, and immediately executes it.
This estimator listens to the same Task and performs a dry-run of that planner
logic without sending robot commands.  It estimates duration by combining:

- Nav2 global path length for each station waypoint segment, when available.
- robocup_waypoint.yaml fallback distances when Nav2 is not available.
- fixed/parameterized vision, robot arm, workbench, backup, and rotation timings.

It also publishes RViz markers and a nav_msgs/Path showing the estimated route.
"""

from __future__ import annotations

import json
import math
import threading
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path as NavPath
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

from robocup_pkg.msg import Step
from sml_messages.msg import Station, Task

from sml_system_pkg.arena_side_utils import (
    normalize_side,
    side_to_fixed_workbench_station,
    side_to_start_goal_station,
)
from sml_system_pkg.planning.planner_config import PRODUCT_MATERIALS
from sml_system_pkg.planning.waypoint_cost import (
    WaypointPose,
    load_waypoint_cost_map,
)

from robocup_planner.execution.cargo_state import CargoManager
from robocup_planner.planning.aidlist_builder import compute_net_aidlist
from robocup_planner.planning.cargo_allocator import CargoAllocator
from robocup_planner.planning.distance_calculator import DistanceCalculator
from robocup_planner.planning.midlist_builder import (
    build_bidlist,
    build_full_midlist,
    build_mid,
    build_storage_midlist,
    check_storage_satisfies,
    merge_into_midlist,
)
from robocup_planner.execution.executor import Plan
from robocup_planner.product_catalog import (
    BATCH_COUNT,
    BATCH_TO_MATERIAL,
    PRODUCTS,
    get_build_order,
    get_material_count,
)


# ---------------------------------------------------------------------------
# Time-estimation defaults
# ---------------------------------------------------------------------------
# These values are intentionally gathered near the top so they can be tuned
# quickly during RoboCup practice.  Every value below is also exposed as a ROS
# parameter, so launch files or command-line overrides can change them without
# editing code.
DEFAULT_AVG_LINEAR_SPEED_MPS = 0.30
DEFAULT_AVG_ANGULAR_SPEED_RADPS = 0.15
DEFAULT_BACKUP_DISTANCE_M = 0.20
DEFAULT_BACKUP_SPEED_MPS = 0.14
DEFAULT_FRONT_ALIGN_TIME_SEC = 2.0
DEFAULT_NAV_SEGMENT_OVERHEAD_SEC = 0.4
DEFAULT_BACKUP_COMMAND_OVERHEAD_SEC = 0.2
DEFAULT_ROTATE_COMMAND_OVERHEAD_SEC = 0.2

# Vision is counted once per object attempt.  If two objects are picked at the
# same station, the estimator adds this twice.
DEFAULT_VISION_DETECT_TIME_SEC = 6.0
DEFAULT_VISION_RETRY_PROBABILITY = 0.0
DEFAULT_VISION_RETRY_EXTRA_TIME_SEC = 0.0

# Robot-arm time is counted per object for LOAD/UNLOAD.
DEFAULT_LOAD_ARM_TIME_SEC_PER_ITEM = 5.0
DEFAULT_UNLOAD_ARM_TIME_SEC_PER_ITEM = 5.0
DEFAULT_ASSEMBLE_TIME_SEC_PER_CONNECTION = 10.0
DEFAULT_WB_PRODUCE_TIME_SEC_PER_CONNECTION = 10.0
DEFAULT_WB_RECYCLE_TIME_SEC_PER_CONNECTION = 10.0



TASK_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

@dataclass
class SegmentEstimate:
    start_name: str
    goal_name: str
    distance_m: float
    source: str
    poses: List[PoseStamped] = field(default_factory=list)


@dataclass
class StepEstimate:
    step_id: int
    step_type: str
    action: str
    station_id: int
    object_ids: List[int]
    from_station_id: Optional[int] = None
    depends_on: List[int] = field(default_factory=list)
    nav_distance_m: float = 0.0
    nav_time_sec: float = 0.0
    align_time_sec: float = 0.0
    vision_time_sec: float = 0.0
    arm_time_sec: float = 0.0
    wb_time_sec: float = 0.0
    post_time_sec: float = 0.0
    total_sec: float = 0.0
    schedule_start_sec: float = 0.0
    schedule_end_sec: float = 0.0
    notes: List[str] = field(default_factory=list)
    segments: List[SegmentEstimate] = field(default_factory=list)


class SmlTimeEstimatorNode(Node):
    """Estimate task duration from planner steps and Nav2 paths."""

    def __init__(self):
        super().__init__('sml_time_estimator_node')
        self.cbg = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._plan_requested = False
        self._plan_timer = None
        self._last_task_signature = None

        self.declare_parameter('task_topic', '/sml/task')
        self.declare_parameter('side', 'a')
        self.declare_parameter('waypoint_yaml_path', self._default_waypoint_yaml())
        self.declare_parameter(
            'rotation_profiles_path',
            self._default_rotation_profiles_yaml(),
        )
        self.declare_parameter('map_yaml_path', self._default_map_yaml())
        self.declare_parameter('nav2_params_path', self._default_nav2_params_yaml())
        self.declare_parameter('compute_path_action_name', 'compute_path_to_pose')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('avg_linear_speed_mps', DEFAULT_AVG_LINEAR_SPEED_MPS)
        self.declare_parameter('avg_angular_speed_radps', DEFAULT_AVG_ANGULAR_SPEED_RADPS)
        self.declare_parameter('backup_distance_m', DEFAULT_BACKUP_DISTANCE_M)
        self.declare_parameter('backup_speed_mps', DEFAULT_BACKUP_SPEED_MPS)
        self.declare_parameter('front_align_time_sec', DEFAULT_FRONT_ALIGN_TIME_SEC)
        self.declare_parameter('nav_segment_overhead_sec', DEFAULT_NAV_SEGMENT_OVERHEAD_SEC)
        self.declare_parameter('backup_command_overhead_sec', DEFAULT_BACKUP_COMMAND_OVERHEAD_SEC)
        self.declare_parameter('rotate_command_overhead_sec', DEFAULT_ROTATE_COMMAND_OVERHEAD_SEC)
        self.declare_parameter('use_nav2_compute_path', True)
        self.declare_parameter('path_server_timeout_sec', 0.5)
        self.declare_parameter('path_result_timeout_sec', 2.0)
        self.declare_parameter('ignore_duplicate_tasks', True)
        self.declare_parameter('log_segment_details', False)
        self.declare_parameter('return_home_after_plan', True)

        # Unknowns / tunables.  These are deliberately conservative defaults
        # until measured values are available from the real AMR.
        self.declare_parameter('vision_detect_time_sec', DEFAULT_VISION_DETECT_TIME_SEC)
        self.declare_parameter('vision_retry_probability', DEFAULT_VISION_RETRY_PROBABILITY)
        self.declare_parameter('vision_retry_extra_time_sec', DEFAULT_VISION_RETRY_EXTRA_TIME_SEC)
        self.declare_parameter('load_arm_time_sec_per_item', DEFAULT_LOAD_ARM_TIME_SEC_PER_ITEM)
        self.declare_parameter('unload_arm_time_sec_per_item', DEFAULT_UNLOAD_ARM_TIME_SEC_PER_ITEM)
        self.declare_parameter('assemble_time_sec_per_connection', DEFAULT_ASSEMBLE_TIME_SEC_PER_CONNECTION)
        self.declare_parameter('wb_produce_time_sec_per_connection', DEFAULT_WB_PRODUCE_TIME_SEC_PER_CONNECTION)
        self.declare_parameter('wb_recycle_time_sec_per_connection', DEFAULT_WB_RECYCLE_TIME_SEC_PER_CONNECTION)
        # Per-workbench-number time overrides. JSON string keyed by the
        # numbered workbench station_id, e.g. '{"4": {"produce": 8.0,
        # "recycle": 9.0}}' — a station not listed falls back to the
        # global wb_produce/recycle_time_sec_per_connection above. Lets
        # different physical workbenches (different arm/gripper speed) be
        # modeled without changing the global defaults.
        self.declare_parameter('wb_time_sec_per_connection_by_station_json', '')
        # Route-ordering weight and Bezier-curve distance model — mirrors
        # robocup_planner_node so this dry-run estimate reflects the same
        # planned route (see planning/distance_calculator.py and
        # planning/midlist_builder.py for details).
        self.declare_parameter('station_priority_weights_json', '')
        self.declare_parameter('bezier_approach_line_length', 0.30)
        self.declare_parameter('bezier_curve_samples', 16)

        self.side = normalize_side(self.get_parameter('side').value)
        self.fixed_workbench_station = side_to_fixed_workbench_station(self.side)
        self.start_station_id = side_to_start_goal_station(self.side)
        self.frame_id = str(self.get_parameter('frame_id').value)

        waypoint_path = self.get_parameter('waypoint_yaml_path').value
        self.waypoint_map = load_waypoint_cost_map(waypoint_path, self.get_logger())
        self.rotation_profiles = self._load_rotation_profiles(
            self.get_parameter('rotation_profiles_path').value
        )
        self.map_info = self._load_map_info(self.get_parameter('map_yaml_path').value)
        self.nav2_info = self._load_nav2_info(self.get_parameter('nav2_params_path').value)

        self._station_weights = self._load_json_id_float_map(
            'station_priority_weights_json', 'station priority weights'
        )
        self._wb_time_overrides = self._load_wb_time_overrides(
            'wb_time_sec_per_connection_by_station_json'
        )

        self._distance_calc = None
        try:
            self._distance_calc = DistanceCalculator(
                waypoint_path,
                approach_line_length=self._p('bezier_approach_line_length'),
                bezier_samples=int(self.get_parameter('bezier_curve_samples').value),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'[TIME EST] DistanceCalculator load failed: {exc}; planning order may be less accurate'
            )

        self.task_sub = self.create_subscription(
            Task,
            self.get_parameter('task_topic').value,
            self._task_callback,
            TASK_QOS,
            callback_group=self.cbg,
        )
        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            self.get_parameter('compute_path_action_name').value,
            callback_group=self.cbg,
        )

        self.summary_pub = self.create_publisher(String, '/sml/time_estimate', 10)
        self.path_pub = self.create_publisher(
            NavPath,
            '/sml/time_estimator/path',
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/sml/time_estimator/markers',
            10,
        )

        self.get_logger().info(
            '[TIME EST] ready | '
            f'side={self.side}, start={self.start_station_id}, '
            f'waypoints={waypoint_path}, '
            f'avg_linear={self._p("avg_linear_speed_mps"):.2f}m/s, '
            f'avg_angular={self._p("avg_angular_speed_radps"):.2f}rad/s, '
            f'bezier_approach_line={self._p("bezier_approach_line_length"):.2f}m, '
            f'station_priority_weights={self._station_weights}, '
            f'wb_time_overrides={self._wb_time_overrides}'
        )
        self._log_navigation_assumptions()

    # ------------------------------------------------------------------
    # Defaults / parameters
    # ------------------------------------------------------------------

    def _default_waypoint_yaml(self) -> str:
        return self._default_param_file(
            'robocup_waypoint.yaml',
            package='robocup_navigator',
        )

    def _default_rotation_profiles_yaml(self) -> str:
        return self._default_param_file(
            'robocup_rotation_profiles.yaml',
            package='robocup_navigator',
        )

    def _default_map_yaml(self) -> str:
        return self._find_workspace_file('amr/map/robocup_map.yaml')

    def _default_nav2_params_yaml(self) -> str:
        return self._find_workspace_file('amr/params/nav2_params.yaml')

    def _default_param_file(self, filename: str, package: str) -> str:
        rel_path = f'{package}/params/{filename}'
        source_path = Path(self._find_workspace_file(rel_path))
        if source_path.exists():
            return str(source_path)
        try:
            share_path = Path(get_package_share_directory(package)) / 'params' / filename
            return str(share_path)
        except Exception:
            return str(source_path)

    def _find_workspace_file(self, relative_path: str) -> str:
        """Find a source file in common RoboCup workspace layouts."""
        candidates = [
            Path.home() / 'cup_ws/src' / relative_path,
            Path.home() / 'robocup_ws/src' / relative_path,
            Path.home() / 'ros2_ws/src' / relative_path,
            Path('/home/st02/cup_ws/src') / relative_path,
            Path('/home/st02/robocup_ws/src') / relative_path,
            Path('/home/st02/ros2_ws/src') / relative_path,
        ]
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        return str(candidates[0])

    def _p(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _load_json_id_float_map(self, param_name: str, label: str) -> Dict[int, float]:
        """Parse a '{"<id>": <float>, ...}' JSON parameter into {int: float}."""
        raw = str(self.get_parameter(param_name).value or '').strip()
        if not raw:
            return {}
        try:
            parsed = json.loads(raw)
            result = {int(k): float(v) for k, v in parsed.items()}
            self.get_logger().info(f'[TIME EST] {label} loaded: {result}')
            return result
        except Exception as exc:
            self.get_logger().warn(
                f'[TIME EST] {param_name} parse failed: {exc}; using defaults'
            )
            return {}

    def _load_wb_time_overrides(self, param_name: str) -> Dict[int, Dict[str, float]]:
        """Parse '{"<station_id>": {"produce": s, "recycle": s}}' overrides."""
        raw = str(self.get_parameter(param_name).value or '').strip()
        if not raw:
            return {}
        try:
            parsed = json.loads(raw)
            result: Dict[int, Dict[str, float]] = {}
            for station_id, entry in parsed.items():
                if not isinstance(entry, dict):
                    continue
                result[int(station_id)] = {
                    str(k).lower(): float(v) for k, v in entry.items()
                }
            self.get_logger().info(f'[TIME EST] per-workbench time overrides loaded: {result}')
            return result
        except Exception as exc:
            self.get_logger().warn(
                f'[TIME EST] {param_name} parse failed: {exc}; using global defaults'
            )
            return {}

    # ------------------------------------------------------------------
    # Task -> plan -> estimate
    # ------------------------------------------------------------------

    def _task_callback(self, msg: Task):
        signature = self._task_signature(msg)
        with self._lock:
            if (
                bool(self.get_parameter('ignore_duplicate_tasks').value)
                and self._last_task_signature == signature
            ):
                return
            if self._plan_requested:
                return
            self._last_task_signature = signature
            self._plan_requested = True

        self.get_logger().info(
            '[TIME EST] task received: '
            f'orders={len(msg.order_list)}, stations={len(msg.arena_layout)} '
            '-> robocup_planner dry-run estimate'
        )

        try:
            plan = self._build_robocup_plan(msg)
            estimates = self._estimate_robocup_plan(plan)
        except Exception as exc:
            self.get_logger().error(f'[TIME EST] estimate failed: {exc}')
            with self._lock:
                self._plan_requested = False
            return

        self.get_logger().info(
            f'[TIME EST] robocup plan estimated: pickups={len(plan.mid)}, '
            f'workbench_products={plan.workbench_products}, '
            f'intransit_products={plan.intransit_products}'
        )
        self._log_estimates(estimates)
        self._publish_summary(estimates)
        self._publish_visualization(estimates)

        with self._lock:
            self._plan_requested = False

    def _task_signature(self, msg: Task):
        orders = tuple(
            (int(o.order_type), int(o.product_id))
            for o in msg.order_list
        )
        stations = tuple(
            (
                int(s.station_id),
                int(s.station_type),
                tuple(int(x) for x in s.material_ids),
            )
            for s in msg.arena_layout
        )
        return orders, stations

    def _build_robocup_plan(self, msg: Task) -> Plan:
        produce_ids = [int(o.product_id) for o in msg.order_list if int(o.order_type) == 1]
        recycle_ids = [int(o.product_id) for o in msg.order_list if int(o.order_type) == 2]

        storage_stations = []
        batch_stations_1080 = []
        batch_stations_90 = []
        workbench_station_id = None
        customer_station_id = None

        for st in msg.arena_layout:
            station_id = int(st.station_id)
            station_type = int(st.station_type)
            if station_type in (int(Station.ST_STORAGE), int(Station.ST_HYBRID)):
                mids = [int(m) for m in st.material_ids]
                regular = [m for m in mids if 1 <= m <= 8]
                b1080 = [m for m in mids if 10 <= m <= 80]
                b90 = [m for m in mids if m == 90]
                if regular:
                    storage_stations.append({
                        'station_id': station_id,
                        'material_ids': regular,
                    })
                if b1080:
                    batch_stations_1080.append({
                        'station_id': station_id,
                        'batch_ids': b1080,
                    })
                if b90:
                    batch_stations_90.append({
                        'station_id': station_id,
                        'batch_ids': [90],
                    })
            if station_type in (int(Station.ST_WORKBENCH), int(Station.ST_HYBRID)):
                if workbench_station_id is None:
                    workbench_station_id = station_id
            if station_type == int(Station.ST_CUSTOMER):
                customer_station_id = station_id

        if workbench_station_id is None:
            workbench_station_id = int(self.fixed_workbench_station)
            self.get_logger().warn(
                f'[TIME EST] no workbench station in task; fallback S{workbench_station_id}'
            )
        if customer_station_id is None:
            raise RuntimeError('No customer station in arena layout')

        home_id = int(self.start_station_id)
        aidlist, net_aidlist, recycled_materials = compute_net_aidlist(
            produce_ids, recycle_ids
        )

        if self._distance_calc:
            storage_mid = build_storage_midlist(
                storage_stations, self._distance_calc, home_id, self._station_weights
            )
        else:
            storage_mid = [
                {
                    'station_id': s['station_id'],
                    'materials': list(s['material_ids']),
                    'distance': 0.0,
                    'is_recycle_pickup': False,
                    'recycle_product_id': None,
                }
                for s in storage_stations
            ]

        satisfied, missing = check_storage_satisfies(storage_mid, net_aidlist)
        use_batch_1080 = False
        if not satisfied and batch_stations_1080:
            use_batch_1080 = True
            if self._distance_calc:
                bidlist_1080 = build_bidlist(
                    batch_stations_1080, self._distance_calc, home_id,
                    station_weights=self._station_weights,
                )
            else:
                bidlist_1080 = [
                    {
                        'station_id': bst['station_id'],
                        'materials': [
                            BATCH_TO_MATERIAL[bid]
                            for bid in bst['batch_ids']
                            for _ in range(BATCH_COUNT)
                        ],
                        'distance': 0.0,
                        'is_recycle_pickup': False,
                        'recycle_product_id': None,
                        'is_batch': True,
                        'is_mix_batch': False,
                    }
                    for bst in batch_stations_1080
                ]
            merged_check = merge_into_midlist(storage_mid, bidlist_1080)
            satisfied, missing = check_storage_satisfies(merged_check, net_aidlist)

        missing_for_mix = missing if (not satisfied and batch_stations_90) else None
        needs_recycling = bool(recycle_ids)
        recycle_orders = [
            {'station_id': customer_station_id, 'product_id': pid}
            for pid in recycle_ids
        ]

        if self._distance_calc:
            full_midlist = build_full_midlist(
                storage_stations=storage_stations,
                customer_stations=[{'station_id': customer_station_id}],
                recycle_orders=recycle_orders,
                calc=self._distance_calc,
                home_station_id=home_id,
                workbench_station_id=workbench_station_id,
                needs_recycling=needs_recycling,
                batch_stations_1080=batch_stations_1080 if use_batch_1080 else None,
                batch_stations_90=batch_stations_90 if missing_for_mix else None,
                missing_for_mix=missing_for_mix,
                station_weights=self._station_weights,
            )
        else:
            full_midlist = [
                {
                    'station_id': o['station_id'],
                    'materials': [],
                    'distance': 0.0,
                    'is_recycle_pickup': True,
                    'recycle_product_id': o['product_id'],
                }
                for o in recycle_orders
            ] + storage_mid

        mid = build_mid(full_midlist, net_aidlist)

        temp_alloc = CargoAllocator()
        intransit_allocated = temp_alloc.allocate(produce_ids)
        intransit_ids = list(intransit_allocated.keys())
        workbench_ids = [pid for pid in produce_ids if pid not in intransit_ids]

        surplus = {}
        for mat, cnt in recycled_materials.items():
            extra = cnt - aidlist.get(mat, 0)
            if extra > 0:
                surplus[mat] = extra

        return Plan(
            mid=mid,
            workbench_products=workbench_ids,
            intransit_products=intransit_ids,
            workbench_station_id=int(workbench_station_id),
            customer_station_id=int(customer_station_id),
            home_station_id=home_id,
            surplus_recycled=surplus,
        )

    def _estimate_robocup_plan(self, plan: Plan) -> List[StepEstimate]:
        estimates: List[StepEstimate] = []
        state = {
            'station_id': int(plan.home_station_id),
            'pose': self._station_departure_pose(int(plan.home_station_id)),
            'next_step_id': 0,
        }
        cargo = CargoManager()
        allocator = CargoAllocator()
        allocator.allocate(plan.intransit_products)
        pending_wb: List[int] = list(plan.workbench_products)
        pending_deliveries: List[int] = []
        cargo1_queue: List[int] = []
        en_route_to_wb = False

        def add_amr(station_id: int, action: str, object_ids: List[int], note: str = '') -> StepEstimate:
            step_id = state['next_step_id']
            state['next_step_id'] += 1
            estimate = self._estimate_virtual_amr_step(
                step_id=step_id,
                from_station_id=int(state['station_id']),
                current_pose=state['pose'],
                station_id=int(station_id),
                action=action,
                object_ids=[int(x) for x in object_ids],
            )
            if step_id > 0:
                estimate.depends_on = [step_id - 1]
            if note:
                estimate.notes.append(note)
            estimates.append(estimate)
            state['station_id'] = int(station_id)
            state['pose'] = self._station_departure_pose(int(station_id))
            return estimate

        def add_wb(action: str, product_id: int) -> StepEstimate:
            step_id = state['next_step_id']
            state['next_step_id'] += 1
            estimate = StepEstimate(
                step_id=step_id,
                step_type='WB',
                action=action,
                station_id=int(plan.workbench_station_id),
                object_ids=[int(product_id)],
                from_station_id=None,
                depends_on=[step_id - 1] if step_id > 0 else [],
            )
            estimate.wb_time_sec = self._workbench_time_for_product(
                int(product_id), action, int(plan.workbench_station_id)
            )
            estimate.total_sec = estimate.wb_time_sec
            estimates.append(estimate)
            return estimate

        def find_materials_for_product(product_id: int):
            needed = Counter(get_material_count(int(product_id)))
            result = []
            used_indices = set()
            available = list(cargo.all_materials())
            for mat_id, count in list(needed.items()):
                for _ in range(count):
                    found = False
                    for idx, (cargo_id, cargo_mat) in enumerate(available):
                        if idx in used_indices:
                            continue
                        if int(cargo_mat) == int(mat_id):
                            used_indices.add(idx)
                            result.append((cargo_id, cargo_mat))
                            found = True
                            break
                    if not found:
                        return None
            return result

        def ready_workbench_product() -> Optional[int]:
            for product_id in pending_wb:
                if find_materials_for_product(int(product_id)) is not None:
                    return int(product_id)
            return None

        def should_go_workbench() -> Optional[int]:
            if en_route_to_wb:
                return None
            return ready_workbench_product()

        def divert_to_workbench(forced: bool = False):
            nonlocal en_route_to_wb
            if en_route_to_wb and not forced:
                return
            en_route_to_wb = True
            add_amr(
                int(plan.workbench_station_id),
                'GO_WORKBENCH',
                [],
                note='workbench detour',
            )
            ready_pid = ready_workbench_product()
            if ready_pid is not None:
                slots = find_materials_for_product(ready_pid) or []
                unload_materials = [mat_id for _, mat_id in slots]
                if unload_materials:
                    add_amr(
                        int(plan.workbench_station_id),
                        'UNLOAD',
                        unload_materials,
                        note=f'unload materials for product {ready_pid}',
                    )
                    for cargo_id, mat_id in slots:
                        cargo.remove_material(cargo_id, mat_id)
                add_wb('PRODUCE', int(ready_pid))
                pending_wb.remove(ready_pid)
                cargo.add_finished_product()
                cargo1_queue.append(int(ready_pid))
                pending_deliveries.append(int(ready_pid))
            elif forced:
                unload_materials = [mat_id for _, mat_id in cargo.all_materials()]
                if unload_materials:
                    add_amr(
                        int(plan.workbench_station_id),
                        'UNLOAD',
                        unload_materials,
                        note='overflow drop to workbench',
                    )
                    for cargo_id, mat_id in list(cargo.all_materials()):
                        cargo.remove_material(cargo_id, mat_id)
            en_route_to_wb = False

        def deliver_all():
            if not pending_deliveries and not allocator.get_completed_slots():
                return
            products = list(pending_deliveries)
            for slot in allocator.get_completed_slots():
                if slot.product_id not in products:
                    products.append(int(slot.product_id))
            if products:
                add_amr(
                    int(plan.customer_station_id),
                    'UNLOAD',
                    products,
                    note='deliver products to customer',
                )
            while cargo.finished_on_cargo1 > 0:
                if cargo1_queue:
                    cargo1_queue.pop(0)
                cargo.consume_finished_product()
            pending_deliveries.clear()
            for slot in list(allocator.get_completed_slots()):
                allocator.free_slot(slot.cargo_id)

        recycle_entries = [e for e in plan.mid if e.get('is_recycle_pickup')]
        for entry in recycle_entries:
            pid = int(entry['recycle_product_id'])
            add_amr(int(entry['station_id']), 'LOAD', [pid], note='recycle pickup')
        if recycle_entries:
            add_amr(int(plan.workbench_station_id), 'GO_WORKBENCH', [], note='recycle products to workbench')
            for entry in recycle_entries:
                pid = int(entry['recycle_product_id'])
                add_wb('RECYCLE', pid)
                for mat_id, count in get_material_count(pid).items():
                    for _ in range(count):
                        cargo.place_material(int(mat_id))

        ready = should_go_workbench()
        if ready is not None:
            divert_to_workbench()
        deliver_all()

        storage_entries = [e for e in plan.mid if not e.get('is_recycle_pickup')]
        for entry in storage_entries:
            station_id = int(entry['station_id'])
            pickup_materials = [int(x) for x in entry.get('pickup_materials', [])]
            if not pickup_materials:
                continue
            add_amr(station_id, 'LOAD', pickup_materials, note='storage pickup')

            for mat_id in pickup_materials:
                cargo_id = allocator.find_slot_for_block(mat_id)
                if cargo_id is None:
                    cargo_id = cargo.place_material(mat_id)
                if cargo_id is None:
                    divert_to_workbench(forced=True)
                    add_amr(station_id, 'LOAD', [mat_id], note='resume pickup after overflow detour')
                    cargo_id = allocator.find_slot_for_block(mat_id)
                    if cargo_id is None:
                        cargo_id = cargo.place_material(mat_id)
                    if cargo_id is None:
                        self.get_logger().warn(
                            f'[TIME EST] no cargo space for material {mat_id}; skipped in dry-run state'
                        )
                        continue
                if cargo_id in (7, 8):
                    complete = allocator.confirm_placed(cargo_id, mat_id)
                    if complete:
                        slot_products = [
                            slot.product_id
                            for slot in allocator.get_completed_slots()
                            if slot.cargo_id == cargo_id
                        ]
                        for product_id in slot_products:
                            if int(product_id) not in pending_deliveries:
                                pending_deliveries.append(int(product_id))

            ready = should_go_workbench()
            if ready is not None:
                divert_to_workbench()
            deliver_all()

        deliver_all()
        if bool(self.get_parameter('return_home_after_plan').value):
            add_amr(int(plan.home_station_id), 'GOAL', [], note='return home')

        self._apply_resource_schedule(estimates)
        return estimates

    def _estimate_virtual_amr_step(
        self,
        step_id: int,
        from_station_id: int,
        current_pose: Optional[WaypointPose],
        station_id: int,
        action: str,
        object_ids: List[int],
    ) -> StepEstimate:
        estimate = StepEstimate(
            step_id=int(step_id),
            step_type='AMR',
            action=str(action),
            station_id=int(station_id),
            object_ids=[int(x) for x in object_ids],
            from_station_id=int(from_station_id),
        )
        segments = self._estimate_station_path(current_pose, int(station_id))
        estimate.segments = segments
        estimate.nav_distance_m = sum(s.distance_m for s in segments)
        estimate.nav_time_sec = (
            estimate.nav_distance_m / max(self._p('avg_linear_speed_mps'), 1e-6)
            + len(segments) * self._p('nav_segment_overhead_sec')
        )

        if int(station_id) not in (0, 9):
            estimate.align_time_sec = self._p('front_align_time_sec')
            estimate.post_time_sec = self._post_process_time(int(station_id))
        else:
            estimate.notes.append('start/goal station: no backup/rotation')

        if action == 'LOAD':
            estimate.vision_time_sec = len(object_ids) * self._vision_time_per_object()
            estimate.arm_time_sec = len(object_ids) * self._p('load_arm_time_sec_per_item')
        elif action == 'UNLOAD':
            estimate.arm_time_sec = len(object_ids) * self._p('unload_arm_time_sec_per_item')
        elif action in ('GOAL', 'GO_WORKBENCH'):
            pass
        else:
            estimate.notes.append(f'unknown AMR action timing model: {action}')

        estimate.total_sec = (
            estimate.nav_time_sec
            + estimate.align_time_sec
            + estimate.vision_time_sec
            + estimate.arm_time_sec
            + estimate.post_time_sec
        )
        if action == 'GOAL':
            estimate.total_sec = estimate.nav_time_sec
        return estimate

    def _workbench_time_for_product(
        self, product_id: int, action: str, station_id: Optional[int] = None
    ) -> float:
        """Per-connection WB time, optionally overridden per numbered
        workbench station (see wb_time_sec_per_connection_by_station_json)."""
        connections = self._product_connection_count(int(product_id))
        key = 'recycle' if action == 'RECYCLE' else 'produce'
        override = None
        if station_id is not None:
            override = self._wb_time_overrides.get(int(station_id), {}).get(key)
        if override is not None:
            return connections * float(override)
        if action == 'RECYCLE':
            return connections * self._p('wb_recycle_time_sec_per_connection')
        return connections * self._p('wb_produce_time_sec_per_connection')

    def _product_connection_count(self, product_id: int) -> int:
        product = PRODUCTS.get(int(product_id), {})
        if 'layers' in product:
            count = sum(len(layer) for layer in product.get('layers', []))
        elif 'blocks' in product:
            count = len(product.get('blocks', []))
        else:
            try:
                count = len(get_build_order(int(product_id)))
            except Exception:
                count = len(str(product_id))
        return max(0, count - 1)

    # ------------------------------------------------------------------
    # Estimation
    # ------------------------------------------------------------------

    def _estimate_steps(self, steps: Iterable[Step]) -> List[StepEstimate]:
        current_station_id = self.start_station_id
        current_pose = self._station_departure_pose(current_station_id)
        estimates: List[StepEstimate] = []

        for step in steps:
            from_station_id = current_station_id if int(step.type) == int(Step.AMR) else None
            estimate = self._estimate_step(step, current_pose, from_station_id)
            estimates.append(estimate)

            if int(step.type) == int(Step.AMR):
                current_station_id = int(step.station_id)
                current_pose = self._station_departure_pose(current_station_id)

        self._apply_resource_schedule(estimates)
        return estimates

    def _estimate_step(
        self,
        step: Step,
        current_pose: Optional[WaypointPose],
        from_station_id: Optional[int] = None,
    ) -> StepEstimate:
        estimate = StepEstimate(
            step_id=int(step.step_id),
            step_type=self._step_type_name(step),
            action=self._action_name(step),
            station_id=int(step.station_id),
            object_ids=[int(x) for x in step.object_ids],
            from_station_id=from_station_id,
            depends_on=[int(x) for x in step.depends_on],
        )

        if int(step.type) == int(Step.AMR):
            self._estimate_amr_step(step, current_pose, estimate)
        elif int(step.type) == int(Step.WB):
            estimate.wb_time_sec = self._estimate_wb_time(step)
            estimate.total_sec = estimate.wb_time_sec
        return estimate

    def _estimate_amr_step(
        self,
        step: Step,
        current_pose: Optional[WaypointPose],
        estimate: StepEstimate,
    ):
        segments = self._estimate_station_path(current_pose, int(step.station_id))
        estimate.segments = segments
        estimate.nav_distance_m = sum(s.distance_m for s in segments)
        estimate.nav_time_sec = (
            estimate.nav_distance_m / max(self._p('avg_linear_speed_mps'), 1e-6)
            + len(segments) * self._p('nav_segment_overhead_sec')
        )

        station_id = int(step.station_id)
        if station_id not in (0, 9):
            estimate.align_time_sec = self._p('front_align_time_sec')
            estimate.post_time_sec = self._post_process_time(station_id)
        else:
            estimate.notes.append('start/goal station: no backup/rotation')

        if int(step.action) == int(Step.LOAD):
            estimate.vision_time_sec = self._vision_time(step)
            estimate.arm_time_sec = len(step.object_ids) * self._p('load_arm_time_sec_per_item')
            estimate.total_sec = (
                estimate.nav_time_sec
                + estimate.align_time_sec
                + estimate.vision_time_sec
                + estimate.arm_time_sec
                + estimate.post_time_sec
            )
        elif int(step.action) == int(Step.UNLOAD):
            estimate.arm_time_sec = len(step.object_ids) * self._p('unload_arm_time_sec_per_item')
            estimate.total_sec = (
                estimate.nav_time_sec
                + estimate.align_time_sec
                + estimate.arm_time_sec
                + estimate.post_time_sec
            )
        elif int(step.action) == int(Step.PRODUCE):
            estimate.arm_time_sec = self._estimate_assemble_time(step)
            # Manager runs NAV and AMR ASSEMBLE concurrently.
            estimate.total_sec = (
                max(estimate.nav_time_sec + estimate.align_time_sec,
                    estimate.arm_time_sec)
                + estimate.post_time_sec
            )
            estimate.notes.append('NAV and ASSEMBLE run in parallel')
        elif int(step.action) == int(Step.GOAL):
            estimate.total_sec = estimate.nav_time_sec
        else:
            estimate.total_sec = estimate.nav_time_sec + estimate.align_time_sec

    def _estimate_station_path(
        self,
        start_pose: Optional[WaypointPose],
        station_id: int,
    ) -> List[SegmentEstimate]:
        target_poses = self._station_sequence_poses(station_id)
        if start_pose is None or not target_poses:
            return []

        segments: List[SegmentEstimate] = []
        prev = start_pose
        for pose in target_poses:
            segment = self._estimate_segment(prev, pose)
            segments.append(segment)
            prev = pose
        return segments

    def _estimate_segment(
        self,
        start: WaypointPose,
        goal: WaypointPose,
    ) -> SegmentEstimate:
        if bool(self.get_parameter('use_nav2_compute_path').value):
            nav2_segment = self._compute_nav2_segment(start, goal)
            if nav2_segment is not None:
                return nav2_segment

        distance = math.hypot(goal.x - start.x, goal.y - start.y)
        poses = [
            self._pose_stamped_from_waypoint(start),
            self._pose_stamped_from_waypoint(goal),
        ]
        return SegmentEstimate(
            start_name=start.name,
            goal_name=goal.name,
            distance_m=distance,
            source='waypoint_fallback',
            poses=poses,
        )

    def _compute_nav2_segment(
        self,
        start: WaypointPose,
        goal: WaypointPose,
    ) -> Optional[SegmentEstimate]:
        if not self.path_client.wait_for_server(
            timeout_sec=self._p('path_server_timeout_sec')
        ):
            return None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = self._pose_stamped_from_waypoint(start)
        goal_msg.goal = self._pose_stamped_from_waypoint(goal)
        goal_msg.planner_id = str(self.get_parameter('planner_id').value)
        if hasattr(goal_msg, 'use_start'):
            goal_msg.use_start = True

        done = threading.Event()
        state = {'accepted': False, 'result': None, 'exception': None}

        def on_goal_response(future):
            try:
                goal_handle = future.result()
                state['accepted'] = bool(goal_handle.accepted)
                if not goal_handle.accepted:
                    done.set()
                    return
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(on_result)
            except Exception as exc:
                state['exception'] = exc
                done.set()

        def on_result(future):
            try:
                state['result'] = future.result().result
            except Exception as exc:
                state['exception'] = exc
            finally:
                done.set()

        try:
            send_future = self.path_client.send_goal_async(goal_msg)
            send_future.add_done_callback(on_goal_response)
        except Exception as exc:
            self.get_logger().warn(
                f'[TIME EST][NAV] ComputePath send failed: {exc}; fallback distance used'
            )
            return None

        if not done.wait(timeout=self._p('path_result_timeout_sec')):
            self.get_logger().warn(
                f'[TIME EST][NAV] ComputePath timeout {start.name}->{goal.name}; fallback distance used'
            )
            return None

        if state['exception'] is not None or not state['accepted']:
            self.get_logger().warn(
                f'[TIME EST][NAV] ComputePath rejected/error {start.name}->{goal.name}; fallback distance used'
            )
            return None

        result = state['result']
        if result is None or not result.path.poses:
            return None

        distance = self._path_length(result.path.poses)
        return SegmentEstimate(
            start_name=start.name,
            goal_name=goal.name,
            distance_m=distance,
            source='nav2_compute_path',
            poses=list(result.path.poses),
        )

    def _apply_resource_schedule(self, estimates: List[StepEstimate]):
        end_times: Dict[int, float] = {}
        amr_available = 0.0
        wb_available = 0.0

        for estimate in estimates:
            deps_ready = max(
                [end_times.get(dep, 0.0) for dep in estimate.depends_on] or [0.0]
            )

            if estimate.step_type == 'AMR':
                start = max(deps_ready, amr_available)
                if self._is_amr_wb_interaction(estimate):
                    start = max(start, wb_available)
                    wb_available = start + estimate.total_sec
                    estimate.notes.append('reserves WB access during AMR interaction')
                amr_available = start + estimate.total_sec
            elif estimate.step_type == 'WB':
                start = max(deps_ready, wb_available)
                wb_available = start + estimate.total_sec
            else:
                start = deps_ready

            end_times[estimate.step_id] = start + estimate.total_sec
            estimate.schedule_start_sec = start
            estimate.schedule_end_sec = end_times[estimate.step_id]
            estimate.notes.append(
                f'schedule_start={estimate.schedule_start_sec:.2f}s, '
                f'schedule_end={estimate.schedule_end_sec:.2f}s'
            )

    # ------------------------------------------------------------------
    # Timing models
    # ------------------------------------------------------------------

    def _vision_time_per_object(self) -> float:
        return (
            self._p('vision_detect_time_sec')
            + self._p('vision_retry_probability')
            * self._p('vision_retry_extra_time_sec')
        )

    def _vision_time(self, step: Step) -> float:
        return len(step.object_ids) * self._vision_time_per_object()

    def _estimate_assemble_time(self, step: Step) -> float:
        total_connections = 0
        for product_id in step.object_ids:
            materials = PRODUCT_MATERIALS.get(int(product_id), [])
            total_connections += max(0, len(materials) - 1)
        return total_connections * self._p('assemble_time_sec_per_connection')

    def _estimate_wb_time(self, step: Step) -> float:
        if not step.object_ids:
            return 0.0
        product_id = int(step.object_ids[0])
        materials = PRODUCT_MATERIALS.get(product_id, [])
        connections = max(0, len(materials) - 1)
        if int(step.action) == int(Step.RECYCLE):
            return connections * self._p('wb_recycle_time_sec_per_connection')
        return connections * self._p('wb_produce_time_sec_per_connection')

    def _post_process_time(self, station_id: int) -> float:
        backup_time = self._p('backup_distance_m') / max(self._p('backup_speed_mps'), 1e-6)
        backup_time += self._p('backup_command_overhead_sec')
        rotate_profile = self._rotation_profile_for_station(station_id)
        if rotate_profile is None:
            return backup_time
        angle_rad = math.radians(abs(float(rotate_profile.get('angle_deg', 0.0))))
        rotate_time = angle_rad / max(self._p('avg_angular_speed_radps'), 1e-6)
        rotate_time += self._p('rotate_command_overhead_sec')
        return backup_time + rotate_time

    # ------------------------------------------------------------------
    # Waypoint / pose helpers
    # ------------------------------------------------------------------

    def _station_sequence_poses(self, station_id: int) -> List[WaypointPose]:
        if self.waypoint_map is None:
            return []
        return self.waypoint_map.station_sequence_poses(int(station_id))

    def _station_goal_pose(self, station_id: int) -> Optional[WaypointPose]:
        if self.waypoint_map is None:
            return None
        return self.waypoint_map.station_goal_pose(int(station_id))

    def _station_departure_pose(self, station_id: int) -> Optional[WaypointPose]:
        goal_pose = self._station_goal_pose(int(station_id))
        if goal_pose is None:
            return None
        if int(station_id) in (0, 9):
            return goal_pose

        yaw = self._yaw_from_quaternion(goal_pose.qx, goal_pose.qy, goal_pose.qz, goal_pose.qw)
        backup = self._p('backup_distance_m')
        x = goal_pose.x - backup * math.cos(yaw)
        y = goal_pose.y - backup * math.sin(yaw)

        rotate_profile = self._rotation_profile_for_station(int(station_id))
        if rotate_profile is not None:
            direction = str(rotate_profile.get('direction', '')).lower()
            angle = math.radians(float(rotate_profile.get('angle_deg', 0.0)))
            yaw += -abs(angle) if direction == 'clockwise' else abs(angle)

        q = self._quaternion_from_yaw(yaw)
        return WaypointPose(
            name=f'station_{station_id}_departure_est',
            x=x,
            y=y,
            z=goal_pose.z,
            qx=q.x,
            qy=q.y,
            qz=q.z,
            qw=q.w,
        )

    def _pose_stamped_from_waypoint(self, pose: WaypointPose) -> PoseStamped:
        msg = PoseStamped()
        msg.header = Header()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = Pose()
        msg.pose.position.x = float(pose.x)
        msg.pose.position.y = float(pose.y)
        msg.pose.position.z = float(pose.z)
        msg.pose.orientation.x = float(pose.qx)
        msg.pose.orientation.y = float(pose.qy)
        msg.pose.orientation.z = float(pose.qz)
        msg.pose.orientation.w = float(pose.qw)
        return msg

    def _path_length(self, poses: List[PoseStamped]) -> float:
        total = 0.0
        for a, b in zip(poses, poses[1:]):
            total += math.hypot(
                b.pose.position.x - a.pose.position.x,
                b.pose.position.y - a.pose.position.y,
            )
        return total

    def _load_rotation_profiles(self, path: str) -> Dict[str, dict]:
        path = str(path or '').strip()
        if not path:
            return {}
        try:
            with open(Path(path).expanduser(), 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            profiles = data.get('rotation_profiles', {}) if isinstance(data, dict) else {}
            self.get_logger().info(
                f'[TIME EST] rotation profiles loaded: {len(profiles)} from {path}'
            )
            return profiles
        except Exception as exc:
            self.get_logger().warn(
                f'[TIME EST] rotation profile load failed: {exc}; rotation estimate disabled'
            )
            return {}

    def _load_map_info(self, path: str) -> Dict[str, object]:
        path = str(path or '').strip()
        if not path:
            return {}
        try:
            with open(Path(path).expanduser(), 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            return {
                'path': path,
                'image': data.get('image'),
                'resolution': data.get('resolution'),
                'origin': data.get('origin'),
            }
        except Exception as exc:
            self.get_logger().warn(f'[TIME EST] map YAML load failed: {exc}')
            return {'path': path}

    def _load_nav2_info(self, path: str) -> Dict[str, object]:
        path = str(path or '').strip()
        if not path:
            return {}
        try:
            with open(Path(path).expanduser(), 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}

            global_params = (
                data.get('global_costmap', {})
                .get('global_costmap', {})
                .get('ros__parameters', {})
            )
            planner_params = data.get('planner_server', {}).get('ros__parameters', {})
            planner_plugins = planner_params.get('planner_plugins', [])
            planner_plugin = None
            if planner_plugins:
                first = str(planner_plugins[0])
                planner_plugin = planner_params.get(first, {}).get('plugin')

            inflation = global_params.get('inflation_layer', {})
            return {
                'path': path,
                'footprint': global_params.get('footprint'),
                'footprint_padding': global_params.get('footprint_padding'),
                'inflation_radius': inflation.get('inflation_radius'),
                'cost_scaling_factor': inflation.get('cost_scaling_factor'),
                'planner_plugins': planner_plugins,
                'planner_plugin': planner_plugin,
            }
        except Exception as exc:
            self.get_logger().warn(f'[TIME EST] nav2 params load failed: {exc}')
            return {'path': path}

    def _log_navigation_assumptions(self):
        map_path = self.map_info.get('path', self.get_parameter('map_yaml_path').value)
        self.get_logger().info(
            '[TIME EST][MAP] '
            f'path={map_path}, image={self.map_info.get("image")}, '
            f'resolution={self.map_info.get("resolution")}, origin={self.map_info.get("origin")}'
        )
        self.get_logger().info(
            '[TIME EST][NAV2] '
            f'params={self.nav2_info.get("path", self.get_parameter("nav2_params_path").value)}, '
            f'planner={self.nav2_info.get("planner_plugin")}, '
            f'footprint={self.nav2_info.get("footprint")}, '
            f'footprint_padding={self.nav2_info.get("footprint_padding")}, '
            f'inflation_radius={self.nav2_info.get("inflation_radius")}, '
            f'cost_scaling_factor={self.nav2_info.get("cost_scaling_factor")}'
        )
        self.get_logger().info(
            '[TIME EST][MODEL] differential drive is handled by Nav2 controller/runtime; '
            'global path estimate uses Nav2 costmap/planner when compute_path_to_pose is available. '
            'Fallback waypoint distance does not include footprint/inflation obstacles.'
        )
    def _rotation_profile_for_station(self, station_id: int) -> Optional[dict]:
        if self.waypoint_map is None:
            return None
        profile = self.waypoint_map.station_profile(int(station_id))
        if profile is None:
            return None
        for waypoint_name in reversed(profile.sequence):
            if waypoint_name in self.rotation_profiles:
                return self.rotation_profiles[waypoint_name]
        goal_name = f'station_{station_id}_goal'
        return self.rotation_profiles.get(goal_name)

    def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _quaternion_from_yaw(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    # ------------------------------------------------------------------
    # Logging / publishing
    # ------------------------------------------------------------------

    def _log_estimates(self, estimates: List[StepEstimate]):
        self.get_logger().info('===== SML 예상 시간 요약 =====')
        for e in estimates:
            if e.step_type == 'AMR':
                from_station = '-' if e.from_station_id is None else str(e.from_station_id)
                self.get_logger().info(
                    f'[{e.step_id:02d}] [station S{from_station} -> S{e.station_id}] '
                    f'{e.action} objects={e.object_ids}'
                )
                self.get_logger().info(
                    f'     nav total: {e.nav_time_sec:.2f}s, distance={e.nav_distance_m:.2f}m'
                )

                if e.segments:
                    for segment in e.segments:
                        segment_time = (
                            segment.distance_m / max(self._p('avg_linear_speed_mps'), 1e-6)
                            + self._p('nav_segment_overhead_sec')
                        )
                        self.get_logger().info(
                            f'     [{segment.start_name} -> {segment.goal_name}] '
                            f'{segment_time:.2f}s ({segment.distance_m:.2f}m, {segment.source})'
                        )
                else:
                    self.get_logger().info('     [nav segment] none')

                if e.align_time_sec > 0.0:
                    self.get_logger().info(f'     [front align] {e.align_time_sec:.2f}s')
                if e.vision_time_sec > 0.0:
                    self.get_logger().info(
                        f'     [vision] {e.vision_time_sec:.2f}s '
                        f'({len(e.object_ids)} object x {self._vision_time_per_object():.2f}s)'
                    )
                if e.arm_time_sec > 0.0:
                    self.get_logger().info(f'     [robot arm] {e.arm_time_sec:.2f}s')
                if e.wb_time_sec > 0.0:
                    self.get_logger().info(f'     [workbench] {e.wb_time_sec:.2f}s')
                if e.post_time_sec > 0.0:
                    self.get_logger().info(f'     [backup + rotation] {e.post_time_sec:.2f}s')
                if e.notes and bool(self.get_parameter('log_segment_details').value):
                    self.get_logger().info(f'     notes: {"; ".join(e.notes)}')

                self.get_logger().info(
                    f'     [step total] {e.total_sec:.2f}s '
                    f'(schedule {e.schedule_start_sec:.2f}s -> {e.schedule_end_sec:.2f}s)'
                )
            else:
                self.get_logger().info(
                    f'[{e.step_id:02d}] [WB] {e.action} objects={e.object_ids}: '
                    f'{e.wb_time_sec:.2f}s -> step={e.total_sec:.2f}s '
                    f'(schedule {e.schedule_start_sec:.2f}s -> {e.schedule_end_sec:.2f}s)'
                )

        serial_sum = sum(e.total_sec for e in estimates)
        wall_est = self._wall_clock_total(estimates)
        self.get_logger().info(
            f'[TOTAL] step 단순 합계={serial_sum:.2f}s | '
            f'병렬/의존성 반영={wall_est:.2f}s'
        )
        self.get_logger().info('=============================')

    def _publish_summary(self, estimates: List[StepEstimate]):
        lines = ['===== SML 예상 시간 =====']
        for e in estimates:
            lines.append(
                f'[{e.step_id:02d}] {e.step_type} {e.action} '
                f'station={e.station_id} total={e.total_sec:.2f}s '
                f'(nav={e.nav_time_sec:.2f}, vision={e.vision_time_sec:.2f}, '
                f'arm={e.arm_time_sec:.2f}, wb={e.wb_time_sec:.2f}, post={e.post_time_sec:.2f})'
            )
        lines.append(f'합계={sum(e.total_sec for e in estimates):.2f}s')
        lines.append(f'wall-clock 예상={self._wall_clock_total(estimates):.2f}s')
        msg = String()
        msg.data = '\n'.join(lines)
        self.summary_pub.publish(msg)

    def _publish_visualization(self, estimates: List[StepEstimate]):
        all_poses: List[PoseStamped] = []
        for estimate in estimates:
            for segment in estimate.segments:
                poses = segment.poses
                if not poses:
                    continue
                if all_poses and poses:
                    all_poses.extend(poses[1:])
                else:
                    all_poses.extend(poses)

        path = NavPath()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = all_poses
        self.path_pub.publish(path)

        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        line = Marker()
        line.header = path.header
        line.ns = 'sml_time_estimator'
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.04
        line.color = ColorRGBA(r=0.1, g=0.6, b=1.0, a=0.9)
        line.points = [
            Point(x=p.pose.position.x, y=p.pose.position.y, z=0.05)
            for p in all_poses
        ]
        markers.markers.append(line)

        marker_id = 10
        for estimate in estimates:
            goal_pose = self._station_goal_pose(estimate.station_id)
            if goal_pose is None:
                continue

            sphere = Marker()
            sphere.header = path.header
            sphere.ns = 'sml_time_estimator_station'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = goal_pose.x
            sphere.pose.position.y = goal_pose.y
            sphere.pose.position.z = 0.12
            sphere.scale.x = 0.18
            sphere.scale.y = 0.18
            sphere.scale.z = 0.18
            sphere.color = ColorRGBA(r=1.0, g=0.8, b=0.1, a=0.95)
            markers.markers.append(sphere)

            text = Marker()
            text.header = path.header
            text.ns = 'sml_time_estimator_text'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = goal_pose.x
            text.pose.position.y = goal_pose.y
            text.pose.position.z = 0.45
            text.scale.z = 0.18
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = (
                f'{estimate.step_id}:{estimate.action.strip()}\n'
                f'S{estimate.station_id} {estimate.total_sec:.1f}s'
            )
            markers.markers.append(text)

        self.marker_pub.publish(markers)

    def _wall_clock_total(self, estimates: List[StepEstimate]) -> float:
        if not estimates:
            return 0.0
        return max(e.schedule_end_sec for e in estimates)

    # ------------------------------------------------------------------
    # Labels / predicates
    # ------------------------------------------------------------------

    def _step_type_name(self, step: Step) -> str:
        if int(step.type) == int(Step.AMR):
            return 'AMR'
        if int(step.type) == int(Step.WB):
            return 'WB'
        return 'UNKNOWN'

    def _action_name(self, step: Step) -> str:
        if int(step.type) == int(Step.AMR) and int(step.action) == int(Step.PRODUCE):
            return 'ASSEMBLE'
        action_map = {
            int(Step.LOAD): 'LOAD',
            int(Step.UNLOAD): 'UNLOAD',
            int(Step.PRODUCE): 'PRODUCE',
            int(Step.RECYCLE): 'RECYCLE',
            int(Step.GOAL): 'GOAL',
        }
        return action_map.get(int(step.action), str(int(step.action)))

    def _is_amr_wb_interaction(self, estimate: StepEstimate) -> bool:
        return (
            estimate.step_type == 'AMR'
            and estimate.action in ('LOAD', 'UNLOAD')
            and int(estimate.station_id) == int(self.fixed_workbench_station)
        )


def main(args=None):
    rclpy.init(args=args)
    node = SmlTimeEstimatorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
