"""
RoboCup Planner Node

Subscribes to the task topic, computes the full plan, then runs the
reactive executor in a background thread while the ROS2 node spins
normally in the main thread.

Interfaces:
  Sub  /eai/task              sml_messages/Task     — task definition
  Sub  <wb_ready_topic>       std_msgs/Int32         — workbench product_id ready
  Act  navigate_to_station    robocup_pkg/NavTask    — navigate to station
  Act  wb_task                robocup_pkg/WbTask     — workbench work
  Srv  /amr_robot_command     robocup_pkg/ArmCommand — arm pick/place

Blocking helper methods (navigate, arm_*, wb_task) are called from the
executor thread and use threading.Event to wait for ROS2 async results.
"""

import hashlib
import json
import os
import threading
import time
from collections import Counter
from datetime import datetime
from typing import Any, Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist

TASK_QOS = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE,
)
LATCHED_TASK_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

from robocup_pkg.action import NavTask, WbTask
from robocup_pkg.srv import ArmCommand
from sml_messages.msg import Station, Task
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from sml_system_pkg.arena_side_utils import (
    normalize_side,
    side_to_fixed_workbench_station,
    side_to_start_goal_station,
)
from robocup_planner.planning.aidlist_builder import compute_net_aidlist
from robocup_planner.planning.cargo_allocator import CargoAllocator
from robocup_planner.planning.distance_calculator import DistanceCalculator
from robocup_planner.planning.midlist_builder import (
    build_full_midlist,
    build_mid,
    build_bidlist,
    build_storage_midlist,
    check_storage_satisfies,
    merge_into_midlist,
)
from robocup_planner.execution.cargo_state import CargoManager
from robocup_planner.execution.executor import Executor, Plan
from robocup_planner.product_catalog import (
    is_intransit_eligible,
    get_material_count,
    product_complexity_score,
    BATCH_TO_MATERIAL,
    BATCH_COUNT,
)

# Workbench WbTask goal strings
WB_PRODUCE = 'PRODUCE'
WB_RECYCLE = 'RECYCLE'

# Arm ArmCommand.srv action strings (matches amr_robot_node / mock_arm_node)
ARM_PICK = 'LOAD'
ARM_PLACE = 'UNLOAD'
ARM_DELIVER = 'UNLOAD'


class IntransitAssemblyHandle:
    """Result holder for one asynchronous in-transit ASSEMBLE command."""

    def __init__(self, product_id: int, cargo_id: int):
        self.product_id = int(product_id)
        self.cargo_id = int(cargo_id)
        self.event = threading.Event()
        self.success: Optional[bool] = None


class WbTaskHandle:
    """Result holder for one asynchronous WbTask (workbench) command."""

    def __init__(self, work_type: str, product_id: int):
        self.work_type = work_type
        self.product_id = int(product_id)
        self.event = threading.Event()
        self.success: Optional[bool] = None


class PlannerNode(Node):

    def __init__(self):
        super().__init__('robocup_planner')

        # --- Parameters ---
        try:
            from ament_index_python.packages import get_package_share_directory
            import os as _os
            _default_wp = _os.path.join(
                get_package_share_directory('robocup_planner'),
                'config',
                'robocup_waypoint.yaml',
            )
        except Exception:
            _default_wp = ''
        self.declare_parameter('waypoint_yaml', _default_wp)
        self.declare_parameter('task_topic', '/eai/task')
        self.declare_parameter('nav_action', 'navigate_to_station')
        self.declare_parameter('wb_action', 'wb_task')
        self.declare_parameter('arm_service', '/amr_robot_command')
        self.declare_parameter('wb_ready_topic', '/workbench/product_ready')
        self.declare_parameter('post_process_service', '/robocup_navigator/post_process')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('wb_recycle_wait_mode', 'auto')  # auto, always, never
        self.declare_parameter('wb_produce_parallelism', 2)
        self.declare_parameter('wb_clearance_backup_distance', 0.10)
        self.declare_parameter('wb_clearance_backup_speed', 0.10)
        self.declare_parameter('motion_period_sec', 0.05)
        self.declare_parameter('driving_velocity', 0.5)
        self.declare_parameter('parking_duration', 1.5)
        self.declare_parameter('exiting_duration', 1.0)
        self.declare_parameter('side', 'a')
        self.declare_parameter('debug_export', False)
        self.declare_parameter('debug_export_dir', '')
        # JSON string: {"product_id": weight, ...}  e.g. '{"8518": 2.0}'
        # Higher weight → cargo 7/8 slot assigned earlier (assembled first).
        self.declare_parameter('product_weights_json', '')

        wp_path = self.get_parameter('waypoint_yaml').get_parameter_value().string_value
        task_topic = self.get_parameter('task_topic').get_parameter_value().string_value
        nav_action = self.get_parameter('nav_action').get_parameter_value().string_value
        wb_action = self.get_parameter('wb_action').get_parameter_value().string_value
        arm_service = self.get_parameter('arm_service').get_parameter_value().string_value
        wb_ready_topic = self.get_parameter('wb_ready_topic').get_parameter_value().string_value
        post_process_service = self.get_parameter('post_process_service').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self._debug_export: bool = self.get_parameter('debug_export').get_parameter_value().bool_value
        _export_dir = self.get_parameter('debug_export_dir').get_parameter_value().string_value
        self._debug_export_dir: str = _export_dir if _export_dir else '/tmp/robocup_planner'

        self._side: str = normalize_side(
            self.get_parameter('side').get_parameter_value().string_value
        )
        _weights_json = self.get_parameter('product_weights_json').get_parameter_value().string_value
        if _weights_json:
            try:
                _raw = json.loads(_weights_json)
                self._product_weights: Dict[int, float] = {int(k): float(v) for k, v in _raw.items()}
                self.get_logger().info(f"Product weights loaded: {self._product_weights}")
            except Exception as e:
                self.get_logger().warning(f"product_weights_json parse failed: {e}; using defaults")
                self._product_weights = {}
        else:
            self._product_weights: Dict[int, float] = {}

        self._wb_recycle_wait_mode = str(
            self.get_parameter('wb_recycle_wait_mode').get_parameter_value().string_value
        ).strip().lower()
        if self._wb_recycle_wait_mode not in {'auto', 'always', 'never'}:
            self.get_logger().warning(
                f"Invalid wb_recycle_wait_mode={self._wb_recycle_wait_mode}; using auto"
            )
            self._wb_recycle_wait_mode = 'auto'
        self._wb_produce_parallelism = max(1, int(
            self.get_parameter('wb_produce_parallelism').value
        ))
        self._wb_clearance_backup_distance = float(
            self.get_parameter('wb_clearance_backup_distance').value
        )
        self._wb_clearance_backup_speed = float(
            self.get_parameter('wb_clearance_backup_speed').value
        )
        self._motion_period_sec = float(self.get_parameter('motion_period_sec').value)

        if not wp_path:
            self.get_logger().warning("waypoint_yaml parameter is empty; distances will be inf")
            self._calc: Optional[DistanceCalculator] = None
        else:
            self._calc = DistanceCalculator(wp_path)

        self._cargo = CargoManager()
        self._cargo_lock = threading.Lock()
        self._arm_call_lock = threading.Lock()
        self._last_navigated_station: Optional[int] = None
        self._post_processed_since_navigation: bool = False
        self._last_task_hash: Optional[str] = None

        # --- ROS interfaces ---
        self._task_sub = self.create_subscription(
            Task, task_topic, self._on_task, TASK_QOS
        )
        self._latched_task_sub = self.create_subscription(
            Task, task_topic, self._on_task, LATCHED_TASK_QOS
        )
        self._wb_ready_sub = self.create_subscription(
            Int32, wb_ready_topic, self._on_wb_ready, 10
        )
        self._nav_client = ActionClient(self, NavTask, nav_action)
        self._wb_client = ActionClient(self, WbTask, wb_action)
        self._arm_client = self.create_client(ArmCommand, arm_service)
        self._post_process_client = self.create_client(Trigger, post_process_service)
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Active executor (one at a time)
        self._executor_thread: Optional[threading.Thread] = None
        self._active_executor: Optional[Executor] = None
        self._exec_lock = threading.Lock()

        # Pending in-transit ASSEMBLE events (arm works asynchronously during travel)
        self._intransit_events: list = []
        self._intransit_lock = threading.Lock()

        self.get_logger().info(
            "RoboCup Planner ready — waiting for task | "
            f"wb_recycle_wait_mode={self._wb_recycle_wait_mode}, "
            f"wb_produce_parallelism={self._wb_produce_parallelism}, "
            f"wb_clearance_backup={self._wb_clearance_backup_distance:.2f}m, "
            f"cmd_vel={cmd_vel_topic}"
        )

    # ------------------------------------------------------------------
    # Task callback — triggers planning + execution
    # ------------------------------------------------------------------

    @staticmethod
    def _copy_station(station: Station) -> Station:
        copied = Station()
        copied.station_type = int(station.station_type)
        copied.name = str(getattr(station, 'name', ''))
        copied.station_id = int(station.station_id)
        copied.material_ids = [int(x) for x in station.material_ids]
        return copied

    @staticmethod
    def _station_name(station: Station) -> str:
        return str(getattr(station, 'name', '') or '').strip().lower()

    @staticmethod
    def _task_digest(msg: Task) -> str:
        """Stable hash used to ignore duplicate QoS deliveries of the same task."""
        payload = {
            'orders': [
                (
                    int(o.order_type),
                    str(getattr(o, 'name', '')),
                    int(o.product_id),
                )
                for o in msg.order_list
            ],
            'stations': [
                (
                    int(st.station_type),
                    str(getattr(st, 'name', '')),
                    int(st.station_id),
                    [int(x) for x in st.material_ids],
                )
                for st in msg.arena_layout
            ],
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(',', ':')).encode('utf-8')
        return hashlib.sha256(encoded).hexdigest()

    @classmethod
    def _is_shared_station(cls, station: Station) -> bool:
        name = cls._station_name(station)
        station_id = int(station.station_id)
        return (
            station_id in (7, 71, 72)
            or name.startswith('shared_')
            or name.endswith('_shared_storage')
            or 'shared_storage' in name
        )

    def _station_matches_side(self, station: Station) -> bool:
        """Return whether a station belongs to this planner's competition side."""
        if self._is_shared_station(station):
            return True

        name = self._station_name(station)
        if name.startswith('side_a_'):
            return self._side == 'a'
        if name.startswith('side_b_'):
            return self._side == 'b'

        station_id = int(station.station_id)
        if self._side == 'b':
            return station_id in {8, 9, 10, 11, 12, 13}
        return station_id in {1, 2, 3, 4, 5, 6}

    def _prepare_task_for_side(self, msg: Task) -> Task:
        """Filter official/full layouts to the selected competition side.

        The competition stack executes one side at a time.  Some task sources
        publish only that side, while test/referee-like sources may include
        mirrored A/B stations.  Keep only the configured side plus shared
        storage, then map shared station id 7 to the real side approach id.
        """
        prepared = Task()
        prepared.order_list = list(msg.order_list)

        kept = []
        dropped = []
        shared_id = 72 if self._side == 'b' else 71
        for station in msg.arena_layout:
            if not self._station_matches_side(station):
                dropped.append(int(station.station_id))
                continue

            copied = self._copy_station(station)
            if int(copied.station_id) == 7:
                copied.station_id = shared_id
                self.get_logger().info(
                    f"Remapped shared storage station_id 7 → {shared_id} (side={self._side})"
                )
            kept.append(copied)

        if not kept:
            self.get_logger().warning(
                "No side-matching stations found in task; using arena_layout as-is"
            )
            prepared.arena_layout = [self._copy_station(st) for st in msg.arena_layout]
            return prepared

        prepared.arena_layout = kept
        if dropped:
            self.get_logger().info(
                f"Filtered task for side={self._side}: kept={len(kept)}, "
                f"dropped_station_ids={dropped}"
            )
        return prepared

    def _on_task(self, msg: Task) -> None:
        msg = self._prepare_task_for_side(msg)
        task_hash = self._task_digest(msg)

        with self._exec_lock:
            if task_hash == self._last_task_hash:
                self.get_logger().info(
                    "Duplicate task delivery ignored "
                    f"(hash={task_hash[:8]}, side={self._side})"
                )
                return
            if self._executor_thread and self._executor_thread.is_alive():
                self.get_logger().warning(
                    "New task received while execution is running — ignoring"
                )
                return
            self._last_task_hash = task_hash

        self.get_logger().info(
            f"Task received — planning (hash={task_hash[:8]}, side={self._side})"
        )
        try:
            plan = self._plan(msg)
        except Exception as e:
            self.get_logger().error(f"Planning failed: {e}")
            return

        executor = Executor(plan, self)
        self._active_executor = executor

        def _run_and_shutdown():
            try:
                executor.run()
            except Exception as e:
                self.get_logger().error(f"Execution failed: {e}")
            finally:
                self.get_logger().info("Execution complete — shutting down")
                if rclpy.ok():
                    rclpy.shutdown()

        thread = threading.Thread(target=_run_and_shutdown, daemon=True, name='executor')
        self._executor_thread = thread
        thread.start()

    # ------------------------------------------------------------------
    # Workbench ready signal — sets the executor's event
    # ------------------------------------------------------------------

    def _on_wb_ready(self, msg: Int32) -> None:
        self.get_logger().info(
            f"Workbench signal: product {msg.data} ready"
        )

    # ------------------------------------------------------------------
    # Planning phase
    # ------------------------------------------------------------------

    def _plan(self, msg: Task) -> Plan:
        _dbg: Dict[str, Any] = {}  # collects intermediate data when debug_export is enabled

        # Parse orders
        produce_ids = [o.product_id for o in msg.order_list if o.order_type == 1]
        recycle_ids = [o.product_id for o in msg.order_list if o.order_type == 2]

        self.get_logger().info(
            f"Plan: produce={produce_ids}, recycle={recycle_ids}"
        )

        # Categorise stations — separate regular storage from batch stations
        storage_stations = []    # material_ids 1-8
        batch_stations_1080 = [] # batch_ids 10-80 (known type, 5 blocks each)
        batch_stations_90 = []   # mix batch ID 90 (unknown content)
        workbench_station_ids = []
        customer_station_id = None
        customer_stations = []

        for st in msg.arena_layout:
            if st.station_type in (Station.ST_STORAGE, Station.ST_HYBRID):
                mids = [int(m) for m in st.material_ids]
                regular = [m for m in mids if 1 <= m <= 8]
                b1080 = [m for m in mids if 10 <= m <= 80]
                b90 = [m for m in mids if m == 90]
                if regular:
                    storage_stations.append({
                        'station_id': st.station_id,
                        'material_ids': regular,
                    })
                if b1080:
                    batch_stations_1080.append({
                        'station_id': st.station_id,
                        'batch_ids': b1080,
                    })
                if b90:
                    batch_stations_90.append({
                        'station_id': st.station_id,
                        'batch_ids': [90],
                    })
                self.get_logger().info(
                    f"Station {st.station_id}: regular={regular} "
                    f"batch_1080={b1080} batch_90={b90}"
                )
            if st.station_type == Station.ST_WORKBENCH:
                workbench_station_ids.append(st.station_id)
            if st.station_type == Station.ST_CUSTOMER:
                customer_station_id = st.station_id
                customer_stations.append(st)

        if not workbench_station_ids:
            raise RuntimeError("No workbench station in arena layout")
        if customer_station_id is None:
            raise RuntimeError("No customer station in arena layout")

        home_id = side_to_start_goal_station(self._side)
        workbench_station_id = self._select_fixed_workbench(workbench_station_ids)
        self.get_logger().info(
            f"Selected workbench station {workbench_station_id} "
            f"from candidates {workbench_station_ids}"
        )

        # Map each material_id to its designated storage station (the "home"
        # position recycled surplus materials should be returned to). When a
        # material is stocked at more than one station, prefer the one closest
        # to the workbench, since that's where disassembly happens.
        material_home_station: Dict[int, int] = {}
        for s in storage_stations:
            for mat_id in s['material_ids']:
                if mat_id not in material_home_station:
                    material_home_station[mat_id] = s['station_id']
                elif self._calc:
                    current = material_home_station[mat_id]
                    if (self._calc.station_to_station(workbench_station_id, s['station_id'])
                            < self._calc.station_to_station(workbench_station_id, current)):
                        material_home_station[mat_id] = s['station_id']

        storage_ledger_seed: Dict[int, Dict[int, int]] = {}
        for s in storage_stations:
            sid = int(s['station_id'])
            for mat_id in s['material_ids']:
                storage_ledger_seed.setdefault(sid, {})
                storage_ledger_seed[sid][int(mat_id)] = (
                    storage_ledger_seed[sid].get(int(mat_id), 0) + 1
                )
        for bs in batch_stations_1080:
            sid = int(bs['station_id'])
            for batch_id in bs['batch_ids']:
                mat_id = BATCH_TO_MATERIAL.get(int(batch_id))
                if mat_id is None:
                    continue
                storage_ledger_seed.setdefault(sid, {})
                storage_ledger_seed[sid][int(mat_id)] = (
                    storage_ledger_seed[sid].get(int(mat_id), 0) + BATCH_COUNT
                )
                if int(mat_id) not in material_home_station:
                    material_home_station[int(mat_id)] = sid

        if self._debug_export:
            _dbg['input'] = {
                'produce_ids': produce_ids,
                'recycle_ids': recycle_ids,
                'stations': [
                    {
                        'station_id': st.station_id,
                        'station_type': st.station_type,
                        'material_ids': list(st.material_ids),
                    }
                    for st in msg.arena_layout
                ],
            }

        # Split produce work: hard products that can be built from recycled
        # workbench materials are assigned to WB PRODUCE first; remaining
        # products stay in the AMR cargo-arm in-transit queue.
        aidlist, _legacy_net, recycled_materials = compute_net_aidlist(
            produce_ids, recycle_ids
        )
        workbench_ids, intransit_ids, recycled_after_wb = (
            self._split_workbench_and_amr_products(produce_ids, recycled_materials)
        )
        amr_aidlist = Counter()
        for pid in intransit_ids:
            amr_aidlist += get_material_count(pid)

        net_aidlist = Counter(amr_aidlist)
        for mat_id, count in recycled_after_wb.items():
            net_aidlist[mat_id] = max(0, net_aidlist.get(mat_id, 0) - count)
            if net_aidlist.get(mat_id, 0) == 0:
                net_aidlist.pop(mat_id, None)

        self.get_logger().info(
            f"aidlist={dict(aidlist)}, amr_net_aidlist={dict(net_aidlist)}, "
            f"wb_produce={workbench_ids}, amr_produce={intransit_ids}"
        )

        # Simulate Phase 1 recycle disassembly to detect cargo overflow at plan time.
        # Materials that would overflow and are still needed get added back to
        # net_aidlist so they are fetched from storage instead of lost silently.
        if recycle_ids:
            overflow_needed = self._check_recycle_overflow(recycle_ids, net_aidlist)
            if overflow_needed:
                net_aidlist += overflow_needed
                self.get_logger().info(
                    f"net_aidlist after recycle overflow correction: {dict(net_aidlist)}"
                )

        # Step A: build storage midlist and check if it alone satisfies net_aidlist
        if self._calc:
            storage_mid = build_storage_midlist(storage_stations, self._calc, home_id)
        else:
            storage_mid = [
                {'station_id': s['station_id'], 'materials': s['material_ids'],
                 'distance': 0.0, 'is_recycle_pickup': False, 'recycle_product_id': None}
                for s in storage_stations
            ]

        satisfied, missing = check_storage_satisfies(storage_mid, net_aidlist)

        if self._debug_export:
            _dbg['computed'] = {
                'aidlist': dict(aidlist),
                'net_aidlist': dict(net_aidlist),
                'recycled_materials': dict(recycled_materials),
                'recycled_after_wb': dict(recycled_after_wb),
                'workbench_products': list(workbench_ids),
                'amr_products': list(intransit_ids),
                'storage_stations': storage_stations,
                'batch_stations_1080': batch_stations_1080,
                'batch_stations_90': batch_stations_90,
                'workbench_station_id': workbench_station_id,
                'customer_station_id': customer_station_id,
            }
            _dbg['midlist'] = {
                'storage_mid': storage_mid,
            }

        # Step B: if not satisfied, add batch 10-80 and re-check
        use_batch_1080 = False
        if not satisfied and batch_stations_1080:
            use_batch_1080 = True
            if self._calc:
                bidlist_1080 = build_bidlist(batch_stations_1080, self._calc, home_id)
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
            self.get_logger().info(
                f"After batch 10-80: satisfied={satisfied}, missing={dict(missing)}"
            )

        # Step C: if still missing (after storage + batch_1080 + recycling already in net_aidlist),
        # assign mix batch 90 to cover remaining shortage
        missing_for_mix = missing if (not satisfied and batch_stations_90) else None
        if missing_for_mix:
            self.get_logger().info(
                f"Mix batch 90 assigned for: {dict(missing_for_mix)}"
            )

        if not satisfied and not missing_for_mix:
            self.get_logger().warning(
                f"Cannot satisfy aidlist — missing: {dict(missing)}"
            )

        # Recycling is always triggered when recycle orders exist
        needs_recycling = bool(recycle_ids)

        # Build recycle orders (map each recycle product to the customer station)
        recycle_orders = [
            {'station_id': customer_station_id, 'product_id': pid}
            for pid in recycle_ids
        ]

        # Build full midlist with batch support
        if self._calc:
            full_midlist = build_full_midlist(
                storage_stations=storage_stations,
                customer_stations=[],
                recycle_orders=recycle_orders,
                calc=self._calc,
                home_station_id=home_id,
                workbench_station_id=workbench_station_id,
                needs_recycling=needs_recycling,
                batch_stations_1080=batch_stations_1080 if use_batch_1080 else None,
                batch_stations_90=batch_stations_90 if missing_for_mix else None,
                missing_for_mix=missing_for_mix,
            )
        else:
            full_midlist = [
                {'station_id': o['station_id'], 'materials': [],
                 'distance': 0.0, 'is_recycle_pickup': True,
                 'recycle_product_id': o['product_id']}
                for o in recycle_orders
            ] + storage_mid
            if use_batch_1080:
                for bst in batch_stations_1080:
                    mats = [
                        BATCH_TO_MATERIAL[bid]
                        for bid in bst['batch_ids']
                        for _ in range(BATCH_COUNT)
                    ]
                    full_midlist.append({
                        'station_id': bst['station_id'],
                        'materials': mats,
                        'distance': float('inf'),
                        'is_recycle_pickup': False,
                        'recycle_product_id': None,
                        'is_batch': True,
                        'is_mix_batch': False,
                    })

        if self._debug_export:
            _dbg['midlist']['full_midlist'] = full_midlist

        # Build final mid list
        mid = build_mid(full_midlist, net_aidlist)

        # Surplus recycled materials after WB PRODUCE reservations and AMR needs.
        surplus = {}
        for mat, cnt in recycled_after_wb.items():
            extra = cnt - amr_aidlist.get(mat, 0)
            if extra > 0:
                surplus[mat] = extra

        plan = Plan(
            mid=mid,
            workbench_products=workbench_ids,
            intransit_products=intransit_ids,
            workbench_station_id=workbench_station_id,
            customer_station_id=customer_station_id,
            home_station_id=home_id,
            surplus_recycled=surplus,
            material_home_station=material_home_station,
            product_weights=self._product_weights,
            storage_ledger_seed=storage_ledger_seed,
        )

        self.get_logger().info(
            f"Plan ready: {len(mid)} pickup entries, "
            f"workbench={workbench_ids}, in-transit={intransit_ids}"
        )

        if self._debug_export:
            _dbg['midlist']['final_mid'] = mid
            _dbg['plan'] = {
                'workbench_products': workbench_ids,
                'intransit_products': intransit_ids,
                'workbench_station_id': workbench_station_id,
                'customer_station_id': customer_station_id,
                'home_station_id': home_id,
                'surplus_recycled': dict(surplus),
                'material_home_station': dict(material_home_station),
                'storage_ledger_seed': storage_ledger_seed,
            }
            self._export_plan_debug(_dbg)

        return plan

    def _split_workbench_and_amr_products(
        self, produce_ids: list, recycled_materials: Counter
    ) -> tuple:
        """Assign recycled-material products to WB, hardest feasible first.

        The workbench is preferred when recycled materials already appear on
        the WB shelf after RECYCLE.  Products not fully covered by those
        materials remain in the AMR cargo-arm queue as a fallback.
        """
        pool = Counter(recycled_materials)
        workbench_indices = set()
        ranked = sorted(
            enumerate(produce_ids),
            key=lambda item: (-product_complexity_score(int(item[1])), item[0]),
        )
        for idx, product_id in ranked:
            need = get_material_count(int(product_id))
            if all(pool.get(mat_id, 0) >= count for mat_id, count in need.items()):
                workbench_indices.add(idx)
                pool.subtract(need)
                pool += Counter()

        workbench_ids = [
            int(pid) for idx, pid in enumerate(produce_ids) if idx in workbench_indices
        ]
        intransit_ids = [
            int(pid) for idx, pid in enumerate(produce_ids) if idx not in workbench_indices
        ]
        if workbench_ids:
            self.get_logger().info(
                f"[PLAN] WB PRODUCE from recycled materials: {workbench_ids}; "
                f"AMR cargo PRODUCE: {intransit_ids}"
            )
        return workbench_ids, intransit_ids, pool

    def _export_plan_debug(self, data: Dict[str, Any]) -> None:
        """Write the full planning snapshot to a timestamped JSON file."""
        try:
            os.makedirs(self._debug_export_dir, exist_ok=True)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
            path = os.path.join(self._debug_export_dir, f'plan_{ts}.json')
            with open(path, 'w', encoding='utf-8') as f:
                json.dump({'timestamp': ts, **data}, f, indent=2, default=str)
            self.get_logger().info(f"[DEBUG] Plan exported → {path}")
        except Exception as e:
            self.get_logger().error(f"[DEBUG] Plan export failed: {e}")

    def _select_fixed_workbench(self, workbench_station_ids):
        """Prefer the official fixed assembly workbench for each arena side."""
        preferred = side_to_fixed_workbench_station(self._side)
        if preferred in workbench_station_ids:
            return preferred
        return workbench_station_ids[0]

    def _check_recycle_overflow(
        self, recycle_ids: list, net_aidlist: Counter
    ) -> Counter:
        """Simulate Phase 1 cargo loading for all recycled products.

        Returns a Counter of materials that would overflow cargo slots 2-6
        AND are still needed (i.e., present in net_aidlist).  Callers should
        add the returned Counter to net_aidlist so those materials are picked
        from storage in Phase 2 instead of being silently lost.
        """
        sim = CargoManager()
        overflow: Counter = Counter()
        for pid in recycle_ids:
            for mat_id, cnt in get_material_count(pid).items():
                for _ in range(cnt):
                    if sim.place_material(mat_id) is None:
                        overflow[mat_id] += 1

        if not overflow:
            return Counter()

        overflow_needed: Counter = Counter()
        for mat_id, cnt in overflow.items():
            recoverable = min(cnt, net_aidlist.get(mat_id, 0))
            if recoverable > 0:
                overflow_needed[mat_id] = recoverable

        total_overflow = sum(overflow.values())
        total_needed = sum(overflow_needed.values())
        self.get_logger().warning(
            f"[PLAN] Recycle cargo overflow detected: {total_overflow} block(s) "
            f"would overflow {dict(overflow)}; {total_needed} needed — "
            "routing overflowed needed materials to storage pickup"
        )
        return overflow_needed

    # ------------------------------------------------------------------
    # Cargo state helpers (thread-safe, called from executor thread)
    # ------------------------------------------------------------------

    def cargo_has_all_materials(self, product_id: int) -> bool:
        """Return True if all materials required to assemble product_id are in cargo 2-6."""
        with self._cargo_lock:
            return self._cargo.find_materials_for_product(product_id) is not None

    def cargo_is_full(self) -> bool:
        """Return True if no cargo 2-6 slot can fit even the smallest (2-unit) block."""
        with self._cargo_lock:
            return not self._cargo.is_any_slot_available(1)

    def arm_unload_all_materials(self) -> bool:
        """Unload every material in cargo 2-6 to the current workbench (overflow buffer)."""
        with self._cargo_lock:
            all_mats = self._cargo.all_materials()
        success = True
        for cargo_id, mat_id in all_mats:
            ok = self.arm_unload_material(mat_id)
            if ok:
                with self._cargo_lock:
                    self._cargo.remove_material(cargo_id, mat_id)
            success = success and ok
        return success

    # ------------------------------------------------------------------
    # Blocking helpers called by Executor (run in executor thread)
    # ------------------------------------------------------------------

    def navigate(self, station_id: int) -> bool:
        """Navigate directly to station_id goal (positive) or sub_goal (negative)."""
        self._nav_client.wait_for_server()

        done = threading.Event()
        success_holder = [False]

        def _result_cb(future):
            result = future.result()
            success_holder[0] = result.result.success
            done.set()

        def _goal_cb(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f"NavTask goal rejected for station {station_id}")
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        goal = NavTask.Goal()
        goal.station_id = station_id
        self._nav_client.send_goal_async(goal).add_done_callback(_goal_cb)
        done.wait()

        if not success_holder[0]:
            self.get_logger().error(f"Navigation to station {station_id} failed")
        else:
            self._last_navigated_station = abs(station_id)
            self._post_processed_since_navigation = False
        return success_holder[0]

    def workbench_produce_parallelism(self) -> int:
        return self._wb_produce_parallelism

    def should_wait_at_workbench_for_recycle(self, is_last_recycle: bool) -> bool:
        """Choose whether to wait beside WB instead of overlapping next travel."""
        if self._wb_recycle_wait_mode == 'always':
            return True
        if self._wb_recycle_wait_mode == 'never':
            return False
        return bool(is_last_recycle)

    def call_workbench_clearance_backup(self) -> bool:
        """Back up a short distance without rotating, keeping WB pick orientation."""
        distance = abs(float(self._wb_clearance_backup_distance))
        speed = abs(float(self._wb_clearance_backup_speed))
        if distance <= 0.0 or speed <= 0.0:
            self.get_logger().warning('[WB CLEARANCE] skipped: invalid distance/speed')
            return True

        duration = distance / speed
        period = max(float(self._motion_period_sec), 0.01)
        self.get_logger().info(
            f'[WB CLEARANCE] backup-only start: distance={distance:.3f}m, speed={speed:.3f}m/s'
        )

        stop = Twist()
        self._cmd_vel_pub.publish(stop)
        deadline = time.monotonic() + duration
        cmd = Twist()
        cmd.linear.x = -speed
        while time.monotonic() < deadline and rclpy.ok():
            self._cmd_vel_pub.publish(cmd)
            time.sleep(period)
        self._cmd_vel_pub.publish(stop)
        self._post_processed_since_navigation = True
        self.get_logger().info('[WB CLEARANCE] backup-only done')
        return True

    def call_post_process(self) -> bool:
        """Trigger post-process exit maneuver (backup + rotate) after docking.

        Calls /robocup_navigator/post_process.  Returns True on success or if
        there was nothing pending (navigator responds NO_PENDING_POST_PROCESS).
        """
        if not self._post_process_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("[POST] post_process service unavailable")
            return False

        future = self._post_process_client.call_async(Trigger.Request())
        done = threading.Event()
        result_holder = [None]

        def _cb(f):
            result_holder[0] = f.result()
            done.set()

        future.add_done_callback(_cb)
        done.wait()

        resp = result_holder[0]
        if resp is None or not resp.success:
            msg = resp.message if resp else 'no response'
            self.get_logger().warning(f"[POST] post_process failed: {msg}")
            return False

        self._post_processed_since_navigation = True
        self.get_logger().info(
            f"[POST] post_process OK"
            + (f": {resp.message}" if resp.message else "")
        )
        return True

    def navigate_subgoal(self, station_id: int) -> bool:
        """Navigate to the sub_goal (approach) position of station_id.

        Convention: negative station_id signals the nav server to stop at
        the sub_goal waypoint (station_N_sub_goal) instead of the docking goal.
        """
        self.get_logger().info(f"[NAV] → sub_goal of station {station_id}")
        return self.navigate(-abs(station_id))

    def navigate_goal(self, station_id: int) -> bool:
        """Navigate the final leg from sub_goal to the docking goal of station_id.

        No arm assembly should occur during this phase (precision parking).
        """
        self.get_logger().info(f"[NAV] → goal of station {station_id}")
        return self.navigate(abs(station_id))

    def arm_assemble_intransit_async(
        self, product_id: int, cargo_id: int
    ) -> IntransitAssemblyHandle:
        """Start in-transit assembly on cargo_id asynchronously.

        The arm stacks blocks on cargo 7/8 while the AMR moves.  Callers must
        invoke wait_for_intransit_assembly() before navigate_goal() to ensure
        the arm is idle during the precision-parking phase.
        """
        handle = IntransitAssemblyHandle(product_id, cargo_id)
        with self._intransit_lock:
            self._intransit_events.append(handle)

        with self._cargo_lock:
            materials_to_consume = self._cargo.find_materials_for_product(product_id) or []

        def _assemble():
            try:
                self.get_logger().info(
                    f"[ARM] ASSEMBLE product={product_id} cargo={cargo_id}"
                )
                success = self._arm_call(
                    'ASSEMBLE',
                    object_ids=[product_id],
                    location=cargo_id,
                    station_id=cargo_id,
                )
                handle.success = success
                if success:
                    with self._cargo_lock:
                        for c_id, mat_id in materials_to_consume:
                            self._cargo.remove_material(c_id, mat_id)
                else:
                    self.get_logger().warning(
                        f"[ARM] ASSEMBLE failed: product={product_id}"
                    )
            except Exception as e:
                handle.success = False
                self.get_logger().error(
                    f"[ARM] ASSEMBLE exception: product={product_id}: {e}"
                )
            finally:
                handle.event.set()

        threading.Thread(
            target=_assemble, daemon=True, name=f'assemble_{product_id}'
        ).start()
        return handle

    def has_pending_intransit_assembly(self) -> bool:
        """Return True when an async cargo ASSEMBLE result still needs handling."""
        with self._intransit_lock:
            return bool(self._intransit_events)

    def wait_for_intransit_assembly(self) -> list:
        """Block until all pending in-transit ASSEMBLE operations finish.

        Call this at sub_goal before navigate_goal() so the arm is idle
        during the sub_goal → goal precision-parking segment.
        """
        with self._intransit_lock:
            events = list(self._intransit_events)
            self._intransit_events.clear()

        if events:
            self.get_logger().info(
                f"[ARM] Waiting for {len(events)} in-transit assembly operation(s)"
            )
            for handle in events:
                handle.event.wait()
            self.get_logger().info("[ARM] All in-transit assemblies complete")
        return events

    def wb_task(self, work_type: str, product_id: int) -> bool:
        """Block until the workbench completes the requested work."""
        self._wb_client.wait_for_server()

        done = threading.Event()
        success_holder = [False]

        def _result_cb(future):
            success_holder[0] = future.result().result.success
            done.set()

        def _goal_cb(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f"WbTask goal rejected ({work_type} {product_id})")
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        goal = WbTask.Goal()
        goal.work_type = work_type
        goal.product_id = product_id
        self._wb_client.send_goal_async(goal).add_done_callback(_goal_cb)
        done.wait()
        return success_holder[0]

    def wb_task_async(self, work_type: str, product_id: int) -> WbTaskHandle:
        """Start a WbTask without blocking, so the AMR can drive off (e.g. to
        fetch the next recycling product) while the workbench is still
        working. Call wait_for_wb_task() before relying on the result.
        """
        handle = WbTaskHandle(work_type, product_id)
        self._wb_client.wait_for_server()

        def _result_cb(future):
            handle.success = future.result().result.success
            handle.event.set()

        def _goal_cb(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(
                    f"WbTask goal rejected ({work_type} {product_id})"
                )
                handle.success = False
                handle.event.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        goal = WbTask.Goal()
        goal.work_type = work_type
        goal.product_id = product_id
        self.get_logger().info(
            f"[WB] {work_type} {product_id} started asynchronously"
        )
        self._wb_client.send_goal_async(goal).add_done_callback(_goal_cb)
        return handle

    def wait_for_wb_task(self, handle: WbTaskHandle) -> bool:
        """Block until the given asynchronous WbTask completes."""
        handle.event.wait()
        return bool(handle.success)

    def _arm_call(
        self,
        action: str,
        object_ids: list,
        location: int = 0,
        station_id: int = None,
    ) -> bool:
        """Send one ArmCommand service call to the arm. Blocks until response."""
        with self._arm_call_lock:
            self._arm_client.wait_for_service()

            req = ArmCommand.Request()
            req.action = action
            req.object_ids = [int(x) for x in object_ids]
            req.location = int(location)
            req.station_id = int(station_id if station_id is not None else location)

            future = self._arm_client.call_async(req)
            done = threading.Event()

            def _cb(f):
                done.set()

            future.add_done_callback(_cb)
            done.wait()
            return future.result().success

    def arm_pick_material(self, station_id: int, material_id: int) -> bool:
        """Pick one material block from a storage station and place it on cargo."""
        success = self._arm_call(
            ARM_PICK,
            object_ids=[material_id],
            location=station_id,
            station_id=station_id,
        )
        if success:
            with self._cargo_lock:
                self._cargo.place_material(material_id)
        return success

    def arm_pick_product(self, station_id: int, product_id: int) -> bool:
        """Pick an assembled product from a customer counter (for recycling)."""
        return self._arm_call(
            ARM_PICK,
            object_ids=[product_id],
            location=station_id,
            station_id=station_id,
        )

    def arm_unload_material(self, object_id: int) -> bool:
        """Unload a material block from cargo to the workbench.
        The arm locates the block via cargo_manager FIND_OBJECT."""
        return self._arm_call(
            ARM_PLACE,
            object_ids=[object_id],
            location=0,
        )

    def arm_return_material_to_storage(self, material_id: int, station_id: int) -> bool:
        """Return a surplus recycled material block from cargo to its designated
        storage station (instead of leaving it stranded in cargo 2-6)."""
        self.get_logger().info(
            f"[ARM] return surplus material_id={material_id} to storage {station_id}"
        )
        success = self._arm_call(
            ARM_PLACE,
            object_ids=[material_id],
            location=station_id,
            station_id=station_id,
        )
        if success:
            with self._cargo_lock:
                for cargo_id, mat_id in self._cargo.all_materials():
                    if mat_id == material_id:
                        self._cargo.remove_material(cargo_id, mat_id)
                        break
        return success

    def arm_unload_product_to_workbench(self, product_id: int, station_id: int) -> bool:
        """Unload a recycled product from cargo 1 to the current workbench."""
        self.get_logger().info(
            f"[ARM] unload recycled product_id={product_id} to workbench {station_id}"
        )
        return self._arm_call(
            ARM_PLACE,
            object_ids=[product_id],
            location=station_id,
            station_id=station_id,
        )

    def get_current_station_id(self) -> Optional[int]:
        """Return the station_id of the last successfully completed navigation, or None."""
        return self._last_navigated_station

    def is_docked_at_station(self, station_id: int) -> bool:
        """True only when the last successful nav ended at station goal and no exit maneuver ran."""
        return (
            self._last_navigated_station == abs(int(station_id))
            and not self._post_processed_since_navigation
        )

    def cargo_materials_snapshot(self) -> list:
        """Return [(cargo_id, material_id), ...] currently carried in material slots 2-6."""
        with self._cargo_lock:
            return list(self._cargo.all_materials())

    def arm_deliver(self, product_id: int, from_cargo_id: int = 0) -> bool:
        """Deliver a finished product to the customer counter.
        The arm locates the product via cargo_manager FIND_OBJECT."""
        self.get_logger().info(
            f"[ARM] deliver product_id={product_id} from cargo {from_cargo_id}"
        )
        return self._arm_call(
            ARM_DELIVER,
            object_ids=[product_id],
            location=0,
        )


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()

    # MultiThreadedExecutor lets action/service callbacks run while
    # the executor thread is blocking inside navigate() / wb_task().
    ros_executor = MultiThreadedExecutor(num_threads=4)
    ros_executor.add_node(node)

    try:
        ros_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
