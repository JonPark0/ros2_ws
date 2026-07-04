#!/usr/bin/env python3
"""Manual EAI-WS order server for the local World Cup 2026 station layout.

side를 고른 뒤 produce/recycle 개수와 제품 ID, station별 material_ids를
직접 입력해 /eai/task 와 side별 topic으로 발행한다.
"""

from __future__ import annotations

from collections import Counter
from pathlib import Path
import sys
import threading
import time
from typing import Dict, List, Optional, Sequence

import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from sml_messages.msg import Order, Station, Task

from eai_task_server.order import (
    BOLD,
    CYAN,
    GREEN,
    MAGENTA,
    PRODUCTS,
    RAW_MATERIAL_IDS,
    SIDES,
    SIDE_LAYOUT,
    YELLOW,
    build_net_materials,
    color,
    make_order,
    make_station,
    product_materials,
    prompt_choice,
    station_name,
)


VALID_MATERIAL_IDS = set(RAW_MATERIAL_IDS) | {10, 20, 30, 40, 50, 60, 70, 80, 90}
# YAML station_type 이름 ↔ sml_messages/Station 상수 매핑 (stations: 명시 모드용)
STATION_TYPE_NAMES = {
    "storage": Station.ST_STORAGE,
    "workbench": Station.ST_WORKBENCH,
    "customer": Station.ST_CUSTOMER,
    "hybrid": Station.ST_HYBRID,
}
TASK_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


def default_order_directories() -> List[Path]:
    """Candidate directories to look for a default order_file in.

    Checked in order: the installed package share directory (works for any
    `ros2 run` regardless of cwd), then the orders/ directory next to this
    package's source (works for --symlink-install / running from source).
    """
    directories: List[Path] = []
    try:
        directories.append(Path(get_package_share_directory("eai_task_server")) / "orders")
    except PackageNotFoundError:
        pass
    directories.append(Path(__file__).resolve().parent.parent / "orders")
    return directories


def find_default_order_file() -> Optional[str]:
    """Auto-pick order_file when exactly one *.yaml/*.yml exists in a default dir."""
    for directory in default_order_directories():
        if not directory.is_dir():
            continue
        order_files = sorted(directory.glob("*.yaml")) + sorted(directory.glob("*.yml"))
        if len(order_files) == 1:
            return str(order_files[0])
        if len(order_files) > 1:
            print(
                color(
                    f"[order_file] {directory}에 order 파일이 여러 개 있어 자동 선택할 수 없습니다: "
                    f"{[f.name for f in order_files]}",
                    YELLOW + BOLD,
                )
            )
            print(color("order_file 파라미터로 사용할 파일을 지정하세요.", YELLOW + BOLD))
            return None
    return None


class ManualOrderServer(Node):
    def __init__(self) -> None:
        super().__init__("manual_order_server")
        self.declare_parameter("task_topic", "/eai/task")
        self.declare_parameter("side_a_topic", "/eai/task/side_a")
        self.declare_parameter("side_b_topic", "/eai/task/side_b")
        self.declare_parameter("order_file", "")

        self.task_topic = self.get_parameter("task_topic").get_parameter_value().string_value
        self.side_a_topic = self.get_parameter("side_a_topic").get_parameter_value().string_value
        self.side_b_topic = self.get_parameter("side_b_topic").get_parameter_value().string_value
        self.order_file = self.get_parameter("order_file").get_parameter_value().string_value
        if not self.order_file:
            default_order_file = find_default_order_file()
            if default_order_file:
                self.order_file = default_order_file
                self.get_logger().info(f"order_file 자동 선택: {default_order_file}")

        self.publisher = self.create_publisher(Task, self.task_topic, TASK_QOS)
        self.side_a_publisher = self.create_publisher(Task, self.side_a_topic, TASK_QOS)
        self.side_b_publisher = self.create_publisher(Task, self.side_b_topic, TASK_QOS)
        self.get_logger().info(
            "manual_order_server ready: "
            f"task_topic={self.task_topic}, side_a_topic={self.side_a_topic}, "
            f"side_b_topic={self.side_b_topic}"
        )

    def publish_task_once(self, task: Task) -> None:
        side_a_task = build_side_only_task(task, "side_a")
        side_b_task = build_side_only_task(task, "side_b")
        self.publisher.publish(task)
        self.side_a_publisher.publish(side_a_task)
        self.side_b_publisher.publish(side_b_task)

    def publish_task(self, task: Task) -> None:
        """Publish once and return immediately.

        A single immediate publish races the DDS discovery handshake: if this
        node exits before the planner's subscription match completes, the
        TRANSIENT_LOCAL history goes away with it and the planner never sees
        the task. Prefer publish_until_planner_seen() for anything that isn't
        kept alive by other means.
        """
        self.publish_task_once(task)
        side_a_task = build_side_only_task(task, "side_a")
        side_b_task = build_side_only_task(task, "side_b")

        self.get_logger().info(
            f"manual task published: orders={len(task.order_list)}, "
            f"stations={len(task.arena_layout)}"
        )
        self.get_logger().info(
            f"side_a published: orders={len(side_a_task.order_list)}, "
            f"stations={len(side_a_task.arena_layout)}"
        )
        self.get_logger().info(
            f"side_b published: orders={len(side_b_task.order_list)}, "
            f"stations={len(side_b_task.arena_layout)}"
        )

    def publish_until_planner_seen(
        self,
        task: Task,
        period_sec: float = 1.0,
        post_subscriber_publishes: int = 3,
        timeout_sec: float = 30.0,
    ) -> None:
        """Publish repeatedly until a /eai/task subscriber is confirmed.

        publish_task() alone is not reliable here: the manual/YAML flows
        publish once and then main() tears the node down right away, so if
        the planner's discovery match hasn't completed yet the message never
        reaches it even under TRANSIENT_LOCAL. Keep republishing (harmless —
        same content) until a subscriber has been observed for a few
        consecutive sends, or until timeout_sec elapses.
        """
        count = 0
        observed = 0
        deadline = time.monotonic() + timeout_sec
        self.get_logger().info(
            f"publishing task on {self.task_topic} until a planner subscriber is observed"
        )
        while rclpy.ok():
            count += 1
            self.publish_task_once(task)
            subscribers = self.publisher.get_subscription_count()
            observed = observed + 1 if subscribers > 0 else 0
            self.get_logger().info(
                f"publish #{count}: /eai/task subscribers={subscribers}, "
                f"confirmed_sends={observed}/{post_subscriber_publishes}"
            )
            if observed >= post_subscriber_publishes:
                self.get_logger().info("task published after planner subscriber was observed")
                return
            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    "no planner subscriber observed before timeout; "
                    "task may not have been delivered"
                )
                return
            time.sleep(max(period_sec, 0.1))


def parse_int_list(raw: str) -> List[int]:
    if not raw.strip():
        return []
    cleaned = raw.replace(",", " ")
    values: List[int] = []
    for token in cleaned.split():
        values.append(int(token))
    return values


def print_catalog() -> None:
    print(color("[Product Catalog]", MAGENTA + BOLD))
    for product_id, product in sorted(PRODUCTS.items()):
        print(f"  {product_id:<5} {product.name:<14} materials={list(product.materials)}")


def prompt_count(label: str) -> int:
    while True:
        raw = input(f"{label} 개수 입력: ").strip()
        if raw.isdigit():
            return int(raw)
        print("0 이상의 정수를 입력하세요.")


def prompt_product_ids(label: str, count: int) -> List[int]:
    if count <= 0:
        return []

    print_catalog()
    while True:
        raw = input(f"{label} product_id {count}개 입력 (예: 81 442): ").strip()
        try:
            product_ids = parse_int_list(raw)
        except ValueError:
            print("숫자만 입력하세요.")
            continue
        unknown = [pid for pid in product_ids if pid not in PRODUCTS]
        if unknown:
            print(f"알 수 없는 product_id: {unknown}")
            continue
        if len(product_ids) != count:
            print(f"{count}개를 입력해야 합니다. 현재 {len(product_ids)}개입니다.")
            continue
        return product_ids


def prompt_customer_initial_ids(recycle_ids: Sequence[int]) -> List[int]:
    """Ask which recycle_ids already sit on the customer counter at plan time.

    The planner (planner_node.py) splits each recycle order into "immediate"
    (Phase 1: picked straight off the counter) or "deferred" (produce it
    first, deliver it, then recycle it — Executor._run_deferred_recycle_phase())
    by checking whether the product_id is present in the customer station's
    material_ids in the *initial* arena_layout. Defaulting to "all on the
    table" (as before) makes every recycle order immediate and makes it
    impossible to exercise the deferred/lifecycle path from this CLI.
    """
    if not recycle_ids:
        return []

    print("")
    print(color("[Customer Table Initial Stock]", MAGENTA + BOLD))
    print("recycle_id 중 지금 customer table 위에 이미 놓여 있는 product만 선택하세요.")
    print("선택되지 않은 recycle_id는 planner가 deferred recycle "
          "(생산 후 납품 -> 회수 -> 분해)로 처리합니다.")
    print(f"recycle 대상: {list(recycle_ids)}")
    while True:
        raw = input(
            "이미 table 위에 있는 product_id 입력 (전부: Enter, 없음: none): "
        ).strip()
        if raw == "":
            return list(recycle_ids)
        if raw.lower() == "none":
            return []
        try:
            chosen = parse_int_list(raw)
        except ValueError:
            print("숫자만 입력하세요.")
            continue
        invalid = [pid for pid in chosen if pid not in recycle_ids]
        if invalid:
            print(f"recycle 대상이 아닌 product_id: {invalid}")
            continue
        return chosen


def prompt_station_materials(side: str) -> Dict[int, List[int]]:
    layout = SIDE_LAYOUT[side]
    # 실제 경기장은 hybrid station에도 batch 재고를 두므로 함께 입력받는다.
    storage_ids = tuple(layout["storage_ids"]) + tuple(layout["hybrid_ids"])
    out: Dict[int, List[int]] = {}

    print("")
    print(color("[Station Material Input]", MAGENTA + BOLD))
    print("storage/shared/hybrid station에 초기 material_ids를 둘 수 있습니다.")
    print("개별 재료는 1~8, known batch는 10/20/.../80, mix batch는 90입니다.")
    print("비워두면 해당 station material_ids=[] 입니다.")

    for station_id in storage_ids:
        while True:
            raw = input(f"  S{station_id:02d} material_ids: ").strip()
            try:
                material_ids = parse_int_list(raw)
            except ValueError:
                print("숫자만 입력하세요.")
                continue
            invalid = [mid for mid in material_ids if mid not in VALID_MATERIAL_IDS]
            if invalid:
                print(f"허용되지 않는 material_id: {invalid}")
                continue
            out[station_id] = material_ids
            break
    return out


def material_availability(material_by_station: Dict[int, List[int]]) -> Counter:
    available: Counter = Counter()
    for material_ids in material_by_station.values():
        for material_id in material_ids:
            if material_id in RAW_MATERIAL_IDS:
                available[material_id] += 1
            elif material_id in (10, 20, 30, 40, 50, 60, 70, 80):
                available[material_id // 10] += 5
            # 90은 mix batch라 특정 재료로 확정하지 않는다.
    return available


def net_counter(product_ids: Sequence[int], recycle_ids: Sequence[int]) -> Counter:
    counter: Counter = Counter()
    for material_id in build_net_materials(product_ids, recycle_ids):
        counter[material_id] += 1
    return counter


def build_task(
    side: str,
    produce_ids: Sequence[int],
    recycle_ids: Sequence[int],
    material_by_station: Dict[int, List[int]],
    selected_workbench_id: int,
    selected_customer_id: int,
    customer_initial_ids: Sequence[int],
) -> Task:
    layout = SIDE_LAYOUT[side]
    task = Task()
    task.order_list = [
        make_order(Order.OT_PRODUCE, pid) for pid in produce_ids
    ] + [
        make_order(Order.OT_RECYCLE, pid) for pid in recycle_ids
    ]

    stations = []
    for idx, station_id in enumerate(layout["storage_ids"], start=1):
        stations.append(
            make_station(
                Station.ST_STORAGE,
                station_name(side, Station.ST_STORAGE, station_id, idx),
                station_id,
                material_by_station.get(station_id, []),
            )
        )

    workbench_ids = [selected_workbench_id] + [
        station_id for station_id in layout["workbench_ids"]
        if station_id != selected_workbench_id
    ]
    for idx, station_id in enumerate(workbench_ids, start=1):
        stations.append(
            make_station(
                Station.ST_WORKBENCH,
                station_name(side, Station.ST_WORKBENCH, station_id, idx),
                station_id,
                [],
            )
        )

    for idx, station_id in enumerate(layout["hybrid_ids"], start=1):
        stations.append(
            make_station(
                Station.ST_HYBRID,
                station_name(side, Station.ST_HYBRID, station_id, idx),
                station_id,
                material_by_station.get(station_id, []),
            )
        )

    stations.append(
        make_station(
            Station.ST_CUSTOMER,
            station_name(side, Station.ST_CUSTOMER, selected_customer_id, 1),
            selected_customer_id,
            list(customer_initial_ids),
        )
    )
    task.arena_layout = stations
    return task


def build_side_only_task(task: Task, side_prefix: str) -> Task:
    side_task = Task()
    side_stations = [
        station for station in task.arena_layout
        if station.name.startswith(f"{side_prefix}_")
    ]
    if not side_stations:
        side_task.order_list = []
        side_task.arena_layout = []
        return side_task

    side_task.order_list = list(task.order_list)
    side_task.arena_layout = [
        station for station in task.arena_layout
        if station.name.startswith(f"{side_prefix}_") or station.name.startswith("shared_")
    ]
    return side_task


def print_summary(
    side: str,
    produce_ids: Sequence[int],
    recycle_ids: Sequence[int],
    material_by_station: Dict[int, List[int]],
    task: Task,
    selected_workbench_id: int,
    selected_customer_id: int,
    customer_initial_ids: Sequence[int],
) -> None:
    selected_raw = sum(len(product_materials(pid)) for pid in list(produce_ids) + list(recycle_ids))
    need = net_counter(produce_ids, recycle_ids)
    available = material_availability(material_by_station)
    missing = Counter(
        {mat: cnt - available.get(mat, 0) for mat, cnt in need.items() if cnt > available.get(mat, 0)}
    )

    print("")
    print(color("================ EAI-WS MANUAL ORDER ================", CYAN + BOLD))
    print(
        f"side={side.upper()}  "
        f"workbench=S{selected_workbench_id:02d}  customer=S{selected_customer_id:02d}"
    )
    print(f"produce={len(produce_ids)}, recycle={len(recycle_ids)}, raw={selected_raw}")

    deferred_ids = [pid for pid in recycle_ids if pid not in customer_initial_ids]

    print("")
    print(color("[ORDER LIST]", MAGENTA + BOLD))
    for pid in produce_ids:
        product = PRODUCTS[pid]
        print(color("  PRODUCE ", GREEN + BOLD) + f"{product.name:<14} id={pid:<5} materials={list(product.materials)}")
    for pid in recycle_ids:
        product = PRODUCTS[pid]
        phase_tag = "immediate" if pid in customer_initial_ids else "deferred"
        print(
            color("  RECYCLE ", YELLOW + BOLD)
            + f"{product.name:<14} id={pid:<5} materials={list(product.materials)} "
            + f"({phase_tag})"
        )
    if deferred_ids:
        print(
            color(
                f"  [lifecycle] deferred recycle (produce -> deliver -> reclaim -> disassemble): "
                f"{deferred_ids}",
                CYAN + BOLD,
            )
        )

    print("")
    print(color("[MATERIAL CHECK]", MAGENTA + BOLD))
    print(f"  need(net)={dict(need)}")
    print(f"  available={dict(available)}")
    if missing:
        print(color(f"  missing={dict(missing)}  -> planner Cannot satisfy aidlist 가능", YELLOW + BOLD))
    else:
        print(color("  storage/shared materials satisfy net aidlist", GREEN + BOLD))

    print("")
    print(color("[ARENA LAYOUT]", MAGENTA + BOLD))
    for station in task.arena_layout:
        print(
            f"  S{station.station_id:02d} type={station.station_type} "
            f"material_ids={list(station.material_ids)} name={station.name}"
        )
    print(color("=====================================================", CYAN + BOLD))


def run_manual_cli(node: ManualOrderServer) -> None:
    print(color("=== EAI-WS Manual Order Server ===", CYAN + BOLD))
    side = prompt_choice("[side]    1) A  2) B", SIDES)
    layout = SIDE_LAYOUT[side]
    selected_workbench_id = layout["workbench_ids"][0]
    selected_customer_id = layout["customer_ids"][0]
    print(
        color(
            f"[fixed] workbench=S{selected_workbench_id:02d}, "
            f"customer=S{selected_customer_id:02d}",
            GREEN + BOLD,
        )
    )
    print(color("[layout] shared_storage_1=S07 is included for both sides.", GREEN + BOLD))

    produce_count = prompt_count("PRODUCE 주문")
    recycle_count = prompt_count("RECYCLE 주문")
    produce_ids = prompt_product_ids("PRODUCE", produce_count)
    recycle_ids = prompt_product_ids("RECYCLE", recycle_count)
    customer_initial_ids = prompt_customer_initial_ids(recycle_ids)
    material_by_station = prompt_station_materials(side)
    task = build_task(
        side,
        produce_ids,
        recycle_ids,
        material_by_station,
        selected_workbench_id,
        selected_customer_id,
        customer_initial_ids,
    )
    print_summary(
        side,
        produce_ids,
        recycle_ids,
        material_by_station,
        task,
        selected_workbench_id,
        selected_customer_id,
        customer_initial_ids,
    )
    input(color("Enter를 누르면 /eai/task 와 side별 topic으로 발행합니다.", GREEN + BOLD))
    node.publish_until_planner_seen(task)


def load_order_config(order_file: str) -> Dict:
    path = Path(order_file)
    if not path.is_file():
        raise ValueError(f"order_file을 찾을 수 없습니다: {path}")
    with path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    if not isinstance(config, dict):
        raise ValueError(f"order_file 최상위 구조는 mapping이어야 합니다: {path}")
    return config


def _parse_station_type(raw) -> int:
    """Accept station_type as a name ("storage") or the Station constant int."""
    if isinstance(raw, int):
        if raw in STATION_TYPE_NAMES.values():
            return raw
        raise ValueError(f"알 수 없는 station_type 값: {raw}")
    key = str(raw).strip().lower()
    if key not in STATION_TYPE_NAMES:
        raise ValueError(
            f"알 수 없는 station_type: {raw!r} (허용: {sorted(STATION_TYPE_NAMES)})"
        )
    return STATION_TYPE_NAMES[key]


def build_task_from_station_list(
    side: str,
    produce_ids: Sequence[int],
    recycle_ids: Sequence[int],
    customer_initial_ids: Optional[Sequence[int]],
    stations_cfg: Sequence[Dict],
) -> tuple[Task, Dict[int, List[int]], int, int, List[int]]:
    """Build a task from an explicit `stations:` list in the order file.

    The hardcoded SIDE_LAYOUT cannot express every real arena (e.g. the
    2026-07-04 field task typed station 3 as ST_STORAGE with batch stock and
    had no second workbench / shared storage at all), so this mode publishes
    exactly the stations the file lists — station_id, station_type,
    material_ids — with nothing added or reordered.
    """
    if not isinstance(stations_cfg, (list, tuple)) or not stations_cfg:
        raise ValueError("stations는 비어 있지 않은 리스트여야 합니다")

    stations: List[Station] = []
    material_by_station: Dict[int, List[int]] = {}
    type_index: Dict[int, int] = {}
    workbench_ids: List[int] = []
    customer_entries: List[tuple[int, List[int]]] = []
    seen_ids: set = set()

    for entry in stations_cfg:
        if not isinstance(entry, dict):
            raise ValueError(f"stations 항목은 mapping이어야 합니다: {entry!r}")
        if "station_id" not in entry or "station_type" not in entry:
            raise ValueError(f"stations 항목에는 station_id와 station_type이 필요합니다: {entry!r}")
        station_id = int(entry["station_id"])
        if station_id in seen_ids:
            raise ValueError(f"station_id가 중복되었습니다: {station_id}")
        seen_ids.add(station_id)
        station_type = _parse_station_type(entry["station_type"])
        material_ids = [int(m) for m in (entry.get("material_ids") or [])]

        if station_type in (Station.ST_STORAGE, Station.ST_HYBRID):
            invalid = [m for m in material_ids if m not in VALID_MATERIAL_IDS]
            if invalid:
                raise ValueError(
                    f"station {station_id}: 허용되지 않는 material_id: {invalid}"
                )
            material_by_station[station_id] = material_ids
        elif station_type == Station.ST_CUSTOMER:
            customer_entries.append((station_id, material_ids))
        elif station_type == Station.ST_WORKBENCH:
            workbench_ids.append(station_id)
            if material_ids:
                raise ValueError(
                    f"workbench station {station_id}에는 material_ids를 둘 수 없습니다"
                )

        type_index[station_type] = type_index.get(station_type, 0) + 1
        name = str(entry.get("name") or "").strip() or station_name(
            side, station_type, station_id, type_index[station_type]
        )
        stations.append(make_station(station_type, name, station_id, material_ids))

    if not workbench_ids:
        raise ValueError("stations에 workbench station이 최소 1개 필요합니다")
    if not customer_entries:
        raise ValueError("stations에 customer station이 1개 필요합니다")
    if len(customer_entries) > 1:
        raise ValueError("customer station은 1개만 지원합니다")

    selected_customer_id, customer_materials = customer_entries[0]
    if customer_materials and customer_initial_ids is not None:
        if sorted(customer_materials) != sorted(int(p) for p in customer_initial_ids):
            raise ValueError(
                "customer station의 material_ids와 customer_initial_ids가 다릅니다 — "
                "둘 중 하나만 지정하세요"
            )
    if customer_materials:
        effective_initial = [int(p) for p in customer_materials]
    elif customer_initial_ids is not None:
        effective_initial = [int(p) for p in customer_initial_ids]
    else:
        effective_initial = list(recycle_ids)
    invalid_customer_ids = [pid for pid in effective_initial if pid not in recycle_ids]
    if invalid_customer_ids:
        raise ValueError(f"recycle 대상이 아닌 customer 초기 재고: {invalid_customer_ids}")

    # customer station의 초기 재고를 확정값으로 채워 넣는다.
    for station in stations:
        if station.station_id == selected_customer_id:
            station.material_ids = list(effective_initial)

    task = Task()
    task.order_list = [
        make_order(Order.OT_PRODUCE, pid) for pid in produce_ids
    ] + [
        make_order(Order.OT_RECYCLE, pid) for pid in recycle_ids
    ]
    task.arena_layout = stations
    return task, material_by_station, workbench_ids[0], selected_customer_id, effective_initial


def build_task_from_config(config: Dict) -> tuple[Task, str, List[int], List[int], Dict[int, List[int]], int, int, List[int]]:
    side = str(config.get("side", "")).strip().lower()
    if side not in SIDES:
        raise ValueError(f"side는 {SIDES} 중 하나여야 합니다: {side!r}")

    produce_ids = [int(pid) for pid in config.get("produce_ids", [])]
    recycle_ids = [int(pid) for pid in config.get("recycle_ids", [])]
    unknown = [pid for pid in produce_ids + recycle_ids if pid not in PRODUCTS]
    if unknown:
        raise ValueError(f"알 수 없는 product_id: {unknown}")

    raw_customer_initial = config.get("customer_initial_ids")

    # 명시적 stations: 모드 — SIDE_LAYOUT을 무시하고 파일의 arena를 그대로 발행.
    stations_cfg = config.get("stations")
    if stations_cfg is not None:
        (
            task,
            material_by_station,
            selected_workbench_id,
            selected_customer_id,
            customer_initial_ids,
        ) = build_task_from_station_list(
            side, produce_ids, recycle_ids, raw_customer_initial, stations_cfg
        )
        return (
            task,
            side,
            produce_ids,
            recycle_ids,
            material_by_station,
            selected_workbench_id,
            selected_customer_id,
            customer_initial_ids,
        )

    if raw_customer_initial is None:
        customer_initial_ids = list(recycle_ids)
    else:
        customer_initial_ids = [int(pid) for pid in raw_customer_initial]
    invalid_customer_ids = [pid for pid in customer_initial_ids if pid not in recycle_ids]
    if invalid_customer_ids:
        raise ValueError(f"recycle 대상이 아닌 customer_initial_ids: {invalid_customer_ids}")

    layout = SIDE_LAYOUT[side]
    # 실제 경기장은 hybrid station에도 재고를 두므로 storage + hybrid 모두 허용.
    material_station_ids = tuple(layout["storage_ids"]) + tuple(layout["hybrid_ids"])
    material_by_station: Dict[int, List[int]] = {}
    for raw_station_id, material_ids in (config.get("material_by_station") or {}).items():
        station_id = int(raw_station_id)
        if station_id not in material_station_ids:
            raise ValueError(
                f"material_by_station의 station_id={station_id}는 side {side!r}의 "
                f"storage/hybrid station이 아닙니다 (허용: {material_station_ids}) — "
                "다른 배치가 필요하면 stations: 명시 모드를 사용하세요"
            )
        parsed_material_ids = [int(mid) for mid in material_ids]
        invalid = [mid for mid in parsed_material_ids if mid not in VALID_MATERIAL_IDS]
        if invalid:
            raise ValueError(f"허용되지 않는 material_id: {invalid}")
        material_by_station[station_id] = parsed_material_ids

    selected_workbench_id = layout["workbench_ids"][0]
    selected_customer_id = layout["customer_ids"][0]
    task = build_task(
        side,
        produce_ids,
        recycle_ids,
        material_by_station,
        selected_workbench_id,
        selected_customer_id,
        customer_initial_ids,
    )
    return (
        task,
        side,
        produce_ids,
        recycle_ids,
        material_by_station,
        selected_workbench_id,
        selected_customer_id,
        customer_initial_ids,
    )


def run_yaml_cli(node: ManualOrderServer, order_file: str) -> None:
    print(color(f"=== EAI-WS Manual Order Server (order_file={order_file}) ===", CYAN + BOLD))
    config = load_order_config(order_file)
    (
        task,
        side,
        produce_ids,
        recycle_ids,
        material_by_station,
        selected_workbench_id,
        selected_customer_id,
        customer_initial_ids,
    ) = build_task_from_config(config)
    print_summary(
        side,
        produce_ids,
        recycle_ids,
        material_by_station,
        task,
        selected_workbench_id,
        selected_customer_id,
        customer_initial_ids,
    )
    node.publish_until_planner_seen(task)


def run_cli(node: ManualOrderServer) -> None:
    if node.order_file:
        run_yaml_cli(node, node.order_file)
        return
    run_manual_cli(node)


def main(args=None) -> None:
    sys.stdout.reconfigure(line_buffering=True)
    rclpy.init(args=args)
    node = ManualOrderServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=False)
    spin_thread.start()
    try:
        run_cli(node)
    except KeyboardInterrupt:
        print("")
        print("manual_order_server interrupted")
    except EOFError:
        # Interactive prompts with no usable stdin (ros2 launch, piped run,
        # or the two-order-file ambiguous fallback in a non-tty context).
        node.get_logger().error(
            "interactive 입력을 읽을 수 없습니다 (stdin 없음) — "
            "order_file 파라미터로 YAML을 지정해 비대화형으로 실행하세요: "
            "ros2 run eai_task_server manual_order_server --ros-args "
            "-p order_file:=src/eai_task_server/orders/adv_lifecycle.yaml"
        )
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()