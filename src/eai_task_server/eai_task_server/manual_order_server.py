#!/usr/bin/env python3
"""Manual EAI-WS order server for the local World Cup 2026 station layout.

난이도, 행동, side를 고른 뒤 제품 ID와 station별 material_ids를 직접 입력해
/eai/task 와 side별 topic으로 발행한다.
"""

from __future__ import annotations

from collections import Counter
import sys
import threading
from typing import Dict, List, Sequence

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
    STAGES,
    STAGE_SPECS,
    TIERS,
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
TASK_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class ManualOrderServer(Node):
    def __init__(self) -> None:
        super().__init__("manual_order_server")
        self.declare_parameter("task_topic", "/eai/task")
        self.declare_parameter("side_a_topic", "/eai/task/side_a")
        self.declare_parameter("side_b_topic", "/eai/task/side_b")

        self.task_topic = self.get_parameter("task_topic").get_parameter_value().string_value
        self.side_a_topic = self.get_parameter("side_a_topic").get_parameter_value().string_value
        self.side_b_topic = self.get_parameter("side_b_topic").get_parameter_value().string_value

        self.publisher = self.create_publisher(Task, self.task_topic, TASK_QOS)
        self.side_a_publisher = self.create_publisher(Task, self.side_a_topic, TASK_QOS)
        self.side_b_publisher = self.create_publisher(Task, self.side_b_topic, TASK_QOS)
        self.get_logger().info(
            "manual_order_server ready: "
            f"task_topic={self.task_topic}, side_a_topic={self.side_a_topic}, "
            f"side_b_topic={self.side_b_topic}"
        )

    def publish_task(self, task: Task) -> None:
        side_a_task = build_side_only_task(task, "side_a")
        side_b_task = build_side_only_task(task, "side_b")

        self.publisher.publish(task)
        self.side_a_publisher.publish(side_a_task)
        self.side_b_publisher.publish(side_b_task)

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
    # Hybrid stations are read by the planner exactly like storage stations
    # when building the initial aidlist (planner_node.py checks
    # station_type in (ST_STORAGE, ST_HYBRID)), so they take the same
    # material_ids input here.
    station_ids = tuple(layout["storage_ids"]) + tuple(layout["hybrid_ids"])
    out: Dict[int, List[int]] = {}

    print("")
    print(color("[Station Material Input]", MAGENTA + BOLD))
    print("storage/shared storage/hybrid station에 초기 material_ids를 둘 수 있습니다.")
    print("개별 재료는 1~8, known batch는 10/20/.../80, mix batch는 90입니다.")
    print("비워두면 해당 station material_ids=[] 입니다.")

    for station_id in station_ids:
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
    tier: str,
    stage: str,
    side: str,
    produce_ids: Sequence[int],
    recycle_ids: Sequence[int],
    material_by_station: Dict[int, List[int]],
    task: Task,
    selected_workbench_id: int,
    selected_customer_id: int,
    customer_initial_ids: Sequence[int],
) -> None:
    spec = STAGE_SPECS[(tier, stage)]
    selected_raw = sum(len(product_materials(pid)) for pid in list(produce_ids) + list(recycle_ids))
    range_ok = spec.raw_min <= selected_raw <= spec.raw_max
    need = net_counter(produce_ids, recycle_ids)
    available = material_availability(material_by_station)
    missing = Counter(
        {mat: cnt - available.get(mat, 0) for mat, cnt in need.items() if cnt > available.get(mat, 0)}
    )

    print("")
    print(color("================ EAI-WS MANUAL ORDER ================", CYAN + BOLD))
    print(
        f"tier={tier}  action={stage}  side={side.upper()}  "
        f"workbench=S{selected_workbench_id:02d}  customer=S{selected_customer_id:02d}"
    )
    print(
        f"produce={len(produce_ids)}/{spec.produce_orders}, "
        f"recycle={len(recycle_ids)}/{spec.recycle_returns}, "
        f"raw={selected_raw} target={spec.raw_min}~{spec.raw_max} "
        + color("[OK]" if range_ok else "[WARN]", GREEN if range_ok else YELLOW)
    )

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


def run_cli(node: ManualOrderServer) -> None:
    print(color("=== EAI-WS Manual Order Server ===", CYAN + BOLD))
    tier = prompt_choice("[난이도]  1) entry   2) beginner   3) advanced", TIERS)
    stage = prompt_choice("[행동]    1) production  2) recycling  3) lifecycle", STAGES)
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

    spec = STAGE_SPECS[(tier, stage)]
    produce_ids = prompt_product_ids("PRODUCE", spec.produce_orders)
    recycle_ids = prompt_product_ids("RECYCLE", spec.recycle_returns)
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
        tier,
        stage,
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
    node.publish_task(task)


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
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
