#!/usr/bin/env python3
"""Shared helpers for EAI task message construction."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from typing import Dict, List, Sequence

from sml_messages.msg import Order, Station


RESET = "\033[0m"
BOLD = "\033[1m"
CYAN = "\033[36m"
GREEN = "\033[32m"
MAGENTA = "\033[35m"
YELLOW = "\033[33m"

TASK_QOS = 10


@dataclass(frozen=True)
class Product:
    name: str
    materials: Sequence[int]


RAW_MATERIAL_IDS = tuple(range(1, 9))

PRODUCTS: Dict[int, Product] = {
    13: Product("Magnet", (1, 3)),
    34: Product("Battery", (3, 4)),
    81: Product("E-Stop", (8, 1)),
    241: Product("Traffic Light", (2, 4, 1)),
    442: Product("Carrot", (4, 4, 2)),
    462: Product("Small Tree", (4, 6, 2)),
    711: Product("Hammer", (7, 1, 1)),
    4482: Product("Big Carrot", (4, 4, 8, 2)),
    8518: Product("Burger", (8, 5, 1, 8)),
    48132: Product("Ice Cream", (4, 8, 1, 3, 2)),
    46262: Product("Big Tree", (4, 6, 2, 6, 2)),
}

SIDES = ("a", "b")

# Current local World Cup 2026 station map.
#
# 01 side_a_storage_1     08 side_b_customer_1
# 02 side_a_storage_2     09 side_b_workbench_1
# 03 side_a_hybrid_1      10 side_b_workbench_2
# 04 side_a_workbench_1   11 side_b_hybrid_1
# 05 side_a_workbench_2   12 side_b_storage_1
# 06 side_a_customer_1    13 side_b_storage_2
# 07 shared_storage_1
SIDE_LAYOUT = {
    "a": {
        "prefix": "side_a",
        "storage_ids": (1, 2, 7),
        "workbench_ids": (4, 5),
        "hybrid_ids": (3,),
        "customer_ids": (6,),
    },
    "b": {
        "prefix": "side_b",
        "storage_ids": (12, 13, 7),
        "workbench_ids": (9, 10),
        "hybrid_ids": (11,),
        "customer_ids": (8,),
    },
}

def color(text: str, style: str) -> str:
    return f"{style}{text}{RESET}"


def product_slug(product_id: int) -> str:
    product = PRODUCTS.get(product_id)
    if product is None:
        return f"product_{product_id}"
    return product.name.lower().replace(" ", "_").replace("-", "_")


def product_materials(product_id: int) -> List[int]:
    product = PRODUCTS[product_id]
    return list(product.materials)


def expanded_material_count(material_ids: Sequence[int]) -> int:
    count = 0
    for material_id in material_ids:
        if material_id in RAW_MATERIAL_IDS:
            count += 1
        elif material_id in (10, 20, 30, 40, 50, 60, 70, 80):
            count += 5
        elif material_id == 90:
            count += 5
    return count


def build_net_materials(product_ids: Sequence[int], recycle_ids: Sequence[int]) -> List[int]:
    needed = Counter()
    for product_id in product_ids:
        needed.update(product_materials(product_id))

    # Returned products may become available raw material after recycling.
    for product_id in recycle_ids:
        needed.subtract(product_materials(product_id))

    materials: List[int] = []
    for material_id in sorted(needed):
        if needed[material_id] > 0:
            materials.extend([material_id] * needed[material_id])
    return materials


def make_order(order_type: int, product_id: int, name: str | None = None) -> Order:
    order = Order()
    order.order_type = order_type
    prefix = "produce" if order_type == Order.OT_PRODUCE else "recycle"
    order.name = name or f"{prefix}_{product_slug(product_id)}"
    order.product_id = product_id
    return order


def make_station(
    station_type: int,
    name: str,
    station_id: int,
    material_ids: Sequence[int],
) -> Station:
    station = Station()
    station.station_type = station_type
    station.name = name
    station.station_id = station_id
    station.material_ids = list(material_ids)
    return station


def station_name(side: str, station_type: int, station_id: int, index: int) -> str:
    if station_id == 7:
        return "shared_storage_1"

    prefix = SIDE_LAYOUT[side]["prefix"]
    if station_type == Station.ST_STORAGE:
        return f"{prefix}_storage_{index}"
    if station_type == Station.ST_WORKBENCH:
        return f"{prefix}_workbench_{index}"
    if station_type == Station.ST_HYBRID:
        return f"{prefix}_hybrid_{index}"
    if station_type == Station.ST_CUSTOMER:
        return f"{prefix}_customer_{index}"
    return f"{prefix}_station_{station_id}"


def prompt_choice(title: str, choices: Sequence[str]) -> str:
    print("")
    print(title)
    for index, choice in enumerate(choices, start=1):
        print(f"  {index}) {choice}")
    while True:
        raw = input("> ").strip().lower()
        if raw.isdigit():
            selected = int(raw)
            if 1 <= selected <= len(choices):
                return choices[selected - 1]
        if raw in choices:
            return raw
        print(f"잘못된 입력입니다. 가능한 값: {', '.join(choices)}")


def prompt_station_choice(title: str, station_ids: Sequence[int]) -> int:
    print("")
    print(title)
    for index, station_id in enumerate(station_ids, start=1):
        print(f"  {index}) S{station_id:02d}")
    while True:
        raw = input("> ").strip()
        if raw.isdigit():
            value = int(raw)
            if value in station_ids:
                return value
            if 1 <= value <= len(station_ids):
                return station_ids[value - 1]
        print("station 번호 또는 목록 번호를 입력하세요.")
