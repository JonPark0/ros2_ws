"""Runtime material location ledger for RoboCup planner execution.

Tracks where raw materials currently live outside the AMR cargo so lifecycle
missions can reuse recycled or buffered materials without forgetting which
station they were placed on.
"""

from __future__ import annotations

from collections import Counter
from typing import Dict, Iterable, List, Optional, Tuple


class StorageLedger:
    """Small mutable inventory model keyed by station_id.

    The ledger intentionally tracks only materials that are physically on a
    station/workbench shelf, not materials already inside AMR cargo.
    """

    def __init__(
        self,
        initial: Optional[Dict[int, Dict[int, int]]] = None,
        distance_calculator=None,
        workbench_station_id: Optional[int] = None,
    ) -> None:
        self._stock: Dict[int, Counter] = {}
        self._calc = distance_calculator
        self.workbench_station_id = int(workbench_station_id) if workbench_station_id is not None else None
        if initial:
            for station_id, mats in initial.items():
                self._stock[int(station_id)] = Counter({int(k): int(v) for k, v in mats.items() if int(v) > 0})

    @classmethod
    def from_station_materials(
        cls,
        station_materials: Iterable[Tuple[int, Iterable[int]]],
        distance_calculator=None,
        workbench_station_id: Optional[int] = None,
    ) -> "StorageLedger":
        seed: Dict[int, Dict[int, int]] = {}
        for station_id, materials in station_materials:
            counter = Counter(int(m) for m in materials)
            seed[int(station_id)] = dict(counter)
        return cls(seed, distance_calculator, workbench_station_id)

    def snapshot(self) -> Dict[int, Dict[int, int]]:
        return {
            int(station_id): {int(mat): int(count) for mat, count in counter.items() if count > 0}
            for station_id, counter in sorted(self._stock.items())
            if any(count > 0 for count in counter.values())
        }

    def count(self, station_id: int, material_id: int) -> int:
        return int(self._stock.get(int(station_id), Counter()).get(int(material_id), 0))

    def add(self, station_id: int, material_id: int, count: int = 1) -> None:
        station_id = int(station_id)
        material_id = int(material_id)
        count = int(count)
        if count <= 0:
            return
        self._stock.setdefault(station_id, Counter())[material_id] += count

    def remove(self, station_id: int, material_id: int, count: int = 1) -> bool:
        station_id = int(station_id)
        material_id = int(material_id)
        count = int(count)
        if count <= 0:
            return True
        available = self.count(station_id, material_id)
        if available < count:
            return False
        self._stock[station_id][material_id] -= count
        if self._stock[station_id][material_id] <= 0:
            del self._stock[station_id][material_id]
        return True

    def where(self, material_id: int, reference_station_id: Optional[int] = None) -> Optional[int]:
        """Return nearest station currently holding material_id."""
        material_id = int(material_id)
        candidates = [
            station_id for station_id, counter in self._stock.items()
            if counter.get(material_id, 0) > 0
        ]
        if not candidates:
            return None
        if self._calc is None or reference_station_id is None:
            return sorted(candidates)[0]

        ref = int(reference_station_id)
        return min(
            candidates,
            key=lambda sid: self._safe_distance(ref, sid),
        )

    def pick_plan(self, needed: Counter, reference_station_id: Optional[int] = None) -> List[Tuple[int, List[int]]]:
        """Group needed materials by station, nearest-first.

        Returns [(station_id, [materials...])].  Does not mutate stock; caller
        should call remove() after a successful arm pick.
        """
        remaining = Counter({int(k): int(v) for k, v in needed.items() if int(v) > 0})
        groups: Dict[int, List[int]] = {}
        current_ref = reference_station_id

        while remaining:
            options = []
            for mat_id in list(remaining.keys()):
                sid = self.where(mat_id, current_ref)
                if sid is not None:
                    options.append((sid, mat_id))
            if not options:
                break

            sid, _ = min(options, key=lambda pair: self._safe_distance(current_ref, pair[0]) if current_ref is not None else pair[0])
            take: List[int] = []
            stock = self._stock.get(sid, Counter())
            for mat_id in list(remaining.keys()):
                n = min(stock.get(mat_id, 0), remaining[mat_id])
                if n > 0:
                    take.extend([mat_id] * n)
                    remaining[mat_id] -= n
                    if remaining[mat_id] <= 0:
                        del remaining[mat_id]
            if take:
                groups.setdefault(sid, []).extend(take)
                current_ref = sid
            else:
                break

        ordered = list(groups.items())
        if reference_station_id is not None:
            ordered.sort(key=lambda item: self._safe_distance(reference_station_id, item[0]))
        return ordered

    def _safe_distance(self, a: Optional[int], b: int) -> float:
        if a is None or self._calc is None:
            return float(abs(int(b)))
        try:
            return float(self._calc.station_to_station(int(a), int(b)))
        except Exception:
            return float(abs(int(a) - int(b)))
