"""
Planner-side cargo slot tracker (material storage, IDs 2-6).

Mirrors the stack model used by cargo_manager_node so planning decisions
(space checks, material lookup, workbench readiness) stay consistent with
the arm's internal state.

Vertical stack model (bottom → top):
  Each slot holds a list of object_ids ordered from bottom to top.
  Block heights (in units):
    2×2 blocks (IDs 1-4) = 2 units high
    4×2 blocks (IDs 5-8) = 4 units high
  Maximum stack height per slot = 6 units.

Cargo 1  — finished products at the customer counter (counted only).
Cargo 7/8 — in-transit assembly, managed by CargoAllocator.
"""

from typing import Dict, List, Optional, Tuple

BLOCK_HEIGHT: Dict[int, int] = {
    1: 2, 2: 2, 3: 2, 4: 2,
    5: 4, 6: 4, 7: 4, 8: 4,
}
MAX_STACK_HEIGHT = 6

MATERIAL_SLOTS = [2, 3, 4, 5, 6]


class CargoSlot:
    """One material cargo tray (IDs 2-6), modelled as a vertical stack."""

    def __init__(self, cargo_id: int):
        self.cargo_id = cargo_id
        self._stack: List[int] = []

    def stack_height(self) -> int:
        return sum(BLOCK_HEIGHT.get(obj, 2) for obj in self._stack)

    def has_space(self, material_id: int) -> bool:
        needed = BLOCK_HEIGHT.get(material_id, 2)
        return self.stack_height() + needed <= MAX_STACK_HEIGHT

    def place(self, material_id: int) -> None:
        self._stack.append(material_id)

    def remove(self, material_id: int) -> None:
        if material_id in self._stack:
            self._stack.remove(material_id)

    def has(self, material_id: int) -> bool:
        return material_id in self._stack

    @property
    def contents(self) -> List[int]:
        return list(self._stack)

    @property
    def is_empty(self) -> bool:
        return len(self._stack) == 0


class CargoManager:
    """Planner-side view of material slots 2-6 and the finished-product counter."""

    def __init__(self):
        self._slots: Dict[int, CargoSlot] = {
            slot_id: CargoSlot(slot_id) for slot_id in MATERIAL_SLOTS
        }
        self.finished_on_cargo1: int = 0
        # Product-transit occupancy of cargo 1 (recycle pickups and
        # WB-produced products ride on cargo 1). Tracked explicitly instead
        # of inferred from call ordering, so a logic bug that tries to load a
        # second product before the first has been unloaded is caught
        # immediately instead of silently corrupting state.
        self._cargo1_product: Optional[int] = None

    def place_material(self, material_id: int) -> Optional[int]:
        """Place material in the first slot with enough space. Returns cargo_id or None."""
        for slot_id in MATERIAL_SLOTS:
            slot = self._slots[slot_id]
            if slot.has_space(material_id):
                slot.place(material_id)
                return slot_id
        return None

    def remove_material(self, cargo_id: int, material_id: int) -> None:
        slot = self._slots.get(cargo_id)
        if slot:
            slot.remove(material_id)

    def find_materials_for_product(
        self, product_id: int
    ) -> Optional[List[Tuple[int, int]]]:
        """
        Return [(cargo_id, material_id)] covering all blocks required by product_id,
        or None if the materials are not all present.
        """
        from robocup_planner.product_catalog import get_material_count
        needed = list(get_material_count(product_id).elements())

        available: Dict[int, List[int]] = {
            cid: list(slot.contents) for cid, slot in self._slots.items()
        }

        result: List[Tuple[int, int]] = []
        for mat_id in needed:
            found = False
            for cid, contents in available.items():
                if mat_id in contents:
                    contents.remove(mat_id)
                    result.append((cid, mat_id))
                    found = True
                    break
            if not found:
                return None
        return result

    def can_assemble_for_workbench(
        self, pending_products: List[int]
    ) -> Optional[int]:
        """Return the first product_id whose materials are fully loaded, or None."""
        for pid in pending_products:
            if self.find_materials_for_product(pid) is not None:
                return pid
        return None

    def all_materials(self) -> List[Tuple[int, int]]:
        """Return all (cargo_id, material_id) pairs currently on cargo 2-6."""
        result = []
        for cid, slot in self._slots.items():
            for mat_id in slot.contents:
                result.append((cid, mat_id))
        return result

    def is_any_slot_available(self, material_id: int) -> bool:
        return any(slot.has_space(material_id) for slot in self._slots.values())

    def add_finished_product(self) -> None:
        self.finished_on_cargo1 += 1

    def consume_finished_product(self) -> None:
        self.finished_on_cargo1 = max(0, self.finished_on_cargo1 - 1)

    # ------------------------------------------------------------------
    # Cargo 1 product-transit occupancy
    # ------------------------------------------------------------------

    def try_occupy_cargo1(self, product_id: int) -> bool:
        """Attempt to record product_id as occupying cargo 1 for transit.

        Returns False (and leaves state unchanged) if cargo 1 is already
        occupied by a different product — callers must treat this as a hard
        stop rather than proceeding, since it means a previous product was
        never unloaded from cargo 1.
        """
        if self._cargo1_product is not None:
            return False
        self._cargo1_product = product_id
        return True

    def release_cargo1(self, expected_product_id: int) -> bool:
        """Clear cargo 1 occupancy after the product has been unloaded.

        Returns False if the occupant did not match expected_product_id
        (or nothing was occupying cargo 1) — a signal of a possible
        identity mismatch between what the planner tracked and what the
        arm actually moved.
        """
        if self._cargo1_product != expected_product_id:
            return False
        self._cargo1_product = None
        return True

    @property
    def cargo1_occupant(self) -> Optional[int]:
        return self._cargo1_product
