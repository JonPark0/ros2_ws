"""
Hybrid executor: follows a static mid list while reacting to in-transit
assembly completions at runtime.

Hybrid assembly rule:
  Simple/remaining products are assembled by the AMR cargo arm on cargo slots
  7/8 while the AMR travels.  Products that can be built from recycled
  materials already on the workbench are sent to WB PRODUCE, hardest first.

Cargo overflow rule:
  If all cargo 2-6 slots are full and no in-transit slot is assembling yet,
  go to the workbench to drop materials as a buffer.

Recycle disassembly rule (Phase 1):
  RECYCLE runs on the workbench asynchronously (wb_task_async), so the AMR
  drives off to fetch the next recycle product instead of idling at the
  workbench while it disassembles. Materials from the previous disassembly
  are always collected before the next product is placed on the workbench
  shelf. If cargo 2-6 fills up while multiple recycle products are in
  flight, the load is dropped at the workbench before heading to the next
  pickup instead of risking an overflow trip mid-route.

In-transit assembly rule (Mod 2):
  Completed products on cargo 7/8 are delivered directly from cargo 7/8
  to the customer counter — they do NOT move to cargo 1 first.
  When a slot is freed after delivery, CargoAllocator auto-assigns the next
  queued product to that slot; _start_ready_intransit_assembly() is called
  immediately in case the new product's materials are already loaded.

Two-phase navigation rule:
  Every station approach is split into two legs:
    1. navigate_subgoal(id)  — fast cruise; arm may ASSEMBLE during this leg.
       At sub_goal: wait_for_intransit_assembly() blocks until arm is idle.
    2. navigate_goal(id)     — precision parking; arm must be idle.

If the last material for a cargo product is picked at the final storage
station, delivery still starts immediately: the AMR drives toward the customer
while ASSEMBLE runs, then waits at the customer sub_goal only if the arm has
not finished yet.
"""

from collections import Counter
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from robocup_planner.planning.cargo_allocator import CargoAllocator
from robocup_planner.product_catalog import get_material_count
from robocup_planner.execution.storage_ledger import StorageLedger


@dataclass
class Plan:
    """Output of the planning phase, consumed by the Executor."""
    mid: List[Dict]                   # ordered pickup sequence (build_mid output)
    workbench_products: List[int]     # product IDs queued for WB PRODUCE
    intransit_products: List[int]     # product IDs queued for cargo arm assembly
    workbench_station_id: int
    customer_station_id: int
    home_station_id: int
    surplus_recycled: Dict[int, int] = field(default_factory=dict)
    material_home_station: Dict[int, int] = field(default_factory=dict)
    product_weights: Dict[int, float] = field(default_factory=dict)
    storage_ledger_seed: Dict[int, Dict[int, int]] = field(default_factory=dict)


class ExecutionFailure(RuntimeError):
    """Raised when a required runtime action fails and the plan cannot continue."""


class Executor:
    """
    Runs the reactive execution loop. Calls blocking helper methods on
    the PlannerNode (navigate, arm_pick, arm_unload, wb_task, arm_deliver).
    Must run in a dedicated thread while the ROS2 node spins separately.
    """

    def __init__(self, plan: Plan, node):
        self._plan = plan
        self._node = node
        self._allocator = CargoAllocator()
        self._allocator.allocate(plan.intransit_products, plan.product_weights)

        # Cargo IDs whose ASSEMBLE command has already been started.
        # Product IDs can repeat in one order, so slot state is keyed by cargo.
        self._started_intransit: set = set()
        # Prevent re-entrant overflow trips.
        self._en_route_to_wb: bool = False
        self._workbench_queue: List[int] = list(plan.workbench_products)
        self._pending_wb_produce: List = []
        self._pending_recycle_outputs: List = []
        self._wb_products_onboard: List[int] = []
        self._ledger = StorageLedger(
            plan.storage_ledger_seed,
            distance_calculator=getattr(node, '_calc', None),
            workbench_station_id=plan.workbench_station_id,
        )
        self._log_ledger('init')

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        self._log("Executor started")

        # Phase 1: recycle pickups (customer counter → workbench → disassembly)
        self._run_recycle_phase()

        # After Phase 1: reclaimed materials may already satisfy queued work.
        self._start_ready_workbench_production()
        self._collect_ready_workbench_products(block=False)
        self._start_ready_intransit_assembly()
        self._deliver_if_useful(
            has_more_pickups=self._has_storage_pickup_entries(),
            include_pending_arm=False,
        )

        # Materials dropped at the workbench as an overflow buffer are still
        # useful. Recover only the ones still needed for unfinished products.
        self._recover_workbench_buffered_materials()

        # Recycled surplus returns are handled after product delivery unless
        # the AMR naturally visits that storage station during pickup.

        # Recovery may have made queued work ready.
        self._start_ready_workbench_production()
        self._collect_ready_workbench_products(block=False)
        self._start_ready_intransit_assembly()
        self._deliver_if_useful(
            has_more_pickups=self._has_storage_pickup_entries(),
            include_pending_arm=False,
        )

        # Phase 2: main pickup loop
        self._run_pickup_phase()

        # Final delivery of anything remaining.  Do not wait in-place for a
        # pending cargo ASSEMBLE; start the customer trip and wait at the
        # customer sub_goal if the arm is still busy.
        self._collect_ready_recycle_outputs(
            block=True, only_if_useful_for_produce=True
        )
        self._collect_ready_workbench_products(block=True)
        self._deliver_until_idle()

        # After the products are delivered, collect any recycle outputs that
        # were pure surplus and return them to their material station.
        self._collect_ready_recycle_outputs(block=True)
        self._return_surplus_materials()

        # Return to home station (0 for A-side, 14 for B-side)
        self._return_to_home()

        self._log("Executor finished")

    # ------------------------------------------------------------------
    # Phase 1 — Recycle pickup
    # ------------------------------------------------------------------

    def _run_recycle_phase(self) -> None:
        """Pick up every recycle product and disassemble it at the workbench.

        RECYCLE runs asynchronously on the workbench so the AMR can drive off
        to fetch the next recycle product instead of idling next to the
        workbench while it disassembles. To avoid stacking a new product on
        top of an unclaimed pile of material blocks, materials from the
        previous disassembly are always collected before the next product is
        placed on the workbench. If cargo 2-6 fills up (typically once 3+
        recycle products' worth of material has accumulated), the load is
        dropped at the workbench before the AMR sets off for the next pickup.
        """
        recycle_entries = [
            e for e in self._plan.mid if e.get('is_recycle_pickup')
        ]
        if not recycle_entries:
            return

        self._log(f"Phase 1: recycling {len(recycle_entries)} product(s)")
        workbench_id = self._plan.workbench_station_id

        pending_wb_handle = None
        pending_wb_pid = None

        for idx, entry in enumerate(recycle_entries):
            station_id = entry['station_id']
            pid = entry['recycle_product_id']

            self._log(f"  → navigate to customer station {station_id}")
            self._node.navigate_subgoal(station_id)
            self._wait_for_intransit_assembly()
            self._node.navigate_goal(station_id)
            if not self._node.arm_pick_product(station_id=station_id, product_id=pid):
                raise ExecutionFailure(
                    f"Failed to pick recycle product {pid} from customer {station_id}"
                )
            self._node.call_post_process()

            self._log(f"  → navigate to workbench {workbench_id} for RECYCLE")
            self._node.navigate_subgoal(workbench_id)
            self._wait_for_intransit_assembly()
            self._node.navigate_goal(workbench_id)

            # Clear out the previous product's disassembled materials first so
            # the workbench shelf is free before we place the next product on it.
            if pending_wb_handle is not None:
                self._collect_recycled_materials(
                    pending_wb_handle, pending_wb_pid, workbench_id
                )
                pending_wb_handle = None
                pending_wb_pid = None

            if not self._node.arm_unload_product_to_workbench(
                product_id=pid,
                station_id=workbench_id,
            ):
                raise ExecutionFailure(
                    f"Failed to unload recycle product {pid} to workbench {workbench_id}"
                )
            pending_wb_handle = self._node.wb_task_async('RECYCLE', pid)
            pending_wb_pid = pid

            # If there is no useful next travel to overlap with the WB recycle
            # time, keep the AMR near the workbench: back up only 10 cm, wait
            # for the WB arms, redock, and pick the disassembled materials.
            is_last = (idx == len(recycle_entries) - 1)
            wait_here = self._should_wait_for_recycle_here(is_last)
            if wait_here:
                self._log(
                    f"  → wait beside workbench for RECYCLE {pid} "
                    "(short backup only, no rotation)"
                )
                self._node.call_workbench_clearance_backup()
                self._collect_recycled_materials(
                    pending_wb_handle, pending_wb_pid, workbench_id
                )
                pending_wb_handle = None
                pending_wb_pid = None
                self._node.call_post_process()
                self._start_ready_intransit_assembly()
            else:
                self._node.call_post_process()
                self._start_ready_intransit_assembly()
                if is_last:
                    self._log(
                        f"  → defer RECYCLE {pid} output collection; "
                        "AMR production work remains"
                    )
                    self._pending_recycle_outputs.append(
                        (pending_wb_handle, pending_wb_pid, workbench_id)
                    )
                    pending_wb_handle = None
                    pending_wb_pid = None

            # Cargo pressure check: with several recycle products in flight,
            # cargo 2-6 can fill up with reclaimed material before it's all
            # needed. Drop the load here (we're already at the workbench)
            # rather than risking an overflow trip mid-route to the next pickup.
            if not is_last and self._node.cargo_is_full():
                self._log(
                    "  ! cargo full during recycle phase — "
                    "dropping materials at workbench before next pickup"
                )
                self._ensure_docked_at_station(
                    workbench_id, "drop recycled overflow materials"
                )
                dropped = self._node.cargo_materials_snapshot()
                if not self._node.arm_unload_all_materials():
                    raise ExecutionFailure(
                        "Failed to unload recycled overflow materials at workbench"
                    )
                for _, mat_id in dropped:
                    self._ledger.add(workbench_id, mat_id)
                self._log_ledger('recycle-overflow-drop')
                self._start_ready_workbench_production()
                self._node.call_post_process()

        # Collect whatever disassembly is still pending after the last pickup.
        if pending_wb_handle is not None:
            self._collect_recycled_materials(
                pending_wb_handle, pending_wb_pid, workbench_id
            )
            self._node.call_post_process()

    def _has_main_production_work_remaining(self) -> bool:
        """True when the AMR still has useful work to overlap with WB RECYCLE."""
        if any(not e.get('is_recycle_pickup') for e in self._plan.mid):
            return True
        if self._remaining_material_need():
            return True
        return bool(self._allocator.queued_products())

    def _should_wait_for_recycle_here(self, is_last_recycle: bool) -> bool:
        """Auto mode waits only when no useful AMR production work remains."""
        mode = getattr(self._node, '_wb_recycle_wait_mode', 'auto')
        if mode == 'always':
            return True
        if mode == 'never':
            return False
        if not is_last_recycle:
            return False
        if self._has_main_production_work_remaining():
            self._log(
                "  → overlap last RECYCLE with remaining AMR production work"
            )
            return False
        return True

    def _recycle_output_has_remaining_need(self, pid: int) -> bool:
        """True if this RECYCLE output can still help AMR production."""
        remaining = self._remaining_material_need()
        if not remaining:
            return False
        for mat_id, cnt in get_material_count(pid).items():
            if remaining.get(mat_id, 0) > 0 and cnt > 0:
                return True
        return False

    def _collect_ready_recycle_outputs(
        self,
        block: bool = False,
        only_if_useful_for_produce: bool = False,
    ) -> None:
        """Collect deferred RECYCLE outputs at task boundaries."""
        if not self._pending_recycle_outputs:
            return

        ready = []
        pending = []
        for handle, pid, workbench_id in self._pending_recycle_outputs:
            useful = self._recycle_output_has_remaining_need(pid)
            if only_if_useful_for_produce and not useful:
                pending.append((handle, pid, workbench_id))
                continue
            if block or handle.event.is_set():
                ready.append((handle, pid, workbench_id))
            else:
                pending.append((handle, pid, workbench_id))
        self._pending_recycle_outputs = pending

        for handle, pid, workbench_id in ready:
            before_docked = (
                hasattr(self._node, 'is_docked_at_station')
                and self._node.is_docked_at_station(workbench_id)
            )
            self._collect_recycled_materials(handle, pid, workbench_id)
            after_docked = (
                hasattr(self._node, 'is_docked_at_station')
                and self._node.is_docked_at_station(workbench_id)
            )
            if after_docked and not before_docked:
                self._node.call_post_process()
                self._start_ready_intransit_assembly()

    def _collect_recycled_materials(
        self, handle, pid: int, workbench_id: int
    ) -> None:
        """Wait for a RECYCLE WbTask to finish, then pick up every material
        block it produced from the workbench shelf."""
        if not self._node.wait_for_wb_task(handle):
            raise ExecutionFailure(f"RECYCLE failed: product={pid}")

        for mat_id, cnt in get_material_count(pid).items():
            for _ in range(cnt):
                self._ledger.add(workbench_id, mat_id)
        self._log_ledger(f"recycle-output-product-{pid}")

        # Let WB arms consume recycled materials for assigned PRODUCE first.
        self._start_ready_workbench_production()
        self._pick_needed_workbench_materials_for_amr(workbench_id)
        self._pick_surplus_workbench_materials_for_return(workbench_id)
        self._collect_ready_workbench_products(block=False)

    # ------------------------------------------------------------------
    # Workbench PRODUCE from recycled materials
    # ------------------------------------------------------------------

    def _start_ready_workbench_production(self) -> bool:
        """Start WB PRODUCE for queued products whose materials are on WB."""
        if not self._workbench_queue:
            return False

        wid = self._plan.workbench_station_id
        parallelism = 1
        if hasattr(self._node, 'workbench_produce_parallelism'):
            parallelism = max(1, int(self._node.workbench_produce_parallelism()))

        started_any = False
        while len(self._pending_wb_produce) < parallelism and self._workbench_queue:
            snapshot = self._ledger.snapshot().get(wid, {})
            start_index = None
            for idx, product_id in enumerate(self._workbench_queue):
                need = get_material_count(product_id)
                if all(snapshot.get(mat_id, 0) >= cnt for mat_id, cnt in need.items()):
                    start_index = idx
                    break
            if start_index is None:
                break

            product_id = self._workbench_queue.pop(start_index)
            for mat_id, cnt in get_material_count(product_id).items():
                if not self._ledger.remove(wid, mat_id, cnt):
                    raise ExecutionFailure(
                        f"WB ledger mismatch while reserving product {product_id}"
                    )
            self._log(
                f"  [WB READY] PRODUCE product {product_id} from recycled materials"
            )
            self._pending_wb_produce.append(
                self._node.wb_task_async('PRODUCE', product_id)
            )
            self._log_ledger(f"wb-produce-start-{product_id}")
            started_any = True

        return started_any

    def _collect_ready_workbench_products(self, block: bool = False) -> None:
        """Pick completed WB-produced products at a task boundary."""
        if not self._pending_wb_produce:
            return

        ready = []
        pending = []
        for handle in self._pending_wb_produce:
            if block:
                if not self._node.wait_for_wb_task(handle):
                    raise ExecutionFailure(
                        f"WB PRODUCE failed: product={handle.product_id}"
                    )
                ready.append(handle)
            elif handle.event.is_set():
                if not handle.success:
                    raise ExecutionFailure(
                        f"WB PRODUCE failed: product={handle.product_id}"
                    )
                ready.append(handle)
            else:
                pending.append(handle)

        self._pending_wb_produce = pending
        if not ready:
            return

        wid = self._plan.workbench_station_id
        self._log(
            f"  → collect WB-produced products {[h.product_id for h in ready]}"
        )
        self._ensure_docked_at_station(wid, "collect WB-produced products")
        for handle in ready:
            if not self._node.arm_pick_product(
                station_id=wid, product_id=handle.product_id
            ):
                raise ExecutionFailure(
                    f"Failed to pick WB-produced product {handle.product_id}"
                )
            self._wb_products_onboard.append(handle.product_id)
        self._node.call_post_process()
        self._start_ready_workbench_production()

    def _pick_needed_workbench_materials_for_amr(self, workbench_id: int) -> None:
        """Pick only WB materials still needed by AMR cargo assembly."""
        remaining_need = self._remaining_material_need()
        buffered = self._ledger.snapshot().get(workbench_id, {})
        if not remaining_need or not buffered:
            return

        to_pick: List[int] = []
        for mat_id, count in sorted(buffered.items()):
            n = min(int(count), int(remaining_need.get(int(mat_id), 0)))
            to_pick.extend([int(mat_id)] * n)
        if not to_pick:
            return

        self._ensure_docked_at_station(
            workbench_id, "pick recycled materials needed by AMR assembly"
        )
        for mat_id in to_pick:
            if self._ledger.count(workbench_id, mat_id) <= 0:
                continue
            if self._node.cargo_is_full():
                self._log(
                    "  ! cargo full; leaving remaining recycled materials on WB"
                )
                return
            self._log(f"    ← pick recycled mat {mat_id} for AMR assembly")
            ok = self._node.arm_pick_material(
                station_id=workbench_id, material_id=mat_id
            )
            if not ok:
                raise ExecutionFailure(
                    f"Failed to pick recycled material {mat_id} from workbench {workbench_id}"
                )
            self._ledger.remove(workbench_id, mat_id)
        self._log_ledger('pick-wb-materials-for-amr')

    def _pick_surplus_workbench_materials_for_return(self, workbench_id: int) -> None:
        """Pick WB surplus materials so they can be returned on route."""
        buffered = self._ledger.snapshot().get(workbench_id, {})
        if not buffered:
            return

        # Keep any material still needed for AMR production on the WB ledger.
        remaining_need = self._remaining_material_need()
        to_pick: List[int] = []
        for mat_id, count in sorted(buffered.items()):
            mat_id = int(mat_id)
            destination = self._plan.material_home_station.get(mat_id)
            if destination is None or destination == workbench_id:
                continue
            surplus_count = max(0, int(count) - int(remaining_need.get(mat_id, 0)))
            to_pick.extend([mat_id] * surplus_count)

        if not to_pick:
            return

        self._ensure_docked_at_station(
            workbench_id, "pick surplus recycled materials for return"
        )
        for mat_id in to_pick:
            if self._ledger.count(workbench_id, mat_id) <= 0:
                continue
            if self._node.cargo_is_full():
                self._log(
                    "  ! cargo full; leaving surplus recycled materials on WB"
                )
                return
            self._log(
                f"    ← pick surplus mat {mat_id} for return to "
                f"station {self._plan.material_home_station.get(mat_id)}"
            )
            ok = self._node.arm_pick_material(
                station_id=workbench_id, material_id=mat_id
            )
            if not ok:
                raise ExecutionFailure(
                    f"Failed to pick surplus material {mat_id} from workbench {workbench_id}"
                )
            self._ledger.remove(workbench_id, mat_id)
        self._log_ledger('pick-wb-surplus-for-return')

    # ------------------------------------------------------------------
    # Workbench overflow-buffer recovery
    # ------------------------------------------------------------------

    def _remaining_product_material_need(self) -> Counter:
        """Product material demand before considering loose cargo materials."""
        needed: Counter = Counter()
        completed_cargos = {
            slot.cargo_id for slot in self._allocator.get_completed_slots()
        }
        for cargo_id in (7, 8):
            product_id = self._allocator.get_slot_product(cargo_id)
            if product_id is None:
                continue
            if cargo_id in completed_cargos or cargo_id in self._started_intransit:
                continue
            needed.update(get_material_count(product_id))
        for product_id in self._allocator.queued_products():
            needed.update(get_material_count(product_id))
        return needed

    def _remaining_material_need(self) -> Counter:
        """Materials still needed for products not yet assembled/delivered."""
        needed = self._remaining_product_material_need()
        for _, mat_id in self._node.cargo_materials_snapshot():
            if needed.get(mat_id, 0) > 0:
                needed[mat_id] -= 1
                if needed[mat_id] <= 0:
                    del needed[mat_id]
        return needed

    def _recover_workbench_buffered_materials(self) -> None:
        """Pick back needed materials that were temporarily unloaded to WB."""
        wid = self._plan.workbench_station_id
        self._start_ready_workbench_production()
        before_docked = (
            hasattr(self._node, 'is_docked_at_station')
            and self._node.is_docked_at_station(wid)
        )
        self._pick_needed_workbench_materials_for_amr(wid)
        after_docked = (
            hasattr(self._node, 'is_docked_at_station')
            and self._node.is_docked_at_station(wid)
        )
        if after_docked and not before_docked:
            self._node.call_post_process()

    # ------------------------------------------------------------------
    # Return surplus recycled materials to their designated storage station
    # ------------------------------------------------------------------

    def _return_surplus_materials(self) -> None:
        """Return recycled materials beyond what's needed for production to
        their designated storage station.

        Blocks of the same material_id are fungible: the plan only tracks how
        many extra units exist, not which physical blocks, so any `count`
        units of that material currently in cargo can be dropped off.
        """
        surplus = {m: c for m, c in self._plan.surplus_recycled.items() if c > 0}
        if not surplus:
            return

        cargo_counts = Counter(
            mat_id for _, mat_id in self._node.cargo_materials_snapshot()
        )
        protected_for_produce = self._remaining_product_material_need()

        # Group by destination station so each station is visited once.
        # If overflow buffering already placed some surplus on the workbench,
        # do not command the arm to unload blocks that are no longer in cargo.
        # Also keep cargo materials that are still needed by AMR cargo assembly.
        by_station: Dict[int, List[int]] = {}
        skipped: Dict[int, int] = {}
        kept_for_produce: Dict[int, int] = {}
        for mat_id, cnt in surplus.items():
            in_cargo = int(cargo_counts.get(mat_id, 0))
            protect = min(in_cargo, int(protected_for_produce.get(mat_id, 0)))
            if protect > 0:
                kept_for_produce[int(mat_id)] = protect
            available = min(int(cnt), max(0, in_cargo - protect))
            missing = int(cnt) - available
            if missing > 0:
                skipped[int(mat_id)] = missing
            if available <= 0:
                continue
            station_id = self._plan.material_home_station.get(
                mat_id, self._plan.workbench_station_id
            )
            by_station.setdefault(station_id, []).extend([mat_id] * available)

        if kept_for_produce:
            self._log(
                f"  → keeping recycled cargo materials for AMR produce: {kept_for_produce}"
            )

        if skipped:
            self._log(
                f"  ! surplus not currently in cargo, leaving buffered: {skipped}"
            )
        if not by_station:
            return

        self._log(f"Returning surplus recycled materials: {surplus}")
        for station_id, mat_ids in by_station.items():
            self._log(f"  → navigate to storage {station_id} to return {mat_ids}")
            self._node.navigate_subgoal(station_id)
            self._wait_for_intransit_assembly()
            self._node.navigate_goal(station_id)
            for mat_id in mat_ids:
                ok = self._node.arm_return_material_to_storage(
                    material_id=mat_id, station_id=station_id
                )
                if not ok:
                    raise ExecutionFailure(
                        f"Failed to return material {mat_id} to storage {station_id}"
                    )
                self._ledger.add(station_id, mat_id)
            self._log_ledger(f"return-surplus-storage-{station_id}")
            self._node.call_post_process()

    # ------------------------------------------------------------------
    # Phase 2 — Main pickup loop
    # ------------------------------------------------------------------

    def _return_matching_surplus_at_station(self, station_id: int) -> None:
        """Unload carried surplus materials when visiting their home station."""
        protected = self._remaining_product_material_need()
        to_return: List[int] = []
        for _, mat_id in self._node.cargo_materials_snapshot():
            mat_id = int(mat_id)
            if self._plan.material_home_station.get(mat_id) != int(station_id):
                continue
            if protected.get(mat_id, 0) > 0:
                protected[mat_id] -= 1
                if protected[mat_id] <= 0:
                    del protected[mat_id]
                continue
            to_return.append(mat_id)

        if not to_return:
            return

        self._log(
            f"  → return carried surplus at station {station_id}: {to_return}"
        )
        for mat_id in to_return:
            ok = self._node.arm_return_material_to_storage(
                material_id=mat_id, station_id=station_id
            )
            if not ok:
                raise ExecutionFailure(
                    f"Failed to return surplus material {mat_id} to station {station_id}"
                )
            self._ledger.add(station_id, mat_id)
        self._log_ledger(f'return-carried-surplus-{station_id}')

    def _run_pickup_phase(self) -> None:
        storage_entries = [
            e for e in self._plan.mid if not e.get('is_recycle_pickup')
        ]

        for idx, entry in enumerate(storage_entries):
            sid = entry['station_id']
            self._log(
                f"[{idx}/{len(storage_entries)-1}] navigate to storage station {sid}"
            )

            self._node.navigate_subgoal(sid)
            self._wait_for_intransit_assembly()
            self._node.navigate_goal(sid)
            self._return_matching_surplus_at_station(sid)

            needs_revisit = False
            for mat_id in entry.get('pickup_materials', []):
                if self._node.cargo_is_full():
                    self._log("  ! cargo 2-6 full — overflow drop at workbench")
                    self._node.call_post_process()
                    self._overflow_drop_at_workbench()
                    # Must revisit the station after returning from workbench.
                    needs_revisit = True

                if needs_revisit:
                    self._node.navigate_subgoal(sid)
                    self._wait_for_intransit_assembly()
                    self._node.navigate_goal(sid)
                    needs_revisit = False

                self._log(f"  → pick mat {mat_id}")
                ok = self._node.arm_pick_material(station_id=sid, material_id=mat_id)
                if not ok:
                    raise ExecutionFailure(
                        f"Failed to pick material {mat_id} from station {sid}"
                    )
                self._ledger.remove(sid, mat_id)

            # Start cargo ASSEMBLE before the exit maneuver so the arm can
            # work during backup/rotation and continue while driving to the
            # next station.
            self._start_ready_intransit_assembly()
            self._node.call_post_process()
            self._collect_ready_recycle_outputs(block=False)

            has_more_pickups = (idx < len(storage_entries) - 1)
            self._deliver_if_useful(
                has_more_pickups=has_more_pickups,
                include_pending_arm=not has_more_pickups,
            )
            self._collect_ready_workbench_products(block=False)

    # ------------------------------------------------------------------
    # In-transit assembly
    # ------------------------------------------------------------------

    def _start_ready_intransit_assembly(self) -> bool:
        """Start ASSEMBLE for the first in-slot product whose materials are all loaded.

        Iterates cargo slots 7 and 8 so the higher-priority slot (assigned first
        by the allocator) is always checked first. Only one ASSEMBLE is started
        per call because the arm is single-threaded.
        Returns True if an ASSEMBLE was started.
        """
        for cargo_id in (7, 8):
            product_id = self._allocator.get_slot_product(cargo_id)
            if product_id is None or cargo_id in self._started_intransit:
                continue
            # Check if all required materials are available in cargo 2-6.
            if not self._node.cargo_has_all_materials(product_id):
                continue

            self._started_intransit.add(cargo_id)
            self._log(
                f"  [READY] in-transit product {product_id} → cargo {cargo_id}; "
                "starting ASSEMBLE async"
            )
            self._node.arm_assemble_intransit_async(product_id, cargo_id)
            return True

        return False

    def _wait_for_intransit_assembly(self) -> None:
        """Block until arm is idle; drain any newly-ready assemblies."""
        while True:
            completed = self._node.wait_for_intransit_assembly()
            for handle in completed:
                if not handle.success:
                    self._started_intransit.discard(handle.cargo_id)
                    raise ExecutionFailure(
                        f"ASSEMBLE failed: product={handle.product_id}, "
                        f"cargo={handle.cargo_id}"
                    )

                if not self._allocator.mark_slot_assembled(
                    handle.cargo_id, handle.product_id
                ):
                    raise ExecutionFailure(
                        f"ASSEMBLE state mismatch: product={handle.product_id}, "
                        f"cargo={handle.cargo_id}"
                    )
                self._log(
                    f"  [DONE] in-transit product {handle.product_id} "
                    f"assembled on cargo {handle.cargo_id}"
                )

            if not self._start_ready_intransit_assembly():
                return

    # ------------------------------------------------------------------
    # Overflow drop at workbench (cargo 2-6 full)
    # ------------------------------------------------------------------

    def _overflow_drop_at_workbench(self) -> None:
        """Navigate to workbench and unload all cargo 2-6 materials as a buffer."""
        if self._en_route_to_wb:
            return
        self._en_route_to_wb = True

        wid = self._plan.workbench_station_id
        self._log(f"  → overflow drop: navigate to workbench {wid}")
        self._node.navigate_subgoal(wid)
        self._wait_for_intransit_assembly()
        self._node.navigate_goal(wid)
        dropped = self._node.cargo_materials_snapshot()
        if not self._node.arm_unload_all_materials():
            raise ExecutionFailure("Failed to unload overflow materials at workbench")
        for _, mat_id in dropped:
            self._ledger.add(wid, mat_id)
        self._log_ledger('overflow-drop')
        self._start_ready_workbench_production()
        self._node.call_post_process()

        self._en_route_to_wb = False

    # ------------------------------------------------------------------
    # Delivery
    # ------------------------------------------------------------------

    def _has_storage_pickup_entries(self) -> bool:
        """True if any storage/hybrid pickup remains in the static plan."""
        return any(
            (not e.get('is_recycle_pickup')) and bool(e.get('pickup_materials'))
            for e in self._plan.mid
        )

    def _delivery_needed_to_free_cargo_slot(self) -> bool:
        """Deliver early only when queued products cannot get cargo 7/8."""
        completed = self._allocator.get_completed_slots()
        queued = self._allocator.queued_products()
        allocated = self._allocator.allocated_products()
        return bool(completed and queued and len(allocated) >= 2)

    def _deliver_if_useful(
        self,
        has_more_pickups: bool,
        include_pending_arm: bool = False,
    ) -> None:
        """Avoid customer detours while useful material pickup work remains."""
        if not self._has_ready_deliveries(include_pending_arm=include_pending_arm):
            return

        if has_more_pickups and not self._delivery_needed_to_free_cargo_slot():
            self._log(
                "  → defer customer delivery; remaining material pickups take priority"
            )
            return

        if has_more_pickups:
            self._log(
                "  → deliver now to free cargo assembly slot before more pickups"
            )
        self._deliver_all()

    def _has_ready_deliveries(self, include_pending_arm: bool = False) -> bool:
        pending_arm = (
            include_pending_arm
            and hasattr(self._node, 'has_pending_intransit_assembly')
            and self._node.has_pending_intransit_assembly()
        )
        return bool(
            self._allocator.get_completed_slots()
            or self._wb_products_onboard
            or pending_arm
        )

    def _deliver_until_idle(self) -> None:
        """At mission end, keep delivering until cargo/WB products are drained."""
        while True:
            self._start_ready_intransit_assembly()
            if not self._has_ready_deliveries(include_pending_arm=True):
                return
            self._deliver_all()
            self._collect_ready_workbench_products(block=False)

    def _deliver_all(self) -> None:
        completed = self._allocator.get_completed_slots()
        wb_onboard = list(self._wb_products_onboard)
        pending_arm = (
            hasattr(self._node, 'has_pending_intransit_assembly')
            and self._node.has_pending_intransit_assembly()
        )
        if not completed and not wb_onboard and not pending_arm:
            return

        cid = self._plan.customer_station_id
        if pending_arm and not completed:
            self._log(
                f"  → navigate to customer {cid} while cargo ASSEMBLE is running"
            )
        else:
            self._log(f"  → navigate to customer {cid}")
        self._node.navigate_subgoal(cid)
        self._wait_for_intransit_assembly()
        self._node.navigate_goal(cid)

        # Deliver each completed in-transit slot directly from cargo 7/8.
        for slot in list(self._allocator.get_completed_slots()):
            self._log(
                f"  → deliver product {slot.product_id} from cargo {slot.cargo_id}"
            )
            if not self._node.arm_deliver(
                product_id=slot.product_id,
                from_cargo_id=slot.cargo_id,
            ):
                raise ExecutionFailure(
                    f"Failed to deliver product {slot.product_id} from cargo {slot.cargo_id}"
                )
            self._started_intransit.discard(slot.cargo_id)
            newly_queued = self._allocator.free_slot(slot.cargo_id)
            if newly_queued is not None:
                self._log(
                    f"  [QUEUE] product {newly_queued} now allocated to cargo {slot.cargo_id}"
                )

        for product_id in wb_onboard:
            self._log(f"  → deliver WB-produced product {product_id}")
            if not self._node.arm_deliver(product_id=product_id, from_cargo_id=0):
                raise ExecutionFailure(
                    f"Failed to deliver WB-produced product {product_id}"
                )
            self._wb_products_onboard.remove(product_id)

        self._node.call_post_process()
        self._start_ready_intransit_assembly()

    # ------------------------------------------------------------------
    # Return to home
    # ------------------------------------------------------------------

    def _return_to_home(self) -> None:
        """Navigate back to the home station (0 for A-side, 14 for B-side)."""
        home_id = self._plan.home_station_id
        self._log(f"Returning to home station {home_id}")
        self._node.navigate_subgoal(home_id)
        self._wait_for_intransit_assembly()
        self._node.navigate_goal(home_id)
        self._log(f"Arrived at home station {home_id} — mission complete")
        current = self._node.get_current_station_id()
        if current is not None and current != home_id:
            self._node.get_logger().error(
                f"[Executor] Home verification FAILED: "
                f"expected station {home_id}, navigator reports {current}"
            )
        else:
            self._log(f"Home verification OK: at station {home_id}")

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    def _ensure_docked_at_station(self, station_id: int, reason: str) -> None:
        if hasattr(self._node, 'is_docked_at_station') and self._node.is_docked_at_station(station_id):
            return
        self._log(f"  → redock station {station_id} before {reason}")
        self._node.navigate_subgoal(station_id)
        self._wait_for_intransit_assembly()
        self._node.navigate_goal(station_id)

    def _log_ledger(self, label: str) -> None:
        self._node.get_logger().info(
            f"[Executor][LEDGER] {label}: {self._ledger.snapshot()}"
        )

    def _log(self, msg: str) -> None:
        self._node.get_logger().info(f"[Executor] {msg}")
