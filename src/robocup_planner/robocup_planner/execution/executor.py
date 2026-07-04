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
  Every station approach is split into two legs, always via _approach(id):
    1. navigate_subgoal(id)  — fast cruise; arm may ASSEMBLE during this leg.
       At sub_goal: wait_for_intransit_assembly() blocks until arm is idle.
    2. navigate_goal(id)     — precision parking; arm must be idle.
  Never call navigate_subgoal()/navigate_goal() directly — always go
  through _approach() so this rule can't be silently skipped at a new
  call site.

Deferred recycle rule (lifecycle produce-then-recycle):
  A recycle order whose product_id has no matching stock on the customer
  counter at plan time can't be disassembled up front — the robot must
  assemble and deliver it first. Plan.deferred_recycle_ids lists those
  product_ids; _run_deferred_recycle_phase() reclaims them from the customer
  counter and disassembles them after every delivery is otherwise complete.

Fail-loud / self-recovery rule:
  Every navigate/arm/wb call the Executor makes is checked against its
  return value; a failure raises ExecutionFailure instead of silently
  continuing with a plan that no longer matches physical reality. run()
  catches any such failure, makes one best-effort attempt to return the AMR
  to its home station anyway (so a mid-mission fault never strands it in
  the arena), and then re-raises so the caller sees a clear error instead of
  a silently-incomplete or hung run.

If the last material for a cargo product is picked at the final storage
station, delivery still starts immediately: the AMR drives toward the customer
while ASSEMBLE runs, then waits at the customer sub_goal only if the arm has
not finished yet.
"""

from collections import Counter
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional

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
    # Product ids that are both produced and recycled this run, with no
    # matching stock on the customer counter at plan time. These must be
    # assembled, delivered, then picked back up for disassembly — handled by
    # Executor._run_deferred_recycle_phase() after normal delivery.
    deferred_recycle_ids: List[int] = field(default_factory=list)


class ExecutionFailure(RuntimeError):
    """Raised when a required runtime action fails and the plan cannot continue."""


def _tall_blocks_first(material_ids: Iterable[int]) -> List[int]:
    """Order picks so 4x2 blocks (IDs 5-8, 4 units tall) load before 2x2s.

    Cargo slots are 6 units tall. 2x2 blocks loaded first pair up into
    slots leaving 2-unit gaps a 4x2 can never use; loading the tall blocks
    while slots are still empty avoids that fragmentation (this exact
    pattern forced an avoidable workbench overflow detour in simulation of
    the 2026-07-04 field mission).
    """
    return sorted((int(m) for m in material_ids), key=lambda m: (m < 5, m))


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

        try:
            self._run_mission()
        except Exception as e:
            self._log(
                f"! Execution failed mid-plan ({e!r}) — attempting best-effort "
                "return to home so the AMR isn't stranded"
            )
            try:
                self._return_to_home()
            except Exception as home_err:
                self._log(f"! Best-effort return-to-home also failed: {home_err!r}")
            raise

        # Return to home station (0 for A-side, 14 for B-side)
        self._return_to_home()

        self._log("Executor finished")

    def _run_mission(self) -> None:
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
        # Last chance for queued WB PRODUCE whose set was split between the
        # shelf and cargo by clear-shelf collection.
        if self._workbench_queue:
            self._feed_workbench_production_from_cargo(self._plan.workbench_station_id)
            self._start_ready_workbench_production()
        self._collect_ready_workbench_products(block=True)
        self._deliver_until_idle()

        # Lifecycle produce-then-recycle products: reclaim from the customer
        # counter now that they've been delivered, disassemble, and return
        # the materials to storage.
        self._run_deferred_recycle_phase()

        # After the products are delivered, collect any recycle outputs that
        # were pure surplus and return them to their material station.
        self._collect_ready_recycle_outputs(block=True)
        self._return_surplus_materials()

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
            self._approach(station_id)
            self._require(
                self._node.arm_pick_product(station_id=station_id, product_id=pid),
                f"arm_pick_product(station={station_id}, product={pid})",
            )
            self._soft(self._node.call_post_process(), "call_post_process (customer)")

            self._log(f"  → navigate to workbench {workbench_id} for RECYCLE")
            self._approach(workbench_id)

            # Clear out the previous product's disassembled materials first —
            # the workbench shelf MUST be completely empty before the next
            # product is placed on it (hard physical invariant).
            if pending_wb_handle is not None:
                self._collect_recycled_materials(
                    pending_wb_handle, pending_wb_pid, workbench_id,
                    clear_shelf=True,
                )
                pending_wb_handle = None
                pending_wb_pid = None

            # Defensive: whatever path led here, never place a product on a
            # shelf that still has loose materials on it.
            self._pick_all_workbench_materials(workbench_id)

            self._require(
                self._node.arm_unload_product_to_workbench(
                    product_id=pid, station_id=workbench_id,
                ),
                f"arm_unload_product_to_workbench(product={pid}, station={workbench_id})",
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
                    pending_wb_handle, pending_wb_pid, workbench_id,
                    clear_shelf=not is_last,
                )
                pending_wb_handle = None
                pending_wb_pid = None
                self._soft(self._node.call_post_process(), "call_post_process (wb wait-here)")
                self._start_ready_intransit_assembly()
            else:
                self._soft(self._node.call_post_process(), "call_post_process (workbench)")
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
            # needed. Dropping it on the WB shelf is NOT an option here —
            # the shelf must be empty before the next product is placed on
            # it, so a drop would have to be re-picked immediately. Instead
            # return cargo surplus to its home storage stations now.
            if not is_last and self._node.cargo_is_full():
                self._log(
                    "  ! cargo full during recycle phase — "
                    "returning cargo surplus to storage before next pickup"
                )
                self._return_cargo_surplus_now()

        # Collect whatever disassembly is still pending after the last pickup.
        if pending_wb_handle is not None:
            self._collect_recycled_materials(
                pending_wb_handle, pending_wb_pid, workbench_id
            )
            self._soft(self._node.call_post_process(), "call_post_process (workbench tail)")

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
                self._soft(self._node.call_post_process(), "call_post_process (recycle outputs)")
                self._start_ready_intransit_assembly()

    def _collect_recycled_materials(
        self, handle, pid: int, workbench_id: int, clear_shelf: bool = False
    ) -> None:
        """Wait for a RECYCLE WbTask to finish, then pick up every material
        block it produced from the workbench shelf.

        clear_shelf=True enforces the hard physical invariant that the WB
        shelf must be COMPLETELY empty of loose materials before the next
        product is placed on it — every remaining shelf material (needed or
        surplus) is collected unconditionally. With clear_shelf=False
        (nothing will be placed on the shelf next), surplus may stay
        buffered on the shelf to keep cargo 2-6 free for production.
        """
        if not self._node.wait_for_wb_task(handle):
            raise ExecutionFailure(f"RECYCLE failed: product={pid}")

        for mat_id, cnt in get_material_count(pid).items():
            for _ in range(cnt):
                self._ledger.add(workbench_id, mat_id)
        self._log_ledger(f"recycle-output-product-{pid}")

        # Let WB arms consume recycled materials for assigned PRODUCE first.
        # Top up the shelf from cargo when that completes a queued product —
        # clear-shelf collection may have carried part of its set away.
        self._feed_workbench_production_from_cargo(workbench_id)
        self._start_ready_workbench_production()
        self._pick_needed_workbench_materials_for_amr(workbench_id)
        if clear_shelf:
            self._pick_all_workbench_materials(workbench_id)
        else:
            self._pick_surplus_workbench_materials_for_return(workbench_id)
        self._collect_ready_workbench_products(block=False)

    def _feed_workbench_production_from_cargo(self, workbench_id: int) -> None:
        """Hand cargo materials back to the WB shelf when that completes a
        queued WB PRODUCE set.

        The clear-shelf rule (shelf must be empty before a product is placed
        on it) can carry away materials a queued WB product was waiting for
        while its set arrives across multiple disassemblies. Once the rest
        of the set reaches the shelf, the missing blocks are sitting in
        cargo — without this hand-back the product would be starved forever.
        Only cargo blocks not needed by the AMR's own unassembled products
        are given away.
        """
        if not self._workbench_queue:
            return

        cargo_counts = Counter(
            mat_id for _, mat_id in self._node.cargo_materials_snapshot()
        )
        amr_need = self._remaining_product_material_need()
        spare = Counter(
            {m: c - amr_need.get(m, 0) for m, c in cargo_counts.items() if c > amr_need.get(m, 0)}
        )

        for product_id in list(self._workbench_queue):
            shelf = Counter(self._ledger.snapshot().get(workbench_id, {}))
            need = get_material_count(product_id)
            missing = Counter(
                {m: c - shelf.get(m, 0) for m, c in need.items() if c > shelf.get(m, 0)}
            )
            if not missing:
                continue  # _start_ready_workbench_production() will take it
            if any(spare.get(m, 0) < c for m, c in missing.items()):
                continue  # cargo can't complete this set without starving AMR work
            self._ensure_docked_at_station(
                workbench_id, f"feed WB PRODUCE {product_id} from cargo"
            )
            self._log(
                f"  → hand back {dict(missing)} from cargo to complete "
                f"WB PRODUCE {product_id}"
            )
            for mat_id, cnt in sorted(missing.items()):
                for _ in range(cnt):
                    self._require(
                        self._node.arm_unload_material(mat_id),
                        f"arm_unload_material({mat_id}) (feed WB PRODUCE {product_id})",
                    )
                    self._ledger.add(workbench_id, mat_id)
                    spare[mat_id] -= 1
        self._log_ledger('feed-wb-produce')

    def _pick_all_workbench_materials(self, workbench_id: int) -> None:
        """Unconditionally collect every loose material from the WB shelf.

        Called right before a product is placed on the workbench: leaving
        even one block there means the product lands on a pile. Frees cargo
        space by assembling first, then by returning cargo surplus to its
        home stations (and re-docking); if the shelf still cannot be
        cleared, fail loudly rather than cause a physical collision.
        """
        buffered = self._ledger.snapshot().get(workbench_id, {})
        if not buffered:
            return

        to_pick: List[int] = []
        for mat_id, count in buffered.items():
            to_pick.extend([int(mat_id)] * int(count))
        to_pick = _tall_blocks_first(to_pick)

        self._ensure_docked_at_station(
            workbench_id, "clear WB shelf before next product placement"
        )
        for mat_id in to_pick:
            if self._ledger.count(workbench_id, mat_id) <= 0:
                continue
            if not self._node.cargo_has_space_for(mat_id):
                if not self._try_free_cargo_space(mat_id):
                    # Last resort: hand back cargo surplus to its home
                    # stations to make room, then come back and continue.
                    self._log(
                        f"  ! shelf-clear needs space for mat {mat_id} — "
                        "returning cargo surplus to storage first"
                    )
                    self._soft(self._node.call_post_process(), "call_post_process (shelf-clear detour)")
                    self._return_cargo_surplus_now()
                    self._ensure_docked_at_station(
                        workbench_id, "resume WB shelf clearing"
                    )
                    if not self._node.cargo_has_space_for(mat_id):
                        raise ExecutionFailure(
                            f"Cannot clear WB shelf: no cargo space for material {mat_id}"
                        )
            self._log(f"    ← clear-shelf pick mat {mat_id}")
            ok = self._node.arm_pick_material(
                station_id=workbench_id, material_id=mat_id
            )
            if not ok:
                raise ExecutionFailure(
                    f"Failed to clear material {mat_id} off workbench {workbench_id}"
                )
            self._ledger.remove(workbench_id, mat_id)
            self._start_ready_intransit_assembly()
        self._log_ledger('clear-wb-shelf')

    def _return_cargo_surplus_now(self) -> None:
        """Return cargo materials not needed by remaining production to their
        home storage stations immediately (mid-mission pressure relief)."""
        protected = self._remaining_product_material_need()
        by_station: Dict[int, List[int]] = {}
        for _, mat_id in self._node.cargo_materials_snapshot():
            mat_id = int(mat_id)
            if protected.get(mat_id, 0) > 0:
                protected[mat_id] -= 1
                continue
            destination = self._plan.material_home_station.get(mat_id)
            if destination is None:
                continue
            by_station.setdefault(destination, []).append(mat_id)

        if not by_station:
            self._log("  ! no returnable cargo surplus — cargo holds only needed materials")
            return
        self._return_grouped_materials(by_station, 'mid-mission-surplus-return')

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

        # A finished product rides on cargo 1 to the customer; cargo 1 holds
        # only one product at a time. Collect at most as many as cargo 1 can
        # take — the rest stay pending on the WB shelf (their handle.event is
        # already set, so the next collect call after a delivery frees
        # cargo 1 picks them up immediately).
        collectable = []
        for handle in ready:
            occupied = (
                hasattr(self._node, 'cargo1_is_occupied')
                and self._node.cargo1_is_occupied()
            )
            if occupied or collectable:
                self._pending_wb_produce.append(handle)
            else:
                collectable.append(handle)
        if not collectable:
            self._log(
                "  → cargo 1 occupied; leaving finished WB products on shelf for now"
            )
            return

        wid = self._plan.workbench_station_id
        self._log(
            f"  → collect WB-produced products {[h.product_id for h in collectable]}"
        )
        self._ensure_docked_at_station(wid, "collect WB-produced products")
        for handle in collectable:
            self._require(
                self._node.arm_pick_product(
                    station_id=wid, product_id=handle.product_id
                ),
                f"arm_pick_product(WB product={handle.product_id})",
            )
            self._wb_products_onboard.append(handle.product_id)
        self._soft(self._node.call_post_process(), "call_post_process (wb collect)")
        self._start_ready_workbench_production()

    def _pick_needed_workbench_materials_for_amr(self, workbench_id: int) -> None:
        """Pick only WB materials still needed by AMR cargo assembly."""
        remaining_need = self._remaining_material_need()
        buffered = self._ledger.snapshot().get(workbench_id, {})
        if not remaining_need or not buffered:
            return

        to_pick: List[int] = []
        for mat_id, count in buffered.items():
            n = min(int(count), int(remaining_need.get(int(mat_id), 0)))
            to_pick.extend([int(mat_id)] * n)
        if not to_pick:
            return
        to_pick = _tall_blocks_first(to_pick)

        self._ensure_docked_at_station(
            workbench_id, "pick recycled materials needed by AMR assembly"
        )
        for mat_id in to_pick:
            if self._ledger.count(workbench_id, mat_id) <= 0:
                continue
            if not self._node.cargo_has_space_for(mat_id):
                if not self._try_free_cargo_space(mat_id):
                    self._log(
                        f"  ! no cargo space for mat {mat_id}; "
                        "leaving it on the WB shelf (ledger keeps it)"
                    )
                    continue
            self._log(f"    ← pick recycled mat {mat_id} for AMR assembly")
            ok = self._node.arm_pick_material(
                station_id=workbench_id, material_id=mat_id
            )
            if not ok:
                raise ExecutionFailure(
                    f"Failed to pick recycled material {mat_id} from workbench {workbench_id}"
                )
            self._ledger.remove(workbench_id, mat_id)
            # Consume complete sets into cargo 7/8 as they arrive.
            self._start_ready_intransit_assembly()
        self._log_ledger('pick-wb-materials-for-amr')

    def _pick_surplus_workbench_materials_for_return(self, workbench_id: int) -> None:
        """Pick WB surplus materials so they can be returned on route.

        Only loads surplus once no production work remains: surplus riding
        in cargo 2-6 through the pickup phase is dead weight that starves
        the slots needed for production materials (the 2026-07-04 field run
        entered Phase 2 with 20/30 units of mostly-surplus cargo and could
        not fit its 4x2 pickups). While work remains, surplus stays on the
        WB shelf — the ledger tracks it and _return_surplus_materials()
        fetches it at mission end.
        """
        buffered = self._ledger.snapshot().get(workbench_id, {})
        if not buffered:
            return

        if self._has_main_production_work_remaining():
            self._log(
                "  → leaving surplus on WB shelf for now; production work remains"
            )
            return

        # Keep any material still needed for AMR production on the WB ledger.
        remaining_need = self._remaining_material_need()
        to_pick: List[int] = []
        for mat_id, count in buffered.items():
            mat_id = int(mat_id)
            destination = self._plan.material_home_station.get(mat_id)
            if destination is None or destination == workbench_id:
                continue
            surplus_count = max(0, int(count) - int(remaining_need.get(mat_id, 0)))
            to_pick.extend([mat_id] * surplus_count)

        if not to_pick:
            return
        to_pick = _tall_blocks_first(to_pick)

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
            self._soft(self._node.call_post_process(), "call_post_process (wb recovery)")

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

        wid = self._plan.workbench_station_id
        cargo_counts = Counter(
            mat_id for _, mat_id in self._node.cargo_materials_snapshot()
        )
        protected_for_produce = self._remaining_product_material_need()

        # Split each surplus material into: already in cargo / still buffered
        # on the WB shelf (deliberately left there during the mission so it
        # never squeezed cargo space) / genuinely unaccounted. Keep cargo
        # materials that are still needed by AMR cargo assembly.
        available_in_cargo: Dict[int, int] = {}
        fetch_from_wb: Dict[int, int] = {}
        kept_for_produce: Dict[int, int] = {}
        skipped: Dict[int, int] = {}
        for mat_id, cnt in surplus.items():
            mat_id = int(mat_id)
            in_cargo = int(cargo_counts.get(mat_id, 0))
            protect = min(in_cargo, int(protected_for_produce.get(mat_id, 0)))
            if protect > 0:
                kept_for_produce[mat_id] = protect
            take_cargo = min(int(cnt), max(0, in_cargo - protect))
            if take_cargo > 0:
                available_in_cargo[mat_id] = take_cargo

            missing = int(cnt) - take_cargo
            destination = self._plan.material_home_station.get(mat_id)
            if missing > 0 and destination is not None and destination != wid:
                fetch = min(missing, self._ledger.count(wid, mat_id))
                if fetch > 0:
                    fetch_from_wb[mat_id] = fetch
                missing -= fetch
            if missing > 0:
                skipped[mat_id] = missing

        if kept_for_produce:
            self._log(
                f"  → keeping recycled cargo materials for AMR produce: {kept_for_produce}"
            )
        if skipped:
            self._log(
                f"  ! surplus neither in cargo nor on WB shelf, leaving: {skipped}"
            )

        # Fetch the WB-buffered surplus now that cargo 2-6 is free.
        fetched: Counter = Counter()
        if fetch_from_wb:
            self._log(
                f"  → fetch buffered surplus from workbench {wid}: {fetch_from_wb}"
            )
            self._ensure_docked_at_station(wid, "fetch buffered surplus for return")
            fetch_order = _tall_blocks_first(
                [m for m, c in fetch_from_wb.items() for _ in range(c)]
            )
            for mat_id in fetch_order:
                if self._ledger.count(wid, mat_id) <= 0:
                    continue
                if not self._node.cargo_has_space_for(mat_id):
                    self._log(
                        f"  ! no cargo space for surplus mat {mat_id}; "
                        "leaving it on the WB shelf"
                    )
                    continue
                self._require(
                    self._node.arm_pick_material(
                        station_id=wid, material_id=mat_id
                    ),
                    f"arm_pick_material(station={wid}, material={mat_id}) (surplus)",
                )
                self._ledger.remove(wid, mat_id)
                fetched[mat_id] += 1
            self._soft(self._node.call_post_process(), "call_post_process (surplus fetch)")
            self._log_ledger('surplus-fetch-from-wb')

        # Group by destination station so each station is visited once.
        by_station: Dict[int, List[int]] = {}
        for mat_id in sorted(set(available_in_cargo) | set(fetched)):
            count = available_in_cargo.get(mat_id, 0) + fetched.get(mat_id, 0)
            if count <= 0:
                continue
            station_id = self._plan.material_home_station.get(mat_id, wid)
            by_station.setdefault(station_id, []).extend([mat_id] * count)

        if not by_station:
            return

        self._log(f"Returning surplus recycled materials: {surplus}")
        self._return_grouped_materials(by_station, 'return-surplus-storage')

    def _return_grouped_materials(self, by_station: Dict[int, List[int]], label: str) -> None:
        """Drive to each destination station once and return the given materials."""
        for station_id, mat_ids in by_station.items():
            self._log(f"  → navigate to storage {station_id} to return {mat_ids}")
            self._approach(station_id)
            for mat_id in mat_ids:
                self._require(
                    self._node.arm_return_material_to_storage(
                        material_id=mat_id, station_id=station_id,
                    ),
                    f"arm_return_material_to_storage(material={mat_id}, station={station_id})",
                )
                self._ledger.add(station_id, mat_id)
            self._log_ledger(f"{label}-{station_id}")
            self._soft(self._node.call_post_process(), "call_post_process (return storage)")

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

            self._approach(sid)
            self._return_matching_surplus_at_station(sid)

            needs_revisit = False
            for mat_id in _tall_blocks_first(entry.get('pickup_materials', [])):
                # Per-material space check: a 4x2 block needs 4 free units,
                # which cargo_is_full() (2-unit check) cannot see. Prefer
                # consuming loaded materials into a cargo 7/8 ASSEMBLE over
                # a workbench overflow detour.
                if not self._node.cargo_has_space_for(mat_id):
                    if not self._try_free_cargo_space(mat_id):
                        self._log(
                            f"  ! no cargo space for mat {mat_id} — "
                            "overflow drop at workbench"
                        )
                        self._soft(self._node.call_post_process(), "call_post_process (pre-overflow)")
                        self._overflow_drop_at_workbench()
                        # Must revisit the station after returning from workbench.
                        needs_revisit = True

                if needs_revisit:
                    self._approach(sid)
                    needs_revisit = False

                self._log(f"  → pick mat {mat_id}")
                self._require(
                    self._node.arm_pick_material(station_id=sid, material_id=mat_id),
                    f"arm_pick_material(station={sid}, material={mat_id})",
                )
                self._ledger.remove(sid, mat_id)
                # Use materials the moment a product's set completes so
                # slots 2-6 drain into cargo 7/8 as pickups come in.
                self._start_ready_intransit_assembly()

            # Start cargo ASSEMBLE before the exit maneuver so the arm can
            # work during backup/rotation and continue while driving to the
            # next station.
            self._start_ready_intransit_assembly()
            self._soft(self._node.call_post_process(), "call_post_process (storage)")
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

    def _try_free_cargo_space(self, material_id: int) -> bool:
        """Free cargo 2-6 space for material_id by consuming loaded materials
        into an in-transit ASSEMBLE right now.

        Assembling moves a complete material set from slots 2-6 onto cargo
        7/8, so it is the preferred way to make room — materials get used
        the moment their product's set is complete instead of piling up.
        Falls through (returns False) when nothing is ready to assemble and
        nothing is already assembling; callers then overflow-drop or leave
        the material where it is.
        """
        for _ in range(3):  # at most: drain pending, then slots 7 and 8
            if self._node.cargo_has_space_for(material_id):
                return True
            started = self._start_ready_intransit_assembly()
            pending = self._node.has_pending_intransit_assembly()
            if not started and not pending:
                return False
            self._log(
                f"  → cargo has no space for mat {material_id}; "
                "assembling loaded materials to free slots"
            )
            self._wait_for_intransit_assembly()
        return self._node.cargo_has_space_for(material_id)

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
        self._approach(wid)
        dropped = self._node.cargo_materials_snapshot()
        self._require(
            self._node.arm_unload_all_materials(),
            "arm_unload_all_materials (overflow drop)",
        )
        for _, mat_id in dropped:
            self._ledger.add(wid, mat_id)
        self._log_ledger('overflow-drop')
        self._start_ready_workbench_production()
        # Take back the dropped materials still needed for AMR production —
        # cargo is empty now, so they fit. Without this they'd be stranded
        # on the shelf and their products would never assemble.
        self._pick_needed_workbench_materials_for_amr(wid)
        self._soft(self._node.call_post_process(), "call_post_process (overflow)")

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
                # A finished WB product may still be on the shelf because
                # cargo 1 was occupied when it completed — collect it now
                # that everything onboard has been delivered.
                if self._pending_wb_produce:
                    self._collect_ready_workbench_products(block=True)
                    continue
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
        self._approach(cid)

        # Deliver each completed in-transit slot directly from cargo 7/8.
        for slot in list(self._allocator.get_completed_slots()):
            self._log(
                f"  → deliver product {slot.product_id} from cargo {slot.cargo_id}"
            )
            self._require(
                self._node.arm_deliver(
                    product_id=slot.product_id, from_cargo_id=slot.cargo_id,
                ),
                f"arm_deliver(product={slot.product_id}, cargo={slot.cargo_id})",
            )
            self._started_intransit.discard(slot.cargo_id)
            newly_queued = self._allocator.free_slot(slot.cargo_id)
            if newly_queued is not None:
                self._log(
                    f"  [QUEUE] product {newly_queued} now allocated to cargo {slot.cargo_id}"
                )

        for product_id in wb_onboard:
            self._log(f"  → deliver WB-produced product {product_id}")
            self._require(
                self._node.arm_deliver(product_id=product_id, from_cargo_id=0),
                f"arm_deliver(WB product={product_id})",
            )
            self._wb_products_onboard.remove(product_id)

        self._soft(self._node.call_post_process(), "call_post_process (delivery)")
        self._start_ready_intransit_assembly()

    # ------------------------------------------------------------------
    # Deferred recycle — produce-then-recycle products with no initial
    # customer-counter stock (run after delivery, before returning home)
    # ------------------------------------------------------------------

    def _run_deferred_recycle_phase(self) -> None:
        """Recycle products that had to be assembled and delivered first.

        These share a product id with a produce order, but had no matching
        stock on the customer counter at plan time — so there was nothing
        to disassemble in Phase 1. Now that the freshly-assembled product
        has been delivered to the customer, pick it back up, disassemble it
        at the workbench, and return every reclaimed material directly to
        its home storage station (production is already done, so nothing
        else needs these materials).
        """
        deferred_ids = self._plan.deferred_recycle_ids
        if not deferred_ids:
            return

        self._log(f"Deferred recycle phase: {len(deferred_ids)} product(s)")
        customer_id = self._plan.customer_station_id
        workbench_id = self._plan.workbench_station_id

        for pid in deferred_ids:
            self._log(f"  → navigate to customer station {customer_id} to reclaim product {pid}")
            self._approach(customer_id)
            self._require(
                self._node.arm_pick_product(station_id=customer_id, product_id=pid),
                f"arm_pick_product(station={customer_id}, product={pid})",
            )
            self._soft(self._node.call_post_process(), "call_post_process (deferred pickup)")

            self._log(f"  → navigate to workbench {workbench_id} for RECYCLE")
            self._approach(workbench_id)
            # The shelf must be empty before the product is placed on it:
            # collect any still-pending RECYCLE outputs, then clear every
            # loose material (e.g. Phase 2 overflow buffers) off the shelf.
            self._collect_ready_recycle_outputs(block=True)
            self._pick_all_workbench_materials(workbench_id)
            self._require(
                self._node.arm_unload_product_to_workbench(
                    product_id=pid, station_id=workbench_id,
                ),
                f"arm_unload_product_to_workbench(product={pid}, station={workbench_id})",
            )
            handle = self._node.wb_task_async('RECYCLE', pid)
            # Nothing else to fetch while this disassembles (deferred recycle
            # runs one product at a time) — retreat to the sub_goal to wait
            # instead of idling docked at the goal.
            self._require(
                self._node.navigate_subgoal(workbench_id),
                f"navigate_subgoal({workbench_id})",
            )
            if not self._node.wait_for_wb_task(handle):
                raise ExecutionFailure(f"Deferred RECYCLE failed: product={pid}")
            self._require(
                self._node.navigate_goal(workbench_id),
                f"navigate_goal({workbench_id})",
            )
            for mat_id, cnt in get_material_count(pid).items():
                for _ in range(cnt):
                    self._log(f"    ← pick recycled mat {mat_id} from workbench")
                    self._require(
                        self._node.arm_pick_material(
                            station_id=workbench_id, material_id=mat_id,
                        ),
                        f"arm_pick_material(station={workbench_id}, material={mat_id})",
                    )
            self._soft(self._node.call_post_process(), "call_post_process (deferred recycle)")

            by_station: Dict[int, List[int]] = {}
            for mat_id, cnt in get_material_count(pid).items():
                station_id = self._plan.material_home_station.get(
                    mat_id, workbench_id
                )
                by_station.setdefault(station_id, []).extend([mat_id] * cnt)
            self._return_grouped_materials(by_station, f'deferred-recycle-return-{pid}')

    # ------------------------------------------------------------------
    # Return to home
    # ------------------------------------------------------------------

    def _return_to_home(self) -> None:
        """Navigate back to the home station (0 for A-side, 14 for B-side)."""
        home_id = self._plan.home_station_id
        self._log(f"Returning to home station {home_id}")
        self._approach(home_id)
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

    def _require(self, ok: bool, desc: str) -> None:
        """Raise ExecutionFailure if a required call did not succeed, instead
        of silently continuing with a plan that no longer matches physical
        reality."""
        if not ok:
            raise ExecutionFailure(f"{desc} failed")

    def _soft(self, ok: bool, desc: str) -> None:
        """Log (but don't abort on) failure of a best-effort call, e.g. the
        post-process exit maneuver — its own failure shouldn't stop the
        mission, but should be visible rather than silently swallowed."""
        if not ok:
            self._node.get_logger().warning(f"[Executor] {desc} did not succeed (continuing)")

    def _approach(self, station_id: int) -> None:
        """Approach station_id following the two-phase navigation rule.

        Always route through the sub_goal (open-space) leg before the
        precision-parking goal leg, and drain any in-transit ASSEMBLE so the
        arm is guaranteed idle before docking. This is the only sanctioned
        way to reach a station's docking goal — calling navigate_goal()
        directly skips the transit window for in-transit assembly and risks
        a close-quarters, bad-heading redock (e.g. from a post_process exit
        pose), which is how the recycle-phase AMR mispositioning bug arose.
        """
        self._require(self._node.navigate_subgoal(station_id), f"navigate_subgoal({station_id})")
        self._wait_for_intransit_assembly()
        self._require(self._node.navigate_goal(station_id), f"navigate_goal({station_id})")

    def _ensure_docked_at_station(self, station_id: int, reason: str) -> None:
        if hasattr(self._node, 'is_docked_at_station') and self._node.is_docked_at_station(station_id):
            return
        self._log(f"  → redock station {station_id} before {reason}")
        self._approach(station_id)

    def _log_ledger(self, label: str) -> None:
        self._node.get_logger().info(
            f"[Executor][LEDGER] {label}: {self._ledger.snapshot()}"
        )

    def _log(self, msg: str) -> None:
        self._node.get_logger().info(f"[Executor] {msg}")
