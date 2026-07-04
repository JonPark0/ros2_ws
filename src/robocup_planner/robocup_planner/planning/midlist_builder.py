"""
Builds the midlist, bidlist, and the final mid (movement id list).

midlist — list-in-list of storage stations sorted by distance.
          Each element represents one station and its available materials.
          When recycling is needed, customer-counter entries are prepended
          as Phase 1, and storage distances are recalculated from the
          workbench position for Phase 2.

bidlist — same format as midlist, but for batch stations.
          IDs 10-80: materials field holds BATCH_COUNT units of the resolved
          raw material per batch ID. ID 90 (mix): is_mix_batch=True; materials
          field is populated via assign_mix_batch_materials() before inclusion.

mid     — midlist filtered to only the materials in net_aidlist,
          preserving distance order. This is the ordered pickup sequence
          the executor follows.
"""

from collections import Counter
from typing import Dict, List, Optional, Tuple

from robocup_planner.planning.distance_calculator import DistanceCalculator
from robocup_planner.product_catalog import BATCH_TO_MATERIAL, BATCH_COUNT, MIX_BATCH_ID


# Each midlist / bidlist entry (one station visit):
# {
#   'station_id':         int,
#   'materials':          [int, ...],   # raw material IDs available at this station
#   'distance':           float,
#   'is_recycle_pickup':  bool,         # True for customer-counter visits in Phase 1
#   'recycle_product_id': int | None,   # which product to pick up (Phase 1 only)
#   'is_batch':           bool,         # True for batch station entries  (optional key)
#   'is_mix_batch':       bool,         # True for ID-90 mix batch entries (optional key)
# }


def build_storage_midlist(
    storage_stations: List[Dict],
    calc: DistanceCalculator,
    ref_station_id: int,
) -> List[Dict]:
    """
    Sort storage stations by Euclidean distance from ref_station_id.
    ref_station_id is 0 (home) normally, or the workbench ID when recycling.
    """
    ref_pos = calc.get_position(ref_station_id) or (0.0, 0.0)
    entries = []
    for st in storage_stations:
        sid = st['station_id']
        dist = calc.point_to_station(ref_pos[0], ref_pos[1], sid)
        entries.append({
            'station_id': sid,
            'materials': list(st['material_ids']),
            'distance': dist,
            'is_recycle_pickup': False,
            'recycle_product_id': None,
        })
    entries.sort(key=lambda e: e['distance'])
    return entries


def build_recycle_phase_entries(
    customer_stations: List[Dict],
    recycle_orders: List[Dict],
    calc: DistanceCalculator,
    ref_station_id: int,
) -> List[Dict]:
    """
    Build Phase 1 entries: visits to customer counters to collect products
    that need to be recycled. Sorted by distance from ref_station_id (home).

    recycle_orders: list of {'station_id': int, 'product_id': int}
                    mapping each recycled product to its customer counter.
    """
    ref_pos = calc.get_position(ref_station_id) or (0.0, 0.0)
    entries = []
    for order in recycle_orders:
        sid = order['station_id']
        dist = calc.point_to_station(ref_pos[0], ref_pos[1], sid)
        entries.append({
            'station_id': sid,
            'materials': [],
            'distance': dist,
            'is_recycle_pickup': True,
            'recycle_product_id': order['product_id'],
        })
    entries.sort(key=lambda e: e['distance'])
    return entries


def build_bidlist(
    batch_stations: List[Dict],
    calc: DistanceCalculator,
    ref_station_id: int,
    include_mix: bool = False,
) -> List[Dict]:
    """
    Build a distance-sorted list of batch station entries.

    batch_stations: [{'station_id': int, 'batch_ids': [int, ...]}]
    - IDs 10-80: each resolved to BATCH_COUNT units of the raw material.
    - ID 90:     included only when include_mix=True; materials=[], is_mix_batch=True.

    Entries sorted by Euclidean distance from ref_station_id.
    """
    ref_pos = calc.get_position(ref_station_id) or (0.0, 0.0)
    entries = []
    for st in batch_stations:
        sid = st['station_id']
        dist = calc.point_to_station(ref_pos[0], ref_pos[1], sid)
        materials: List[int] = []
        is_mix = False
        for bid in st['batch_ids']:
            bid = int(bid)
            if bid == MIX_BATCH_ID:
                if include_mix:
                    is_mix = True
            else:
                raw = BATCH_TO_MATERIAL.get(bid)
                if raw is not None:
                    materials.extend([raw] * BATCH_COUNT)
        if materials or is_mix:
            entries.append({
                'station_id': sid,
                'materials': materials,
                'distance': dist,
                'is_recycle_pickup': False,
                'recycle_product_id': None,
                'is_batch': True,
                'is_mix_batch': is_mix,
            })
    entries.sort(key=lambda e: e['distance'])
    return entries


def merge_into_midlist(midlist: List[Dict], bidlist: List[Dict]) -> List[Dict]:
    """
    Insert bidlist entries into the storage phase of midlist, maintaining
    distance order.  Phase 1 (is_recycle_pickup=True) entries are not moved.
    """
    phase1 = [e for e in midlist if e.get('is_recycle_pickup')]
    phase2 = [e for e in midlist if not e.get('is_recycle_pickup')]
    merged = sorted(phase2 + bidlist, key=lambda e: e['distance'])
    return phase1 + merged


def assign_mix_batch_materials(
    bidlist_90: List[Dict],
    missing: Counter,
) -> List[Dict]:
    """
    Assign still-missing materials to mix batch (ID 90) stations.
    Stations are processed in distance order (closest first).
    Each station absorbs up to BATCH_COUNT units per material type.
    Returns entries with the materials field populated; empty-assignment
    entries are omitted.
    """
    remaining = Counter(missing)
    result = []
    for entry in bidlist_90:
        if not remaining:
            break
        assigned: List[int] = []
        for mat_id in list(remaining.keys()):
            take = min(remaining[mat_id], BATCH_COUNT)
            assigned.extend([mat_id] * take)
            remaining[mat_id] -= take
            if remaining[mat_id] == 0:
                del remaining[mat_id]
        if assigned:
            new_entry = dict(entry)
            new_entry['materials'] = assigned
            result.append(new_entry)
    return result


def build_full_midlist(
    storage_stations: List[Dict],
    customer_stations: List[Dict],
    recycle_orders: List[Dict],
    calc: DistanceCalculator,
    home_station_id: int,
    workbench_station_id: int,
    needs_recycling: bool,
    batch_stations_1080: Optional[List[Dict]] = None,
    batch_stations_90: Optional[List[Dict]] = None,
    missing_for_mix: Optional[Counter] = None,
) -> List[Dict]:
    """
    Assemble the complete midlist:
      Phase 1 (if recycling needed): customer counter visits, sorted from home.
      Phase 2: storage pickups, sorted from the correct reference point
               (workbench if recycling, home otherwise).
               Batch entries (10-80 and 90) are merged in distance order.

    batch_stations_1080: [{'station_id', 'batch_ids'}] — merged into Phase 2 when provided.
    batch_stations_90:   [{'station_id', 'batch_ids': [90]}] — merged when missing_for_mix given.
    missing_for_mix:     Counter of materials to assign arbitrarily to mix batch stations.

    Returns the concatenated list.  Phase 1 entries are marked is_recycle_pickup=True.
    """
    phase2_ref = workbench_station_id if needs_recycling else home_station_id

    if needs_recycling:
        phase1 = build_recycle_phase_entries(
            customer_stations, recycle_orders, calc, home_station_id
        )
    else:
        phase1 = []

    phase2 = build_storage_midlist(storage_stations, calc, phase2_ref)

    if batch_stations_1080:
        bidlist_1080 = build_bidlist(batch_stations_1080, calc, phase2_ref)
        phase2 = merge_into_midlist(phase2, bidlist_1080)

    if batch_stations_90 and missing_for_mix:
        bidlist_90 = build_bidlist(batch_stations_90, calc, phase2_ref, include_mix=True)
        assigned_90 = assign_mix_batch_materials(bidlist_90, missing_for_mix)
        if assigned_90:
            phase2 = merge_into_midlist(phase2, assigned_90)

    return phase1 + phase2


def check_storage_satisfies(
    storage_midlist: List[Dict],
    net_aidlist: Counter,
) -> Tuple[bool, Counter]:
    """
    Check whether the materials in storage can satisfy net_aidlist.
    Returns (satisfied, missing_materials).
    """
    available: Counter = Counter()
    for entry in storage_midlist:
        for mat in entry['materials']:
            available[mat] += 1

    missing: Counter = Counter()
    for mat_id, needed in net_aidlist.items():
        shortfall = needed - available.get(mat_id, 0)
        if shortfall > 0:
            missing[mat_id] = shortfall

    return (len(missing) == 0, missing)


def build_mid(
    midlist: List[Dict],
    net_aidlist: Counter,
) -> List[Dict]:
    """
    Filter midlist to only the materials needed by net_aidlist, preserving
    distance order.  Recycle-pickup entries (Phase 1) pass through unchanged.

    Each returned entry:
    {
      'station_id':         int,
      'pickup_materials':   [int, ...],
      'is_recycle_pickup':  bool,
      'recycle_product_id': int | None,
    }
    """
    remaining = Counter(net_aidlist)
    mid: List[Dict] = []

    for entry in midlist:
        if entry['is_recycle_pickup']:
            mid.append({
                'station_id': entry['station_id'],
                'pickup_materials': [],
                'is_recycle_pickup': True,
                'recycle_product_id': entry['recycle_product_id'],
            })
            continue

        pickup: List[int] = []
        for mat in entry['materials']:
            if remaining.get(mat, 0) > 0:
                pickup.append(mat)
                remaining[mat] -= 1
                if remaining[mat] == 0:
                    del remaining[mat]

        if pickup:
            mid.append({
                'station_id': entry['station_id'],
                'pickup_materials': pickup,
                'is_recycle_pickup': False,
                'recycle_product_id': None,
            })

    return mid
