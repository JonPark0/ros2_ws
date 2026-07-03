"""
Product and material definitions for RoboCup assembly challenge.

Material IDs:
  2x2: Red=1, Green=2, Blue=3, Yellow=4
  2x4: Red=5, Green=6, Blue=7, Yellow=8

Product IDs encode the material sequence used in the name / assembly diagram.
"""

from collections import Counter
from typing import List, Optional

# Size of each material block
MATERIAL_SIZE: dict = {
    1: '2x2', 2: '2x2', 3: '2x2', 4: '2x2',
    5: '2x4', 6: '2x4', 7: '2x4', 8: '2x4',
}

# Batch ID → raw material ID (IDs 10-80: known type, ≥5 blocks guaranteed)
BATCH_TO_MATERIAL: dict = {
    10: 1, 20: 2, 30: 3, 40: 4,
    50: 5, 60: 6, 70: 7, 80: 8,
}
BATCH_COUNT: int = 5    # treat each batch station as having exactly this many blocks
MIX_BATCH_ID: int = 90  # unknown type/count batch

# All 11 product definitions.
# Single-column products list 'blocks' in bottom-to-top build order.
# Workbench-only products list 'layers' in bottom-to-top order; each
# layer is a list of block IDs placed side-by-side at that level.
PRODUCTS: dict = {
    81: {
        'name': 'E-Stop',
        'workbench_only': False,
        'blocks': [8, 1],
    },
    34: {
        'name': 'Battery',
        'workbench_only': False,
        'blocks': [3, 4],
    },
    13: {
        'name': 'Magnet',
        'workbench_only': False,
        'blocks': [1, 3],
    },
    442: {
        'name': 'Carrot',
        'workbench_only': False,
        'blocks': [4, 4, 2],
    },
    241: {
        'name': 'Traffic Light',
        'workbench_only': False,
        'blocks': [2, 4, 1],
    },
    462: {
        'name': 'Small Tree',
        'workbench_only': False,
        'blocks': [4, 6, 2],
    },
    4482: {
        'name': 'Big Carrot',
        'workbench_only': False,
        'blocks': [4, 4, 8, 2],
    },
    711: {
        'name': 'Hammer',
        'workbench_only': False,
        'blocks': [1, 1, 7],
    },
    # --- Workbench-only: contain side-by-side layers ---
    8518: {
        'name': 'Big Tree',
        'workbench_only': True,
        # Bottom layer: [8], middle: [5, 1] side-by-side, top: [8]
        'layers': [[8], [5, 1], [8]],
    },
    46262: {
        'name': 'Ice Cream',
        'workbench_only': True,
        # Bottom: [4], then [6,2] side-by-side, then [6], top: [2]
        'layers': [[4], [6, 2], [6], [2]],
    },
    48132: {
        'name': 'Burger',
        'workbench_only': True,
        # Bottom: [4], then [8], then [1,3] side-by-side, top: [2]
        'layers': [[4], [8], [1, 3], [2]],
    },
}


def get_material_count(product_id: int) -> Counter:
    """Frequency map of materials required for one unit of product_id."""
    p = PRODUCTS[product_id]
    if p['workbench_only']:
        materials = [m for layer in p['layers'] for m in layer]
    else:
        materials = p['blocks']
    return Counter(materials)


def is_intransit_eligible(product_id: int) -> bool:
    """True if the AMR cargo arm should assemble this product.

    Products not assigned to WB PRODUCE are eligible for AMR cargo-arm
    assembly.  Multi-layer products can still be flattened for cargo-arm
    fallback when they are not covered by recycled workbench materials.
    """
    return product_id in PRODUCTS


def get_base_block(product_id: int) -> int:
    """The bottom-most block for a single-column product."""
    p = PRODUCTS[product_id]
    if p['workbench_only']:
        raise ValueError(f"Product {product_id} is workbench-only and has no single base block")
    return p['blocks'][0]


def get_build_order(product_id: int) -> List[int]:
    """Block IDs in assembly order (bottom → top) for AMR cargo assembly.

    Multi-layer products are flattened layer-by-layer.  If the arm service
    later supports side-by-side placement metadata, get_all_layers() can be
    used there without changing planner sequencing.
    """
    p = PRODUCTS[product_id]
    if p['workbench_only']:
        return [m for layer in p['layers'] for m in layer]
    return list(p['blocks'])


def get_all_layers(product_id: int) -> List[List[int]]:
    """All layers for a workbench-only product, bottom → top."""
    p = PRODUCTS[product_id]
    if not p['workbench_only']:
        return [[b] for b in p['blocks']]
    return [list(layer) for layer in p['layers']]


def product_complexity_score(product_id: int) -> int:
    """Higher score means the product is better suited to workbench arms."""
    p = PRODUCTS[product_id]
    material_count = sum(get_material_count(product_id).values())
    layer_bonus = 100 if p.get('workbench_only') else 0
    wide_layer_bonus = 10 * sum(1 for layer in p.get('layers', []) if len(layer) > 1)
    return layer_bonus + wide_layer_bonus + material_count


def product_name(product_id: int) -> str:
    return PRODUCTS.get(product_id, {}).get('name', f'Unknown({product_id})')


def validate_product_id(product_id: int) -> bool:
    return product_id in PRODUCTS
