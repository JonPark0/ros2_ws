# eai_task_server

ROS 2 Python package that publishes hardcoded EAI task messages.

**Note:** Examples in this package are created for testing purposes. There is no guarantee for identical tasks during competitions.

## Dependencies

- ROS 2 (tested on Humble; expected to work on Jazzy and newer distros as well)
- [`sml_messages`](https://github.com/robocup-sml/sml_messages) (must be available in your workspace and built before this package)

## Build + Setup

```bash
cd ~/ros2_ws && colcon build --packages-select sml_messages eai_task_server
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
```

Run the `source` command in each new terminal before using `ros2 run` / `ros2 launch`.

## Executables

| Command | Purpose |
|---|---|
| `task_publisher` | Publishes preset scenario/stage tasks on a timer |
| `task_listener` | Echoes received tasks for inspection |
| `task_complexity_publisher` | Interactive CLI that builds tier/stage tasks by complexity |
| `manual_order_server` | Builds one task by hand or from a YAML order file (see below) |
| `entry_server_0702` / `beginner_server_0702` | Snapshot servers for the 2026-07-02 entry/beginner runs |

## Run

```bash
ros2 run eai_task_server task_publisher --ros-args -p scenario:=production -p stage:=beginner
```

In a second terminal, run:

```bash
ros2 run eai_task_server task_listener
```

## Parameters

- `scenario`: `production`, `recycling`, or `lifecycle`
- `stage`: `entry`, `beginner`, or `advanced`
- `topic_name`: full task topic (default `/eai/task`)
- `side_a_topic_name`: side A only task topic (default `/eai/task/side_a`)
- `side_b_topic_name`: side B only task topic (default `/eai/task/side_b`)
- `publish_period_sec`: publish period in seconds (default `1.0`)
- `publish_once`: publish a single message and exit (default `false`)

## Publishing Topics

- `/eai/task`: full task (orders + stations from side A and side B)
- `/eai/task/side_a`: same orders, but only side A stations
- `/eai/task/side_b`: same orders, but only side B stations

## How to Adapt

Teams can adjust task definitions for local testing by editing the task builders directly in [./eai_task_server/task_publisher.py](./eai_task_server/task_publisher.py#L77).

Useful places to modify:

- Scenario/stage task contents (orders + required materials): [./eai_task_server/task_publisher.py](./eai_task_server/task_publisher.py#L77)
- Arena layout station map (side A + side B station list): [./eai_task_server/task_publisher.py](./eai_task_server/task_publisher.py#L21)
- Scenario/stage routing map (`scenario` + `stage` to builder function): [./eai_task_server/task_publisher.py](./eai_task_server/task_publisher.py#L239)

After changes, rebuild and source again:

```bash
cd ~/ros2_ws && colcon build --packages-select sml_messages eai_task_server
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
```

## Launch Server

```bash
ros2 launch eai_task_server task_server.launch.py scenario:=lifecycle stage:=advanced
```

One-shot launch example:

```bash
ros2 launch eai_task_server task_server.launch.py scenario:=production stage:=beginner publish_once:=true
```

## Launch Server + Listener

```bash
ros2 launch eai_task_server task_server_with_listener.launch.py scenario:=recycling stage:=advanced
```

Combined one-shot example:

```bash
ros2 launch eai_task_server task_server_with_listener.launch.py scenario:=production stage:=beginner publish_once:=true
```

## Manual Order Server

`manual_order_server` builds one task by hand (side, produce/recycle product IDs,
storage material_ids) and publishes it to `/eai/task` and the per-side topics.

After you confirm, it keeps republishing (same content, harmless) until it has
observed a `/eai/task` subscriber (e.g. `planner_node`) for a few consecutive
sends, then exits. This avoids losing the task to a DDS discovery race where
the node would otherwise publish once and tear down before the planner's
subscription match completes.

```bash
ros2 run eai_task_server manual_order_server
```

Without `order_file`, it looks for a default order first (see below); if none is
found it runs an interactive CLI that asks for the side, how many produce/recycle
orders to place, the product IDs, which recycled products already sit on the
customer counter, and the initial material_ids per storage station.

### Loading an order from a YAML file

Pass a relative path (resolved against the current working directory) via the
`order_file` ROS parameter to skip the interactive prompts and publish
non-interactively:

```bash
ros2 run eai_task_server manual_order_server --ros-args -p order_file:=src/eai_task_server/orders/example_order.yaml
```

### Default order file

If `order_file` is not given, `manual_order_server` looks in
[`orders/`](./orders) (the installed package share directory first, falling
back to the source-tree `orders/` folder next to this package) and, **only if
exactly one `*.yaml`/`*.yml` file is found there**, uses it automatically —
so `ros2 run eai_task_server manual_order_server` with no arguments publishes
that file non-interactively. If zero or more than one order file is present,
it falls back to the interactive CLI (and prints the ambiguous file list so
you can pass `order_file` explicitly instead).

> **Note:** this package currently ships two order files
> ([`orders/example_order.yaml`](./orders/example_order.yaml) and
> [`orders/adv_lifecycle.yaml`](./orders/adv_lifecycle.yaml)), so the no-args
> run is ambiguous and drops into the interactive CLI — pass `order_file`
> explicitly to publish one of them.

See [`orders/example_order.yaml`](./orders/example_order.yaml) for the file format:

```yaml
side: a
produce_ids: [13, 462, 711]
recycle_ids: [81, 442, 711]
customer_initial_ids: [81, 711]   # optional; omit to treat all recycle_ids as immediate
material_by_station:
  1: [1, 3, 7]
  2: [4, 6, 2]
  7: [8, 1]
```

- `side`: `a` or `b`
- `produce_ids` / `recycle_ids`: product IDs from the catalog in [`eai_task_server/order.py`](./eai_task_server/order.py)
- `customer_initial_ids`: subset of `recycle_ids` already on the customer table at plan time (drives the immediate vs. deferred recycle split); omit to default to all of `recycle_ids`
- `material_by_station`: initial `material_ids` keyed by station_id — storage **and hybrid** stations of the selected side are accepted (side a: `1`, `2`, `7`, `3`; side b: `12`, `13`, `7`, `11`)

### Explicit arena mode (`stations:`)

The hardcoded side layout cannot express every real arena (e.g. the
2026-07-04 field task typed station 3 as a storage station with batch stock
and had no second workbench or shared storage). Add a `stations:` list to the
order file to publish exactly those stations instead — see
[`orders/adv_lifecycle.yaml`](./orders/adv_lifecycle.yaml) for a
field-accurate example:

```yaml
side: a
produce_ids: [81, 711, 8518]
recycle_ids: [241, 462, 48132]
stations:
  - {station_id: 1, station_type: storage, material_ids: [10, 70]}
  - {station_id: 2, station_type: storage, material_ids: [20, 40, 60]}
  - {station_id: 3, station_type: storage, material_ids: [30, 50, 80]}
  - {station_id: 4, station_type: workbench}
  - {station_id: 6, station_type: customer, material_ids: [48132, 462, 241]}
```

- `station_type`: `storage`, `workbench`, `hybrid`, or `customer` (or the raw
  `sml_messages/Station` constant)
- `material_ids` on `storage`/`hybrid`: materials 1-8, known batches
  10/20/…/80, mix batch 90
- `material_ids` on the `customer` station: its initial product stock — every
  `recycle_id` listed there is an *immediate* recycle, the rest become
  *deferred* (produce → deliver → reclaim → disassemble). Omit it to fall back
  to `customer_initial_ids` (or all of `recycle_ids`). Specifying both and
  disagreeing is an error.
- `name` is optional; `side_<x>_<type>_<n>` names are generated when omitted
  (the planner's side filter relies on these prefixes)
- at least one `workbench` and exactly one `customer` station are required;
  when `stations:` is present, `material_by_station` is ignored and
  `SIDE_LAYOUT` is bypassed entirely
