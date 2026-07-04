# robocup_planner

RoboCup SML 경기용 계획·실행 통합 노드.  
`sml_planning_node` + `sml_manager_node` 의 역할을 단일 노드로 수행합니다.

---

## 아키텍처

```
/eai/task
      │
      ▼
[PlannerNode]
  ┌── 계획 단계 ──────────────────────────────────────────────────┐
  │  order_list       → produce / recycle 분류                    │
  │  recycle 주문     → immediate(초기 customer 재고 있음) vs     │
  │                     deferred(생산→납품→회수→분해 lifecycle)   │
  │  produce 주문     → workbench_products (재활용 재료로 조립    │
  │                     가능한 제품, 복잡한 것 우선) vs           │
  │                     intransit_products (AMR cargo 조립)       │
  │  aidlist          → amr_net_aidlist (재활용 차감, WB 예약 후) │
  │  arena_layout     → mid (거리순 pickup 시퀀스, batch 지원)    │
  │                   → material_home_station (재료별 원래 보관소)│
  │                   → storage_ledger_seed (초기 재고 원장)      │
  │  잉여 재료        → surplus_recycled                          │
  │  우선순위         → 경로상 완성 순서 + lifecycle boost        │
  └────────────────────────────────────────────────────────────────┘
      │
      ▼
[Executor] (백그라운드 스레드) + [StorageLedger] (런타임 재고 원장)
  Phase 1  : immediate 재활용 픽업 (customer → workbench 분해, 비동기)
             분해 산출물 → WB PRODUCE 우선 소비 → AMR 필요분 픽업
  Phase 1.5: WB 버퍼 재료 회수 / WB 완성품 수거
  Phase 2  : 재료 픽업 루프 (storage/batch, 경유 시 잉여 반납)
  Phase 3  : 납품 (cargo 7/8 완성품 + WB 완성품)
  Phase 4  : deferred 재활용 (납품한 제품 회수 → 분해 → 재료 반납)
  Phase 5  : 잉여 재활용 재료 반납 → home 복귀
```

### 하이브리드 조립 분배 (WB PRODUCE / AMR in-transit)

RECYCLE 분해로 워크벤치 선반에 생기는 재료로 완전히 조립 가능한 produce
제품은 **워크벤치 PRODUCE**에 배정됩니다(`product_complexity_score`가 높은
제품 — workbench 전용 side-by-side 레이어 제품 — 우선). 나머지 제품은 기존처럼
AMR cargo 7/8 **in-transit 조립** 큐에 남습니다.

- WB PRODUCE는 `wb_task_async()`로 논블로킹 실행되며, `wb_produce_parallelism`
  (기본 2, 워크벤치 로봇팔 2대)만큼 동시에 진행됩니다.
- 완성된 WB 제품은 cargo 1에 실어 customer로 납품합니다. cargo 1은 한 번에
  한 제품만 적재하므로(`try_occupy_cargo1` 가드), 초과분은 납품으로 cargo 1이
  빌 때까지 WB 선반에 남겨 둡니다.
- workbench 전용 제품(8518 Burger, 46262 Big Tree, 48132 Ice Cream)도 재활용
  재료가 부족하면 레이어를 평탄화(flatten)해 AMR cargo 조립으로 폴백합니다.

### StorageLedger — 런타임 재고 원장

`execution/storage_ledger.py`는 AMR cargo 밖(보관소/워크벤치 선반)에 물리적으로
존재하는 재료의 위치·수량을 추적합니다. 분해 산출물, 오버플로 버퍼 드랍,
잉여 반납이 모두 원장에 기록되어 lifecycle 미션 중 재료를 잃어버리지 않습니다.
WB PRODUCE 시작 시 원장에서 재료를 예약하며, 불일치 시 즉시 실패(fail-loud)합니다.

### Phase 1 상세 — immediate 재활용 픽업 및 분해 (비동기 처리)

초기 arena_layout의 customer counter에 이미 놓여 있는(immediate) 재활용
제품만 Phase 1에서 처리합니다. 워크벤치 분해(RECYCLE)는 `wb_task_async()`로
**논블로킹 실행**되어 워크벤치 분해와 AMR 주행이 동시에 진행됩니다.

- 워크벤치에 새 제품을 내려놓기 **전에**, 이전 분해 산출물을 먼저 회수합니다
  (`_collect_recycled_materials()`).
- 분해 산출물은 ① WB PRODUCE 예약분 소비 → ② AMR 조립에 필요한 만큼만 픽업
  순으로 처리합니다. **잉여분은 생산 작업이 남아 있는 동안 WB 선반(원장)에
  남겨 두고**, 미션 마지막의 잉여 반납 단계에서 회수합니다 — 잉여가 cargo
  2-6을 점유해 생산용 픽업 공간을 잠식하지 않게 하기 위함입니다.
- cargo 2-6이 가득 차면(`cargo_is_full()`) 이미 워크벤치에 있는 김에 자재를
  버퍼로 내려놓습니다(`arm_unload_all_materials()` + 원장 기록).
- **대기 정책** (`wb_recycle_wait_mode`): `auto`(기본)는 겹칠 만한 AMR 생산
  작업이 남아 있지 않은 마지막 recycle에 한해 WB 옆에서 대기합니다. 대기 시
  회전 없이 `wb_clearance_backup_distance`(기본 10cm)만 후진했다가 재도킹해
  분해 산출물을 바로 회수합니다. `always`/`never`로 강제할 수 있습니다.

### Phase 4 — deferred 재활용 (lifecycle produce-then-recycle)

같은 product_id가 produce와 recycle 주문에 모두 있고 초기 customer 재고가
없으면, 그 제품은 **만들어 납품한 뒤에야** 분해할 수 있습니다
(`Plan.deferred_recycle_ids`). 납품이 모두 끝난 후
`_run_deferred_recycle_phase()`가 customer counter에서 제품을 회수해
워크벤치에서 분해하고, 재료를 `material_home_station`으로 반납합니다.
분해 대기 중에는 sub_goal로 물러나 대기합니다. 이 제품들은
`deferred_recycle_priority_boost`(기본 1000배)로 cargo 7/8 우선순위가
증폭되어 produce→deliver→reclaim 루프를 최대한 일찍 돌기 시작합니다.

### Phase 5 — 잉여 재활용 재료 반납

분해로 얻은 재료 중 주문에 필요한 개수를 초과하는 만큼(`surplus_recycled`)은
`material_home_station`(재료별 원래 보관소)으로 반납합니다. Phase 2 픽업 루프
중 해당 보관소를 자연스럽게 경유하면 그때 반납하고
(`_return_matching_surplus_at_station`), 남은 것은 미션 마지막에 station별로
모아서 한 번에 반납합니다.

---

## 카고 공간 관리 (조립 우선 소진)

- **재료별 공간 체크**: 픽업 전 `cargo_has_space_for(material_id)`로 해당
  재료(2×2=2유닛, 4×2=4유닛)가 실제로 들어갈 슬롯이 있는지 확인합니다.
  (`cargo_is_full()`은 2유닛 기준이라 4×2 블록의 공간 부족을 못 봅니다 —
  2026-07-04 필드에서 픽업이 조용히 유실된 원인.)
- **조립으로 공간 확보**: 공간이 없으면 먼저 `_try_free_cargo_space()`가
  이미 재료 세트가 완성된 제품의 ASSEMBLE을 즉시 실행해 cargo 2-6 재료를
  7/8 슬롯으로 소진시킵니다. 그래도 안 되면 워크벤치 overflow drop으로
  폴백하고, drop 직후 생산에 필요한 재료는 바로 되찾아옵니다.
- **픽업 즉시 조립 시작**: 재료를 하나 실을 때마다
  `_start_ready_intransit_assembly()`를 호출해, 세트가 완성되는 순간
  ASSEMBLE이 시작됩니다(주행·이탈 기동과 겹쳐 실행).
- **fail-loud 픽업**: `arm_pick_material()`은 공간 없는 픽업을 사전
  거부하고, 적재 후 슬롯 배정 실패(상태 불일치)도 오류로 드러냅니다.

---

## 견고성 (fail-loud / self-recovery)

- **Bounded wait + 재시도**: 모든 블로킹 ROS2 호출(navigate / arm / wb_task)은
  `nav_timeout_sec` / `arm_timeout_sec` / `wb_timeout_sec`으로 대기가 제한되고,
  `call_max_retries`회까지 재시도 후 실패 처리됩니다. 죽은 서버가 executor
  스레드를 영원히 멈추게 할 수 없습니다.
- **Nav 취소 유예**: navigator는 한 번에 하나의 goal만 받으므로, 타임아웃된
  goal은 명시적으로 cancel하고 `nav_cancel_grace_sec` 동안 busy 해제를 기다린
  뒤 재시도합니다.
- **WB watchdog**: `wb_task_async()`는 action 서버가 콜백을 영영 안 주더라도
  `wb_timeout_sec` 내에 handle을 실패로 마감하는 watchdog 스레드를 둡니다.
- **`_require` / `_soft`**: 필수 호출 실패는 `ExecutionFailure`로 즉시 중단,
  best-effort 호출(post_process 등)은 경고만 남기고 계속합니다.
- **Best-effort 귀환**: 미션 중간 실패 시에도 home 복귀를 한 번 시도해 AMR이
  경기장에 방치되지 않게 한 뒤 오류를 다시 던집니다.
- **cargo 1 점유 가드**: 이전 제품이 하역되지 않은 채 두 번째 제품을 실으려는
  시퀀싱 버그를 조용한 상태 손상 대신 즉시 실패로 드러냅니다.
- **중복 task 무시**: 동일 task의 QoS 재전송은 내용 해시로 걸러냅니다.
  (planning 실패 시 해시를 지워 재전송으로 재시도 가능)

---

## 인터페이스

| 방향 | 인터페이스 | 타입 | 설명 |
|---|---|---|---|
| Sub | `/eai/task` | `sml_messages/msg/Task` | 주문 수신 트리거 (VOLATILE + TRANSIENT_LOCAL 이중 구독) |
| Sub | `/workbench/product_ready` | `std_msgs/Int32` | 워크벤치 완료 신호 |
| Pub | `cmd_vel` | `geometry_msgs/Twist` | WB 대기용 후진(clearance backup) |
| Act | `navigate_to_station` | `robocup_pkg/action/NavTask` | AMR 이동 |
| Srv | `/robocup_navigator/post_process` | `std_srvs/srv/Trigger` | station 작업 후 후진/회전 이탈 |
| Act | `wb_task` | `robocup_pkg/action/WbTask` | 워크벤치 조립/분해 |
| Srv | `/amr_robot_command` | `robocup_pkg/srv/ArmCommand` | 로봇팔 제어 |

---

## 두 단계 내비게이션 (sub_goal / goal)

모든 station 접근은 `Executor._approach(id)` 한 곳을 통해서만 이뤄지며, 두
단계로 분리됩니다:

1. **`navigate_subgoal(id)`** — `station_id = -abs(id)` 로 전달. AMR이 sub_goal 웨이포인트에 정지.  
   이 구간 동안 cargo 7/8 ASSEMBLE 비동기 실행 가능.
2. **`wait_for_intransit_assembly()`** — 진행 중인 ASSEMBLE 완료 대기.
3. **`navigate_goal(id)`** — `station_id = +abs(id)` 로 전달. 저속 정밀 도킹.  
   이 구간에는 팔 명령 없음.

`navigate_goal()`을 직접 호출하면 post_process 이탈 자세에서의 근접 재도킹
(과거 recycle phase AMR 위치 오류의 원인)이 재발할 수 있으므로 금지합니다.

---

## AMR 슬롯 구조

| 슬롯 | 용도 |
|---|---|
| 1 | 제품 운반 (recycle 픽업 + WB 완성품; 동시 1개, 점유 가드) |
| 2~6 | 재료 (최대 5종) |
| 7~8 | 주행 중 조립(in-transit) 전용 |

cargo 7/8 할당 우선순위는 `product_weights_json`(사용자 가중치) ×
경로상 재료 완성 순서(`compute_completion_indices`) ×
lifecycle boost(`deferred_recycle_priority_boost`)로 결정됩니다.

---

## 파라미터 (`config/params.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `side` | `a` | 경기 side (`a`/`b`) — 반대편 station 필터링 |
| `task_topic` | `/eai/task` | 주문 수신 토픽 |
| `nav_action` | `navigate_to_station` | 내비게이션 액션 이름 |
| `wb_action` | `wb_task` | 워크벤치 액션 이름 |
| `arm_service` | `/amr_robot_command` | 로봇팔 서비스 이름 |
| `wb_ready_topic` | `/workbench/product_ready` | 워크벤치 완료 토픽 |
| `post_process_service` | `/robocup_navigator/post_process` | station 작업 후 이탈 서비스 |
| `cmd_vel_topic` | `cmd_vel` | clearance backup용 속도 토픽 |
| `wb_recycle_wait_mode` | `auto` | WB 분해 대기 정책 (`auto`/`always`/`never`) |
| `wb_produce_parallelism` | `2` | WB PRODUCE 동시 실행 수 (로봇팔 2대 기준) |
| `wb_clearance_backup_distance` | `0.10` | WB 대기 시 후진 거리 [m] (회전 없음) |
| `wb_clearance_backup_speed` | `0.10` | WB 대기 후진 속도 [m/s] |
| `motion_period_sec` | `0.05` | cmd_vel 발행 주기 [s] |
| `product_weights_json` | `""` | `{"product_id": weight}` — cargo 7/8 우선순위 가중치 |
| `nav_timeout_sec` | `60.0` | navigate 1회 대기 한도 [s] |
| `arm_timeout_sec` | `30.0` | arm/post_process 1회 대기 한도 [s] |
| `wb_timeout_sec` | `120.0` | wb_task 1회 대기 한도 + async watchdog [s] |
| `call_max_retries` | `2` | 타임아웃/실패 시 재시도 횟수 |
| `nav_cancel_grace_sec` | `5.0` | 타임아웃 goal cancel 후 busy 해제 대기 [s] |
| `deferred_recycle_priority_boost` | `1000.0` | lifecycle produce-then-recycle 제품의 cargo 슬롯 우선순위 배수 |
| `driving_velocity` | `0.5` | 주행 속도 [m/s] |
| `parking_duration` | `1.5` | 주차 시간 [s] |
| `exiting_duration` | `1.0` | 이탈 시간 [s] |
| `debug_export` | `true` | 실행 전 계획 전체를 JSON으로 내보내기 |
| `debug_export_dir` | `/tmp/robocup_planner` | 계획 JSON 저장 경로 |
| `waypoint_yaml` | *(ament_index 자동 해석)* | 웨이포인트 YAML 경로 |

`waypoint_yaml`은 빌드 시 `config/robocup_waypoint.yaml`을 `share/robocup_planner/config/`에 설치하며, 노드 시작 시 `ament_index`로 자동 해석합니다.

---

## 빌드 및 실행

```bash
# 빌드
colcon build --packages-select sml_messages robocup_pkg sml_system_pkg robocup_planner
source install/setup.bash

# 시뮬레이션 (mock 노드 포함 전체 스택)
ros2 launch robocup_planner robocup_planner_sim.launch.py

# 인자 지정 예시
ros2 launch robocup_planner robocup_planner_sim.launch.py \
  side:=b tier:=beginner stage:=lifecycle
```

수동 주문은 별도 launch 파일 대신 `eai_task_server`의 `manual_order_server`로
발행합니다 (`src/eai_task_server/README.md` 참조):

```bash
ros2 run eai_task_server manual_order_server --ros-args \
  -p order_file:=src/eai_task_server/orders/example_order.yaml
```

### launch 인자

| 인자 | 기본값 | 선택값 |
|---|---|---|
| `side` | `a` | `a`, `b` |
| `tier` | `beginner` | `entry`, `beginner`, `advanced`, `expert` |
| `stage` | `production` | `production`, `recycling`, `lifecycle` |

---

## 파일 구조

```
robocup_planner/
├── config/
│   ├── params.yaml              # 노드 파라미터 (전체 스택)
│   └── robocup_waypoint.yaml    # 웨이포인트 좌표 (distance calculator용)
├── launch/
│   └── robocup_planner_sim.launch.py   # 시뮬레이션 통합 런치
└── robocup_planner/
    ├── planner_node.py          # 메인 노드 (PlannerNode) — 계획 + 블로킹 헬퍼
    ├── product_catalog.py       # 제품·재료 카탈로그 (workbench_only 구분)
    ├── execution/
    │   ├── executor.py          # 반응형 실행 루프 (Executor, Plan)
    │   ├── storage_ledger.py    # 런타임 재고 원장 (StorageLedger)
    │   └── cargo_state.py       # cargo 1~6 상태 추적 (CargoManager)
    └── planning/
        ├── aidlist_builder.py   # aidlist / net_aidlist 계산
        ├── cargo_allocator.py   # in-transit 슬롯(7/8) 할당
        ├── distance_calculator.py  # 웨이포인트 기반 거리 계산
        └── midlist_builder.py   # pickup 시퀀스 생성 (batch·완성 순서 지원)
```

---

## 디버깅

```bash
# 주문 확인
ros2 topic echo /eai/task --once

# 계획 JSON 확인 (debug_export=true일 때)
ls /tmp/robocup_planner/

# 내비게이션 액션 단독 테스트 (sub_goal)
ros2 action send_goal /navigate_to_station robocup_pkg/action/NavTask "{station_id: -2}" --feedback

# 내비게이션 액션 단독 테스트 (goal)
ros2 action send_goal /navigate_to_station robocup_pkg/action/NavTask "{station_id: 2}" --feedback

# 로봇팔 서비스 단독 테스트
ros2 service call /amr_robot_command robocup_pkg/srv/ArmCommand \
  "{action: 'LOAD', object_ids: [1], location: 1, station_id: 1, slide_ids: []}"
```
