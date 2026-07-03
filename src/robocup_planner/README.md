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
  ┌── 계획 단계 ──────────────────────────────────────────────┐
  │  order_list     → aidlist / net_aidlist (재활용 차감)     │
  │  arena_layout   → midlist (거리순 pickup 시퀀스)          │
  │                 → material_home_station (재료별 원래 보관소) │
  │  produce_ids    → workbench_products / intransit_products  │
  │  recycled_materials vs aidlist → surplus_recycled (잉여 재료) │
  └────────────────────────────────────────────────────────────┘
      │
      ▼
[Executor] (백그라운드 스레드)
  Phase 1  : 재활용 픽업 (customer → workbench 분해, 비동기)
  Phase 1.5: 잉여 재활용 재료 반납 (→ 원래 보관소)
  Phase 2  : 재료 픽업 루프
    ├── navigate_subgoal(id)          ← 고속 주행
    ├── wait_for_intransit_assembly() ← cargo 7/8 조립 완료 대기
    └── navigate_goal(id)             ← 저속 정밀 도킹
  Phase 3  : 납품
```

### Phase 1 상세 — 재활용 픽업 및 분해 (비동기 처리)

재활용 제품이 여러 개인 경우, 워크벤치의 분해(RECYCLE) 작업은 `wb_task_async()`로
**논블로킹 실행**됩니다. AMR은 분해가 끝나기를 기다리며 워크벤치 앞에 정차하지 않고,
곧바로 다음 재활용 제품을 가지러 이동합니다. 워크벤치 분해와 AMR 주행이 동시에
진행되는 구조입니다.

- 워크벤치에 새 제품을 내려놓기 **전에**, 이전에 분해가 끝난 제품의 재료 블록을
  먼저 회수합니다 (`_collect_recycled_materials()`). 워크벤치 선반에 재료가
  쌓인 채로 다음 제품을 올려두지 않기 위함입니다.
- 재활용 제품이 여러 개 몰려 cargo 2-6이 가득 차면(`cargo_is_full()`), 다음
  제품을 가지러 가기 전에 이미 워크벤치에 있는 김에 자재를 내려놓습니다
  (`arm_unload_all_materials()`).
- 마지막 제품의 분해가 끝날 때까지는 Phase 1 종료 시점에 반드시 회수합니다.

### Phase 1.5 — 잉여 재활용 재료 반납

분해로 얻은 재료 중 현재 주문에 필요한 개수를 초과하는 만큼(`surplus_recycled`)은
카고에 남겨두지 않고, 계획 단계에서 계산해 둔 `material_home_station`(재료별
원래 보관소)으로 이동해 반납합니다. 같은 보관소로 갈 재료는 한 번에 모아서
반납하여 불필요한 재방문을 피합니다.

---

## 인터페이스

| 방향 | 인터페이스 | 타입 | 설명 |
|---|---|---|---|
| Sub | `/eai/task` | `sml_messages/msg/Task` | 주문 수신 트리거 |
| Sub | `/workbench/product_ready` | `std_msgs/Int32` | 워크벤치 완료 신호 |
| Act | `navigate_to_station` | `robocup_pkg/action/NavTask` | AMR 이동 |
| Srv | `/robocup_navigator/post_process` | `std_srvs/srv/Trigger` | station 작업 후 후진/회전 이탈 |
| Act | `wb_task` | `robocup_pkg/action/WbTask` | 워크벤치 조립/분해 |
| Srv | `/amr_robot_command` | `robocup_pkg/srv/ArmCommand` | 로봇팔 제어 |

---

## 두 단계 내비게이션 (sub_goal / goal)

모든 station 접근이 두 단계로 분리됩니다:

1. **`navigate_subgoal(id)`** — `station_id = -abs(id)` 로 전달. AMR이 sub_goal 웨이포인트에 정지.  
   이 구간 동안 cargo 7/8 ASSEMBLE 비동기 실행 가능.
2. **`wait_for_intransit_assembly()`** — 진행 중인 ASSEMBLE 완료 대기.
3. **`navigate_goal(id)`** — `station_id = +abs(id)` 로 전달. 저속 정밀 도킹.  
   이 구간에는 팔 명령 없음.

납품 시 customer counter에서도 동일 패턴을 적용합니다:  
sub_goal 도착 후 in-transit assembly가 아직 실행 중이면 그 자리에서 완료를 기다린 뒤 docking goal로 진입합니다.

---

## AMR 슬롯 구조

| 슬롯 | 용도 |
|---|---|
| 1 | 워크벤치 완성품 |
| 2~6 | 재료 (최대 5종) |
| 7~8 | 주행 중 조립(in-transit) 전용 |

cargo 7/8에는 `is_intransit_eligible` 조건을 만족하는 제품만 할당됩니다 (레이어 중첩 없는 단순 조립 구조).

---

## 파라미터 (`config/params.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `task_topic` | `/eai/task` | 주문 수신 토픽 |
| `nav_action` | `navigate_to_station` | 내비게이션 액션 이름 |
| `post_process_service` | `/robocup_navigator/post_process` | station 작업 후 이탈 서비스 |
| `wb_action` | `wb_task` | 워크벤치 액션 이름 |
| `arm_service` | `/amr_robot_command` | 로봇팔 서비스 이름 |
| `wb_ready_topic` | `/workbench/product_ready` | 워크벤치 완료 토픽 |
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
    ├── planner_node.py          # 메인 노드 (PlannerNode)
    ├── product_catalog.py       # 제품·재료 카탈로그
    ├── execution/
    │   ├── executor.py          # 반응형 실행 루프 (Executor)
    │   └── cargo_state.py       # 실시간 cargo 2~6 상태 추적
    └── planning/
        ├── aidlist_builder.py   # aidlist / net_aidlist 계산
        ├── cargo_allocator.py   # in-transit 슬롯(7/8) 할당
        ├── distance_calculator.py  # 웨이포인트 기반 거리 계산
        └── midlist_builder.py   # pickup 시퀀스 생성 (batch 지원)
```

---

## 디버깅

```bash
# 주문 확인
ros2 topic echo /eai/task --once

# 내비게이션 액션 단독 테스트 (sub_goal)
ros2 action send_goal /navigate_to_station robocup_pkg/action/NavTask "{station_id: -2}" --feedback

# 내비게이션 액션 단독 테스트 (goal)
ros2 action send_goal /navigate_to_station robocup_pkg/action/NavTask "{station_id: 2}" --feedback

# 로봇팔 서비스 단독 테스트
ros2 service call /amr_robot_command robocup_pkg/srv/ArmCommand \
  "{action: 'LOAD', object_ids: [1], location: 1, station_id: 1, slide_ids: []}"
```
