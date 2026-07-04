import rclpy
from rclpy.node import Node
from arm_interfaces.srv import Cargo


MATERIAL_NAMES = {
    1: "2x2_red",
    2: "2x2_green",
    3: "2x2_blue",
    4: "2x2_yellow",
    5: "4x2_red",
    6: "4x2_green",
    7: "4x2_blue",
    8: "4x2_yellow",
}

# 블록별 높이 (칸 단위)
BLOCK_HEIGHT = {
    1: 2, 2: 2, 3: 2, 4: 2,  # 2×2 블록: 2칸
    5: 4, 6: 4, 7: 4, 8: 4,  # 4×2 블록: 4칸
}

# 블록별 픽업 오프셋 (바닥에서 몇 칸 위를 집는가)
PICK_OFFSET = {
    1: 0, 2: 0, 3: 0, 4: 0,  # 2×2: 바닥에서 0칸
    5: 1, 6: 1, 7: 1, 8: 1,  # 4×2: 바닥에서 1칸
}

MAX_STACK_HEIGHT = 6  # 슬롯당 최대 적재 높이 (칸)

PRODUCT_SLOT = 1
MATERIAL_SLOTS = [2, 3, 4, 5, 6]
ASSEMBLY_SLOTS = [7, 8]  # 조립 슬롯 (FIND_EMPTY 검색 대상 아님)

# 완성품 전용 고정 배달 자리 개수 (인덱스 0~5). 워크벤치가 아닌 스테이션에
# 비전 없이 완성품을 내려놓을 때, station_id별로 이 중 몇 번 자리까지
# 찼는지를 여기서 기억한다 (amr_robot_node의 PRODUCT_DELIVERY_JOINTS와 짝).
PRODUCT_DELIVERY_SLOTS = 6


class CargoManagerNode(Node):
    def __init__(self):
        super().__init__('cargo_manager_node')
        self.srv = self.create_service(Cargo, '/cargo', self.cargo_cb)

        # {slot: [bottom, ..., top] 순서의 object_id 리스트}
        self.slot_state = {
            slot: []
            for slot in [PRODUCT_SLOT] + MATERIAL_SLOTS + ASSEMBLY_SLOTS
        }

        # station_id별 완성품 고정 배달 자리(0~5) 점유 상태. 처음 보는 station_id는
        # 6자리 모두 빈 상태(0)로 lazy 초기화된다. slot_state와 달리 이 상태는
        # CLEAR_PRODUCT_DELIVERY로 명시적으로 비우기 전까지 리셋되지 않는다 — 로봇이
        # 다른 작업을 하다가 나중에 같은 station으로 다시 와도 기억을 유지해야 하기 때문.
        self.product_delivery_state = {}

        self.get_logger().info('[CARGO] cargo_manager_node started')
        self.get_logger().info(f'[CARGO] slots: {list(self.slot_state.keys())}')

    # ── 내부 헬퍼 ──────────────────────────────────────────────────

    def _stack_height(self, slot):
        return sum(BLOCK_HEIGHT.get(obj, 2) for obj in self.slot_state[slot])

    def _layer_index(self, slot, object_id):
        """슬롯 스택에서 object_id의 픽업 layer_index를 계산한다."""
        h = 0
        for obj in self.slot_state[slot]:
            if obj == object_id:
                return h + PICK_OFFSET.get(obj, 0)
            h += BLOCK_HEIGHT.get(obj, 2)
        return None

    def _product_delivery_slots(self, station_id):
        return self.product_delivery_state.setdefault(
            station_id, [0] * PRODUCT_DELIVERY_SLOTS)

    # ── 서비스 콜백 ────────────────────────────────────────────────

    def cargo_cb(self, request, response):
        action = request.action.upper()
        response.layer_index = 0

        if action == 'FIND_EMPTY':
            if request.object_id > 8:
                search_slots = [PRODUCT_SLOT]
            else:
                search_slots = MATERIAL_SLOTS

            needed = BLOCK_HEIGHT.get(request.object_id, 2)
            for slot in search_slots:
                if self._stack_height(slot) + needed <= MAX_STACK_HEIGHT:
                    future_layer = self._stack_height(slot) + PICK_OFFSET.get(request.object_id, 0)
                    response.success = True
                    response.slot = slot
                    response.layer_index = future_layer
                    response.message = f'empty slot found: slot={slot}, future_layer={future_layer}'
                    self.get_logger().info(f'[CARGO] {response.message}')
                    return response

            response.success = False
            response.slot = -1
            response.message = 'no empty slot'
            self.get_logger().warn(f'[CARGO] {response.message}')

        elif action == 'FIND_EMPTY_ASSEMBLY_SLOT':
            # 조립(ASSEMBLE) 요청이 들어왔을 때 사용할 슬롯(7/8)을 매니저가 직접 배정한다.
            # 호출자(amr_robot_node)가 station_id 같은 걸 슬롯 번호로 오용하지 않도록,
            # "비어있는 조립 슬롯을 앞에서부터(7 -> 8) 고른다"는 정책을 여기서만 관리한다.
            for slot in ASSEMBLY_SLOTS:
                if not self.slot_state[slot]:
                    response.success = True
                    response.slot = slot
                    response.message = f'empty assembly slot found: slot={slot}'
                    self.get_logger().info(f'[CARGO] {response.message}')
                    return response

            response.success = False
            response.slot = -1
            response.message = 'no empty assembly slot'
            self.get_logger().warn(f'[CARGO] {response.message}')

        elif action == 'FIND_OBJECT':
            for slot, stack in self.slot_state.items():
                if request.object_id in stack:
                    layer = self._layer_index(slot, request.object_id)
                    response.success = True
                    response.slot = slot
                    response.layer_index = layer
                    response.message = (
                        f'object found: object_id={request.object_id}, '
                        f'slot={slot}, layer_index={layer}'
                    )
                    self.get_logger().info(f'[CARGO] {response.message}')
                    return response

            response.success = False
            response.slot = -1
            response.message = f'object_id={request.object_id} not found'
            self.get_logger().warn(f'[CARGO] {response.message}')

        elif action == 'FIND_OBJECT_EXCLUDING':
            # FIND_OBJECT와 동일하지만, request.slot으로 넘어온 슬롯은 검색에서 제외한다.
            # 빅트리 조립처럼 같은 object_id가 두 슬롯에 나뉘어 있을 때, 이미 확보한
            # 슬롯 말고 "다른" 슬롯에 있는 걸 찾아야 하는 경우에 쓴다.
            exclude_slot = request.slot
            for slot, stack in self.slot_state.items():
                if slot == exclude_slot:
                    continue
                if request.object_id in stack:
                    layer = self._layer_index(slot, request.object_id)
                    response.success = True
                    response.slot = slot
                    response.layer_index = layer
                    response.message = (
                        f'object found (excluding slot={exclude_slot}): '
                        f'object_id={request.object_id}, slot={slot}, layer_index={layer}'
                    )
                    self.get_logger().info(f'[CARGO] {response.message}')
                    return response

            response.success = False
            response.slot = -1
            response.message = f'object_id={request.object_id} not found (excluding slot={exclude_slot})'
            self.get_logger().warn(f'[CARGO] {response.message}')

        elif action == 'FIND_SLOT_STACK':
            slot = request.slot
            if slot not in self.slot_state:
                response.success = False
                response.message = f'invalid slot={slot}'
            else:
                response.success = True
                response.slot = slot
                response.stack = list(self.slot_state[slot])
                response.message = f'slot={slot} stack={self.slot_state[slot]}'
                self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'SET':
            slot = request.slot
            if slot not in self.slot_state:
                response.success = False
                response.message = f'invalid slot={slot}'
            else:
                obj = request.object_id
                self.slot_state[slot].append(obj)
                layer = self._layer_index(slot, obj)
                name = MATERIAL_NAMES.get(obj, f'product_id={obj}')
                response.success = True
                response.slot = slot
                response.layer_index = layer
                response.message = (
                    f'slot={slot} set: object_id={obj} ({name}), '
                    f'layer_index={layer}, stack={self.slot_state[slot]}'
                )
                self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'SET_AT':
            # 수동 보정용: 슬롯 스택의 특정 위치(층, station_id로 전달)를 직접 덮어쓴다.
            # layer가 현재 스택 길이보다 크면 0(placeholder)으로 채운 뒤 지정한다.
            slot = request.slot
            layer = request.station_id
            obj = request.object_id
            if slot not in self.slot_state:
                response.success = False
                response.message = f'invalid slot={slot}'
            elif layer < 0:
                response.success = False
                response.message = f'invalid layer={layer}'
            else:
                stack = self.slot_state[slot]
                while len(stack) <= layer:
                    stack.append(0)
                stack[layer] = obj
                response.success = True
                response.slot = slot
                response.layer_index = layer
                response.stack = list(stack)
                name = MATERIAL_NAMES.get(obj, f'product_id={obj}')
                response.message = (
                    f'slot={slot} layer={layer} set: object_id={obj} ({name}), '
                    f'stack={stack}'
                )
                self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'CLEAR':
            slot = request.slot
            obj = request.object_id
            if slot not in self.slot_state:
                response.success = False
                response.message = f'invalid slot={slot}'
            elif obj not in self.slot_state[slot]:
                response.success = False
                response.message = f'object_id={obj} not in slot={slot}'
            else:
                self.slot_state[slot].remove(obj)
                response.success = True
                response.slot = slot
                response.message = (
                    f'slot={slot} cleared: object_id={obj}, '
                    f'remaining={self.slot_state[slot]}'
                )
                self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'FIND_EMPTY_PRODUCT_DELIVERY':
            # 워크벤치가 아닌 스테이션에 완성품을 비전 없이 내려놓을 때 쓸 다음
            # 빈 자리(0~5)를 station_id별로 찾아준다. 다 찼으면 실패를 반환해서
            # 호출자(amr_robot_node)가 기존 비전(666) 방식으로 폴백하게 한다.
            station_id = request.station_id
            slots = self._product_delivery_slots(station_id)
            for idx, obj in enumerate(slots):
                if obj == 0:
                    response.success = True
                    response.slot = idx
                    response.message = (
                        f'empty product delivery slot found: station={station_id}, idx={idx}'
                    )
                    self.get_logger().info(f'[CARGO] {response.message}')
                    return response

            response.success = False
            response.slot = -1
            response.message = f'no empty product delivery slot at station={station_id}'
            self.get_logger().warn(f'[CARGO] {response.message}')

        elif action == 'SET_PRODUCT_DELIVERY':
            # 완성품을 station_id의 idx(request.slot, 0~5) 자리에 내려놓았음을 기록한다.
            station_id = request.station_id
            idx = request.slot
            obj = request.object_id
            slots = self._product_delivery_slots(station_id)
            if not (0 <= idx < PRODUCT_DELIVERY_SLOTS):
                response.success = False
                response.message = f'invalid product delivery idx={idx}'
            else:
                slots[idx] = obj
                name = MATERIAL_NAMES.get(obj, f'product_id={obj}')
                response.success = True
                response.slot = idx
                response.message = (
                    f'station={station_id} idx={idx} set: object_id={obj} ({name}), '
                    f'slots={slots}'
                )
                self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'CLEAR_PRODUCT_DELIVERY':
            # 수동 보정용: 사람이 완성품을 수거해간 뒤 station_id의 idx(request.slot,
            # 0~5) 자리를 다시 비운다. 자동으로는 호출되지 않는다.
            station_id = request.station_id
            idx = request.slot
            slots = self._product_delivery_slots(station_id)
            if not (0 <= idx < PRODUCT_DELIVERY_SLOTS):
                response.success = False
                response.message = f'invalid product delivery idx={idx}'
            else:
                slots[idx] = 0
                response.success = True
                response.slot = idx
                response.message = f'station={station_id} idx={idx} cleared, slots={slots}'
                self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'STATUS':
            lines = []
            for slot, stack in self.slot_state.items():
                if not stack:
                    lines.append(f'slot={slot}: empty')
                else:
                    items = []
                    for obj in stack:
                        name = MATERIAL_NAMES.get(obj, f'product_id={obj}')
                        layer = self._layer_index(slot, obj)
                        items.append(f'{name}(layer={layer})')
                    lines.append(f'slot={slot}: [{", ".join(items)}]')
            for station_id, slots in self.product_delivery_state.items():
                items = [
                    (MATERIAL_NAMES.get(obj, f'product_id={obj}') if obj else 'empty')
                    for obj in slots
                ]
                lines.append(f'product_delivery[station={station_id}]: {items}')
            response.success = True
            response.message = ' | '.join(lines)
            self.get_logger().info(f'[CARGO] STATUS: {response.message}')

        else:
            response.success = False
            response.message = f'unknown action: {action}'
            self.get_logger().error(f'[CARGO] {response.message}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CargoManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
