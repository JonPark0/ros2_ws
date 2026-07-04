import rclpy
from rclpy.node import Node
from robocup_pkg.srv import ArmCommand


class ManualCommandNode(Node):
    def __init__(self):
        super().__init__('manual_command_node')
        self.client = self.create_client(ArmCommand, '/amr_robot_command')
        self.get_logger().info('[MANUAL] manual_command_node started')
        self.get_logger().info('[MANUAL] commands: load / unload / exit')

    def call_arm(self, action, object_ids, location=0, station_id=0):
        req = ArmCommand.Request()
        req.action = action
        req.object_ids = object_ids
        req.location = location
        req.station_id = station_id

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('[MANUAL] waiting for /amr_robot_command service...')

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res:
            self.get_logger().info(
                f'[MANUAL] result: success={res.success}, '
                f'slots={list(res.slots)}, object_ids={list(res.object_ids)}, '
                f'message={res.message}'
            )
        else:
            self.get_logger().error('[MANUAL] no response')

    def parse_object_ids(self, raw):
        object_ids = [int(x.strip()) for x in raw.split(',') if x.strip()]
        if not object_ids:
            raise ValueError('empty object_id list')
        return object_ids

    def run(self):
        print('\n' + '=' * 40)
        print('ARM Manual Command Node')
        print('=' * 40)
        print('object_id (재료): 1=2x2_red  2=2x2_green  3=2x2_blue  4=2x2_yellow')
        print('                  5=4x2_red  6=4x2_green  7=4x2_blue  8=4x2_yellow')
        print('object_id (완성품): 34=battery  13=magnet  81=e_stop  442=carrot')
        print('                    241=traffic_light  462=small_tree  711=hammer')
        print('                    4482=big_carrot')
        print('여러 개 입력 시 쉼표로 구분: 1,3,5')
        print('station_id: 6=워크벤치(세우기 포함)  8=고객센터(그냥 전달)')
        print('=' * 40)

        while rclpy.ok():
            try:
                cmd = input('\ncommand (load/unload/assemble/exit): ').strip().lower()

                if cmd == 'exit':
                    break

                elif cmd == 'load':
                    raw = input('object_id (쉼표 구분, 예: 1,3,5): ').strip()
                    object_ids = self.parse_object_ids(raw)
                    sid = int(input('station_id: ').strip())
                    self.call_arm('LOAD', object_ids, station_id=sid)

                elif cmd == 'unload':
                    raw = input('object_id (쉼표 구분, 예: 34,13): ').strip()
                    object_ids = self.parse_object_ids(raw)
                    sid = int(input('station_id (6=워크벤치 / 8=고객센터): ').strip())
                    self.call_arm('UNLOAD', object_ids, station_id=sid)

                elif cmd == 'assemble':
                    raw = input('product_id (예: 34): ').strip()
                    object_ids = self.parse_object_ids(raw)
                    sid = int(input('station_id (target_slot, 예: 7 또는 8): ').strip())
                    self.call_arm('ASSEMBLE', object_ids, station_id=sid)

                elif cmd == '':
                    continue

                else:
                    print(f'unknown command: {cmd}')

            except ValueError:
                print('[ERROR] invalid input')
            except KeyboardInterrupt:
                break


def main(args=None):
    rclpy.init(args=args)
    node = ManualCommandNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
