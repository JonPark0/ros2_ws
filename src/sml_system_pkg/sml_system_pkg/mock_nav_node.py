"""
mock_nav_node.py
navigate_to_station Action 서버 mock.
+ /robocup_navigator/post_process (std_srvs/Trigger) 서비스 mock.

시간 모델:
  - use_distance_time=False: 기존처럼 delay_sec 고정 지연
  - use_distance_time=True : 현재 station → 목표 station 거리 / amr_speed_mps + nav_overhead_sec
"""

import json
import math
import time

import rclpy
import yaml
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from robocup_pkg.action import NavTask

from sml_system_pkg.arena_side_utils import (
    amr_station_to_planner_station,
    normalize_side,
    side_to_start_goal_station,
)

DEFAULT_STATION_COORD_JSON_PATH = (
    '/home/st02/ros2_ws/src/sml_system_pkg/config/station_coordinates_a_zone.json'
)

try:
    from ament_index_python.packages import get_package_share_directory
    import os as _os
    DEFAULT_WAYPOINT_YAML_PATH = _os.path.join(
        get_package_share_directory('robocup_planner'), 'config', 'robocup_waypoint.yaml'
    )
except Exception:
    DEFAULT_WAYPOINT_YAML_PATH = ''

ODOM_PUBLISH_HZ = 10.0


class MockNavNode(Node):

    def __init__(self):
        super().__init__('mock_nav_node')
        self.cbg = MutuallyExclusiveCallbackGroup()

        self.declare_parameter('delay_sec', 1.0)
        self.declare_parameter('use_distance_time', False)
        self.declare_parameter('amr_speed_mps', 0.50)
        self.declare_parameter('nav_overhead_sec', 0.0)
        self.declare_parameter('post_process_delay_sec', 0.0)
        self.declare_parameter('side', 'a')
        self.declare_parameter('start_station_id', -1)
        self.declare_parameter('station_coord_json_path', DEFAULT_STATION_COORD_JSON_PATH)
        self.declare_parameter('waypoint_yaml', DEFAULT_WAYPOINT_YAML_PATH)
        self.declare_parameter('publish_odom', True)

        self.side = normalize_side(self.get_parameter('side').value)
        configured_start = int(self.get_parameter('start_station_id').value)
        self.current_station_id = (
            side_to_start_goal_station(self.side)
            if configured_start < 0
            else configured_start
        )
        # Track whether the AMR is stopped at a sub_goal (mid-approach).
        # When the next goal leg targets the same station, only 20% remains.
        self._at_subgoal_of: int = -1
        # Prefer the real robocup_waypoint.yaml (same file the real planner and
        # navigator use) over the legacy station_coordinates_a_zone.json;
        # fall back to the JSON entries for any station id the YAML lacks
        # (the navigator team's waypoint file is still WIP as of this writing).
        self.station_coords = self._load_station_coords()
        self.station_coords.update(self._load_waypoint_yaml())

        # Current simulated (x, y) position in the same coordinate space as
        # station_coords, used only to drive the odom publisher below —
        # entirely separate from current_station_id (which drives NavTask
        # logic/timing and is left untouched).
        self._current_xy = self._station_coord(self.current_station_id)

        publish_odom = bool(self.get_parameter('publish_odom').value)
        self._odom_pub = (
            self.create_publisher(Odometry, '/odom', 10) if publish_odom else None
        )

        self._action_server = ActionServer(
            self,
            NavTask,
            'navigate_to_station',
            execute_callback=self._execute_cb,
            callback_group=self.cbg,
        )
        self.get_logger().info(
            '[MOCK NAV] navigate_to_station 서버 시작 | '
            f'side={self.side}, start={self.current_station_id}, '
            f'use_distance_time={bool(self.get_parameter("use_distance_time").value)}, '
            f'fixed_delay={float(self.get_parameter("delay_sec").value):.2f}s, '
            f'speed={float(self.get_parameter("amr_speed_mps").value):.2f}m/s'
        )

        self._post_process_srv = self.create_service(
            Trigger,
            '/robocup_navigator/post_process',
            self._post_process_cb,
            callback_group=self.cbg,
        )
        self.get_logger().info('[MOCK NAV] post_process 서비스 시작')

    def _load_station_coords(self):
        path = str(self.get_parameter('station_coord_json_path').value).strip()
        if not path:
            return {}
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            raw = data.get('station_coordinates', data)
            coords = {}
            for key, value in raw.items():
                station_id = int(key)
                coords[station_id] = (float(value['x']), float(value['y']))
            self.get_logger().info(
                f'[MOCK NAV] station 좌표 로드 완료: {len(coords)}개, path={path}'
            )
            return coords
        except Exception as exc:
            self.get_logger().warn(
                f'[MOCK NAV] station 좌표 로드 실패: {exc}. fixed delay를 사용합니다.'
            )
            return {}

    def _load_waypoint_yaml(self):
        """Load station_N_goal positions from the real robocup_waypoint.yaml.

        Same shape as robocup_planner/planning/distance_calculator.py, so the
        odometry this node simulates stays consistent with whatever the real
        planner/navigator would use for the same file.
        """
        path = str(self.get_parameter('waypoint_yaml').value).strip()
        if not path:
            return {}
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            waypoints = data.get('waypoints', {})
            coords = {}
            for i in range(0, 21):
                wp = waypoints.get(f'station_{i}_goal')
                if wp:
                    coords[i] = (
                        float(wp['position']['x']),
                        float(wp['position']['y']),
                    )
            self.get_logger().info(
                f'[MOCK NAV] waypoint_yaml 좌표 로드 완료: {len(coords)}개, path={path}'
            )
            return coords
        except Exception as exc:
            self.get_logger().warn(
                f'[MOCK NAV] waypoint_yaml 로드 실패: {exc}. '
                'station_coordinates_a_zone.json만 사용합니다.'
            )
            return {}

    def _station_coord(self, station_id: int):
        station_id = int(station_id)
        if station_id in self.station_coords:
            return self.station_coords[station_id]
        local_station_id = amr_station_to_planner_station(station_id, self.side)
        if local_station_id in self.station_coords:
            return self.station_coords[local_station_id]
        return (float(station_id), 0.0)

    def _travel_delay(self, target_station_id: int) -> float:
        use_distance_time = bool(self.get_parameter('use_distance_time').value)
        if not use_distance_time or not self.station_coords:
            return max(0.0, float(self.get_parameter('delay_sec').value))

        start = int(self.current_station_id)
        target = int(target_station_id)
        if start == target:
            dist = 0.0
        else:
            x1, y1 = self._station_coord(start)
            x2, y2 = self._station_coord(target)
            dist = math.hypot(x2 - x1, y2 - y1)

        speed = max(float(self.get_parameter('amr_speed_mps').value), 1e-6)
        overhead = max(0.0, float(self.get_parameter('nav_overhead_sec').value))
        return overhead + dist / speed

    def _publish_odom(self, x: float, y: float, yaw: float = 0.0) -> None:
        if self._odom_pub is None:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose = PoseWithCovariance()
        msg.pose.pose = Pose(
            position=Point(x=float(x), y=float(y), z=0.0),
            orientation=Quaternion(
                x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
            ),
        )
        msg.twist = TwistWithCovariance()
        msg.twist.twist = Twist()
        self._odom_pub.publish(msg)

    def _sleep_and_publish_odom(self, target_xy, duration: float) -> None:
        """Sleep for `duration` seconds, publishing interpolated /odom updates
        along the way (10 Hz) instead of a single blind time.sleep(). This is
        the only source of live AMR position telemetry during simulation —
        the real robot publishes the same Odometry message via
        robocup_navigator/current_pose.py, so GUI consumers use one code path
        for both sim and real-world monitoring."""
        sx, sy = self._current_xy
        tx, ty = target_xy
        yaw = math.atan2(ty - sy, tx - sx) if (tx, ty) != (sx, sy) else 0.0

        if duration <= 0.0 or self._odom_pub is None:
            time.sleep(max(0.0, duration))
            self._current_xy = (tx, ty)
            self._publish_odom(tx, ty, yaw)
            return

        steps = max(1, int(duration * ODOM_PUBLISH_HZ))
        step_dt = duration / steps
        for i in range(1, steps + 1):
            time.sleep(step_dt)
            t = i / steps
            x = sx + (tx - sx) * t
            y = sy + (ty - sy) * t
            self._current_xy = (x, y)
            self._publish_odom(x, y, yaw)

    def _execute_cb(self, goal_handle):
        raw_id = int(goal_handle.request.station_id)

        # Convention: negative station_id = navigate to sub_goal of abs(station_id).
        # The sub_goal is the approach waypoint; the AMR does not dock yet.
        # Positive station_id = navigate to the docking goal (final parking position).
        is_subgoal = raw_id < 0
        station_id = abs(raw_id)

        # If the AMR is already at the sub_goal of this station (because
        # navigate_subgoal() was just called), only ~20% of the distance remains
        # for the short precision-parking leg.
        already_at_subgoal = (not is_subgoal) and (self._at_subgoal_of == station_id)

        delay_sec = self._travel_delay(station_id)
        if is_subgoal:
            # sub_goal is ~80% of the full station-to-station distance.
            delay_sec *= 0.8
        elif already_at_subgoal:
            # Only the short sub_goal → goal leg remains (~20% of total).
            delay_sec *= 0.2

        phase = 'sub_goal' if is_subgoal else 'goal'
        self.get_logger().info(
            f'[MOCK NAV] goal 수신: from={self.current_station_id}, '
            f'to={station_id} ({phase}), delay={delay_sec:.2f}s'
        )

        fb = NavTask.Feedback()
        fb.status = 'MOVING'
        goal_handle.publish_feedback(fb)

        target_xy = self._station_coord(station_id)
        self._sleep_and_publish_odom(target_xy, delay_sec)

        fb.status = 'AT_SUBGOAL' if is_subgoal else 'ARRIVED'
        goal_handle.publish_feedback(fb)

        if is_subgoal:
            # Record that the AMR is now waiting at the sub_goal of station_id.
            self._at_subgoal_of = station_id
        else:
            # Fully docked — update effective position and clear sub_goal state.
            self.current_station_id = station_id
            self._at_subgoal_of = -1

        goal_handle.succeed()

        result = NavTask.Result()
        result.success = True
        result.fail_reason = ''
        self.get_logger().info(
            f'[MOCK NAV] 완료: station_id={station_id} ({phase})'
        )
        return result

    def _post_process_cb(self, request, response):
        delay_sec = max(0.0, float(self.get_parameter('post_process_delay_sec').value))
        if delay_sec > 0.0:
            time.sleep(delay_sec)
        self.get_logger().info(f'[MOCK NAV] post_process 호출됨 → success, delay={delay_sec:.2f}s')
        response.success = True
        response.message = ''
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockNavNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
