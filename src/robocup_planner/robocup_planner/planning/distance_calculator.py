"""
Distance calculator using the AMR team's waypoint YAML.

Uses the station_N_goal position for each station as the reference point.
The YAML will be updated by the AMR team on competition day, so all
distances are recalculated at runtime by loading the file fresh.

Two distance models are provided:

- station_to_station()/point_to_station(): straight-line (Euclidean)
  distance. Kept for callers that only need a rough proximity check
  (e.g. picking the nearest storage station for a material's "home").

- bezier_station_to_station()/bezier_point_to_station(): a quadratic
  Bezier-curve length estimate. The AMR never actually drives a straight
  line — Nav2's A*-based global planner bends the path, and every station
  has a docking approach line (the station_N_sub_goal -> station_N_goal
  segment) that the robot must align to and enter along straight, roughly
  perpendicular to the station, to be able to dock. Modelling the route as
  a straight line ignores that final approach bend; modelling it as a
  curve that is tangent to the station's approach line at the destination
  gives a closer approximation of the real driven distance, without
  needing a live Nav2 path query. This is what route-ordering and
  travel-time estimates should use.
"""

import math
import re
import yaml
from typing import Dict, Optional, Tuple

# Default length (m) of the destination approach line used to build the
# Bezier control point when a station has no explicit sub_goal waypoint
# (e.g. the home station). Matches the AMR team's typical docking
# approach distance.
DEFAULT_APPROACH_LINE_LENGTH = 0.30
# Number of line segments used to numerically approximate the quadratic
# Bezier curve's arc length. Higher = more accurate, cheaper than it looks
# since this only runs at planning time.
DEFAULT_BEZIER_SAMPLES = 16

_GOAL_RE = re.compile(r'^station_(\d+)_goal$')
_SUB_GOAL_RE = re.compile(r'^station_(\d+)_sub_goal$')


class DistanceCalculator:
    def __init__(
        self,
        waypoint_yaml_path: str,
        approach_line_length: float = DEFAULT_APPROACH_LINE_LENGTH,
        bezier_samples: int = DEFAULT_BEZIER_SAMPLES,
    ):
        with open(waypoint_yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        self._waypoints: dict = data['waypoints']
        self._approach_line_length = float(approach_line_length)
        self._bezier_samples = max(2, int(bezier_samples))

        self._positions: Dict[int, Tuple[float, float]] = {}
        self._yaws: Dict[int, float] = {}
        self._sub_goal_positions: Dict[int, Tuple[float, float]] = {}
        self._parse_waypoints()

    def _parse_waypoints(self) -> None:
        """Index every station_<id>_goal / station_<id>_sub_goal entry.

        Station ids are discovered by pattern match rather than a fixed
        numeric range, since arena ids aren't contiguous (e.g. shared
        storage uses ids 71/72 alongside the regular 0-14 range).
        """
        for name, wp in self._waypoints.items():
            if not isinstance(wp, dict) or 'position' not in wp:
                continue

            goal_match = _GOAL_RE.match(name)
            if goal_match:
                station_id = int(goal_match.group(1))
                pos = wp['position']
                self._positions[station_id] = (pos['x'], pos['y'])
                orientation = wp.get('orientation')
                if orientation:
                    self._yaws[station_id] = self._yaw_from_quaternion(
                        orientation.get('z', 0.0), orientation.get('w', 1.0)
                    )
                continue

            sub_match = _SUB_GOAL_RE.match(name)
            if sub_match:
                station_id = int(sub_match.group(1))
                pos = wp['position']
                self._sub_goal_positions[station_id] = (pos['x'], pos['y'])

    @staticmethod
    def _yaw_from_quaternion(z: float, w: float) -> float:
        # Waypoints only ever rotate about Z (planar arena), so x=y=0.
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def get_position(self, station_id: int) -> Optional[Tuple[float, float]]:
        return self._positions.get(station_id)

    # ------------------------------------------------------------------
    # Straight-line (Euclidean) distance — proximity checks only.
    # ------------------------------------------------------------------

    def station_to_station(self, from_id: int, to_id: int) -> float:
        """Euclidean distance between two station goal positions."""
        a = self._positions.get(from_id)
        b = self._positions.get(to_id)
        if a is None or b is None:
            return float('inf')
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def point_to_station(self, x: float, y: float, station_id: int) -> float:
        """Euclidean distance from an arbitrary (x, y) to a station goal position."""
        pos = self._positions.get(station_id)
        if pos is None:
            return float('inf')
        return math.hypot(x - pos[0], y - pos[1])

    # ------------------------------------------------------------------
    # Bezier-curve distance — route ordering / travel-time estimates.
    # ------------------------------------------------------------------

    def _entry_control_point(self, station_id: int) -> Optional[Tuple[float, float]]:
        """Point the approach curve should bend through before the goal.

        Prefers the station's real sub_goal waypoint (the physical
        alignment line the navigator drives: sub_goal -> goal, straight,
        perpendicular to the station). Falls back to a synthesized point
        `approach_line_length` meters back along the goal's heading when
        no sub_goal is defined (e.g. the home station).
        """
        goal = self._positions.get(station_id)
        if goal is None:
            return None

        sub_goal = self._sub_goal_positions.get(station_id)
        if sub_goal is not None:
            return sub_goal

        yaw = self._yaws.get(station_id)
        if yaw is None:
            return None
        return (
            goal[0] - math.cos(yaw) * self._approach_line_length,
            goal[1] - math.sin(yaw) * self._approach_line_length,
        )

    def _quadratic_bezier_length(
        self,
        p0: Tuple[float, float],
        p1: Tuple[float, float],
        p2: Tuple[float, float],
    ) -> float:
        total = 0.0
        prev = p0
        n = self._bezier_samples
        for i in range(1, n + 1):
            t = i / n
            mt = 1.0 - t
            x = mt * mt * p0[0] + 2.0 * mt * t * p1[0] + t * t * p2[0]
            y = mt * mt * p0[1] + 2.0 * mt * t * p1[1] + t * t * p2[1]
            total += math.hypot(x - prev[0], y - prev[1])
            prev = (x, y)
        return total

    def bezier_station_to_station(self, from_id: int, to_id: int) -> float:
        """Approximate driven distance from one station to another.

        Builds a quadratic Bezier curve from from_id's goal position to
        to_id's goal position, using to_id's approach-line point as the
        control point so the curve arrives tangent to the station's real
        docking heading — approximating the bend Nav2's A*-based planner
        actually drives instead of a straight line.
        """
        if from_id == to_id:
            return 0.0
        p0 = self._positions.get(from_id)
        p2 = self._positions.get(to_id)
        if p0 is None or p2 is None:
            return float('inf')
        p1 = self._entry_control_point(to_id) or p2
        return self._quadratic_bezier_length(p0, p1, p2)

    def bezier_point_to_station(self, x: float, y: float, station_id: int) -> float:
        """Same as bezier_station_to_station(), but starting from an
        arbitrary (x, y) point instead of a named station."""
        p2 = self._positions.get(station_id)
        if p2 is None:
            return float('inf')
        if p2 == (x, y):
            return 0.0
        p1 = self._entry_control_point(station_id) or p2
        return self._quadratic_bezier_length((x, y), p1, p2)

    def estimate_travel_time(
        self,
        from_id: int,
        to_id: int,
        driving_velocity: float,
        parking_duration: float,
        exiting_duration: float,
    ) -> float:
        """
        Rough travel time estimate between two stations.
        Assumes constant velocity cruise along the Bezier-curve distance;
        parking/exiting add fixed overhead.
        """
        dist = self.bezier_station_to_station(from_id, to_id)
        if driving_velocity <= 0:
            return float('inf')
        return dist / driving_velocity + parking_duration + exiting_duration
