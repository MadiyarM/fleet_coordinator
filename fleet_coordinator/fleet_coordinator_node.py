import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math
from itertools import combinations


# Goal waypoints per robot (map frame, metres).
# Meeting point shared by all three so they converge and trigger blocking,
# then disperse to their home points and release.
MEETING = (8.29, 6.41, 0.0)

GOALS = {
    'carter1': [MEETING, (8.17, 0.38, 0.0)],   # north home
    'carter2': [MEETING, (5.65, 11.78, 0.0)],   # west home
    'carter3': [MEETING, (16.3, 7.12, 0.0)],   # south home
}

ROBOT_NAMES = ['carter1', 'carter2', 'carter3']

BLOCK_DIST   = 3.0   # metres - block when closer than this
RELEASE_DIST = 5.0   # metres - release when farther than this


class RobotAgent:
    def __init__(self, name, node):
        self.name = name
        self.node = node

        self.x = 0.0
        self.y = 0.0
        self.pose_received = False      # bug 3: don't act before first pose

        self.blocked = False
        self.blocked_by = None          # who is blocking us
        self.goal_index = 0
        self.goal_handle = None
        self.nav_active = False         # bug 4: true while a goal is in flight

        self._nav_client = ActionClient(
            node, NavigateToPose, f'/{name}/navigate_to_pose'
        )
        node.create_subscription(
            PoseWithCovarianceStamped,
            f'/{name}/amcl_pose',
            self._pose_cb,
            10,
        )

    def _pose_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if not self.pose_received:
            self.pose_received = True
            self.node.get_logger().info(f'[{self.name}] First pose received: ({self.x:.2f}, {self.y:.2f})')

    def send_next_goal(self):
        if self.nav_active:             # bug 4: already navigating, don't double-send
            return
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().warn(f'[{self.name}] Nav2 action server not ready')
            return

        goals = GOALS[self.name]
        gx, gy, gyaw = goals[self.goal_index % len(goals)]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.orientation.w = 1.0

        self.node.get_logger().info(
            f'[{self.name}] Sending goal #{self.goal_index % len(goals)}: ({gx}, {gy})'
        )
        self.nav_active = True
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.node.get_logger().warn(f'[{self.name}] Goal rejected')
            self.nav_active = False
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self.nav_active = False
        self.goal_handle = None
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f'[{self.name}] Goal reached -> next waypoint')
            self.goal_index += 1
        elif status == GoalStatus.STATUS_CANCELED:
            # we cancelled it ourselves due to blocking - stay put
            self.node.get_logger().info(f'[{self.name}] Goal cancelled (blocked)')
            return
        else:
            self.node.get_logger().warn(
                f'[{self.name}] Goal aborted (status {status}) - retrying same waypoint'
            )

        # advance only if not blocked
        if not self.blocked:
            self.send_next_goal()

    def cancel_current_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            # nav_active cleared in _result_cb when cancellation completes

    def resume(self):
        if not self.nav_active:
            self.send_next_goal()


class FleetCoordinator(Node):
    def __init__(self):
        super().__init__('fleet_coordinator')

        self.robots = {name: RobotAgent(name, self) for name in ROBOT_NAMES}

        self._initial_done = False
        self.create_timer(3.0, self._initial_goals)
        self.create_timer(0.2, self._check_distances)   # 5 Hz hysteresis

    def _initial_goals(self):
        if self._initial_done:
            return
        # wait until every robot has a real pose before starting
        if not all(r.pose_received for r in self.robots.values()):
            self.get_logger().info('Waiting for all robot poses before sending goals...')
            return
        self._initial_done = True
        self.get_logger().info('Sending initial goals to all robots')
        for agent in self.robots.values():
            agent.send_next_goal()

    def _check_distances(self):
        if not self._initial_done:
            return

        agents = [self.robots[n] for n in ROBOT_NAMES]

        for a, b in combinations(agents, 2):
            # bug 3: skip pairs where we don't have a real position yet
            if not (a.pose_received and b.pose_received):
                continue

            dist = math.hypot(a.x - b.x, a.y - b.y)

            # --- bug 2: dynamic blocker ---
            # If they get too close, the lower-priority (later name) yields.
            # An already-blocked robot is not re-evaluated.
            if dist < BLOCK_DIST:
                if a.blocked or b.blocked:
                    continue

                if ROBOT_NAMES.index(a.name) < ROBOT_NAMES.index(b.name):
                    blocker, yielder = a, b
                else:
                    blocker, yielder = b, a

                self.get_logger().info(
                    f'[coordinator] {yielder.name} blocked by {blocker.name} '
                    f'(dist={dist:.2f}m < {BLOCK_DIST}m)'
                )
                yielder.blocked = True
                yielder.blocked_by = blocker.name
                yielder.cancel_current_goal()

            elif dist > RELEASE_DIST:
                for yielder, other in ((a, b), (b, a)):
                    if yielder.blocked and yielder.blocked_by == other.name:
                        self.get_logger().info(
                            f'[coordinator] {yielder.name} released from {other.name} '
                            f'(dist={dist:.2f}m > {RELEASE_DIST}m)'
                        )
                        yielder.blocked = False
                        yielder.blocked_by = None
                        yielder.resume()


def main(args=None):
    rclpy.init(args=args)
    node = FleetCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
