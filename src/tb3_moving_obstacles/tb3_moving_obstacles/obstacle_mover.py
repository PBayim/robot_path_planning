"""
Drives Gazebo obstacle models along scripted ping-pong trajectories via /set_entity_state.
"""
import math

import rclpy
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node


class ObstacleTrajectory:
    """Deterministic ping-pong motion between two waypoints at constant speed."""

    def __init__(self, name, x0, x1, y0, y1, speed, z, phase_offset):
        self.name = name
        self.x0, self.x1 = x0, x1
        self.y0, self.y1 = y0, y1
        self.z = z
        self.speed = speed
        self.phase_offset = phase_offset

        dx, dy = x1 - x0, y1 - y0
        self.segment_length = math.hypot(dx, dy)
        if self.segment_length > 1e-6:
            self.dir_x, self.dir_y = dx / self.segment_length, dy / self.segment_length
        else:
            self.dir_x, self.dir_y = 0.0, 0.0

        self.leg_duration = (
            self.segment_length / self.speed if self.speed > 1e-6 else math.inf
        )

    def pose_and_velocity_at(self, t):
        """Return ((x, y, z), (vx, vy)) for elapsed time t (seconds)."""
        t_eff = t - self.phase_offset
        if t_eff <= 0.0 or not math.isfinite(self.leg_duration):
            return (self.x0, self.y0, self.z), (0.0, 0.0)

        cycle = 2.0 * self.leg_duration
        phase = math.fmod(t_eff, cycle)

        if phase <= self.leg_duration:
            frac = phase / self.leg_duration
            x = self.x0 + frac * (self.x1 - self.x0)
            y = self.y0 + frac * (self.y1 - self.y0)
            vx, vy = self.speed * self.dir_x, self.speed * self.dir_y
        else:
            frac = (phase - self.leg_duration) / self.leg_duration
            x = self.x1 + frac * (self.x0 - self.x1)
            y = self.y1 + frac * (self.y0 - self.y1)
            vx, vy = -self.speed * self.dir_x, -self.speed * self.dir_y

        return (x, y, self.z), (vx, vy)


class ObstacleMover(Node):

    def __init__(self):
        super().__init__('obstacle_mover')

        self.declare_parameter('obstacle_names', [''])
        self.declare_parameter('update_rate_hz', 20.0)

        obstacle_names = [
            n for n in self.get_parameter('obstacle_names').value if n
        ]
        update_rate_hz = self.get_parameter('update_rate_hz').value

        self.trajectories = []
        for name in obstacle_names:
            self.declare_parameter(f'{name}.waypoints_x', [0.0, 0.0])
            self.declare_parameter(f'{name}.waypoints_y', [0.0, 0.0])
            self.declare_parameter(f'{name}.speed', 0.0)
            self.declare_parameter(f'{name}.z', 0.4)
            self.declare_parameter(f'{name}.phase_offset', 0.0)

            wx = self.get_parameter(f'{name}.waypoints_x').value
            wy = self.get_parameter(f'{name}.waypoints_y').value
            speed = self.get_parameter(f'{name}.speed').value
            z = self.get_parameter(f'{name}.z').value
            phase_offset = self.get_parameter(f'{name}.phase_offset').value

            self.trajectories.append(
                ObstacleTrajectory(name, wx[0], wx[1], wy[0], wy[1], speed, z, phase_offset)
            )
            self.get_logger().info(
                f"Loaded trajectory for '{name}': "
                f"({wx[0]:.2f},{wy[0]:.2f}) <-> ({wx[1]:.2f},{wy[1]:.2f}) "
                f"@ {speed:.2f} m/s, phase_offset={phase_offset:.1f}s"
            )

        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.start_time = self.get_clock().now()

        period = 1.0 / update_rate_hz
        self.timer = self.create_timer(period, self.update_obstacles)
        self._service_ready_logged = False

    def update_obstacles(self):
        if not self.set_state_client.service_is_ready():
            if not self._service_ready_logged:
                self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
                self._service_ready_logged = True
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        for traj in self.trajectories:
            (x, y, z), (vx, vy) = traj.pose_and_velocity_at(elapsed)

            request = SetEntityState.Request()
            request.state.name = traj.name
            request.state.pose.position.x = x
            request.state.pose.position.y = y
            request.state.pose.position.z = z
            request.state.pose.orientation.w = 1.0
            request.state.twist.linear.x = vx
            request.state.twist.linear.y = vy
            request.state.reference_frame = 'world'

            self.set_state_client.call_async(request)


def main():
    rclpy.init()
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
