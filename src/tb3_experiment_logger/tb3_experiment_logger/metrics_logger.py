"""
Passive experiment logger for NavigateToPose runs. Detects goal start/end via the action
status topic and appends one CSV row per completed run. Runs indefinitely.
"""
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy, qos_profile_action_status_default)
from tf_transformations import euler_from_quaternion

CSV_FIELDS = [
    'run', 'timestamp', 'controller',
    'goal_x', 'goal_y', 'goal_yaw_deg',
    'status', 'time_to_goal_s', 'num_recoveries',
    'final_x', 'final_y', 'final_yaw_deg', 'xy_error_m', 'yaw_error_deg',
    'planned_path_length_m', 'executed_path_length_m', 'path_length_ratio',
    'mean_linear_vel', 'max_linear_vel',
    'mean_linear_accel', 'max_linear_accel',
    'mean_angular_vel', 'max_angular_vel',
    'mean_tracking_error_m', 'max_tracking_error_m',
]

_TERMINAL = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
             GoalStatus.STATUS_CANCELED}
_STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED:   'ABORTED',
    GoalStatus.STATUS_CANCELED:  'CANCELED',
}


def _yaw(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def _path_length(pts):
    return sum(math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
               for i in range(1, len(pts)))


def _nearest(point, path):
    if not path:
        return float('nan')
    return min(math.hypot(point[0] - p[0], point[1] - p[1]) for p in path)


def _mean(v):
    return sum(v) / len(v) if v else float('nan')


def _fmax(v):
    return max(v) if v else float('nan')


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')

        self.declare_parameter('output_csv', '/tmp/tb3_experiment_results.csv')
        self.declare_parameter('controller', 'unspecified')

        self._output_csv = self.get_parameter('output_csv').value
        self._controller = self.get_parameter('controller').value
        self._run = 0

        # recording state
        self._recording = False
        self._tracked_uuid = None          # UUID bytes of the current goal
        self._prev_statuses = {}           # {uuid_bytes: status} from last msg
        self._goal_x = self._goal_y = self._goal_yaw_deg = float('nan')
        self._goal_time = None
        self._result_time = None
        self._final_status = None
        self._odom = []       # (t, x, y, yaw)
        self._cmdvel = []     # (t, v, w)
        self._plan = []       # [(x, y)] first plan only
        self._amcl = None     # (x, y, yaw)
        self._recoveries = 0

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        plan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        amcl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        feedback_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(Odometry, '/odom', self._odom_cb, sensor_qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmdvel_cb, sensor_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, amcl_qos)
        self.create_subscription(Path, '/plan', self._plan_cb, plan_qos)
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._status_cb,
            qos_profile_action_status_default)
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self._feedback_cb,
            feedback_qos)

        os.makedirs(os.path.dirname(self._output_csv), exist_ok=True)
        self.get_logger().info(
            f'Passive metrics logger ready.\n'
            f'Controller: {self._controller}\n'
            f'Output: {self._output_csv}\n'
            f'Send goals from RViz to start recording.')

    # ------------------------------------------------------------------
    # data callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        if not self._recording:
            return
        self._odom.append((
            time.time(),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            _yaw(msg.pose.pose.orientation),
        ))

    def _cmdvel_cb(self, msg):
        if not self._recording:
            return
        self._cmdvel.append((time.time(), msg.linear.x, msg.angular.z))

    def _amcl_cb(self, msg):
        self._amcl = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            _yaw(msg.pose.pose.orientation),
        )

    def _plan_cb(self, msg):
        if not self._recording or self._plan:
            return
        self._plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if msg.poses:
            last = msg.poses[-1]
            self._goal_x = last.pose.position.x
            self._goal_y = last.pose.position.y
            self._goal_yaw_deg = math.degrees(_yaw(last.pose.orientation))
        self.get_logger().info(
            f'[Run {self._run + 1}] Plan captured: {len(self._plan)} poses, '
            f'{_path_length(self._plan):.2f} m — '
            f'goal ({self._goal_x:.2f}, {self._goal_y:.2f})')

    def _feedback_cb(self, msg):
        if self._recording:
            self._recoveries = msg.feedback.number_of_recoveries

    # ------------------------------------------------------------------
    # action status — drives the recording state machine
    # ------------------------------------------------------------------

    def _status_cb(self, msg):
        current = {bytes(s.goal_info.goal_id.uuid): s.status
                   for s in msg.status_list}

        if not self._recording:
            for uuid, status in current.items():
                if (status == GoalStatus.STATUS_EXECUTING
                        and self._prev_statuses.get(uuid) != GoalStatus.STATUS_EXECUTING):
                    self._start_recording(uuid)
                    break
        else:
            if self._tracked_uuid in current:
                status = current[self._tracked_uuid]
                if status in _TERMINAL:
                    self._finish(status)
            elif self._tracked_uuid not in current:
                # Goal dropped off the list — use previous status if terminal
                prev = self._prev_statuses.get(self._tracked_uuid)
                if prev in _TERMINAL:
                    self._finish(prev)
                # If prev was still EXECUTING, the goal was cancelled abruptly
                elif prev == GoalStatus.STATUS_EXECUTING:
                    self._finish(GoalStatus.STATUS_ABORTED)

        self._prev_statuses = current

    def _start_recording(self, uuid):
        self._recording = True
        self._tracked_uuid = uuid
        self._goal_time = time.time()
        self._result_time = None
        self._final_status = None
        self._goal_x = self._goal_y = self._goal_yaw_deg = float('nan')
        self._odom = []
        self._cmdvel = []
        self._plan = []
        self._recoveries = 0
        self.get_logger().info(f'[Run {self._run + 1}] New goal detected — recording started')

    def _finish(self, status):
        self._final_status = status
        self._result_time = time.time()
        self._recording = False
        self._write_row()

    # ------------------------------------------------------------------
    # CSV output
    # ------------------------------------------------------------------

    def _write_row(self):
        self._run += 1
        elapsed = self._result_time - self._goal_time
        status_name = _STATUS_NAMES.get(self._final_status, str(self._final_status))

        final_x = final_y = final_yaw_deg = xy_err = yaw_err = float('nan')
        if self._amcl:
            final_x, final_y, final_yaw = self._amcl
            final_yaw_deg = math.degrees(final_yaw)
            if not math.isnan(self._goal_x):
                xy_err = math.hypot(final_x - self._goal_x, final_y - self._goal_y)
                yaw_err = math.degrees(abs(math.atan2(
                    math.sin(final_yaw - math.radians(self._goal_yaw_deg)),
                    math.cos(final_yaw - math.radians(self._goal_yaw_deg)))))

        odom_xy = [(x, y) for (_, x, y, _) in self._odom]
        plan_len = _path_length(self._plan)
        exec_len = _path_length(odom_xy)
        ratio = exec_len / plan_len if plan_len > 0 else float('nan')

        lin_v = [abs(v) for (_, v, _) in self._cmdvel]
        ang_v = [abs(w) for (_, _, w) in self._cmdvel]
        accels = [
            (self._cmdvel[i][1] - self._cmdvel[i-1][1])
            / (self._cmdvel[i][0] - self._cmdvel[i-1][0])
            for i in range(1, len(self._cmdvel))
            if self._cmdvel[i][0] > self._cmdvel[i-1][0]
        ]
        tracking = [_nearest(p, self._plan) for p in odom_xy]

        row = {
            'run':                    self._run,
            'timestamp':              datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'controller':             self._controller,
            'goal_x':                 round(self._goal_x, 3),
            'goal_y':                 round(self._goal_y, 3),
            'goal_yaw_deg':           round(self._goal_yaw_deg, 1),
            'status':                 status_name,
            'time_to_goal_s':         round(elapsed, 2),
            'num_recoveries':         self._recoveries,
            'final_x':                round(final_x, 3),
            'final_y':                round(final_y, 3),
            'final_yaw_deg':          round(final_yaw_deg, 1),
            'xy_error_m':             round(xy_err, 3),
            'yaw_error_deg':          round(yaw_err, 1),
            'planned_path_length_m':  round(plan_len, 3),
            'executed_path_length_m': round(exec_len, 3),
            'path_length_ratio':      round(ratio, 3),
            'mean_linear_vel':        round(_mean(lin_v), 3),
            'max_linear_vel':         round(_fmax(lin_v), 3),
            'mean_linear_accel':      round(_mean([abs(a) for a in accels]), 3),
            'max_linear_accel':       round(_fmax([abs(a) for a in accels]), 3),
            'mean_angular_vel':       round(_mean(ang_v), 3),
            'max_angular_vel':        round(_fmax(ang_v), 3),
            'mean_tracking_error_m':  round(_mean(tracking), 3),
            'max_tracking_error_m':   round(_fmax(tracking), 3),
        }

        write_header = not os.path.exists(self._output_csv)
        with open(self._output_csv, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            if write_header:
                writer.writeheader()
            writer.writerow(row)

        self.get_logger().info(
            f'[Run {self._run}] {status_name} in {elapsed:.1f}s — '
            f'xy_error={xy_err:.3f}m — row written')


def main():
    rclpy.init()
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
