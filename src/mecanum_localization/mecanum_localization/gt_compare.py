# Copyright 2026 Aryan Hegde
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Measure odometry drift against the simulation ground truth (simulation only).

Pairs each estimate on ``odom_topic`` (/odom from the LIO adapter) with the ground truth on
``gt_topic`` (/sim/ground_truth/odom, 50 Hz), interpolated at the estimate's stamp. Both
trajectories are taken relative to their pose at the first estimate, so the frames they are
published in do not matter. Planar only: x, y and yaw.

Reports every ``report_period`` seconds and once more on exit (``duration`` elapsed, or Ctrl-C):
path length, final and maximum position error, drift as a percentage of the path, final and
maximum yaw error, RMS position error, mean twist error, and the number of jumps (an estimate
step that exceeds the true step by more than ``jump_threshold``, a sign of a LIO reset or
divergence). The run passes when both the final and the maximum position error are within
``max_drift_percent`` of the whole path, the maximum yaw error is within ``max_yaw_error_deg``,
and there are no jumps. The maximum matters for out-and-back runs, where an estimate that never
moved still ends with no final error.
"""

import bisect
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .planar_math import flatten, relative, wrap_angle


def interpolate_pose(t, t0, pose0, t1, pose1):
    """Interpolate a planar pose at time ``t`` between two samples; yaw takes the short way."""
    if t1 <= t0:
        return pose1
    a = min(max((t - t0) / (t1 - t0), 0.0), 1.0)
    x0, y0, yaw0 = pose0
    x1, y1, yaw1 = pose1
    return (x0 + a * (x1 - x0), y0 + a * (y1 - y0),
            wrap_angle(yaw0 + a * wrap_angle(yaw1 - yaw0)))


class GroundTruthBuffer:
    """Time-ordered ground-truth samples (stamp, planar pose, planar twist), interpolated."""

    def __init__(self, keep_seconds=10.0):
        self._keep = keep_seconds
        self._t = []
        self._samples = []

    @property
    def latest(self):
        """Return the newest stamp, or None when empty."""
        return self._t[-1] if self._t else None

    def add(self, t, pose, twist):
        """Append a sample; samples must arrive in time order. Old samples are dropped."""
        if self._t and t <= self._t[-1]:
            return
        self._t.append(t)
        self._samples.append((pose, twist))
        cut = bisect.bisect_left(self._t, t - self._keep)
        if cut:
            del self._t[:cut]
            del self._samples[:cut]

    def at(self, t):
        """Return (pose, twist) at time ``t``, or None if ``t`` is outside the buffer."""
        if not self._t or t < self._t[0] or t > self._t[-1]:
            return None
        i = bisect.bisect_left(self._t, t)
        if self._t[i] == t:
            return self._samples[i]
        (pose0, _), (pose1, twist1) = self._samples[i - 1], self._samples[i]
        return interpolate_pose(t, self._t[i - 1], pose0, self._t[i], pose1), twist1


class DriftTracker:
    """Accumulate paired estimate / ground-truth poses and compute drift statistics."""

    def __init__(self, jump_threshold=0.2):
        self._jump_threshold = jump_threshold
        self._est0 = None
        self._gt0 = None
        self._prev = None
        self.samples = 0
        self.path_length = 0.0
        self.final_error = 0.0
        self.max_error = 0.0
        self.final_yaw_error = 0.0
        self.max_yaw_error = 0.0
        self.jumps = 0
        self._sum_sq_error = 0.0
        self._sum_twist_error = [0.0, 0.0, 0.0]
        self.rows = []

    def add(self, t, est_pose, gt_pose, est_twist=None, gt_twist=None):
        """Add one pair of planar poses (and optionally planar twists (vx, vy, wz))."""
        if self._est0 is None:
            self._est0, self._gt0 = est_pose, gt_pose
        est = relative(self._est0, est_pose)
        gt = relative(self._gt0, gt_pose)
        if self._prev is not None:
            prev_est, prev_gt = self._prev
            gt_step = math.hypot(gt[0] - prev_gt[0], gt[1] - prev_gt[1])
            est_step = math.hypot(est[0] - prev_est[0], est[1] - prev_est[1])
            self.path_length += gt_step
            if est_step - gt_step > self._jump_threshold:
                self.jumps += 1
        self._prev = (est, gt)

        error = math.hypot(est[0] - gt[0], est[1] - gt[1])
        yaw_error = wrap_angle(est[2] - gt[2])
        self.samples += 1
        self.final_error = error
        self.max_error = max(self.max_error, error)
        self.final_yaw_error = yaw_error
        self.max_yaw_error = max(self.max_yaw_error, abs(yaw_error))
        self._sum_sq_error += error * error
        if est_twist is not None and gt_twist is not None:
            for k in range(3):
                self._sum_twist_error[k] += abs(est_twist[k] - gt_twist[k])
        self.rows.append((t, *est, *gt))

    @property
    def drift_percent(self):
        """Return the final position error as a percentage of the true path length."""
        return 100.0 * self.final_error / self.path_length if self.path_length > 0 else 0.0

    @property
    def max_drift_percent(self):
        """Return the maximum position error as a percentage of the whole true path."""
        return 100.0 * self.max_error / self.path_length if self.path_length > 0 else 0.0

    @property
    def rms_error(self):
        """Return the root-mean-square position error over all samples."""
        return math.sqrt(self._sum_sq_error / self.samples) if self.samples else 0.0

    @property
    def mean_twist_error(self):
        """Return the mean absolute (vx, vy, wz) error over all samples."""
        n = max(self.samples, 1)
        return tuple(e / n for e in self._sum_twist_error)

    def passed(self, max_drift_percent, max_yaw_error_deg):
        """Return True when the position and yaw errors are within the targets, with no jumps."""
        return (self.samples > 0 and self.drift_percent <= max_drift_percent
                and self.max_drift_percent <= max_drift_percent
                and math.degrees(self.max_yaw_error) <= max_yaw_error_deg and self.jumps == 0)

    def summary(self, max_drift_percent, max_yaw_error_deg):
        """Return a multi-line text report."""
        vx, vy, wz = self.mean_twist_error
        verdict = 'PASS' if self.passed(max_drift_percent, max_yaw_error_deg) else 'FAIL'
        return '\n'.join([
            f'samples {self.samples}, true path {self.path_length:.2f} m',
            f'position error: final {self.final_error:.3f} m '
            f'({self.drift_percent:.2f}% of path, target < {max_drift_percent:g}%), '
            f'max {self.max_error:.3f} m ({self.max_drift_percent:.2f}%), '
            f'RMS {self.rms_error:.3f} m',
            f'yaw error: final {math.degrees(self.final_yaw_error):+.2f} deg, '
            f'max {math.degrees(self.max_yaw_error):.2f} deg (target < {max_yaw_error_deg:g} deg)',
            f'mean twist error: vx {vx:.3f} m/s, vy {vy:.3f} m/s, wz {wz:.3f} rad/s',
            f'jumps {self.jumps}',
            verdict,
        ])


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def _planar(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    tw = msg.twist.twist
    return (flatten(p.x, p.y, q.x, q.y, q.z, q.w),
            (tw.linear.x, tw.linear.y, tw.angular.z))


class GtCompare(Node):

    def __init__(self):
        super().__init__('gt_compare')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('gt_topic', '/sim/ground_truth/odom')
        self.declare_parameter('report_period', 10.0)
        self.declare_parameter('duration', 0.0)
        # STAND-IN: drift targets from docs/plan.md phase 4, not derived from a requirement yet.
        self.declare_parameter('max_drift_percent', 1.0)
        self.declare_parameter('max_yaw_error_deg', 2.0)
        self.declare_parameter('jump_threshold', 0.2)
        self.declare_parameter('csv_path', '')

        self._max_drift = float(self.get_parameter('max_drift_percent').value)
        self._max_yaw = float(self.get_parameter('max_yaw_error_deg').value)
        self._duration = float(self.get_parameter('duration').value)
        self._report_period = float(self.get_parameter('report_period').value)
        self.tracker = DriftTracker(float(self.get_parameter('jump_threshold').value))
        self._gt = GroundTruthBuffer()
        self._pending = []
        self._start = None
        self._last_report = None
        self.done = False

        odom_topic = self.get_parameter('odom_topic').value
        gt_topic = self.get_parameter('gt_topic').value
        self.create_subscription(Odometry, odom_topic, self._on_estimate, 50)
        self.create_subscription(Odometry, gt_topic, self._on_gt, 200)
        self.get_logger().info(f'comparing {odom_topic} against {gt_topic}')

    def _on_estimate(self, msg):
        self._pending.append((_stamp(msg), *_planar(msg)))
        self._process()

    def _on_gt(self, msg):
        pose, twist = _planar(msg)
        self._gt.add(_stamp(msg), pose, twist)
        self._process()

    def _process(self):
        latest = self._gt.latest
        if latest is None:
            return
        keep = []
        for t, pose, twist in self._pending:
            if t > latest:
                keep.append((t, pose, twist))
                continue
            gt = self._gt.at(t)
            if gt is None:
                continue  # older than the ground-truth buffer (only before the first pair)
            self.tracker.add(t, pose, gt[0], twist, gt[1])
            if self._start is None:
                self._start = self._last_report = t
            if self._report_period > 0 and t - self._last_report >= self._report_period:
                self._last_report = t
                self.get_logger().info(
                    f'[{t - self._start:.0f} s] ' + self.report().replace('\n', ' | '))
            if self._duration > 0 and t - self._start >= self._duration:
                self.done = True
        self._pending = keep

    def report(self):
        return self.tracker.summary(self._max_drift, self._max_yaw)

    def write_csv(self):
        path = self.get_parameter('csv_path').value
        if not path:
            return
        with open(path, 'w') as f:
            f.write('t,est_x,est_y,est_yaw,gt_x,gt_y,gt_yaw\n')
            for row in self.tracker.rows:
                f.write(','.join(f'{v:.6f}' for v in row) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = GtCompare()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        # SIGTERM shuts the context down under spin_once, which then raises RCLError.
        if rclpy.ok():
            raise
    finally:
        print('gt_compare final report:\n' + node.report(), flush=True)
        node.write_csv()
        passed = node.tracker.passed(node._max_drift, node._max_yaw)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
