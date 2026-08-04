#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class KinematicsPlannerNode(Node):
    def __init__(self):
        super().__init__('kinematics_planner_node')

        self.get_logger().info('Initializing Kinematics Planner Node...')

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for Joint States
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Current Joint Angles state (initial zero configuration)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # 20 Hz Timer for smooth, steady joint state publishing (prevents RViz flickering)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Action Client for Arm Controller
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # Subscriber for Target Pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_pose_callback,
            10
        )

        self.get_logger().info('Kinematics Planner Node initialized successfully.')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_joint_angles
        self.joint_state_pub.publish(msg)

    def target_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f'Received target pose in frame: {msg.header.frame_id}')

        # Transform pose to base_link if necessary
        target_pose = msg
        if msg.header.frame_id != 'base_link':
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0)
                )
                target_pose.pose.position.x += transform.transform.translation.x
                target_pose.pose.position.y += transform.transform.translation.y
                target_pose.pose.position.z += transform.transform.translation.z
                target_pose.header.frame_id = 'base_link'
                self.get_logger().info('Transformed target pose to base_link frame.')
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warning(f'Could not transform {msg.header.frame_id} to base_link: {e}')
                return

        # Compute Inverse Kinematics for target position
        x = target_pose.pose.position.x
        y = target_pose.pose.position.y
        z = target_pose.pose.position.z

        joint_angles = self.compute_inverse_kinematics(x, y, z)
        if joint_angles is None:
            self.get_logger().error('Failed to compute IK solution for target pose.')
            return

        self.get_logger().info(f'Computed IK Joint Angles (deg): {[round(math.degrees(a), 2) for a in joint_angles]}')

        # Update joint states for steady 20 Hz publishing
        self.current_joint_angles = joint_angles

        # Send goal to FollowJointTrajectory action server
        self.send_trajectory_goal(joint_angles)

    def compute_inverse_kinematics(self, x, y, z):
        """
        Analytical 6-DOF geometric IK solver for joint1..joint6.
        """
        try:
            q1 = math.atan2(y, x)
            r = math.sqrt(x**2 + y**2)

            shoulder_z = 0.3
            dz = z - shoulder_z
            d = math.sqrt(r**2 + dz**2)

            l1 = 0.4
            l2 = 0.5

            cos_q3 = (d**2 - l1**2 - l2**2) / (2 * l1 * l2)
            cos_q3 = max(-1.0, min(1.0, cos_q3))
            q3 = math.acos(cos_q3)

            alpha = math.atan2(dz, r)
            beta = math.atan2(l2 * math.sin(q3), l1 + l2 * math.cos(q3))

            q2 = (math.pi / 2) - (alpha + beta)

            q4 = 0.0
            q5 = - (q2 + q3) / 2.0
            q6 = 0.0

            return [q1, q2, q3, q4, q5, q6]

        except Exception as e:
            self.get_logger().error(f'IK computation error: {e}')
            return None

    def send_trajectory_goal(self, joint_angles):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('Action server /arm_controller/follow_joint_trajectory not responding; steady joint states active.')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0

        goal_msg.trajectory.points = [point]

        self.get_logger().info('Sending joint trajectory goal...')
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Trajectory goal rejected by controller.')
            return

        self.get_logger().info('Trajectory goal accepted. Executing motion...')


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
