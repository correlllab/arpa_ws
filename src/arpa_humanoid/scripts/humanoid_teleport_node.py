#!/usr/bin/env python3
"""
Humanoid teleport: floor_link -> pelvis (H12) or g1_base (G1) TF + Gazebo SetEntityState.
Static model in Gazebo (no physics). Topics under /humanoid/ by default.

Subscribes:
    /humanoid/teleport_delta  (geometry_msgs/Point)
    /humanoid/teleport_pose   (geometry_msgs/Pose)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, TransformStamped
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from tf2_ros import TransformBroadcaster
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive


class HumanoidTeleportNode(Node):
    def __init__(self):
        super().__init__("humanoid_teleport_node")

        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 1.3)
        self.declare_parameter("initial_z", 0.78)
        self.declare_parameter("initial_yaw", -math.pi / 2.0)
        self.declare_parameter("model_name", "h12_humanoid")
        self.declare_parameter("tf_child_frame", "pelvis")
        self.declare_parameter("collision_object_id", "h12_humanoid_bbox")
        self.declare_parameter("publish_collision_box", False)

        self._x = self.get_parameter("initial_x").value
        self._y = self.get_parameter("initial_y").value
        self._z = self.get_parameter("initial_z").value
        self._yaw = self.get_parameter("initial_yaw").value
        self._model_name = self.get_parameter("model_name").value
        self._tf_child = self.get_parameter("tf_child_frame").value
        self._collision_id = self.get_parameter("collision_object_id").value
        self._publish_collision_box = bool(
            self.get_parameter("publish_collision_box").value
        )

        self._tf_broadcaster = TransformBroadcaster(self)

        self._set_state_clients = [
            self.create_client(SetEntityState, "/set_entity_state"),
            self.create_client(SetEntityState, "/gazebo/set_entity_state"),
        ]

        self.create_subscription(
            Point, "/humanoid/teleport_delta", self._on_delta, 10
        )
        self.create_subscription(
            Pose, "/humanoid/teleport_pose", self._on_pose, 10
        )

        self._planning_scene_pub = None
        if self._publish_collision_box:
            self._planning_scene_pub = self.create_publisher(
                PlanningScene, "/planning_scene", 10
            )

        self._tf_timer = self.create_timer(0.02, self._publish_tf)

        self._initial_collision_timer = None
        if self._publish_collision_box:
            self._initial_collision_timer = self.create_timer(
                5.0, self._publish_initial_collision
            )

        self.get_logger().info(
            f"Humanoid teleport model={self._model_name} tf={self._tf_child} "
            f"at ({self._x:.2f}, {self._y:.2f}, {self._z:.2f}), yaw={self._yaw:.2f}"
        )

    def _yaw_to_quat(self, yaw: float):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "floor_link"
        t.child_frame_id = self._tf_child
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = self._z
        qx, qy, qz, qw = self._yaw_to_quat(self._yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

        # Separate prefixed visual tree for RViz so it doesn't conflict with the
        # unprefixed transform the GUI uses for planning.
        t_visual = TransformStamped()
        t_visual.header = t.header
        t_visual.header.stamp = t.header.stamp
        t_visual.header.frame_id = "floor_link"
        t_visual.child_frame_id = "humanoid_visual/" + self._tf_child
        t_visual.transform = t.transform
        self._tf_broadcaster.sendTransform(t_visual)

    def _teleport_gazebo(self):
        client = None
        for candidate in self._set_state_clients:
            if candidate.wait_for_service(timeout_sec=0.5):
                client = candidate
                break
        if client is None:
            self.get_logger().warn("Gazebo set_entity_state service not available yet")
            return

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self._model_name
        req.state.pose.position.x = self._x
        req.state.pose.position.y = self._y
        req.state.pose.position.z = self._z
        qx, qy, qz, qw = self._yaw_to_quat(self._yaw)
        req.state.pose.orientation.x = qx
        req.state.pose.orientation.y = qy
        req.state.pose.orientation.z = qz
        req.state.pose.orientation.w = qw
        req.state.reference_frame = "world"

        future = client.call_async(req)
        future.add_done_callback(self._teleport_done)

    def _teleport_done(self, future):
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warn(f"Teleport failed: {result.status_message}")
        except Exception as e:
            self.get_logger().error(f"Teleport service call failed: {e}")

    def _publish_initial_collision(self):
        if not self._publish_collision_box:
            return
        self._publish_planning_scene_collision()
        self.get_logger().info("Published humanoid collision box to planning scene")
        if self._initial_collision_timer is not None:
            self._initial_collision_timer.cancel()
            self._initial_collision_timer = None

    def _publish_planning_scene_collision(self):
        if not self._publish_collision_box or self._planning_scene_pub is None:
            return
        co = CollisionObject()
        co.header.frame_id = "floor_link"
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = self._collision_id
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.45, 0.55, 1.55]

        box_pose = Pose()
        box_pose.position.x = self._x
        box_pose.position.y = self._y
        box_pose.position.z = self._z
        qx, qy, qz, qw = self._yaw_to_quat(self._yaw)
        box_pose.orientation.x = qx
        box_pose.orientation.y = qy
        box_pose.orientation.z = qz
        box_pose.orientation.w = qw

        co.primitives.append(box)
        co.primitive_poses.append(box_pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        self._planning_scene_pub.publish(scene)

    def _on_delta(self, msg: Point):
        self._x += msg.x
        self._y += msg.y
        self._z += msg.z
        self.get_logger().info(
            f"Teleport delta -> ({self._x:.3f}, {self._y:.3f}, {self._z:.3f})"
        )
        self._teleport_gazebo()
        self._publish_planning_scene_collision()

    def _on_pose(self, msg: Pose):
        self._x = msg.position.x
        self._y = msg.position.y
        self._z = msg.position.z
        qz = msg.orientation.z
        qw = msg.orientation.w
        self._yaw = 2.0 * math.atan2(qz, qw)
        self.get_logger().info(
            f"Teleport abs -> ({self._x:.3f}, {self._y:.3f}, {self._z:.3f}), yaw={self._yaw:.2f}"
        )
        self._teleport_gazebo()
        self._publish_planning_scene_collision()


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidTeleportNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
