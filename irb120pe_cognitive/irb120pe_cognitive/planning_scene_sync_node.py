from __future__ import annotations

import rclpy
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from irb120pe_cognitive_interfaces.srv import GetDetectedObjects


class PlanningSceneSyncNode(Node):
    """Synchronizes perceived cubes into the MoveIt 2 Planning Scene."""

    def __init__(self) -> None:
        super().__init__("irb120pe_planning_scene_sync")
        self.declare_parameter("detected_objects_service", "/irb120pe/perception/get_detected_objects")
        self.declare_parameter("planning_frame", "world")
        self.declare_parameter("cube_size_m", 0.035)
        self.declare_parameter("sync_period_sec", 1.0)

        self.detected_objects_client = self.create_client(
            GetDetectedObjects,
            self.get_parameter("detected_objects_service").value,
        )
        self.apply_scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.known_object_ids: set[str] = set()
        self.create_timer(float(self.get_parameter("sync_period_sec").value), self._sync_once)

    def _sync_once(self) -> None:
        if not self.detected_objects_client.wait_for_service(timeout_sec=0.1):
            return
        future = self.detected_objects_client.call_async(GetDetectedObjects.Request())
        future.add_done_callback(self._objects_received)

    def _objects_received(self, future: rclpy.task.Future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Could not read detected objects: {exc}")
            return
        current_ids = {obj.object_id for obj in response.objects}
        collision_objects: list[CollisionObject] = []

        # Add/update every currently perceived object. MoveIt treats ADD with an
        # existing id as a replacement, so moving cubes are kept current.
        for obj in response.objects:
            collision_objects.append(self._make_box(obj.object_id, obj.pose, CollisionObject.ADD))

        # Remove objects that disappeared from perception so stale obstacles do not
        # block valid plans. The perception node controls object freshness.
        for stale_id in sorted(self.known_object_ids - current_ids):
            removal = CollisionObject()
            removal.id = stale_id
            removal.header.frame_id = self.get_parameter("planning_frame").value
            removal.operation = CollisionObject.REMOVE
            collision_objects.append(removal)

        if collision_objects:
            self._apply_scene(collision_objects)
        self.known_object_ids = current_ids

    def _make_box(self, object_id: str, pose_stamped, operation: int) -> CollisionObject:
        collision_object = CollisionObject()
        collision_object.header = pose_stamped.header
        if not collision_object.header.frame_id:
            collision_object.header.frame_id = self.get_parameter("planning_frame").value
        collision_object.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        size = float(self.get_parameter("cube_size_m").value)
        primitive.dimensions = [size, size, size]
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose_stamped.pose)
        collision_object.operation = operation
        return collision_object

    def _apply_scene(self, collision_objects: list[CollisionObject]) -> None:
        if not self.apply_scene_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("/apply_planning_scene is not available yet.")
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_client.call_async(request)
        future.add_done_callback(self._scene_applied)

    def _scene_applied(self, future: rclpy.task.Future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Planning Scene update failed: {exc}")
            return
        if not result.success:
            self.get_logger().warn("MoveIt rejected the Planning Scene update.")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlanningSceneSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
