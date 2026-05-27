from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

from irb120pe_cognitive_interfaces.msg import DetectedObject
from irb120pe_cognitive_interfaces.srv import GetDetectedObjects


class CognitivePerceptionNode(Node):
    """Publishes structured cube detections while keeping vision separate from reasoning."""

    def __init__(self) -> None:
        super().__init__("irb120pe_cognitive_perception")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("detections_2d_topic", "/irb120pe/perception/detections_2d")
        self.declare_parameter("detected_objects_service", "/irb120pe/perception/get_detected_objects")
        self.declare_parameter("planning_frame", "world")
        self.declare_parameter("camera_frame", "camera")
        self.declare_parameter("yolo_model_path", "")
        self.declare_parameter("yolo_confidence", 0.35)
        self.declare_parameter("detection_period_sec", 0.5)
        self.declare_parameter("table_height_m", 0.88)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_image: Any | None = None
        self.camera_info: CameraInfo | None = None
        self.model = self._load_yolo_model()
        self.objects: dict[str, DetectedObject] = {}

        self.detections_pub = self.create_publisher(
            Detection2DArray,
            self.get_parameter("detections_2d_topic").value,
            10,
        )
        # SensorDataQoS so we receive Gazebo's camera publisher regardless of its
        # reliability/durability settings (different Gazebo versions vary).
        self.create_subscription(Image, self.get_parameter("camera_topic").value, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value, self._camera_info_cb, qos_profile_sensor_data)
        self.create_service(GetDetectedObjects, self.get_parameter("detected_objects_service").value, self._get_objects_cb)
        self.create_timer(float(self.get_parameter("detection_period_sec").value), self._detect_once)

    def _load_yolo_model(self) -> Any | None:
        model_path = self.get_parameter("yolo_model_path").value
        if not model_path:
            try:
                model_path = str(
                    Path(get_package_share_directory("irb120pe_detection"))
                    / "yolov8"
                    / "cubeDETECTION_Gz.pt"
                )
            except Exception:
                model_path = str(
                    Path(os.path.expanduser("~"))
                    / "irb120_ws/src/irb120_PoseEstimation/irb120pe_detection/yolov8/cubeDETECTION_Gz.pt"
                )
        try:
            from ultralytics import YOLO

            return YOLO(model_path)
        except Exception as exc:
            self.get_logger().warn(f"YOLO model unavailable; perception node will publish no detections: {exc}")
            return None

    def _image_cb(self, msg: Image) -> None:
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self.camera_info is None:
            self.get_logger().info(
                f"camera_info received: frame_id={msg.header.frame_id} fx={msg.k[0]:.1f} fy={msg.k[4]:.1f}"
            )
        self.camera_info = msg

    def _get_objects_cb(self, _request: GetDetectedObjects.Request, response: GetDetectedObjects.Response) -> GetDetectedObjects.Response:
        response.objects = list(self.objects.values())
        response.status = f"{len(response.objects)} detected object(s) available."
        return response

    def _detect_once(self) -> None:
        if self.model is None or self.latest_image is None:
            return
        confidence_threshold = float(self.get_parameter("yolo_confidence").value)
        try:
            results = self.model(self.latest_image, conf=confidence_threshold, verbose=False)
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return

        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("planning_frame").value
        next_objects: dict[str, DetectedObject] = {}

        names = getattr(self.model, "names", {})
        label_counts: dict[str, int] = {}
        for box in results[0].boxes:
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            cls_id = int(box.cls[0])
            label = str(names.get(cls_id, cls_id))
            label_counts[label] = label_counts.get(label, 0) + 1
            object_id = f"{label}_{label_counts[label]}"
            confidence = float(box.conf[0])
            center_u = 0.5 * (x1 + x2)
            center_v = 0.5 * (y1 + y2)

            detection = Detection2D()
            detection.header = msg.header
            detection.bbox = self._make_bbox(center_u, center_v, x2 - x1, y2 - y1)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = label
            hypothesis.hypothesis.score = confidence
            detection.results.append(hypothesis)
            msg.detections.append(detection)

            obj = self._make_detected_object(object_id, label, confidence, center_u, center_v)
            next_objects[obj.object_id] = obj

        self.objects = next_objects
        self.detections_pub.publish(msg)

    def _make_bbox(self, center_u: float, center_v: float, size_x: float, size_y: float) -> BoundingBox2D:
        bbox = BoundingBox2D()
        bbox.center.position.x = center_u
        bbox.center.position.y = center_v
        bbox.size_x = size_x
        bbox.size_y = size_y
        return bbox

    def _make_detected_object(self, object_id: str, label: str, confidence: float, u: float, v: float) -> DetectedObject:
        obj = DetectedObject()
        obj.object_id = object_id
        obj.label = label
        obj.confidence = confidence
        obj.pose = self._estimate_pose(u, v)
        obj.stamp = self.get_clock().now().to_msg()
        return obj

    def _estimate_pose(self, u: float, v: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.get_parameter("planning_frame").value
        tf_pose = self._estimate_pose_with_tf(u, v)
        if tf_pose is not None:
            return tf_pose

        # Fallback for dry-run/mock demos when camera intrinsics or TF are not ready.
        # The upstream project estimates workspace x/y after perspective correction;
        # this maps image coordinates into the ArUco-bounded tray region.
        width = float(self.latest_image.shape[1]) if self.latest_image is not None else 1920.0
        height = float(self.latest_image.shape[0]) if self.latest_image is not None else 1080.0
        pose.pose.position.x = 0.47 + (u / width) * 0.16
        pose.pose.position.y = 0.27 + (v / height) * 0.51
        pose.pose.position.z = float(self.get_parameter("table_height_m").value) + 0.02
        pose.pose.orientation.w = 1.0
        return pose

    def _estimate_pose_with_tf(self, u: float, v: float) -> PoseStamped | None:
        if self.camera_info is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter("planning_frame").value,
                self.get_parameter("camera_frame").value,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except Exception as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}", throttle_duration_sec=5.0)
            return None

        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        if fx == 0.0 or fy == 0.0:
            return None

        # Image-plane ray (u-cx)/fx, (v-cy)/fy, 1 is in OPTICAL convention
        # (X right, Y down, Z forward). The IRB-120 URDF uses camera_link with
        # X-forward / Y-left / Z-up. Convert before rotating into world.
        ray_camera = (1.0, -(u - cx) / fx, -(v - cy) / fy)
        origin = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        ray_world = self._rotate_vector(transform.transform.rotation, ray_camera)
        plane_z = float(self.get_parameter("table_height_m").value) + 0.02
        if abs(ray_world[2]) < 1e-9:
            return None
        t = (plane_z - origin[2]) / ray_world[2]
        if t <= 0.0:
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.get_parameter("planning_frame").value
        pose.pose.position.x = origin[0] + t * ray_world[0]
        pose.pose.position.y = origin[1] + t * ray_world[1]
        pose.pose.position.z = plane_z
        pose.pose.orientation.w = 1.0
        return pose

    def _rotate_vector(self, q, vector: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, z, w = q.x, q.y, q.z, q.w
        vx, vy, vz = vector
        # Quaternion-vector multiplication, expanded to avoid extra dependencies.
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CognitivePerceptionNode()
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
