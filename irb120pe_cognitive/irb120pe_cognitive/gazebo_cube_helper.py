import argparse
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
import xacro


SUPPORTED_CUBES = ("BlueCube", "BlackCube", "WhiteCube", "Cube")


def cube_urdf_path(cube: str) -> Path:
    if cube not in SUPPORTED_CUBES:
        choices = ", ".join(SUPPORTED_CUBES)
        raise ValueError(f"Unsupported cube '{cube}'. Expected one of: {choices}.")

    gazebo_share = Path(get_package_share_directory("irb120pe_gazebo"))
    return gazebo_share / "urdf" / "cube" / f"{cube}.urdf"


def expanded_cube_xml(cube: str, entity_name: str) -> str:
    path = cube_urdf_path(cube)
    if not path.exists():
        raise FileNotFoundError(f"Cube URDF does not exist: {path}")

    return xacro.process_file(str(path), mappings={"name": entity_name}).toxml()


def pose_from_args(args: argparse.Namespace) -> Pose:
    pose = Pose()
    pose.position.x = args.x
    pose.position.y = args.y
    pose.position.z = args.z
    pose.orientation.x = args.qx
    pose.orientation.y = args.qy
    pose.orientation.z = args.qz
    pose.orientation.w = args.qw
    return pose


class GazeboCubeClient(Node):
    def __init__(self) -> None:
        super().__init__("irb120pe_gazebo_cube_helper")
        self._spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self._delete_client = self.create_client(DeleteEntity, "/delete_entity")

    def spawn(
        self,
        *,
        cube: str,
        entity_name: str,
        pose: Pose,
        reference_frame: str,
        timeout_sec: float,
    ) -> SpawnEntity.Response:
        if not self._spawn_client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("/spawn_entity service was not available before timeout.")

        request = SpawnEntity.Request()
        request.name = entity_name
        request.xml = expanded_cube_xml(cube, entity_name)
        request.initial_pose = pose
        request.reference_frame = reference_frame

        future = self._spawn_client.call_async(request)
        return self._wait_for_future(future, timeout_sec)

    def delete(self, *, entity_name: str, timeout_sec: float) -> DeleteEntity.Response:
        if not self._delete_client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("/delete_entity service was not available before timeout.")

        request = DeleteEntity.Request()
        request.name = entity_name

        future = self._delete_client.call_async(request)
        return self._wait_for_future(future, timeout_sec)

    def _wait_for_future(self, future, timeout_sec: float):
        start = self.get_clock().now()
        timeout_ns = int(timeout_sec * 1_000_000_000)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start).nanoseconds > timeout_ns:
                raise TimeoutError("Gazebo service call did not finish before timeout.")
        return future.result()


def _add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--timeout", type=float, default=30.0, help="Seconds to wait for Gazebo services.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Spawn or delete Gazebo cube stimuli through xacro-expanded /spawn_entity XML."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    spawn = subparsers.add_parser("spawn", help="Spawn a xacro-expanded cube entity.")
    _add_common_args(spawn)
    spawn.add_argument("--cube", choices=SUPPORTED_CUBES, default="BlueCube")
    spawn.add_argument("--name", default=None, help="Gazebo entity name. Defaults to --cube.")
    spawn.add_argument("--x", type=float, default=0.55)
    spawn.add_argument("--y", type=float, default=0.52)
    spawn.add_argument("--z", type=float, default=0.88)
    spawn.add_argument("--qx", type=float, default=0.0)
    spawn.add_argument("--qy", type=float, default=0.0)
    spawn.add_argument("--qz", type=float, default=0.0)
    spawn.add_argument("--qw", type=float, default=1.0)
    spawn.add_argument("--reference-frame", default="world")
    spawn.add_argument(
        "--replace",
        action="store_true",
        help="Delete an existing entity with the same name before spawning.",
    )

    delete = subparsers.add_parser("delete", help="Delete a cube entity from Gazebo.")
    _add_common_args(delete)
    delete.add_argument("--name", default="BlueCube")

    return parser


def run(args: argparse.Namespace) -> int:
    rclpy.init()
    node = GazeboCubeClient()
    try:
        if args.command == "spawn":
            entity_name = args.name or args.cube
            if args.replace:
                delete_response = node.delete(entity_name=entity_name, timeout_sec=args.timeout)
                print(f"delete {delete_response.success} {delete_response.status_message}")
                if not delete_response.success and "does not exist" not in delete_response.status_message:
                    return 1

            response = node.spawn(
                cube=args.cube,
                entity_name=entity_name,
                pose=pose_from_args(args),
                reference_frame=args.reference_frame,
                timeout_sec=args.timeout,
            )
            print(f"spawn {response.success} {response.status_message}")
            return 0 if response.success else 1

        response = node.delete(entity_name=args.name, timeout_sec=args.timeout)
        print(f"delete {response.success} {response.status_message}")
        return 0 if response.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        sys.exit(run(args))
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
