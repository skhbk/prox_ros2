from prox2f_detection.object_detection_params import object_detection

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray

import cv2
import message_filters
import numpy as np
import open3d as o3d
import quaternion
from cv_bridge import CvBridge

import ultralytics
from ultralytics.engine.results import Results


class ObjectDetection(Node):
    def __init__(self):
        super().__init__("object_detection")

        self.param_listener = object_detection.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.model = ultralytics.YOLO(self.params.model_path)

        self.tracked_poses: dict[int, PoseWithCovariance] = {}

        #  Subscriptions
        self.color_subscription = message_filters.Subscriber(
            self, Image, "/color/image_raw"
        )
        self.depth_subscription = message_filters.Subscriber(
            self, Image, "/depth/image_raw"
        )
        self.depth_info_subscription = message_filters.Subscriber(
            self, CameraInfo, "/depth/camera_info"
        )
        self.time_sync = message_filters.TimeSynchronizer(
            (
                self.color_subscription,
                self.depth_subscription,
                self.depth_info_subscription,
            ),
            10,
        )
        self.time_sync.registerCallback(self.topic_callback)

        # Publishers
        self.image_publisher = self.create_publisher(
            Image, "~/image", qos_profile_sensor_data
        )
        self.markers_publisher = self.create_publisher(
            MarkerArray, "~/markers", qos_profile_sensor_data
        )
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "~/pose", qos_profile_sensor_data
        )

    def topic_callback(
        self, color_msg: Image, depth_msg: Image, depth_info_msg: CameraInfo
    ):
        if color_msg.width != depth_msg.width or color_msg.height != depth_msg.height:
            self.get_logger().error(
                "Image sizes do not match. "
                f"color: {color_msg.width}x{color_msg.height}, "
                f"depth: {depth_msg.width}x{depth_msg.height}"
            )
            return

        cv_bridge = CvBridge()

        color_img = cv_bridge.imgmsg_to_cv2(color_msg)
        depth_img = cv_bridge.imgmsg_to_cv2(depth_msg)

        depth_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            depth_info_msg.width,
            depth_info_msg.height,
            depth_info_msg.k.reshape([3, 3]),
        )

        results: list[Results] = self.model.track(
            source=color_img,
            verbose=False,
            conf=self.params.confidence_threshold,
            iou=self.params.iou_threshold,
            device=self.params.device,
            persist=True,
        )

        assert len(results) == 1
        results = results[0].cpu()

        track_ids = (
            results.boxes.id.int().tolist() if results.boxes.id is not None else []
        )
        masks = results.masks if results.masks is not None else []

        markers_msg = MarkerArray()

        track_ids_to_delete = set(self.tracked_poses.keys()) - set(track_ids)

        for track_id, mask in zip(track_ids, masks):
            # Create mask image
            mask_img = np.zeros(depth_img.shape, dtype=np.uint16)
            mask_img = cv2.fillPoly(
                mask_img,
                [np.array(mask.xy).astype(int)],
                np.iinfo(np.uint16).max,
            )
            erosion_kernel = np.ones(
                (self.params.point_cloud.erosion.kernel_size) * 2, np.uint8
            )
            mask_img = cv2.erode(mask_img, erosion_kernel)
            masked_depth_img = cv2.bitwise_and(depth_img, mask_img)

            # Convert masked depth image to pointcloud
            cloud = o3d.geometry.PointCloud.create_from_depth_image(
                o3d.geometry.Image(masked_depth_img),
                depth_intrinsic,
            )
            cloud = cloud.voxel_down_sample(
                voxel_size=self.params.point_cloud.down_sample.voxel_size
            )
            cloud, _ = cloud.remove_statistical_outlier(
                self.params.point_cloud.outlier_removal.n_neighbors,
                self.params.point_cloud.outlier_removal.std_ratio,
            )

            try:
                # Get bounding box from pointcloud
                bounding_box = cloud.get_oriented_bounding_box()
            except RuntimeError:
                track_ids_to_delete.add(track_id)
                continue

            point = Point()
            point.x = bounding_box.center[0]
            point.y = bounding_box.center[1]
            point.z = bounding_box.center[2]

            orientation = Quaternion()
            q = quaternion.from_rotation_matrix(bounding_box.R)
            orientation.w = q.w
            orientation.x = q.x
            orientation.y = q.y
            orientation.z = q.z

            current_pose = Pose()
            current_pose.position = point
            current_pose.orientation = orientation

            if track_id not in self.tracked_poses:
                pose = PoseWithCovariance()
                pose.pose = current_pose
                pose.covariance = np.identity(6, np.float64).flatten()
                self.tracked_poses[track_id] = pose
            else:
                pose = self.tracked_poses[track_id]
                covariances = pose.covariance.reshape([6, 6])

                xyz = point_to_xyz(pose.pose.position)
                rpy = quaternion_to_rpy(pose.pose.orientation)
                xyz_variances = get_xyz_variances(covariances)
                rpy_variances = get_rpy_variances(covariances)

                current_xyz = point_to_xyz(current_pose.position)
                current_rpy = quaternion_to_rpy(current_pose.orientation)

                alpha = self.params.pose.update_ratio

                new_xyz = (1 - alpha) * xyz + alpha * current_xyz
                new_rpy = (1 - alpha) * rpy + alpha * current_rpy

                new_xyz_variances = (
                    (1 - alpha) * (xyz_variances + xyz**2)
                    + alpha * current_xyz**2
                    - new_xyz**2
                )
                new_rpy_variances = (
                    (1 - alpha) * (rpy_variances + rpy**2)
                    + alpha * current_rpy**2
                    - new_rpy**2
                )

                pose.pose.position = xyz_to_point(new_xyz)
                pose.pose.orientation = rpy_to_quaternion(new_rpy)
                pose.covariance = get_covariances(
                    np.concatenate([new_xyz_variances, new_rpy_variances])
                ).flatten()

            marker = Marker()
            marker.header = depth_msg.header
            marker.ns = self.get_namespace()
            marker.id = track_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = self.tracked_poses[track_id].pose
            marker.scale.x = bounding_box.extent[0]
            marker.scale.y = bounding_box.extent[1]
            marker.scale.z = bounding_box.extent[2]
            marker.color.r = 1.0
            marker.color.a = 0.5
            markers_msg.markers.append(marker)

        for track_id in track_ids_to_delete:
            if track_id in self.tracked_poses:
                del self.tracked_poses[track_id]

            # Delete markers
            marker = Marker()
            marker.header = depth_msg.header
            marker.ns = self.get_namespace()
            marker.id = track_id
            marker.action = Marker.DELETE
            markers_msg.markers.append(marker)

        if any(self.tracked_poses):
            poses = list(self.tracked_poses.values())
            poses.sort(key=lambda x: x.covariance.mean())

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header = depth_msg.header
            pose_msg.pose = poses[0]
            self.pose_publisher.publish(pose_msg)

        self.markers_publisher.publish(markers_msg)

        # Plot results image
        plot = results.plot()
        image_msg = cv_bridge.cv2_to_imgmsg(
            plot, encoding=color_msg.encoding, header=color_msg.header
        )
        self.image_publisher.publish(image_msg)


def point_to_xyz(point: Point):
    return np.array([point.x, point.y, point.z], np.float64)


def xyz_to_point(xyz: np.ndarray):
    point = Point()
    point.x = xyz[0]
    point.y = xyz[1]
    point.z = xyz[2]
    return point


def quaternion_to_rpy(q_msg: Quaternion):
    q = np.quaternion(q_msg.w, q_msg.x, q_msg.y, q_msg.z)
    rpy = quaternion.as_euler_angles(q)
    return np.array(rpy, np.float64)


def rpy_to_quaternion(rpy: np.ndarray):
    q = quaternion.from_euler_angles(rpy[0], rpy[1], rpy[2])
    q_msg = Quaternion()
    q_msg.w = q.w
    q_msg.x = q.x
    q_msg.y = q.y
    q_msg.z = q.z
    return q_msg


def get_xyz_variances(covariance: np.ndarray):
    return np.array([covariance[0, 0], covariance[1, 1], covariance[2, 2]], np.float64)


def get_rpy_variances(covariance: np.ndarray):
    return np.array([covariance[3, 3], covariance[4, 4], covariance[5, 5]], np.float64)


def get_covariances(variances: np.ndarray):
    covariances = np.zeros([6, 6], np.float64)
    for i in range(6):
        covariances[i, i] = variances[i]
    return covariances


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetection()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
