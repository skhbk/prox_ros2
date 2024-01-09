from prox2f_detection.object_detection_params import object_detection

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
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

        self.track_ids = []

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
        self.poses_publisher = self.create_publisher(
            PoseArray, "~/poses", qos_profile_sensor_data
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

        poses_msg = PoseArray()
        poses_msg.header = depth_msg.header

        markers_msg = MarkerArray()

        track_ids_to_delete = set(self.track_ids) - set(track_ids)
        self.track_ids = track_ids

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

            pose = Pose()
            pose.position = point
            pose.orientation = orientation
            poses_msg.poses.append(pose)

            marker = Marker()
            marker.header = depth_msg.header
            marker.ns = self.get_namespace()
            marker.id = track_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = bounding_box.extent[0]
            marker.scale.y = bounding_box.extent[1]
            marker.scale.z = bounding_box.extent[2]
            marker.color.r = 1.0
            marker.color.a = 0.5
            markers_msg.markers.append(marker)

        # Delete markers
        for track_id in track_ids_to_delete:
            marker = Marker()
            marker.header = depth_msg.header
            marker.ns = self.get_namespace()
            marker.id = track_id
            marker.action = Marker.DELETE
            markers_msg.markers.append(marker)

        self.poses_publisher.publish(poses_msg)
        self.markers_publisher.publish(markers_msg)

        # Plot results image
        plot = results.plot()
        image_msg = cv_bridge.cv2_to_imgmsg(
            plot, encoding=color_msg.encoding, header=color_msg.header
        )
        self.image_publisher.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetection()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
