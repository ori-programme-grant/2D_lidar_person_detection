import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker

from dr_spaam.detector import Detector

class DrSpaamROS(Node):
    def __init__(self):
        super().__init__('dr_spaam_ros')
        self._read_params()
        self._detector = Detector(
            self.weight_file,
            model=self.detector_model,
            gpu=True,
            stride=self.stride,
            panoramic_scan=self.panoramic_scan,
        )
        self._init()

    def _read_params(self):
        self.declare_parameter('weight_file', '')
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('stride', 1)
        self.declare_parameter('detector_model', 'default')
        self.declare_parameter('panoramic_scan', False)
        self.declare_parameter('scan_topic', 'scan')

        self.weight_file = self.get_parameter('weight_file').value
        self.conf_thresh = self.get_parameter('conf_thresh').value
        self.stride = self.get_parameter('stride').value
        self.detector_model = self.get_parameter('detector_model').value
        self.panoramic_scan = self.get_parameter('panoramic_scan').value
        self.scan_topic = self.get_parameter('scan_topic').value

    def _init(self):
        self._dets_pub = self.create_publisher(PoseArray, 'people_detections', 10)
        self._rviz_pub = self.create_publisher(Marker, 'rviz', 10)
        self._scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, 10
        )

    def _scan_callback(self, msg):
        if self._dets_pub.get_subscription_count() == 0 and self._rviz_pub.get_subscription_count() == 0:
            return

        if not self._detector.is_ready():
            self._detector.set_laser_fov(
                np.rad2deg(msg.angle_increment * len(msg.ranges))
            )

        scan = np.array(msg.ranges)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        dets_xy, dets_cls, _ = self._detector(scan)
        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        dets_msg = detections_to_pose_array(dets_xy, dets_cls)
        dets_msg.header = msg.header
        self._dets_pub.publish(dets_msg)

        rviz_msg = detections_to_rviz_marker(dets_xy, dets_cls)
        rviz_msg.header = msg.header
        self._rviz_pub.publish(rviz_msg)


def detections_to_rviz_marker(dets_xy, dets_cls):
    msg = Marker()
    msg.action = Marker.ADD
    msg.ns = 'dr_spaam_ros'
    msg.id = 0
    msg.type = Marker.LINE_LIST
    msg.pose.orientation.w = 1.0
    msg.scale.x = 0.03
    msg.color.r = 1.0
    msg.color.a = 1.0
    
    r = 0.4
    ang = np.linspace(0, 2 * np.pi, 20)
    xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

    for d_xy, _ in zip(dets_xy, dets_cls):
        for i in range(len(xy_offsets) - 1):
            p0 = Point(x=d_xy[0] + xy_offsets[i, 0], y=d_xy[1] + xy_offsets[i, 1], z=0.0)
            p1 = Point(x=d_xy[0] + xy_offsets[i + 1, 0], y=d_xy[1] + xy_offsets[i + 1, 1], z=0.0)
            msg.points.extend([p0, p1])
    
    return msg


def detections_to_pose_array(dets_xy, dets_cls):
    pose_array = PoseArray()
    for d_xy, _ in zip(dets_xy, dets_cls):
        p = Pose()
        p.position.x = float(d_xy[0])
        p.position.y = float(d_xy[1])
        p.position.z = 0.0
        pose_array.poses.append(p)
    
    return pose_array


def main(args=None):
    rclpy.init(args=args)
    node = DrSpaamROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
