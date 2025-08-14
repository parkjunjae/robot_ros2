#!/usr/bin/env python3
# reframe_cloud_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class ReframeCloudNode(Node):
    def __init__(self):
        super().__init__('reframe_cloud_node')

        # 파라미터(런치/YAML에서 덮어쓰기 가능)
        self.declare_parameter('input_topic', '/cloud_registered')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('output_topic', '/cloud_registered_odom')
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('prefer_latest_tf', True)      # ★ 기본값: 항상 최신 TF 사용
        self.declare_parameter('future_slop_sec', 0.1)        # msg가 now보다 이만큼 미래면 최신 TF 강제

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.tf_timeout = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))
        self.prefer_latest = bool(self.get_parameter('prefer_latest_tf').value)
        self.future_slop = float(self.get_parameter('future_slop_sec').value)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # I/O
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos_profile_sensor_data)

        self.get_logger().info(f'[{self.get_name()}] {self.input_topic} -> {self.output_topic}, target_frame={self.target_frame}, prefer_latest_tf={self.prefer_latest}')

        self._warn_count = 0  # 과도한 로그 방지

    def cb(self, msg: PointCloud2):
        # 1) XYZ만 추출해 dtype 문제 회피
        try:
            pts_xyz = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=False))
            xyz_cloud = pc2.create_cloud_xyz32(msg.header, pts_xyz)
        except Exception as ex:
            self.get_logger().error(f'build XYZ cloud failed: {ex}')
            return

        # 2) 사용할 TF 시각 결정
        request_time = Time.from_msg(xyz_cloud.header.stamp)
        now_time = self.get_clock().now()

        use_latest = self.prefer_latest
        if not use_latest:
            # msg가 now보다 future_slop 이상 "미래"면 최신으로 강제
            if (request_time - now_time) > Duration(seconds=self.future_slop):
                use_latest = True

        # 3) TF 조회 (메시지 시각 또는 최신)
        try:
            if use_latest:
                tf = self.tf_buffer.lookup_transform(self.target_frame,
                                                     xyz_cloud.header.frame_id,
                                                     Time(),  # latest
                                                     self.tf_timeout)
            else:
                tf = self.tf_buffer.lookup_transform(self.target_frame,
                                                     xyz_cloud.header.frame_id,
                                                     request_time,
                                                     self.tf_timeout)
        except TransformException as ex:
            # 메시지 시각 실패 시 최신으로 한 번 더 시도
            if not use_latest:
                try:
                    tf = self.tf_buffer.lookup_transform(self.target_frame,
                                                         xyz_cloud.header.frame_id,
                                                         Time(),
                                                         self.tf_timeout)
                    if self._warn_count % 50 == 0:
                        self.get_logger().warn(f'TF at msg time unavailable ({ex}); using latest TF.')
                    self._warn_count += 1
                except TransformException as ex2:
                    if self._warn_count % 50 == 0:
                        self.get_logger().warn(f'TF latest lookup also failed: {ex2}')
                    self._warn_count += 1
                    return
            else:
                if self._warn_count % 50 == 0:
                    self.get_logger().warn(f'TF latest lookup failed: {ex}')
                self._warn_count += 1
                return

        # 4) 변환 + TF 시각으로 재스탬프
        try:
            out = do_transform_cloud(xyz_cloud, tf)
            out.header.frame_id = self.target_frame
            out.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(out)
        except Exception as ex:
            self.get_logger().error(f'cloud transform failed: {ex}')

def main():
    rclpy.init()
    node = ReframeCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
