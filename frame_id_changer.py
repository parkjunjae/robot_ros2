#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# reframe_cloud_node.py
#
# - 입력 PointCloud2를 target_frame으로 변환해서 다시 퍼블리시
# - "latest TF"를 사용해 변환했다면, 출력 메시지의 stamp도 latest 시각으로 맞춰서
#   costmap 등 소비자 쪽에서 시간 외삽으로 드랍하지 않도록 함.
#

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

        # -----------------------------
        # Parameters (launch/YAML에서 override 가능)
        # -----------------------------
        self.declare_parameter('input_topic', '/cloud_registered_body')
        self.declare_parameter('target_frame', 'base_link')   # ← 기본값: base_link 권장
        self.declare_parameter('output_topic', '/cloud_registered_base')
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('prefer_latest_tf', True)      # ← 기본값 True 로 변경
        self.declare_parameter('future_slop_sec', 0.05)       # msg가 now보다 이만큼 미래면 latest 강제
        # (옵션) 포인트 필터 파라미터
        self.declare_parameter('z_min', -0.10)
        self.declare_parameter('z_max',  1.20)
        self.declare_parameter('radius_max', 6.0)

        # Read params
        self.input_topic   = self.get_parameter('input_topic').get_parameter_value().string_value
        self.target_frame  = self.get_parameter('target_frame').get_parameter_value().string_value
        self.output_topic  = self.get_parameter('output_topic').get_parameter_value().string_value
        self.tf_timeout    = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))
        self.prefer_latest = bool(self.get_parameter('prefer_latest_tf').value)
        self.future_slop   = float(self.get_parameter('future_slop_sec').value)
        self.z_min         = float(self.get_parameter('z_min').value)
        self.z_max         = float(self.get_parameter('z_max').value)
        self.radius_max2   = float(self.get_parameter('radius_max').value)**2

        # TF buffer/listener
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # I/O
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f'[{self.get_name()}] {self.input_topic} -> {self.output_topic}, '
            f'target_frame={self.target_frame}, prefer_latest_tf={self.prefer_latest}'
        )
        self._warn_count = 0  # 과도한 경고 억제용

    def cb(self, msg: PointCloud2):
        # 1) XYZ만 추출(필요시 필터)
        try:
            pts = []
            for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                if self.z_min <= z <= self.z_max and (x*x + y*y) <= self.radius_max2:
                    pts.append((x, y, z))
            if not pts:
                return  # 빈 메시지면 조용히 리턴
            xyz_cloud = pc2.create_cloud_xyz32(msg.header, pts)
        except Exception as ex:
            self.get_logger().error(f'build XYZ cloud failed: {ex}')
            return

        # 2) 사용할 TF 시각 결정
        request_time = Time.from_msg(xyz_cloud.header.stamp)
        now_time = self.get_clock().now()

        use_latest = self.prefer_latest
        # msg가 now보다 future_slop 이상 미래면 latest 사용 강제
        if not use_latest and (request_time - now_time) > Duration(seconds=self.future_slop):
            use_latest = True

        # 3) TF 조회 (메시지 시각 또는 최신)
        used_latest = False
        try:
            if use_latest:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    xyz_cloud.header.frame_id,
                    rclpy.time.Time(),              # latest
                    self.tf_timeout
                )
                used_latest = True
            else:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    xyz_cloud.header.frame_id,
                    request_time,                    # at message time
                    self.tf_timeout
                )
        except TransformException as ex:
            # 메시지 시각 조회 실패 시 latest로 한 번 더 시도
            if not use_latest:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        xyz_cloud.header.frame_id,
                        rclpy.time.Time(),
                        self.tf_timeout
                    )
                    used_latest = True
                    if (self._warn_count % 50) == 0:
                        self.get_logger().warning(f'TF at msg time unavailable ({ex}); using latest TF.')
                    self._warn_count += 1
                except TransformException as ex2:
                    if (self._warn_count % 50) == 0:
                        self.get_logger().warning(f'TF latest lookup also failed: {ex2}')
                    self._warn_count += 1
                    return
            else:
                if (self._warn_count % 50) == 0:
                    self.get_logger().warning(f'TF latest lookup failed: {ex}')
                self._warn_count += 1
                return

        # 4) 변환 + 스탬프/프레임 정합
        try:
            out = do_transform_cloud(xyz_cloud, tf)
            out.header.frame_id = self.target_frame
            # ★ 핵심: latest TF를 썼다면 출력 메시지 stamp도 latest로 맞춘다.
            if used_latest:
                # TF가 가진 정확한 최신 시각으로 맞춘다.
                out.header.stamp = tf.header.stamp
                # 대안) now 사용: out.header.stamp = self.get_clock().now().to_msg()
            else:
                # 요청 시각으로 변환했으니 원래 시각 유지
                out.header.stamp = request_time.to_msg()

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
