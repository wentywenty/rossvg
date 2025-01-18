#!/usr/bin/env python

import rospy
from svgpathtools import svg2paths
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
import numpy as np
import xml.etree.ElementTree as ET
import tf2_ros
import tf_conversions

class SVGParser:
    def __init__(self):
        rospy.init_node('svg_parser')
        self.path_pub = rospy.Publisher('/svg_path', Path, queue_size=1)
        self.pose_sub = rospy.Subscriber('/turtle1/pose_converted', Pose, self.pose_callback)
        self.svg_file = rospy.get_param('~svg_file', '3.svg')
        self.sample_rate = rospy.get_param('~sample_rate', 100)
        self.boundary_tolerance = 0.1  # 边界容忍度
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        self.all_points = []
        self.current_pose = None
        self.goal_tolerance = 0.1  # 目标容忍度
        self.turtle_boundary = 4.0  # 乌龟的活动范围
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # 发布静态变换：map -> odom
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = "map"
        static_transform_stamped.child_frame_id = "odom"
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_transform_stamped)
        
    def is_near_boundary(self, point, width, height):
        return (point.real < self.boundary_tolerance or
                point.real > width - self.boundary_tolerance or
                point.imag < self.boundary_tolerance or
                point.imag > height - self.boundary_tolerance)
        
    def parse_svg(self):
        paths, _ = svg2paths(self.svg_file)
        
        tree = ET.parse(self.svg_file)
        root = tree.getroot()
        width = float(root.attrib['width'])
        height = float(root.attrib['height'])
        
        min_x, min_y = float('inf'), float('inf')
        max_x, max_y = float('-inf'), float('-inf')
        
        # 找到最左上角和最右下角的点
        for path in paths:
            for segment in path:
                points = np.linspace(0, 1, self.sample_rate)
                for t in points:
                    point = segment.point(t)
                    if not self.is_near_boundary(point, width, height):
                        min_x = min(min_x, float(point.real))
                        min_y = min(min_y, float(point.imag))
                        max_x = max(max_x, float(point.real))
                        max_y = max(max_y, float(point.imag))
        
        # 计算缩放比例
        scale_x = (self.turtle_boundary - 2 * self.boundary_tolerance) / (max_x - min_x)
        scale_y = (self.turtle_boundary - 2 * self.boundary_tolerance) / (max_y - min_y)
        scale = min(scale_x, scale_y)
        
        # 平移和缩放所有点，使最左上角的点成为原点，并进行一笔画连线
        for path in paths:
            for segment in path:
                points = np.linspace(0, 1, self.sample_rate)
                for t in points:
                    point = segment.point(t)
                    if not self.is_near_boundary(point, width, height):
                        pose = PoseStamped()
                        pose.header.frame_id = "map"
                        pose.pose.position.x = (float(point.real) - min_x) * scale + self.boundary_tolerance
                        pose.pose.position.y = (float(point.imag) - min_y) * scale + self.boundary_tolerance
                        pose.pose.orientation.w = 1.0
                        self.path_msg.poses.append(pose)
                        self.all_points.append(((float(point.real) - min_x) * scale + self.boundary_tolerance, (float(point.imag) - min_y) * scale + self.boundary_tolerance))
        
        # 平移路径到乌龟的初始位置
        if self.path_msg.poses:
            initial_pose = self.path_msg.poses[0].pose
            dx = initial_pose.position.x - 5.544445
            dy = initial_pose.position.y - 5.544445
            for pose in self.path_msg.poses:
                pose.pose.position.x -= dx
                pose.pose.position.y -= dy
        
        rospy.loginfo(f"Parsed {len(self.path_msg.poses)} poses from SVG.")
        
    def pose_callback(self, msg):
        self.current_pose = msg
        self.update_path()
        self.publish_tf()
        
    def update_path(self):
        if self.current_pose is None or not self.path_msg.poses:
            return
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 移除已经走过的路径点
        while self.path_msg.poses and self.get_distance(current_x, current_y, self.path_msg.poses[0].pose.position.x, self.path_msg.poses[0].pose.position.y) < self.goal_tolerance:
            self.path_msg.poses.pop(0)
        
        self.path_pub.publish(self.path_msg)
        
    def get_distance(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
    def publish_tf(self):
        if self.current_pose is None:
            return
        
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "turtle1"
        t.transform.translation.x = self.current_pose.position.x
        t.transform.translation.y = self.current_pose.position.y
        t.transform.translation.z = 0.0
        q = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        )
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
        
    def run(self):
        self.parse_svg()
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.path_msg.poses:
                self.path_pub.publish(self.path_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        parser = SVGParser()
        parser.run()
    except rospy.ROSInterruptException:
        pass