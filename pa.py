#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from turtlesim.msg import Pose

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower')
        self.cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.path_sub = rospy.Subscriber('/svg_path', Path, self.path_callback)
        
        self.current_pose = None
        self.path = None
        self.current_goal_idx = 0
        self.goal_tolerance = 0.1
        self.boundary_tolerance = 0.5  # 距离边界的最小距离
        self.max_linear_speed = 12.0  # 最大线速度
        self.max_angular_speed = 90.0  # 最大角速度
        
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def path_callback(self, msg):
        self.path = msg
        self.current_goal_idx = 0
        
    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
        
    def is_near_boundary(self, pose):
        return (pose.x < self.boundary_tolerance or
                pose.x > 11.0 - self.boundary_tolerance or
                pose.y < self.boundary_tolerance or
                pose.y > 11.0 - self.boundary_tolerance)
        
    def run(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if not all([self.current_pose, self.path]):
                rate.sleep()
                continue
                
            if self.current_goal_idx >= len(self.path.poses):
                self.cmd_pub.publish(Twist())
                return
                
            if self.is_near_boundary(self.current_pose):
                self.cmd_pub.publish(Twist())  # 停止运动
                rospy.logwarn("Turtle is near the boundary, stopping to avoid collision.")
                rate.sleep()
                continue
                
            goal = self.path.poses[self.current_goal_idx].pose
            dist = self.get_distance(
                self.current_pose.x, 
                self.current_pose.y,
                goal.position.x,
                goal.position.y
            )
            
            if dist < self.goal_tolerance:
                self.current_goal_idx += 1
                continue
                
            target_angle = math.atan2(
                goal.position.y - self.current_pose.y,
                goal.position.x - self.current_pose.x
            )
            
            angle_diff = target_angle - self.current_pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            cmd = Twist()
            if abs(angle_diff) > 0.1:
                cmd.linear.x = 0.0
                cmd.angular.z = self.max_angular_speed * angle_diff
            else:
                cmd.linear.x = min(self.max_linear_speed, 0.5 * dist)
                cmd.angular.z = self.max_angular_speed * angle_diff
            
            self.cmd_pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        follower = PathFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass