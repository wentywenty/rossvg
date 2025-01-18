#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Path

class PathVisualizer:
    def __init__(self):
        rospy.init_node('path_visualizer')
        self.path_sub = rospy.Subscriber('/svg_path', Path, self.path_callback)
        self.points = []

    def path_callback(self, msg):
        self.points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.visualize_path()

    def visualize_path(self):
        if not self.points:
            return
        x, y = zip(*self.points)
        plt.figure()
        plt.plot(x, y, 'b-', label='Path')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('SVG Path Visualization')
        plt.legend()
        plt.grid()
        plt.show()

if __name__ == '__main__':
    try:
        visualizer = PathVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass