# filepath: /home/flora/github/zuoye/tfff.py
#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose as TurtlesimPose
from geometry_msgs.msg import Pose as GeometryPose
import tf_conversions

class PoseConverter:
    def __init__(self):
        rospy.init_node('pose_converter')
        self.pose_sub = rospy.Subscriber('/turtle1/pose', TurtlesimPose, self.pose_callback)
        self.pose_pub = rospy.Publisher('/turtle1/pose_converted', GeometryPose, queue_size=1)
        
    def pose_callback(self, msg):
        converted_pose = GeometryPose()
        converted_pose.position.x = msg.x
        converted_pose.position.y = msg.y
        converted_pose.position.z = 0.0
        quaternion = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        converted_pose.orientation.x = quaternion[0]
        converted_pose.orientation.y = quaternion[1]
        converted_pose.orientation.z = quaternion[2]
        converted_pose.orientation.w = quaternion[3]
        self.pose_pub.publish(converted_pose)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = PoseConverter()
        converter.run()
    except rospy.ROSInterruptException:
        pass