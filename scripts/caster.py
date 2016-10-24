#!/usr/bin/env python
import sys
import rospy
import cv2
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped,Transform,Quaternion,Vector3
from std_srvs.srv import Empty


class robot_caster(object):
    def __init__(self):
        rospy.init_node('robot_caster', anonymous=False)
        self.transform = np.eye(4,dtype=float)
        self.br = tf.TransformBroadcaster()
        self.rob_tf = ((0,0,0),(0,0,0,1))

        self.image_sub = rospy.Subscriber("/iiwa/state/CartesianPose",PoseStamped,self.callback)

    def run(self):
        rospy.spin()

    def callback(self,data):
        vec3 = (
            data.pose.position.x*1000,
            data.pose.position.y*1000,
            data.pose.position.z*1000
        )
        quat = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        )

        self.br.sendTransform(
                vec3,
                quat,
                rospy.Time.now(),
                "tcp",
                "world"
                )

        self.rob_tf = (vec3,quat)

def main(args):
    rb = robot_caster()
    try: 
        rb.run()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)