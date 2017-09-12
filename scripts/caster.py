#!/usr/bin/env python
import sys, os
import rospy
import tf
import yaml
import numpy as np
from geometry_msgs.msg import PoseStamped

def load_inverse_transform(yaml_tf):
    tr = [(),()]
    for k,v in yaml_tf.items():
        tr[0] = (
            -v["translation"]["x"],
            -v["translation"]["y"],
            -v["translation"]["z"]
        )
        tr[1] = tf.transformations.quaternion_inverse((
            v["rotation"]["x"],
            v["rotation"]["y"],
            v["rotation"]["z"],
            v["rotation"]["w"]
        ))
    
    print("Loading {}: {}".format(k,tr))
    return tr

class RobotCaster(object):
    def __init__(self):
        rospy.init_node('robot_caster', anonymous=False)
        self.transform = np.eye(4,dtype=float)
        self.br = tf.TransformBroadcaster()
        self.tool_tf = [(0,0,0),(0,0,0,1)]
        self.rob_tf = [(0,0,0),(0,0,0,1)]
        self.load_tool()

        self.image_sub = rospy.Subscriber("/iiwa/state/CartesianPose",PoseStamped,self.callback)
        super(RobotCaster,self).__init__()

    def load_tool(self):
        tool_name = rospy.get_param("~tool_name","")
        if tool_name:
            path = os.path.join(rospy.get_param("~tool_config_path",""),tool_name+".yaml")
            try:
                with open(path) as f:
                    yaml_tf = yaml.safe_load(f)
                self.tool_tf = load_inverse_transform(yaml_tf)
                print("Loaded tool: {}".format(tool_name))
                return
            except Exception as e:
                print("Error loading tool: {}".format(e))
        else:
            print("Tool not specified.")
        print("Loading flange")

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
                "robot_base"
                )

        self.br.sendTransform(
                self.tool_tf[0],
                self.tool_tf[1],
                rospy.Time.now(),
                "flange",
                "tcp"
                )

        self.rob_tf = (vec3,quat)

def main(args):
    rb = RobotCaster()
    try: 
        rb.run()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)