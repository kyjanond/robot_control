#!/usr/bin/env python
import sys,os, yaml
import rospy
import numpy as np
import tf, math
import minieigen as me
from std_srvs.srv import Empty,SetBool,EmptyRequest
from geometry_msgs.msg import PoseStamped,Point,Quaternion


class robot_controler(object):
    def __init__(self,tool_transform):
        rospy.init_node('robot_controler', anonymous=False)
        self.tool_tr = tool_transform
        self.br = tf.TransformBroadcaster()
        self.li = tf.TransformListener()
        self.is_following = False
        self.follow_frame = (
            (0,100,500),
            tf.transformations.quaternion_from_euler(0,math.pi,-math.pi*0.5)
        )
        self.target_id = None

        self.move_pub = rospy.Publisher("/iiwa/command/CartesianPose",PoseStamped,queue_size=10)
        self.move_srv = rospy.Service("~move",Empty,self.handleMove)
        self.move_srv = rospy.Service("~follow",SetBool,self.handleMove)
    
    def calcTarget(self):
        id = "target {}".format(self.target_id)
        now = rospy.Time.now()
        self.br.sendTransform(
                self.follow_frame[0],
                self.follow_frame[1],
                now,
                id,
                "id {}".format(self.target_id)
                )
        self.li.waitForTransform("/world",id,now,rospy.Duration(2.0))
        (trans,rot) = self.li.lookupTransform("/world",id,now)
        return trans,rot

    def followTarget(self):
        id = "target {}".format(self.target_id)
        now = rospy.Time.now()
        self.br.sendTransform(
                self.follow_frame[0],
                self.follow_frame[1],
                now,
                id,
                "id {}".format(self.target_id)
                )
        (trans,rot) = self.li.lookupTransform("/world",id,rospy.Time(0))
        return trans,rot
    
    def publishTarget(self,trans,rot):
        msg = PoseStamped()
        msg.pose.position = Point(*tuple([x*0.001 for x in trans]))
        msg.pose.orientation = Quaternion(*rot)
        self.move_pub.publish(msg)
    
    def moveToTarget(self):
        try:
            t,r = self.calcTarget()
        except Exception as e:
            print e
            return
        self.publishTarget(t,r)

    def handleMove(self,req):
        self.target_id = rospy.get_param("target_id",default=None)
        try:
            t,r = self.calcTarget()
            self.publishTarget(t,r)
        except Exception as e:
            print e
            return e
        
        print("moving to id {}".format(self.target_id))
        if type(req) is not EmptyRequest:
            self.is_following = req.data
            print("is following: {}".format(self.is_following))
            return [True,""]
        else:
            return []


    def run(self):
        rate = rospy.Rate(25)

        while not rospy.is_shutdown():
            self.br.sendTransform(
                self.tool_tr[0],
                self.tool_tr[1],
                rospy.Time.now(),
                "camera",
                "flange"
                )
            if self.is_following:
                try:
                    t,r = self.followTarget()
                    self.publishTarget(t,r)
                except Exception as e:
                    print e
            rate.sleep()
        self.shutdown()

    def shutdown(self):
        rospy.loginfo("Shutting down...")

def loadTransform(yaml_tr):
    tr = [(),()]
    for k,v in yaml_tr.items():
        tr[0] = (
            v["translation"]["x"],
            v["translation"]["y"],
            v["translation"]["z"]
        )
        tr[1] = (
            v["rotation"]["x"],
            v["rotation"]["y"],
            v["rotation"]["z"],
            v["rotation"]["w"]
        )
    print("Loading {}: {}".format(k,tr))
    return tr



def main(args):
    if len(args) > 1:
        path = os.path.join("/home/ok/robot_info/tool_cfg",args[1]+".yaml")
        try:
            with open(path) as f:
                yaml_tr = yaml.safe_load(f)
            tool_tr = loadTransform(yaml_tr)
        except Exception as e:
            print("Error loading tool: {}".format(e))
            print("Loading flange")
            tool_tr = [(0,0,0),(0,0,0,1)]
    else:
        print("Tool not specified.")
        print("Loading flange")
        tool_tr = [(0,0,0),(0,0,0,1)]
    rc = robot_controler(tool_tr)
    try: 
        rc.run()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)