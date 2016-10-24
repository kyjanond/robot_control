#!/usr/bin/env python
import sys,os, yaml
import rospy
import numpy as np
import tf, math
from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import PoseStamped,Point,Quaternion
import enum
from dice_recognition.srv import SetTarget
from std_srvs.srv import Empty

class RobMovement(object):
    def __init__(self,approximation,*args):
        self.points = args
        self.position = 0
        self.is_running = False
        self.aprox = approximation

PROXIMITY_TRNSL = 150.0
PROXIMITY_ROT = 10.0

CUP_GRAB_POS = (
    (283.8,-245.9,90.2),
    (0.686368,0.350942,0.578221,-0.267206)
)

CUP_GRAB_PREPOS = (
    (283.8,-245.9,200.0),
    (0.0,1.0,0.0,0.0)
)

THROW_PREPOS = (
    (492.0,47.7,200.0),
    (0.537945,0.54738,0.473214,-0.432502)
)

THROW_POS = (
    (475.0,47.7,175.0),
    (-0.291946,0.709684,0.583431,0.265939)
)

CUP_POS = (
    (450,-30,125),
    (0.0,1.0,0.0,0.0)
)

OBSERVER_POS = (
    (460,340,200),
    (0.0,1.0,0.0,0.0)
)

@enum.unique
class RobotState(enum.Enum):
    idle = 0
    starting = 1
    picking = 2
    predrop = 3
    droping = 4
    returning = 5


class RobotControler(object):
    def __init__(self):
        rospy.init_node('RobotControler', anonymous=False)
        self.li = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.is_picking = False
        self.move_pub = rospy.Publisher("/iiwa/command/CartesianPose",PoseStamped,queue_size=10)
        self.gripper_pub = rospy.Publisher("/iiwa/command/OpenGripper",Bool,queue_size=10)
        self.status_pub = rospy.Publisher("status",Int8,queue_size=10)
        self.move_srv = rospy.Service("run_pick",SetTarget,self.handlePick)
        self.move_srv = rospy.Service("run_home",Empty,self.handleHome)
        self.state = RobotState.idle
        self.target_dice = -1
        self.target = OBSERVER_POS
        self.starting_pos = None
        self.is_moving = False
        self.approximation = False
    
    def handleHome(self,req):
        self.target = OBSERVER_POS
        self.publishTarget(self.target[0],self.target[1])
        return []


    def calcTarget(self):
        t = "dice {}".format(self.target_dice)
        now = rospy.Time.now()
        self.li.waitForTransform("world",t,now,rospy.Duration(2.0))
        trans,rot = self.li.lookupTransform("world",t,now)
        self.target = (trans,rot) 
    
    def publishTarget(self,trans,rot):
        msg = PoseStamped()
        msg.pose.position = Point(*tuple([x*0.001 for x in trans]))
        msg.pose.orientation = Quaternion(*rot)
        self.move_pub.publish(msg)
    
    def publishGripper(self,open_gripper):
        msg = Bool()
        msg.data = open_gripper
        self.gripper_pub.publish(msg)
    
    def checkPosition(self):
        now = rospy.Time.now()
        self.br.sendTransform(
                    self.target[0],
                    self.target[1],
                    now,
                    "pick_target",
                    "world"
                )
        self.li.waitForTransform("tcp","pick_target",now,rospy.Duration(2.0))
        trnsl,rot = self.li.lookupTransform("tcp","pick_target",rospy.Time(0))
        rot_mag = np.linalg.norm(rot)
        trnsl_mag = np.linalg.norm(trnsl) 
        if trnsl_mag <= 1.0 and rot_mag <= 1.0:
            self.is_moving = False
        elif self.approximation and trnsl_mag <= PROXIMITY_TRNSL and rot_mag <= PROXIMITY_ROT:
            self.is_moving = False
        else:
            self.is_moving = True


    def handlePick(self,req):
        if self.is_picking:
            return [False]
        try:
            self.target_dice = req.dice
            self.is_picking = True
            self.state = RobotState.starting
        except Exception as e:
            print e
            return e
    
        return [True]

    def checkState(self):
        if self.state == RobotState.starting and self.is_picking:
            try:
                self.calcTarget()
            except Exception as e:
                print "Error: {}".format(e)
                self.is_picking = False
                return
            self.starting_pos = self.li.lookupTransform("tcp","world",rospy.Time(0))
            self.publishGripper(True)
            self.publishTarget(self.target[0],self.target[1])
            self.approximation = False
            self.state = RobotState.picking

        elif self.state == RobotState.picking:
            self.publishGripper(False)
            rospy.sleep(1)
            self.target = OBSERVER_POS
            self.publishTarget(self.target[0],self.target[1])
            self.approximation = True
            self.state = RobotState.predrop

        elif self.state == RobotState.predrop:
            self.target = CUP_POS
            self.publishTarget(self.target[0],self.target[1])
            self.approximation = False
            self.state = RobotState.droping

        elif self.state == RobotState.droping:
            self.publishGripper(True)
            rospy.sleep(1)
            self.publishGripper(False)
            self.target = OBSERVER_POS
            self.publishTarget(self.target[0],self.target[1])
            self.approximation = False
            self.state = RobotState.returning

        elif self.state == RobotState.returning:
            self.is_picking = False
            self.approximation = False
            self.state = RobotState.idle
        
        self.status_pub.publish(self.state.value)



    def run(self):
        rate = rospy.Rate(25)
        while not rospy.is_shutdown():
            if self.is_picking:
                if not self.is_moving:
                    self.checkState()
                self.checkPosition()
            rate.sleep()
        self.shutdown()

    def shutdown(self):
        rospy.loginfo("Shutting down...")

def main(args):
    rc = RobotControler()
    try: 
        rc.run()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)