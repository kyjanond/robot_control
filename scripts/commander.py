#!/usr/bin/env python
import sys, os, yaml
import rospy
import numpy as np
import tf, math
import enum
from std_msgs.msg import Bool, Int8MultiArray, Int8, String
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray

#services
from std_srvs.srv import Empty
from iiwa_msgs.srv import ConfigureSmartServo, ConfigureSmartServoRequest
from robot_control.msg import Motion, MotionArray

PROXIMITY_TRNSL = 15.0
PROXIMITY_ROT = 5.0

POSITIONS = {
    "home_pos":(
        (500,0,500),
        (0.0,1.0,0.0,0.0)
    ),
    "pos_a":(
        (400,-400,500),
        (0.0,1.0,0.0,0.0)
    ),
    "pos_b":(
        (400,400,500),
        (0.0,1.0,0.0,0.0)
    ),
}

@enum.unique
class RobotState(enum.Enum):
    idle = 0
    moving = 1

class RobotCommander(object):
    
    def __init__(self):
        rospy.init_node('robot_commander', anonymous=False)
        self.li = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.move_pub = rospy.Publisher("/iiwa/command/CartesianPose",PoseStamped,queue_size=10)
        #self.gripper_pub = rospy.Publisher("/iiwa/command/OpenGripper",Bool,queue_size=10)
        self.state_pub = rospy.Publisher("~state",Int8MultiArray,queue_size=10)

        sub_ns = rospy.get_param("~bridge_ns","")
        if sub_ns:
            sub_ns = "/{}/".format(sub_ns)
        self.teach_sub = rospy.Subscriber(sub_ns+"TeachPoseArray",MotionArray,self.teach_callback)
        self.predefined_sub = rospy.Subscriber(sub_ns+"PredefinedMotion",String,self.predefined_motion_callback)
        self.cmd_sub = rospy.Subscriber(sub_ns+"Command",String,self.cmd_callback)
        
        self.home_srv = rospy.Service(sub_ns+"RunHome",Empty,self.handle_home)
        self.approx_srv = rospy.Service(sub_ns+"SetApprox",Empty,self.handle_home)
        self.impedance_srv = rospy.Service(sub_ns+"SetImpedance",Empty,self.handle_set_impedance)

        self.state = RobotState.idle
        self.pos_index = 0
        
        self.is_moving = False
        self.approx = (5,1)
        self.pos_enumerate = enumerate([])
        self.target = [(),()]
        self.motions = {
            "line":[
                [False,0,POSITIONS["home_pos"],(PROXIMITY_TRNSL,PROXIMITY_ROT)],
                [False,0,POSITIONS["pos_a"],(PROXIMITY_TRNSL,PROXIMITY_ROT)],
                [False,0,POSITIONS["pos_b"],(PROXIMITY_TRNSL,PROXIMITY_ROT)],
                [False,0,POSITIONS["home_pos"],(PROXIMITY_TRNSL,PROXIMITY_ROT)]
            ]
        }
    
    def pub_target(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position = Point(*tuple([x*0.001 for x in self.target[0]]))
        msg.pose.orientation = Quaternion(*self.target[1])
        self.move_pub.publish(msg)

    def check_pos(self):
        now = rospy.Time.now()
        self.br.sendTransform(
                    self.target[0],
                    self.target[1],
                    now,
                    "target",
                    "world"
                )
        try:
            self.li.waitForTransform("tcp","target",now,rospy.Duration(2.0))
        except Exception as e:
            print("Comm error:{}".format(e))
            print("Line no. {}".format(sys.exc_info()[2].tb_lineno))
            rospy.signal_shutdown("probably a comm error")
            return

        trnsl,rot = self.li.lookupTransform("tcp","target",now)
        rot_mag = np.linalg.norm(rot)
        trnsl_mag = np.linalg.norm(trnsl) 
        if (trnsl_mag <= self.approx[0] and rot_mag <= self.approx[1]):
            self.is_moving = False
        else:
            self.is_moving = True
    
    def handle_home(self,req):
        try:
            self.target = POSITIONS["home_pos"]
            self.pub_target()
        except Exception as e:
            print "Service call failed: %s"%e
            return e

        return [] 

    def handle_set_impedance(self,req):
        msg = ConfigureSmartServoRequest()
        msg.control_mode = 2 #CARTESIAN IMPEDANCE
        msg.cartesian_impedance.cartesian_stiffness.x = 800
        msg.cartesian_impedance.cartesian_stiffness.y = 800
        msg.cartesian_impedance.cartesian_stiffness.z = 800
        msg.cartesian_impedance.cartesian_stiffness.a = 150
        msg.cartesian_impedance.cartesian_stiffness.b = 150
        msg.cartesian_impedance.cartesian_stiffness.c = 150
        msg.cartesian_impedance.nullspace_stiffness = 7

        msg.cartesian_impedance.cartesian_damping.x = 0.7
        msg.cartesian_impedance.cartesian_damping.y = 0.7
        msg.cartesian_impedance.cartesian_damping.z = 0.7
        msg.cartesian_impedance.cartesian_damping.a = 0.7
        msg.cartesian_impedance.cartesian_damping.b = 0.7
        msg.cartesian_impedance.cartesian_damping.c = 0.7
        msg.cartesian_impedance.nullspace_damping = 1.0
        try:
            rospy.wait_for_service('/iiwa/configuration/configureSmartServo',1)
            configure_servo = rospy.ServiceProxy('/iiwa/configuration/configureSmartServo', ConfigureSmartServo)
            ret = configure_servo(msg)
            if not ret.success:
                print ret
        except Exception as e:
            print "Service call failed: %s"%e
            return e
        return []
    
    def predefined_motion_callback(self,msg):
        try:
            motion = self.motions[msg.data][:]
        except Exception as e:
            print "Error: {}".format(e)
            print("Line no. {}".format(sys.exc_info()[2].tb_lineno))
        print("Received predefined motion: {}".format(msg.data))
        self.build_motion(motion)
    
    def cmd_callback(self,msg):
        print("Received cmd: {}".format(msg))
        if (msg.data == "stop"):
            self.state = RobotState.idle
            self.pos_enumerate = enumerate([])
            self.is_moving = False
    
    def teach_callback(self, msg):
        self.state = RobotState.idle
        self.is_moving = False
        motion = []
        for m in msg.motions:
            pose = (
                (m.pose.position.x,m.pose.position.y,m.pose.position.z),
                (m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w)
                )
            motion.append([m.trigger,m.sleep,pose,(m.pos_aprox,m.ori_aprox)])
        self.build_motion(motion)
    
    def execute_motion(self):
        try:
            self.pos_index,pos = self.pos_enumerate.next()
        except StopIteration:
            self.pos_index = -1
            self.state_pub.publish(data=[self.state.value,self.pos_index])
            self.state = RobotState.idle
            return
        #self.gripper_pub.publish(pos[0])
        if pos[2]:
            self.target = pos[2]
        if pos[1]:
            rospy.sleep(pos[1])
            self.approx = (5,1)
        elif pos[3]:
            self.approx = pos[3]
        
        self.state_pub.publish(data=[self.state.value,self.pos_index])
        self.pub_target()
    
    def build_motion(self,motion):
        try:
            self.pos_enumerate = enumerate(motion)
        except Exception as e:
            print "Error: {}".format(e)
            print("Line no. {}".format(sys.exc_info()[2].tb_lineno))
            self.state = RobotState.idle
            return False
        self.state = RobotState.moving
        return True

    def run(self):
        rate = rospy.Rate(25)
        while not rospy.is_shutdown():
            if not self.state == RobotState.idle:
                if not self.is_moving:
                    self.execute_motion()
                self.check_pos()
            rate.sleep()
        self.shutdown()

    def shutdown(self):
        rospy.loginfo("Shutting down...")

def main(*args,**kwargs):
    rc = RobotCommander()
    try: 
        rc.run()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()