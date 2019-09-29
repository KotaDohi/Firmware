#!/usr/bin/env python
# ROS python API
import rospy
import numpy as np
import set_point

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
from gazebo_msgs.msg import LinkStates
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

class Controller:
    def __init__(self):
        #Drone state
        self.state = State()
        self.uav_pos = Point(0.0, 0.0, 3.0)
        self.load_pos = Point(0.0, 0.0, 0.0)
        self.rel_pos = Point(0.0, 0.0, 3.0)
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        #http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        self.sp.type_mask = int('010000111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        self.sp.acceleration_or_force.x = 0
        self.sp.acceleration_or_force.y = 0
        self.sp.position.z = 0

        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 3.0

    def posLoad(self,msg):
        self.load_pos.x = msg.pose[28].position.x
        self.load_pos.y = msg.pose[28].position.y
        self.load_pos.z = msg.pose[28].position.z

        self.uav_pos.x = msg.pose[1].position.x
        self.uav_pos.y = msg.pose[1].position.y
        self.uav_pos.z = msg.pose[1].position.z

    def posUAV(self,msg):
        self.uav_pos.x = msg.pose.position.x
        self.uav_pos.y = msg.pose.position.y
        self.uav_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def updateSp(self):
        self.sp.position.x = 0
        self.sp.position.y = 0
        self.sp.position.z = 5.0

    def commander(self):
        self.rel_pos.x = self.uav_pos.x - self.load_pos.x
        self.rel_pos.y = self.uav_pos.y - self.load_pos.y
        self.rel_pos.z = self.uav_pos.z - self.load_pos.z
        gain = 0.1
        self.sp.velocity.x = -self.rel_pos.y*gain
        self.sp.velocity.y = self.rel_pos.x*gain
        self.sp.position.z = 5.0

def main():
    # initiate node
    rospy.init_node('odometry_node', anonymous=True)

    # flight mode object
    modes = set_point.fcuModes()

    # flight Controller
    cnt = Controller()
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to UAV Position
    #rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posUAV)

    # Subscribe to payload Position
    rospy.Subscriber('/gazebo/link_states', LinkStates, cnt.posLoad)

    # Setpoint publisher
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)


    print(cnt.state.armed)
    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    print('armed')

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k = 0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    cnt.sp.type_mask = int('010111111000', 2)
    while cnt.uav_pos.z<5:
        print("uav_pos_z:",cnt.uav_pos.z)
        print("load_pos_z:",cnt.load_pos.z)
        cnt.updateSp()
    	sp_pub.publish(cnt.sp)
    print("reached")


    # ROS main loop
    cnt.sp.type_mask = int('010111100011', 2)
    while not rospy.is_shutdown():
    	cnt.commander()
        sp_pub.publish(cnt.sp)
        if cnt.uav_pos.z>1:
            print("uav_x",nt.uav_pos.x)
            print("uav_y",cnt.uav_pos.y)
            print("load_x",cnt.load_pos.x)
            print("load_y",cnt.load_pos.y)
            print("diff_x:",cnt.rel_pos.x)
            print("diff_y:",cnt.rel_pos.y)
            print("velocity_x:",cnt.sp.velocity.x)
            print("velocity_y:",cnt.sp.velocity.y)
            print("velocity_z:",cnt.sp.velocity.z)
            print("force_x:",cnt.sp.acceleration_or_force.x)
            print("force_y:",cnt.sp.acceleration_or_force.y)
            print("force_z:",cnt.sp.acceleration_or_force.z)
        print("----------------")
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
