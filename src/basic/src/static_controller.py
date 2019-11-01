#!/usr/bin/env python
# ROS python API
import rospy
import numpy as np
import set_point
import time
import scipy.linalg
import control

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
from gazebo_msgs.msg import LinkStates
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from tf.transformations import euler_from_quaternion

class Controller:
    def __init__(self):
        #Drone state
        self.state = State()
        self.uav_pos = Point(0.0, 0.0, 3.0)
        self.load_pos = Point(0.0, 0.0, 0.0)
        self.uav_vel = Point(0.0, 0.0, 0.0)
        self.load_vel = Point(0.0, 0.0, 0.0)
        self.uav_att = Point(0.0, 0.0, 0.0)
        # Instantiate a attitude setpoints message
        self.sp = AttitudeTarget()
        # set the flag to use body_rates and thrust
        #http://docs.ros.org/api/mavros_msgs/html/msg/AttitudeTarget.html
        self.sp.type_mask = int('10000000', 2)

        self.sp.body_rate.x = 0
        self.sp.body_rate.y = 0
        self.sp.body_rate.z = 0
        self.sp.thrust = 0

        # Desired rotational rate for UAV(rad/s)
        self.omega = np.pi

        # Instantiate a position setpoints message
        self.pos_sp = PositionTarget()
        self.pos_sp.type_mask = int('010111111000', 2)
        self.pos_sp.coordinate_frame = 1
        self.pos_sp.position.x = 0
        self.pos_sp.position.y = 0
        self.pos_sp.position.z = 3

        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a squareself.omegas[0],self.omegas[1],self.omegas[2],float(self.u[2]) fence for now
        self.FENCE_LIMIT = 5.0

        # We will fly at a fixed altitude for now
        self.ALT_SP = 3.0

        # parameers of the system
        self.l = 4.01 #length of the tether
        self.r = 3.0 #radius of the UAV circle
        self.p0 = 0.8 #radius of the load circle
        self.g = 9.80665 #gravity

    def init_position(self):
        self.pos_sp.position.x = 2.0
        self.pos_sp.position.y = 0
        self.pos_sp.position.z = 5.0

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def get_position(self,msg):
        self.load_pos.x = msg.pose[9].position.x - msg.pose[1].position.x
        self.load_pos.y = msg.pose[9].position.y - msg.pose[1].position.y
        self.load_pos.z = msg.pose[9].position.z

        self.uav_pos.x = msg.pose[1].position.x
        self.uav_pos.y = msg.pose[1].position.y
        self.uav_pos.z = msg.pose[1].position.z

        self.load_vel.x = msg.twist[9].linear.x - msg.twist[1].linear.x
        self.load_vel.y = msg.twist[9].linear.y - msg.twist[1].linear.y
        self.load_vel.z = msg.twist[9].linear.z

        self.uav_vel.x = msg.twist[1].linear.x
        self.uav_vel.y = msg.twist[1].linear.y
        self.uav_vel.z = msg.twist[1].linear.z

        orientation_q = msg.pose[1].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.uav_att.x = roll
        self.uav_att.y = pitch
        self.uav_att.z = yaw

    def lqr_xr(self):
        x = np.matrix([self.load_pos.x, self.load_vel.x, self.uav_pos.x, self.uav_vel.x, self.uav_att.y]).T
        A = np.array([[0, 1, 0, 0, 0],
                      [-self.g/self.l, 0, 0, 0, -self.g],
                      [0, 0, 0, 1, 0],
                      [0, 0, 0, 0, self.g],
                      [0, 0, 0, 0, 0]])
        B = np.matrix([0, 0, 0, 0, 1]).T
        Q = np.array([[1, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0]])
        R = np.array([[1]])
        self.omega_y = float(self.lqr(x,A,B,Q,R))

    def lqr_ys(self):
        x = np.matrix([self.load_pos.y, self.load_vel.y, self.uav_pos.y, self.uav_vel.y, self.uav_att.x]).T
        A = np.array([[0, 1, 0, 0, 0],
                      [-self.g/self.l, 0, 0, 0, self.g],
                      [0, 0, 0, 1, 0],
                      [0, 0, 0, 0, -self.g],
                      [0, 0, 0, 0, 0]])
        B = np.matrix([0, 0, 0, 0, 1]).T
        Q = np.array([[1, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0]])
        R = np.array([[1]])
        self.omega_x = float(self.lqr(x,A,B,Q,R))

    def lqr_z(self):
        x = np.matrix([self.uav_pos.z - 6, self.uav_vel.z]).T
        A = np.array([[0, 1],
                      [0, 0]])
        B = np.matrix([0, 1]).T
        Q = np.array([[1, 0],
                     [0, 0]])
        R = np.array([[1]])
        self.a = float(self.lqr(x,A,B,Q,R))
        #calculate waypoint value for u
        xe = np.matrix([6.0, 0.0]).T
        ue = -scipy.linalg.pinv(B)*(A*xe)


    def lqr(self,x,A,B,Q,R):
        K, S, E = control.lqr(A, B, Q, R)
        u = -scipy.linalg.inv(R)*(B.T*(S*x))
        print("x",x)
        #print("B",self.B)
        return u

    def output(self,t):
        self.t = t
        self.lqr_xr()
        self.lqr_ys()
        self.lqr_z()
        self.sp.body_rate.x = self.omega_x
        self.sp.body_rate.y = self.omega_y
        if self.a >=0:
            self.sp.thrust = 0.613 + self.a/40.54
        else:
            self.sp.thrust = 0.613 + self.a/15.98
        print("omegas",self.sp.body_rate.x, self.sp.body_rate.y, self.sp.thrust)

def main():
    # initiate node
    rospy.init_node('static_controller_node', anonymous=True)

    # flight mode object
    modes = set_point.fcuModes()

    # flight Controller
    cnt = Controller()
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to UAV and Payload Position
    rospy.Subscriber('/gazebo/link_states', LinkStates, cnt.get_position)

    # Attitude Setpoint publisher
    at_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

    # Position Setpoint Publisher
    pos_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    print('armed')

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k = 0
    while k<10:
        pos_pub.publish(cnt.pos_sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    # wait until the altitude of UAV is 5m
    cnt.init_position()
    while cnt.uav_pos.z<4.95 or cnt.uav_pos.x>2.05:
        print("uav_pos_z:",cnt.uav_pos.z)
        #print("load_pos_z:",cnt.load_pos.z)
        print("uav_x",cnt.uav_pos.x)
        #print("uav_y",cnt.uav_pos.y)
        print("load_x",cnt.load_pos.x)
        #print("load_y",cnt.load_pos.y)
        print("----------------")
    	pos_pub.publish(cnt.pos_sp)
    print("reached")
    time.sleep(0.1)

    # ROS main loop
    t_start = time.time()
    cnt.p0 = cnt.uav_pos.x - cnt.load_pos.x
    while not rospy.is_shutdown():
        t = time.time() - t_start
        cnt.output(t)
        at_pub.publish(cnt.sp)
        print("uav_x",cnt.uav_pos.x)
        print("uav_y",cnt.uav_pos.y)
        print("uav_pos_z:",cnt.uav_pos.z)
        print("----------------")
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
