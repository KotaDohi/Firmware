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

class Controller:
    def __init__(self):
        #Drone state
        self.state = State()
        self.uav_pos = Point(0.0, 0.0, 3.0)
        self.load_pos = Point(0.0, 0.0, 0.0)
        self.uav_vel = Point(0.0, 0.0, 0.0)
        self.load_vel = Point(0.0, 0.0, 0.0)
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
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # We will fly at a fixed altitude for now
        self.ALT_SP = 3.0

        # parameers of the system
        self.l = 4.01 #length of the tether
        self.r = 3.0 #radius of the UAV circle
        self.p0 = 0.8 #radius of the load circle
        self.g = 9.80665 #gravity

    def init_position(self):
        self.pos_sp.position.x = 3.0
        self.pos_sp.position.y = 0
        self.pos_sp.position.z = 5.0

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def get_position(self,msg):
        self.load_pos.x = msg.pose[28].position.x - msg.pose[1].position.x
        self.load_pos.y = msg.pose[28].position.y - msg.pose[1].position.y
        self.load_pos.z = msg.pose[28].position.z

        self.uav_pos.x = msg.pose[1].position.x
        self.uav_pos.y = msg.pose[1].position.y
        self.uav_pos.z = msg.pose[1].position.z

        self.load_vel.x = msg.twist[28].linear.x - msg.twist[1].linear.x
        self.load_vel.y = msg.twist[28].linear.y - msg.twist[1].linear.y
        self.load_vel.z = msg.twist[28].linear.z

        self.uav_vel.x = msg.twist[1].linear.x
        self.uav_vel.y = msg.twist[1].linear.y
        self.uav_vel.z = msg.twist[1].linear.z

    def cal_x(self):
        uav_pos = np.array([self.uav_pos.x, self.uav_pos.y, self.uav_pos.z])
        load_pos = np.array([self.load_pos.x, self.load_pos.y])
        uav_vel = np.array([self.uav_vel.x, self.uav_vel.y, self.uav_vel.z])
        load_vel = np.array([self.load_vel.x, self.load_vel.y])

        rot_matrix1 = np.array([[np.cos(self.omega*self.t), np.sin(self.omega*self.t), 0], [-np.sin(self.omega*self.t), np.cos(self.omega*self.t), 0], [0, 0, 1]])
        rot_matrix2 = np.array([[np.sin(self.omega*self.t), -np.cos(self.omega*self.t), 0], [np.cos(self.omega*self.t), np.sin(self.omega*self.t), 0], [0, 0, 0]])

        inv_uav_pos = np.dot(rot_matrix1, uav_pos)
        inv_uav_vel = np.dot(rot_matrix1, uav_vel) - self.omega*np.dot(rot_matrix2, uav_pos)
        inv_load_pos = np.dot(rot_matrix1[:2,:2], load_pos)
        inv_load_vel = np.dot(rot_matrix1[:2,:2], load_vel) - self.omega*np.dot(rot_matrix2[:2,:2], load_pos)

        self.lqr_x = np.matrix([inv_load_pos[0],  inv_load_vel[0], inv_load_pos[1], inv_load_vel[1], inv_uav_pos[0],
        inv_uav_vel[0], inv_uav_pos[1], inv_uav_vel[1], inv_uav_pos[2], inv_uav_vel[2]]).T
        #print("lqr_x",self.lqr_x)

    def cal_AB(self):
        #calc all the components
        xi = np.sqrt(self.l**2-self.p0**2)
        a0 = np.sqrt(self.g**2+self.omega**4*self.r**2)
        mu0 = np.arctan(-self.omega**2*self.r/self.g)
        p1 = xi**2/self.l**2*(self.omega**2+self.g/xi**3*(4*self.p0-self.l**2))
        p2 = xi**2/self.l**2*2*self.omega
        p3 = xi**2/self.l**2*(self.p0/xi*a0*np.sin(mu0)-a0*np.cos(mu0))
        p4 = -xi**2/self.l**2*(self.p0/xi*np.cos(mu0)-np.sin(mu0))
        q1 = self.omega**2-self.g/xi
        q2 = -2*self.omega
        q3 = a0
        u1 = np.sin(mu0)
        u2 = a0*np.cos(mu0)
        u3 = 2*self.omega
        u4 = self.omega**2
        v1 = -a0
        v2 = -2*self.omega
        v3 = self.omega**2
        w1 = np.cos(mu0)
        w2 = -a0*np.sin(mu0)

        self.A = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                           [p1,0, 0, p2,0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                           [0, q2,q1,0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 0, u4,0, 0,u3, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0,v2,v3, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

        self.B = np.array([[0, 0, 0],
                           [p3,0,p4],
                           [0, 0, 0],
                           [0,q3, 0],
                           [0, 0, 0],
                           [u2,0,u1],
                           [0, 0, 0],
                           [0,v1, 0],
                           [0, 0, 0],
                           [w2,0,w1]])
        #print("A",self.A)
        #print("B",self.B)

    def lqr(self):
        Q = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        #Q = np.zeros([10,10])
        R = 1000*np.eye(3)
        K, S, E = control.lqr(self.A, self.B, Q, R)
        #P = np.matrix(scipy.linalg.solve_discrete_are(self.A, self.B, Q, R))
        self.u = -scipy.linalg.inv(R)*(self.B.T*(S*self.lqr_x))
        #self.u = -np.matrix(scipy.linalg.inv(self.B.T*P*self.B+R)*(self.B.T*P*self.A))*self.lqr_x
        #print("S",S)
        #print("B",self.B)
        #print("x",self.lqr_x)
        #print("u",self.u)

    def cal_omegas(self):
        gamma = np.arcsin(np.sin(float(self.u[1]))*np.cos(self.omega*self.t) - np.sin(float(self.u[0]))*np.cos(float(self.u[1]))*np.sin(self.omega*self.t))
        print("gamma",gamma)
        beta = np.arccos(np.cos(float(self.u[0]))*np.cos(float(self.u[1]))/np.cos(gamma))
        print("beta",beta)
        gamma_dot = (self.r*self.omega**3*np.arccos(gamma)*np.cos(self.omega*self.t))/np.sqrt(self.g**2+self.omega**4*self.r**2)
        beta_dot = self.r*self.omega**3*np.arccos(gamma)*(np.tan(beta)*np.tan(gamma)*np.cos(self.omega*self.t)+np.arccos(beta)*np.sin(self.omega*self.t))/np.sqrt(self.g**2+self.omega**4*self.r**2)

        rot_matrix = np.array([[np.cos(beta)*np.cos(gamma),-np.sin(gamma),0],[np.cos(beta)*np.sin(gamma),np.cos(gamma),0],[-np.sin(beta),0,1]])

        euler = np.array([gamma_dot,beta_dot,0])
        self.omegas = np.dot(rot_matrix,euler)

    def output(self,t):
        self.t = t
        self.cal_x()
        self.cal_AB()
        self.lqr()
        self.cal_omegas()
        self.sp.body_rate.x = self.omegas[0]
        self.sp.body_rate.y = self.omegas[1]
        self.sp.body_rate.z = self.omegas[2]
        self.sp.thrust = float(self.u[2])/15.56
        print("omegas",self.omegas[0],self.omegas[1],self.omegas[2],float(self.u[2]))

def main():
    # initiate node
    rospy.init_node('controller_node', anonymous=True)

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
    while cnt.uav_pos.z<5 or cnt.uav_pos.x<1 or cnt.uav_pos.x>3:
        print("uav_pos_z:",cnt.uav_pos.z)
        print("load_pos_z:",cnt.load_pos.z)
        print("uav_x",cnt.uav_pos.x)
        print("uav_y",cnt.uav_pos.y)
        print("load_x",cnt.load_pos.x)
        print("load_y",cnt.load_pos.y)
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
        print("----------------")
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
