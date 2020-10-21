#! /usr/bin/python2

import rospy
from std_msgs.msg import Float64MultiArray
from autoware_msgs.msg import lane
from geometry_msgs.msg import PoseStamped
from autoware_msgs.msg import lane
#from nav_msgs.msg import Odometry
#from ackermann_msgs.msg import Ackered

import math

import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la

import Cubic_Spine



Q = np.eye(5)
R = np.eye(2)


dt = 0.1
L = 1.53  
max_steer = np.deg2rad(25.0)  
target_speed = 5
goal_dis = 1.5




class State:

    def __init__(self, x=0.0, y=0.0, yaw=np.deg2rad(90), v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


class lqr_control_node():
    def __init__(self):
        self._tx = []
        self._ty = []
        self.goal = []
        self.count = 0
        self.state = State(x = 0.0, y = 0.0, yaw = 0.0, v = 0.0)
        
        self.e = 0
        
        self.e_th = 0
        self.v = 0
        self.delta = 0

        self.IsResGo = 0
        
        self.Run = Float64MultiArray()
        self.Run.data = [0,0,0,0,0]


        #self.info = AckermannDriveStamped()
        self._pub = rospy.Publisher("command",Float64MultiArray,queue_size=10)
        self._sub_path = rospy.Subscriber("final_waypoints",lane,self.callback_from_path,queue_size=10)
        self._sub_state = rospy.Subscriber("current_pose",PoseStamped,self.callback_from_pose,queue_size=10)
        #self._sub_ResGo = rospy.Subscriber("ecu_msg",Float64MultiArray,self.callback_from_ResGo,queue_size=10)
        rospy.spin()


    def callback_from_pose(self,msg):
        self.state.x = msg.pose.position.x
        self.state.y = msg.pose.position.y
        # x = msg.pose.orientation.x
        # y = msg.pose.orientation.y
        # z = msg.pose.orientation.z
        # w = msg.pose.orientation.w

        # r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        # p = math.asin(2 * (w * y - z * x))
        # y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        # self.state.yaw = y
        # print(self.state.yaw)
        # self.state.v = msg.pose.position.z
        self.state.yaw = msg.pose.position.z
        print(self.state.yaw)
        self.state.v = msg.pose.orientation.w
        


    def callback_from_path(self,msg):
       
        for i in range(msg.waypoints.__len__()):
            self._tx.append(msg.waypoints[i].pose.pose.position.x)
            self._ty.append(msg.waypoints[i].pose.pose.position.y)
        self.count+=1
        self.goal = [self._tx[-1],self._ty[-1]]
        print(self._ty)
        cx, cy, cyaw, ck, s = Cubic_Spine.calc_spline_course(
            self._tx, self._ty, ds=0.1)
        self._tx = []
        self._ty = []
        sp = self.speed_set(cx, cy, cyaw, target_speed)
        while True:
            delta, target_ind, self.e, self.e_th, ai = self.LQR_control(
                self.state, cx, cy, cyaw, ck, self.e, self.e_th,sp)
            if delta >max_steer:
                delta = max_steer
            if delta < -max_steer:
                delta = -max_steer
            self.v= self.state.v + ai*dt
            if self.v>target_speed:
                self.v = target_speed
            
            if(abs(self.delta - delta)> 0.1):
                self.delta = delta
                
            self.Run.data[0] = self.v
            #self.Run.data[0] = 0
            self.Run.data[1] = self.delta*180/math.pi
            #self.Run.data[1] = -10
            self.Run.data[2] = 1
            self.Run.data[3] = 0
            self.Run.data[4] = 2
            self._pub.publish(self.Run)
            #self.info.drive.steering_angle = delta
            #self.info.drive.speed = self.state.v + ai*dt
      
            #self._pub.publish(self.info)


            dx = self.state.x - self.goal[0]
            dy = self.state.y - self.goal[1]
            rospy.sleep(0.01)
            if self.state.y>=30:
                self.Run.data[0] = 0
                self.Run.data[1] = 0
                self.Run.data[2] = 1
                self.Run.data[3] = 0
                self.Run.data[4] = 3
                break
            if math.sqrt(dx ** 2 + dy ** 2) <= goal_dis:
                print("Goal")
                break













    def pi_2_pi(self,angle):

        return (angle + math.pi) % (2*math.pi) - math.pi

    def compute_riccati(self,A,B,Q,R):
        
        Pn= Q
        Pn_Plus = Q
        maxiter = 200
        eps = 0.01

        for i in range(maxiter):
            # Pn = A.T @ P @ A - A.T @ P @ B @ la.inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
            Pn_Plus=Q + np.matmul(np.matmul(A.T,Pn),A)-np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(A.T,Pn),B),la.inv(R+np.matmul(np.matmul(B.T,Pn),B))),B.T),Pn),A)
            if (abs(Pn_Plus - Pn)).max() < eps:
                break
            Pn = Pn_Plus

        return Pn_Plus


    def cal_feedback_K(self,A,B,Q,R):

        P = self.compute_riccati(A, B, Q, R)

        K = np.matmul(np.matmul(np.matmul(la.inv(R+np.matmul(np.matmul(B.T,P),B)),B.T),P),A)


        return K

    def LQR_control(self,state, cx, cy, cyaw, ck, pe, pth_e, sp):
        INDEX, e = self.calc_nearest_index(state, cx, cy, cyaw)

        tv = sp[INDEX]

        k = ck[INDEX]
        v = state.v
        th_e = self.pi_2_pi(state.yaw - cyaw[INDEX])

        A = np.zeros((5, 5))
        A[0, 0] = 1.0
        A[0, 1] = dt
        A[1, 2] = v
        A[2, 2] = 1.0
        A[2, 3] = dt
        A[4, 4] = 1.0
        # print(A)

        B = np.zeros((5, 2))
        B[3, 0] = v / L
        B[4, 1] = dt

        K = self.cal_feedback_K(A, B, Q, R)

        x = np.zeros((5, 1))

        x[0, 0] = e
        x[1, 0] = (e - pe) / dt
        x[2, 0] = th_e
        x[3, 0] = (th_e - pth_e) / dt
        x[4, 0] = v - tv

        U_optimal = np.matmul(-K,x)


        ff = math.atan2(L * k, 1)
        fb = self.pi_2_pi(U_optimal[0, 0])


        ai = U_optimal[1, 0]

        delta = ff + fb

        return delta, INDEX, e, th_e, ai

    def calc_nearest_index(self,state, cx, cy, cyaw):
        dx = [state.x - icx for icx in cx]
        dy = [state.y - icy for icy in cy]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        distance = min(d)

        INDEX = d.index(distance)

        distance = math.sqrt(distance)

        dxl = cx[INDEX] - state.x
        dyl = cy[INDEX] - state.y

        angle = self.pi_2_pi(cyaw[INDEX] - math.atan2(dyl, dxl))
        if angle < 0:
            distance *= -1

        return INDEX, distance

    def speed_set(self,cx, cy, cyaw, target_speed):
        speed_profile = [target_speed] * len(cx)

        direction = 1.0

        # Set stop point
        for i in range(len(cx) - 1):
            dyaw = abs(cyaw[i + 1] - cyaw[i])
            switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

            if switch:
                direction *= -1

            if direction != 1.0:
                speed_profile[i] = 0
            else:
                speed_profile[i] = target_speed

            if switch:
                speed_profile[i] = 0.0

        return speed_profile



def main():
    print("LQR control node start!!")
    rospy.init_node("lqr_control",anonymous=True)
    lqr_control_node()




if __name__ == '__main__':
    main()
