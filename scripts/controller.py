"""
author: Pulishen
UAV formation controller
"""
import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import String 
import time
import math
import numpy as np
import sys


class UAV_node:
    """
    Distributed UAV node
    """
    def __init__(self,id_number,uav_num,obs_pos,terminal_position):
        self.id = id_number # id
        self.uav_type = "iris" # UAV type
        self.uav_n = uav_num # UAV number
        self.obs_p = obs_pos # Obstacle location
        self.obs_n = len(self.obs_p) # Obstacle number
        self.terminal_p = terminal_position # terminal position
        self.vel_pub = rospy.Publisher("/xtdrone/"+self.uav_type+'_'+str(self.id-1)+"/cmd_vel_flu", Twist,queue_size=10) # velocity punlisher 
        self.pos_sub = self.pos_sub_init() # position subscriber
        self.vel_sub = self.vel_sub_init() # velocity subscriber 
        self.pos = [np.zeros(3)]*self.uav_n
        self.vel = [np.zeros(3)]*self.uav_n 
        self.formation = self.get_formation_shape()


    def vel_sub_init(self):
        """
        initialize velocity subscriber
        """
        vel_sub = [None]*self.uav_n
        for i in range(self.uav_n):
            vel_sub[i] = rospy.Subscriber(self.uav_type+'_'+str(i)+'/mavros/local_position/velocity_body',TwistStamped,self.vel_callback,i)
        return vel_sub

    def pos_sub_init(self):
        """
        initialize position subscriber
        """
        pos_sub = [None]*self.uav_n
        for i in range(self.uav_n):
            pos_sub[i] = rospy.Subscriber(self.uav_type+'_'+str(i)+'/mavros/vision_pose/pose',PoseStamped,self.pos_callback,i)
        return pos_sub

    def pos_callback(self,msg,id):
        """
        position subscriber callback function
        """
        self.pos[id] = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def vel_callback(self,msg,id):
        self.vel[id] = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    def get_formation_shape(self):
        """
        UAV formation shape
        3uav:    o      o
                       
                      
                        o
        4uav:    o      o


                 o      o
        """
        if self.uav_n == 3:
            f_s = [[0,-5,0],[0,0,-5],[0,0,0]]
        elif self.uav_n == 4:
            f_s = [[0,-5,0,-5],[0,0,-5,-5],[0,0,0,0]]
        return f_s


    def formation_control(self):
        """
        Consensus formation controller for second order systems
        Course paper chapter 2 
        beta:k_xy,k_z
        gama:k_v
        """
        k_xy = 3
        k_z = 3
        k_v = 10
        a_x = 0
        a_y = 0
        a_z = 0
        for i in range(self.uav_n):
            if i != self.id:
                a_x -= k_xy*(-self.pos[i][0]+self.pos[self.id-1][0]+self.formation[0][i]-self.formation[0][self.id-1])+k_v*(-self.vel[i][0]+self.vel[self.id-1][0])
                a_y -= k_xy*(-self.pos[i][1]+self.pos[self.id-1][1]+self.formation[1][i]-self.formation[1][self.id-1])+k_v*(-self.vel[i][1]+self.vel[self.id-1][1])
                a_z -= k_z*(-self.pos[i][2]+self.pos[self.id-1][2]+self.formation[2][i]-self.formation[2][self.id-1])+k_v*(-self.vel[i][2]+self.vel[self.id-1][2])
        return [a_x,a_y,a_z]

    def terminal_to_leader(self):
        """
        force of the target point to leader
        Course paper chapter 3.2
        kxoy:k_xy
        kz:k_z
        kv:k_v
        """
        vel_u = math.sqrt(self.pos[0][0]**2+self.pos[0][1]**2)
        k_v = 20
        k_xy = 15
        k_z = 3
        a_xy = k_xy*math.sqrt((self.pos[0][0]-self.terminal_p[0])**2+(self.pos[0][1]-self.terminal_p[1])**2)
        if a_xy > 30:
           a_xy = 30
        a_z = -k_z*(self.pos[0][2]-self.terminal_p[2])
        dis_xy = np.array([-self.pos[0][0]+self.terminal_p[0],-self.pos[0][1]+self.terminal_p[1]])
        cos = dis_xy.dot([1,0])/(np.linalg.norm(dis_xy))
        sin = dis_xy.dot([0,1])/(np.linalg.norm(dis_xy))
        a_x = a_xy*cos-k_v*self.vel[0][0]
        a_y = a_xy*sin-k_v*self.vel[0][1]
        return [a_x,a_y,a_z]

    def obstacle_to_uav(self):
        """
        force of the obstacle to each uav
        Course paper chapter 3.4
        ko:k_o
        ruo:r_o
        kuo:g
        m:m
        """
        k_o = 300
        pos = self.pos[self.id-1]
        vel = np.linalg.norm(self.vel[self.id-1])
        dis_xy = np.array([-self.pos[0][0]+self.terminal_p[0],-self.pos[0][1]+self.terminal_p[1]])
        r_xoy = np.linalg.norm(dis_xy)
        r_o = 15
        m = 0.002
        g = k_o*(1+math.exp(-1/(vel+1)))
        a_x = 0
        a_y = 0
        for i in range(self.obs_n):
            dis = np.array([-pos[0]+self.obs_p[i][0],-pos[1]+self.obs_p[i][1]])
            d = np.linalg.norm(dis[:2]) 
            if d < r_o:
                a1 = g*(1/d-1/r_o)*((1/d)**2)*(r_xoy**m)
                cos1 = dis.dot([1,0])/np.linalg.norm(dis)
                sin1 = dis.dot([0,1])/np.linalg.norm(dis)
                a2 = (m/2)*g*((1/d-1/r_o)**2)*(r_xoy**(m-1))
                cos2 = dis_xy.dot([1,0])/(r_xoy)
                sin2 = dis_xy.dot([0,1])/(r_xoy)
                a_x = a_x-a1*cos1-a2*cos2
                a_y = a_y-a1*sin1-a2*sin2
        return [a_x,a_y,0]
       
    def uav_to_uav(self):
        """
        Obstacle avoidance within the cluster
        Course paper chapter 3.3
        kuu:k_f
        ruu:r_f
        """
        k_f = 20
        r_f = 4
        a_x = 0
        a_y = 0
        a_z = 0
        num = self.id-1
        pos = self.pos
        for i in range(self.uav_n): 
            if i != num: 
                dis = np.array([-pos[i][0]+pos[num][0],-pos[i][1]+pos[num][1],-pos[i][2]+pos[num][2]])
                r = np.linalg.norm(dis)
            
                if r < r_f:
                    a_f = k_f*(1/r - 1/r_f)*1/(r**2)
                    sin1 = dis.dot([0,0,1])/r
                    cos1 = math.sqrt(1-sin1**2)
                    cos2 = dis[:2].dot([1,0])/np.linalg.norm(dis[:2])
                    sin2 = dis[:2].dot([0,1])/np.linalg.norm(dis[:2])
                    a_x = a_x + a_f*cos2*cos1
                    a_y = a_y + a_f*sin2*cos1
                    a_z = a_z + a_f*sin1
        return [a_x,a_y,a_z]

    def disturbance(self):
        """
        The solution of local minimum problem is simplified to generate random disturbance
        """
        num = self.id-1
        dis_xy = np.array([-self.pos[num][0]+self.terminal_p[0],-self.pos[num][1]+self.terminal_p[1]])
        cos = dis_xy.dot([1,0])/(np.linalg.norm(dis_xy))
        sin = dis_xy.dot([0,1])/(np.linalg.norm(dis_xy))
        a_y = 0.5*np.sign(2*np.random.random()-1)*a_xy*cos
        a_x = 0.5*np.sign(2*np.random.random()-1)*a_xy*sin
        return [a_x,a_y,0]

    def run(self):
        """
        Run ROS thread
        """
        vel = Twist()
        dt = 0.02
        rospy.init_node("UAV"+str(self.id), anonymous=True)
        rate = rospy.Rate(50)
        rospy.loginfo("UAV"+str(self.id)+" loading...")
        a = [0,0,0]
        while not rospy.is_shutdown():
            if self.id != 1:
                a1 = self.formation_control()
            else:
                a1 = self.terminal_to_leader()
            a2 = self.uav_to_uav()
            a3 = self.obstacle_to_uav()
            a[0] = a1[0]+a2[0]+a3[0]
            a[1] = a1[1]+a2[1]+a3[1]
            a[2] = a1[2]+a2[2]
            if np.linalg.norm(a) < 0.5: # If it falls into local minimum
                a = a + self.disturbance()
            vel.linear.x = a[0]*dt + self.vel[self.id-1][0]
            vel.linear.y = a[1]*dt + self.vel[self.id-1][1]
            vel.linear.z = a[2]*dt + self.vel[self.id-1][2]
            self.vel_pub.publish(vel)
            rate.sleep()


if __name__ == '__main__':
    pos_o = [[20,20,10],[38.6,39.6,10],[41.6,32.7,10],[47.8,54.39,10],[48.2,44.96,10],[59.63,55.55,10]]
    pos_t = [80,65,5]
    a = UAV_node(int(sys.argv[1]),3,pos_o,pos_t)

    a.run()
