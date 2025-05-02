#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
import math
import numpy as np
import quaternion
import tf
# メッセージの型等のimport
from collections import deque
from shipcon_pcc.msg import pos_binder_msgs
from shipcon_pcc.msg import gps_convert
from geometry_msgs.msg import PoseStamped
from shipcon_pcc.msg import gyro

class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.pub_pos_binder = rospy.Publisher('pos_binder', pos_binder_msgs, queue_size=10)
        self.msg_pos_binder = pos_binder_msgs()
        # messageの型を作成
    def make_msg(self,msg_pos_binder):
        print('a')
    def send_msg(self,msg_pos_binder):
        #self.make_msg(self.msg_gps_convert)
        self.pub_pos_binder.publish(msg_pos_binder)

class Subscribers():
    def __init__(self):
        #importしたメッセージファイルを箱に代入
        self.gps_convert_tmp = gps_convert()
        self.ndt_pose_tmp = PoseStamped()
        self.gyro_tmp = gyro()
        # Converted states
        self.gps_convert = rospy.Subscriber('gps_convert', gps_convert, self.convert_cb)   
        # LiDAR AutoWare
        self.ndt_pose = rospy.Subscriber('ndt_pose', PoseStamped, self.ndt_pose_cb)
        self.gyro = rospy.Subscriber('gyro', gyro, self.gyro_cb)
    def convert_cb(self, msg):
        self.gps_convert_tmp = msg
    def ndt_pose_cb(self, msg):
        self.ndt_pose_tmp = msg
    def gyro_cb(self, msg):
        self.gyro_tmp = msg

class pos_binder():
    def __init__(self, queue_size=2):
        self.time_queue = deque(maxlen=queue_size)
        self.x_pos_queue = deque(maxlen=queue_size)
        self.y_pos_queue = deque(maxlen=queue_size)
        self.yaw_angle_queue = deque(maxlen=queue_size)
        self.listener = tf.TransformListener()
    def refrsh_pos(self, sub):     #センサからのデータを常に更新する
        #GNSSから得た位置情報
        self.x_pos_gnss = sub.gps_convert_tmp.X_chosen
        self.y_pos_gnss = sub.gps_convert_tmp.Y_chosen
        self.u_gnss = sub.gps_convert_tmp.velo_dop_u_mid
        self.vm_gnss = sub.gps_convert_tmp.velo_dop_vm_mid
        #gyro data
        self.yaw_gyro = sub.gyro_tmp.ang_yaw
        self.r_gyro = sub.gyro_tmp.vel_yaw
        # #LiDARのndt_matchingより得た位置情報
        # self.x_pos_ndt = sub.ndt_pose_tmp.pose.position.x
        # self.y_pos_ndt = sub.ndt_pose_tmp.pose.position.y
        # self.z_pos_ndt = sub.ndt_pose_tmp.pose.position.z
        # #LiDARのndt_matchingより得たquatanion
        # self.quaternion = np.array([sub.ndt_pose_tmp.pose.orientation.w, sub.ndt_pose_tmp.pose.orientation.x, sub.ndt_pose_tmp.pose.orientation.y, sub.ndt_pose_tmp.pose.orientation.z])
        # pose and quaternion of base_link from world frame
        self.global_ndt = self.listener.lookupTransform("/world", "/base_link", rospy.Time(0))
    def calc_velo_lidar(self):
        # time
        time_aft = rospy.Time.now()
        self.time_queue.append(time_aft)
        # position x,y
        self.x_pos_queue.append(self.global_ndt[0][0])
        self.y_pos_queue.append(self.global_ndt[0][1])
        # quaternion
        quat_tmp = (self.global_ndt[1][0], self.global_ndt[1][1], self.global_ndt[1][2], self.global_ndt[1][3])
        # quaternion to euler angle
        self.euler = tf.transformations.euler_from_quaternion(quat_tmp) #euler[0]:pitch, euler[1]=roll, euler[2]=yaw
        # yaw by LiDAR
        self.yaw_angle_queue.append(self.euler[2])
        if len(self.time_queue) < 2:
            self.u_ndt = 0.0 # this is for publishing something. this value has no meaning
            self.vm_ndt = 0.0 # this is for publishing something. this value has no meaning
            self.r_ndt = 0.0 # this is for publishing something. this value has no meaning 
        else:
            # time
            time_bef = self.time_queue[0]
            dt = (time_aft - time_bef).to_sec()
            if dt == 0:
                return
            # dx,dy   
            x_pos_bef = self.x_pos_queue[0]
            y_pos_bef = self.y_pos_queue[0]
            x_pos_aft = self.x_pos_queue[1]
            y_pos_aft = self.y_pos_queue[1] 
            dx_dy = np.array([[x_pos_aft - x_pos_bef], [y_pos_aft - y_pos_bef]])
            # rotation matrix
            R = np.array([[math.cos(self.euler[2]), math.sin(self.euler[2])],[-math.sin(self.euler[2]), math.cos(self.euler[2])]]) 
            # delta_x, delta_y
            delta_x_y = R @ dx_dy
            # velocity u,vm
            u_vm = delta_x_y / dt
            self.u_ndt = u_vm[0]
            self.vm_ndt = u_vm[1]
            # delta yaw
            yaw_angel_bef = self.yaw_angle_queue[0]
            yaw_angel_aft = self.yaw_angle_queue[1]
            dyaw = yaw_angel_aft - yaw_angel_bef
            # yaw angle velocity
            self.r_ndt = dyaw / dt
    def calc_velo_gyro(self):
        time_aft = rospy.Time.now()
        self.time_queue.append(time_aft)
        if len(self.time_queue) < 2:
            self.u_gyro = 0    # this is for publishing something. this value has no meaning
            self.vm_gyro = 0   # this is for publishing something. this value has no meaning
        time_bef = self.time_queue[0]
        dt = (time_aft - time_bef).to_sec()
        self.u_gyro = self.acc_surge_gyro*dt
        self.vm_gyro = self.acc_sway_gyro*dt

def main():
    # nodeの立ち上げ
    rospy.init_node('NODE_pos_binder', anonymous = True)
    # インスタンスの作成と実行
    pub = Publishsers()
    sub = Subscribers()
    sub_and_pub = pos_binder()
    #rospy.spin()
    rate = rospy.Rate(10)
    msg_pos_binder = pub.msg_pos_binder
    while not rospy.is_shutdown():
        try:
            sub_and_pub.refrsh_pos(sub)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        sub_and_pub.calc_velo_lidar()
        # sub_and_pub.calc_velo_gyro()
        #以下pubするための処理
        msg_pos_binder.x_ndt = sub_and_pub.global_ndt[0][0]
        msg_pos_binder.y_ndt = sub_and_pub.global_ndt[0][1]
        msg_pos_binder.z_ndt = sub_and_pub.global_ndt[0][2]
        msg_pos_binder.x_GNSS = sub_and_pub.x_pos_gnss
        msg_pos_binder.y_GNSS = sub_and_pub.y_pos_gnss
        msg_pos_binder.u_ndt = sub_and_pub.u_ndt    
        msg_pos_binder.vm_ndt = sub_and_pub.vm_ndt
        msg_pos_binder.u_GNSS = sub_and_pub.u_gnss
        msg_pos_binder.vm_GNSS = sub_and_pub.vm_gnss
        msg_pos_binder.yaw_gyro = sub_and_pub.yaw_gyro
        msg_pos_binder.r_gyro = sub_and_pub.r_gyro
        msg_pos_binder.yaw_ndt = sub_and_pub.euler[2]
        msg_pos_binder.r_ndt = sub_and_pub.r_ndt
        pub.send_msg(msg_pos_binder)
        print("Hello World!")
        rate.sleep()

if __name__ == '__main__':
   main()
