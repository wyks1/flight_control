#创建日期：2020年10月10日
#版本：初版
#此程序对应北醒TF系列默认配置下串口版本有效
#此程序只提供参考和学习
# -*- coding: utf-8 -*-
import serial.tools.list_ports
from geometry_msgs.msg import PoseStamped,Vector3Stamped,TwistStamped
import time
import rospy
from nav_msgs.msg  import Odometry
import threading
import numpy as np

ser = serial.Serial()
ser.port = '/dev/ttyUSB1'    #设置端口
ser.baudrate = 115200 #设置雷达的波特率

def thread_job():
    rospy.spin()

class VelocityBody:
 def __init__(self):
        self.pos_pub = rospy.Publisher("/drone_1/Lidar_position", PoseStamped, queue_size = 10)
        self.add_thread = threading.Thread(target = thread_job)
        self.add_thread.start()
        self.current_position = Odometry()
        self.last_position = Odometry()



def getTFminiData():

   rospy.init_node('Lidar_position', anonymous=True)
   cnt = VelocityBody()
   last_time = time.time()
   rate = rospy.Rate(100)
   pos_est_pub = PoseStamped()
   last_pos_z = []
   last_vel_x = 0
   last_vel_y = 0
   last_vel_z = 0
   while True:
      count = ser.in_waiting #获取接收到的数据长度
      if count > 8:
         recv = ser.read(9)#读取数据并将数据存入recv
         #print('get data from serial port:', recv)
         ser.reset_input_buffer()#清除输入缓冲区
         if recv[0] == 0x59 and recv[1] == 0x59:  # python3
            distance = np.float(recv[2] + np.float(recv[3] << 8))
            strength = recv[4] + recv[5] * 256
            temp = (np.int16(recv[6] + np.int16(recv[7] << 8)))/8-256 #计算芯片温度
            current_time = time.time()
            deltat = current_time - last_time
            last_pos_z.append(distance/100)
            if(len(last_pos_z) > 5):
              last_pos_z.pop(0)
            pose_est_z = np.sum(np.array(last_pos_z)) / 5
            last_pos_z = list(last_pos_z)
            vel_z_cur = (pose_est_z - cnt.last_position.pose.pose.position.z) / deltat
            pos_est_pub.pose.position.z = pose_est_z
            pos_est_pub.pose.orientation.z = vel_z_cur
            #print('distance = %f  velocity = %f  temperature = %5d' % (pose_est_z, vel_z_cur, temp))
            pos_est_pub.header.stamp=rospy.Time.now()
            cnt.pos_pub.publish(pos_est_pub)
            cnt.last_position.pose.pose.position.z = pose_est_z
            last_time = current_time
            rate.sleep()
            ser.reset_input_buffer()
      else:
         time.sleep(0.005) #50ms
if __name__ == '__main__':
   try:
      if ser.is_open == False:
         try:
            ser.open()
         except:
            print('Open COM failed!')
      getTFminiData()
   except rospy.ROSInterruptException:
      pass
   except KeyboardInterrupt:  # Ctrl+C 停止输出数据
      if ser != None:
         ser.close()

