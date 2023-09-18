#创建日期：2020年10月10日
#版本：初版
#此程序对应北醒TF系列默认配置下串口版本有效
#此程序只提供参考和学习
# -*- coding: utf-8 -*-
import serial.tools.list_ports
from geometry_msgs.msg import PoseStamped,Vector3Stamped,TwistStamped
import time
import rospy
import threading
import numpy as np

ser = serial.Serial()
ser.port = '/dev/ttyUSB1'    #设置端口
ser.baudrate = 115200 #设置雷达的波特率

def thread_job():
    rospy.spin()

class VelocityBody:
 def __init__(self):
        self.pos_pub = rospy.Publisher("Lidar_position", PoseStamped, queue_size = 10)
        self.add_thread = threading.Thread(target = thread_job)
        self.add_thread.start()


def getTFminiData():

   rospy.init_node('Lidar_position', anonymous=True)
   cnt = VelocityBody()
   rate = rospy.Rate(100)
   pos_est_pub = PoseStamped()
   while True:
      count = ser.in_waiting #获取接收到的数据长度
      if count > 8:
         recv = ser.read(9)#读取数据并将数据存入recv
         #print('get data from serial port:', recv)
         ser.reset_input_buffer()#清除输入缓冲区
         if recv[0] == 0x59 and recv[1] == 0x59:  # python3
            distance = np.int16(recv[2] + np.int16(recv[3] << 8))
            strength = recv[4] + recv[5] * 256
            temp = (np.int16(recv[6] + np.int16(recv[7] << 8)))/8-256 #计算芯片温度
            #print('distance = %5d  strengh = %5d  temperature = %5d' % (distance, strength, temp))
            pos_est_pub.pose.position.z = distance/100
            cnt.pos_pub.publish(pos_est_pub)
            rate.sleep()
            ser.reset_input_buffer()
         if recv[0] == 'Y' and recv[1] == 'Y':  # python2 //此处标示出文件读取成功
            lowD = int(recv[2].encode('hex'), 16)
            highD = int(recv[3].encode('hex'), 16)
            lowS = int(recv[4].encode('hex'), 16)
            highS = int(recv[5].encode('hex'), 16)
            lowT = int(recv[6].encode('hex'), 16)
            highT = int(recv[7].encode('hex'), 16)
            distance = np.int16(lowD + np.int16(highD << 8))
            strength = lowS + highS * 256
            temp = (np.int16(lowD + np.int16(highD << 8)))/8-256 #计算芯片温度
            print('distance = %5d  strengh = %5d  temperature = %5d' % (distance, strength, temp))

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

