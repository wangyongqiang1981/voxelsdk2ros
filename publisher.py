#!/usr/bin/env python
from __future__ import print_function
import rospy
from sensor_msgs.msg import PointCloud2,PointField
import numpy as np
from Voxel import *
from struct import *
import time

g_seq = 0
g_pub = None
g_msg = None
g_tic = 0
def cb(dc,fr,ty):
    global g_seq,g_pub,g_msg,g_tic
    pcf = PointCloudFrame.typeCast(fr)
    tic = time.time()
    if (tic-g_tic) > 0.15:
        g_tic = tic
        now = rospy.get_rostime()
        g_seq += 1
        g_msg.header.seq = g_seq
        g_msg.header.stamp.secs = now.secs
        g_msg.header.stamp.nsecs = now.nsecs
        data = []
        for i in range(pcf.size()):
            try:
                b = pack('<f',pcf[i].x)
                data.append(ord(b[0]))
                data.append(ord(b[1]))
                data.append(ord(b[2]))
                data.append(ord(b[3]))
                b = pack('<f',pcf[i].y)
                data.append(ord(b[0]))
                data.append(ord(b[1]))
                data.append(ord(b[2]))
                data.append(ord(b[3]))
                b = pack('<f',pcf[i].z)
                data.append(ord(b[0]))
                data.append(ord(b[1]))
                data.append(ord(b[2]))
                data.append(ord(b[3]))
            except Exception as e:
                print(e)
        g_msg.data = data
        toc = time.time()
        g_pub.publish(g_msg)
    
def publisher():
    global g_pub,g_msg
    sys = CameraSystem()
    dev = sys.scan()
    cam = sys.connect(dev[0])
    if cam.isInitialized()==False:
        print('Fail init')
    #
    g_pub = rospy.Publisher('cloud_point', PointCloud2, queue_size=10)
    rospy.init_node('voxel_publisher', anonymous=True)
    g_msg = PointCloud2()
    g_msg.header.frame_id = 'base_link'
    g_msg.height = 240
    g_msg.width = 320
    g_msg.fields.append(PointField(name='x',offset=0,datatype=7,count=1))
    g_msg.fields.append(PointField(name='y',offset=4,datatype=7,count=1))
    g_msg.fields.append(PointField(name='z',offset=8,datatype=7,count=1))
    g_msg.point_step = 12
    g_msg.row_step = 320*12
    #
    cam.registerCallback(3,cb)
    ret,rate = cam.getFrameRate()
    if ret==False:
        print("Fail")
    print("fr:{}".format(rate.numerator))
    time.sleep(2) # necessory
    cam.start()        
    rospy.spin()

if __name__=='__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        print('E')
        
        
