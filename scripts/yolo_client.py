#!/usr/bin/python
# coding: utf-8

import socket
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import QuaternionStamped

target_corner_msg = QuaternionStamped()
pickup_corner_msg = QuaternionStamped()


def clint():
    rospy.init_node('yolo_clint', anonymous=True)
    yolo_target_pub = rospy.Publisher('yolo_target_corner', QuaternionStamped, queue_size=1)
    yolo_pickup_pub = rospy.Publisher('yolo_pickup_corner', QuaternionStamped, queue_size=1)

    rate = rospy.Rate(30)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 连接服务端
    print ('connect state: ', s.connect_ex(('127.0.0.1', 8000)))

    while not rospy.is_shutdown():
        target_get_flag = False
        pickup_get_flag = False
        receive_msg = s.recv(100).decode()
        # print (receive_msg)
        msg = receive_msg.split(',')
        print(msg)
        if msg[0] == '0':
            target_get_flag = True
            print ("target corner")
            """ QuaternionStamped.x, y, z, w = xmin, ymin, xmax, ymax """
            target_corner_msg.header.stamp = rospy.get_rostime()
            target_corner_msg.quaternion.x = float(msg[1])
            target_corner_msg.quaternion.y = float(msg[2])
            target_corner_msg.quaternion.z = float(msg[3])
            target_corner_msg.quaternion.w = float(msg[4])

        if msg[0] == '1':
            pickup_get_flag = True
            print ("pickup corner")
            """ QuaternionStamped.x, y, z, w = xmin, ymin, xmax, ymax """
            pickup_corner_msg.header.stamp = rospy.get_rostime()
            pickup_corner_msg.quaternion.x = float(msg[1])
            pickup_corner_msg.quaternion.y = float(msg[2])
            pickup_corner_msg.quaternion.z = float(msg[3])
            pickup_corner_msg.quaternion.w = float(msg[4])

        if not target_get_flag:
            print (" target not found ... ")
            target_corner_msg.header.stamp = rospy.get_rostime()
            target_corner_msg.quaternion.x = -1.0
            target_corner_msg.quaternion.y = 0.0
            target_corner_msg.quaternion.z = 0.0
            target_corner_msg.quaternion.w = 0.0

        if not pickup_get_flag:
            print (" pickup not found ... ")
            pickup_corner_msg.header.stamp = rospy.get_rostime()
            pickup_corner_msg.quaternion.x = -1.0
            pickup_corner_msg.quaternion.y = 0.0
            pickup_corner_msg.quaternion.z = 0.0
            pickup_corner_msg.quaternion.w = 0.0

        yolo_target_pub.publish(target_corner_msg)
        yolo_pickup_pub.publish(pickup_corner_msg)
        rate.sleep()


if __name__ == "__main__":
    clint()