#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PointCloud2 msgに対して，最大距離の閾値を設け，閾値以上の点を黒でマスクし，画像に変換するプログラム
maintainer: Ryohei Kobayashi and Ryo Terashima

"""

import math
import numpy as np

import cv2

import rospy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo

import ros_numpy # apt install ros-noetic-ros-numpy

class PCLMaskByDist:


    def __init__(self):

        # 距離方向の閾値
        self.max_depth_threshold = 1.5

        self.camera_info_msg = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
        self.camera_height = self.camera_info_msg.height
        self.camera_width = self.camera_info_msg.width

        self.pcl_msg = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.pcl_callback, queue_size=10)
    
    def pcl_callback(self, msg):

        # pclのmsgをros_numpyで配列化
        pcl_data = ros_numpy.numpify(msg)
        # pclからrgbを抜き出す
        get_rgb = pcl_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]

        # bgr色空間からrgb色空間へ
        bgr2rgb = cv2.cvtColor(get_rgb , cv2.COLOR_BGR2RGB)
        rospy.loginfo(bgr2rgb)

        # pclからx,y,zを抜き出す
        x = pcl_data['x']
        y = pcl_data['y']
        z = pcl_data['z']

        # マスクを作成: x,y,zにnanの値が含まれていないか & zが最大深度しきい値以下であるかどうか
        valid_mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z <= self.max_depth_threshold)

        # bgr2rgb(入力サイズ)と同じサイズを持つ要素0(black)の配列を作成
        filtered_image = np.zeros_like(bgr2rgb)
        # マスクを適用 
        filtered_image[valid_mask] = bgr2rgb[valid_mask]

        # 画像の表示
        cv2.imshow('PCL MASK BY DISTANCE NODE: BY RYOHEISOFT', filtered_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('pcl_mask_by_dist_node')
    pmbd = PCLMaskByDist()
    rospy.spin()

