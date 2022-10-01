'''
Date: 2022-10-01 20:03:03
LastEditors: Freddd13
LastEditTime: 2022-10-01 22:53:01
Description: 
yzdyzd13@gmail.com
'''

#! /usr/bin/python

import rospy
import numpy as np
import ros_numpy
import pcl
from pypcd import pypcd
import time

from sensor_msgs.msg import PointCloud2
import sensor_msgs

def uncalib_pointcloud_cb(msg):
    global calib_data, num_beams, num_calib_intensity
    pc = pypcd.PointCloud.from_msg(msg)
    x = pc.pc_data['x']
    y = pc.pc_data['y']
    z = pc.pc_data['z']
    intensity = pc.pc_data['intensity']
    arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0],
                   dtype=np.float32)
    # arr[::4] = x
    # arr[1::4] = y
    # arr[2::4] = z
    # arr[3::4] = intensity
    # scan = arr.reshape(-1, 4)

    calibed = np.zeros(arr.shape[0])
    # get beam id
    angles = np.arctan(z / np.sqrt(x*x + y*y)) / np.pi * 180.0
    beam_ids = np.around((angles+ 15.0) / 2.0)


    # np.savetxt("raw.txt", intensity[intensity < 100].astype(np.int), newline=" ",fmt="%i")

    # correct intensity
    intensity[intensity < 100] = calib_data[beam_ids[intensity < 100].astype(np.int), intensity[intensity < 100].astype(np.int)]
    pc.pc_data['intensity'] = intensity
    # np.savetxt("calib.txt", intensity[intensity < 100].astype(np.int), newline=" ",fmt="%i")
    # time.sleep(5)
    calibed_pc_msg = pypcd.PointCloud.to_msg(pc)
    calibed_pc_msg.header.frame_id = "velodyne"
    calibed_pc_puber.publish(calibed_pc_msg)

if __name__ == "__main__":
    # init ros
    rospy.init_node('view_realtime_pc', anonymous=True)
    rospy.Subscriber("/velodyne_points", PointCloud2, uncalib_pointcloud_cb)
    # uncalibed_pc_puber = rospy.Publisher('/calibed_pc', PointCloud2, queue_size=2)
    calibed_pc_puber = rospy.Publisher('/calibed_pc', PointCloud2, queue_size=2)

    # read calib file
    url_calib_file = "/home/yxt/code/slam/intensity_calib_fred_ws/src/intensity_calibration/results/full_cloud_voxel0.200000_dis0.000000_intensity100.000000_var0.500000_eps0.050000_loss0.000001.txt"
    num_beams = 16
    num_calib_intensity = 100
    calib_data = np.loadtxt(url_calib_file)
    rospy.spin()
