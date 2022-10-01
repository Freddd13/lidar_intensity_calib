'''
Date: 2022-10-01 19:47:08
LastEditors: Freddd13
LastEditTime: 2022-10-01 19:52:56
Description: 
yzdyzd13@gmail.com
'''
#!/home/yxt/miniconda3/bin/python

import open3d as o3d
import numpy as np
import yaml

if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud("/home/yxt/code/slam/intensity_calib_fred_ws/src/intensity_calibration/sample/test_uniform_0.1.pcd")   #传入自己当前的pcd文件
    o3d.visualization.draw_geometries([pcd])
