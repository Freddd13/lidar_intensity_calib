'''
Date: 2022-09-29 23:10:32
LastEditors: Freddd13
LastEditTime: 2022-09-30 10:44:10
Description: 
yzdyzd13@gmail.com
'''

import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # read calib file
    url_calib_file = "/home/yxt/code/slam/intensity_calib_fred_ws/src/intensity_calibration/results/test_uniform_0.1_voxel0.400000_dis0.000000_intensity100.000000_var1.200000_eps0.050000_loss0.000000.txt"
    num_beams = 16
    num_calib_intensity = 100
    calib_data = np.loadtxt(url_calib_file)

    # plot!
    fig, ax = plt.subplots()
    intensity_original = np.linspace(0, num_calib_intensity - 1, num=num_calib_intensity)
    intensity_calibed = np.zeros(intensity_original.shape)

    for i in range(num_beams):
        intensity_calibed = calib_data[i, :]
        ax.plot(intensity_original, intensity_calibed)

    plt.show()