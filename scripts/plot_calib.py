'''
Date: 2022-09-29 23:10:32
LastEditors: Freddd13
LastEditTime: 2022-10-01 18:23:05
Description: 
yzdyzd13@gmail.com
'''

import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # read calib file
    url_calib_file = "/home/yxt/code/slam/intensity_calib_fred_ws/src/intensity_calibration/results/test_uniform_0.1_voxel0.400000_dis0.000000_intensity100.000000_var0.300000_eps0.050000_loss0.000000.txt"
    num_beams = 16
    num_calib_intensity = 100
    calib_data = np.loadtxt(url_calib_file)

    # plot calib res!
    fig, ax = plt.subplots()
    intensity_original = np.linspace(0, num_calib_intensity - 1, num=num_calib_intensity)
    intensity_calibed = np.zeros(intensity_original.shape)

    for i in range(num_beams):
        intensity_calibed = calib_data[i, :]
        ax.plot(intensity_original, intensity_calibed)

    plt.show()

    # plot loss
    url_loss_file = "/home/yxt/code/slam/intensity_calib_fred_ws/src/intensity_calibration/results/loss.txt"
    loss_data = np.loadtxt(url_loss_file)
    num_epoch = loss_data.shape[1]

    # plot calib res!
    fig, ax = plt.subplots()
    epoch = np.linspace(0, num_epoch - 1, num=num_epoch)
    loss = np.zeros(epoch.shape)
    print(loss.shape)

    for i in range(loss_data.shape[0]):
        loss = loss_data[i, :]
        ax.plot(epoch, loss)

    plt.show()