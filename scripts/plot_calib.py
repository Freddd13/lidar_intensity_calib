'''
Date: 2022-09-29 23:10:32
LastEditors: Freddd13
LastEditTime: 2022-10-01 23:19:19
Description: 
yzdyzd13@gmail.com
'''

import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # read calib file
    url_calib_file = "/home/yxt/code/slam/intensity_calib_fred_ws/src/intensity_calibration/results/full_cloud_voxel0.200000_dis0.000000_intensity100.000000_var0.500000_eps0.050000_loss0.000001.txt"
    num_beams = 16
    num_calib_intensity = 100
    calib_data = np.loadtxt(url_calib_file)

    ##### plot calib res! #####
    fig, ax = plt.subplots()
    intensity_original = np.linspace(0, num_calib_intensity - 1, num=num_calib_intensity)
    intensity_calibed = np.zeros(intensity_original.shape)

    dict_beam_num = {}
    for i in range(num_beams):
        dict_beam_num[i] = str(i)
        
    for i in range(num_beams):
        intensity_calibed = calib_data[i, :]
        ax.plot(intensity_original, intensity_calibed, label=dict_beam_num[i])


    plt.legend()
    plt.xlabel("Original Intensity")
    plt.ylabel("Calibed Intensity")
    plt.title("Calibration Result")
    plt.show()

    ##### plot loss #####
    url_loss_file = "/home/yxt/code/slam/intensity_calib_fred_ws/src/intensity_calibration/results/loss.txt"
    loss_data = np.loadtxt(url_loss_file)
    num_epoch = loss_data.shape[1]

    # plot calib res!
    fig, ax = plt.subplots()
    epoch = np.linspace(0, num_epoch - 1, num=num_epoch)
    loss = np.zeros(epoch.shape)
    print(loss.shape)

    loss_type_dict = {0: "E loss", 1: "M loss"}
    for i in range(loss_data.shape[0]):
        loss = loss_data[i, :]
        ax.plot(epoch, loss, label=loss_type_dict[i])
    plt.legend()
    plt.xlabel("Epoch")
    plt.ylabel("Loss")
    plt.title("EM Loss")
    plt.show()