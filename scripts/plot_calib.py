'''
Date: 2022-09-29 23:10:32
LastEditors: Freddd13
LastEditTime: 2022-09-29 23:37:50
Description: 
yzdyzd13@gmail.com
'''

import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # read calib file
    url_calib_file = "build/calib_res.txt"
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