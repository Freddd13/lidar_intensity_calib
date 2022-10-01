'''
Date: 2022-10-01 21:10:22
LastEditors: Freddd13
LastEditTime: 2022-10-01 21:36:45
Description: 
yzdyzd13@gmail.com
'''


import numpy as np


mapping_a = np.array([[4,3,2,1],[6,7,8,9]])

raw = np.array([[4,4,4,2],[4,4,4,0]])
# 2 ->beam1->8, 0->beam0->4

beam = np.array([1,0])
raw[:,3] = mapping_a[beam, raw[:,3]]
print(raw)

# a_s = 
# Output : array([ 1,  4,  9, 16, 25])