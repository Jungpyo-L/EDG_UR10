# imports 
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio

# load mat file
data = sio.loadmat('/home/edg/EDG_Experiment/250521/DataLog_2025_0521_145817_digit_data_log.mat')

print(data.keys())