import os
from scipy.io import loadmat
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime


def getLastMatFileSaved():
  ResultSavingDirectory = os.path.expanduser('~') + '/JPExperiment/' + datetime.now().strftime("%y%m%d")
  if not os.path.exists(ResultSavingDirectory):
    os.makedirs(ResultSavingDirectory)
  fileList = []
  for file in os.listdir(ResultSavingDirectory):
  
    if file.endswith(".mat"):
      fileList.append(file)
  try:
    return sorted(fileList)[-1]
  except Exception as e:
      print(e)
  return "none"


folderName = os.path.expanduser('~') + '/JPExperiment/' + datetime.now().strftime("%y%m%d")
# fileName = "DataLog_2021_0204_164929_idx_0_30Hz30duty.mat"

mat_contents = loadmat(folderName + '/' + getLastMatFileSaved())

print(( sorted(mat_contents.keys()) ))

#%% Plot the data
# dataBuffer = mat_contents['SensorPacket_data']
# sensorData = dataBuffer[:,1:]
# plt.figure()
# plt.plot(sensorData)
# plt.ylabel('posD_pwr')
# plt.xlabel('sample')
# plt.grid()
# plt.show(block=False)
# plt.title('Mark Position')
# plt.pause(1)


# dataBuffer = mat_contents['SensorPacket_data']
# sensorData = dataBuffer[:,1:]
# plt.figure()
# plt.plot(dataBuffer[:,1:4])
# plt.ylabel('acc')
# plt.xlabel('sample')
# plt.grid()
# plt.show(block=False)
# plt.title('ACC')
# plt.pause(1)

# plt.clf()
# plt.plot(dataBuffer[:,4:])
# plt.ylabel('quat')
# plt.xlabel('sample')
# plt.grid()
# plt.show(block=False)
# plt.title('Quaternion')
# plt.pause(1)

dataBuffer = mat_contents['netftdata_data']

Forces = dataBuffer[:,1:4]
F_mean = np.mean(Forces[1:30,:], axis= 0)
Forces_noOffset = Forces - F_mean
NormForces = np.linalg.norm(Forces_noOffset,axis = 1)
print(np.amax(NormForces))

plt.clf()
plt.plot(Forces_noOffset)
plt.ylabel('F/T')
plt.xlabel('sample')
plt.grid()
plt.show(block=False)
plt.title('FT sensor')
plt.pause(1)

dataBuffer = mat_contents['endEffectorPose_data']
plt.clf()
plt.plot(dataBuffer[:,-3:])
plt.ylabel('Robot Position')
plt.xlabel('sample')
plt.grid()
plt.show(block=False)
plt.title('Robot Position')
plt.pause(1)


dataBuffer = mat_contents['SensorPacket_data']
plt.clf()
plt.plot(dataBuffer[:,1:4])
plt.ylabel('Robot Position')
plt.xlabel('sample')
plt.grid()
plt.show(block=False)
plt.title('Robot Position')
plt.pause(1)

plt.close()

#%% Plot the data
plt.figure()
dataBuffer = mat_contents['netftdata_data']

plt.plot(dataBuffer[:,1:4])
plt.ylabel('F/T')
plt.xlabel('sample')
plt.grid()
plt.show(block=False)
plt.title('FT sensor')
plt.pause(1)

dataBuffer = mat_contents['endEffectorPose_data']
plt.clf()
plt.plot(dataBuffer[:,-3:])
plt.ylabel('Robot Position')
plt.xlabel('sample')
plt.grid()
plt.show(block=False)
plt.title('Robot Position')
plt.pause(1)

plt.close()

# plt.show()