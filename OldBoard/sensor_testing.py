import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import numpy as np
from numpy import loadtxt
import matplotlib.pyplot as plt
import time
from spatialmath import SE3

top = 0
forty5 = 0

# ANSI escape codes for text color
class Color:
    RED = '\033[91m'
    GREEN = '\033[92m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'

#import data
filename = 'to_test/FA6.txt'
data = np.loadtxt(filename, delimiter=',')
# data = data[0:10000,:]

#temporary input data format
# [1,    2,  3,  4,  5-12,    13-16,    17,      18,    19,     20,      21,        22,        23,              24,         25,           26,          27,                28,         29,   30,      31,    32,     33,        34   ]
# [time, Fx, Fy, Fz, s1 - s8,   tof,  x_des,  y_des,  z_des, contact, pitch_des, roll_des, x_act pulse , y1_act pulse, y2_act pulse, z_act pulse, pitch_act pulse, roll_act pulse, x_act, y1_act, y2_act, z_act, pitch_act, roll_act]

#filter out non-contact points
contact_inds = data[:,19] == 1
contact_inds = contact_inds * 1
contact_transitions = np.diff(data[:,19]) #this might not be doing what i think it is?
ATI_offsets_inds = contact_transitions == 1
ATI_offsets_inds = ATI_offsets_inds * 1

# Find indices where ATI_offsets_inds_top is equal to 1
transition_inds = np.where(ATI_offsets_inds == 1)[0]

# Take the first two occurrences
transition_inds = transition_inds[:2]

# Calculate the difference between the second and first indices
gl = transition_inds[1] - transition_inds[0]

gl_new = len(ATI_offsets_inds) - gl + 20 + 1

#process ATI force data -> zero ATI forces for each new contact and transform coordinate frame from ATI frame to sensor frame
for ii in range(len(ATI_offsets_inds)):
    if ii == gl_new:
        data[ii+1:ii+gl-20,1:4] = data[ii+1:ii+gl-20,1:4] - data[ii, 1:4]
    elif ATI_offsets_inds[ii]==1:
        # x1 = data[ii:ii+gl-1,1:4]
        aa = data[ii+1:ii+gl,1:4] - data[ii,1:4]
        data[ii+1:ii+gl,1:4] = data[ii+1:ii+gl,1:4] - data[ii,1:4]

contact_data = data[contact_inds.flatten().astype(bool), :]

ATI_forces = contact_data[:,1:4]
fingertip_forces = np.zeros(ATI_forces.shape)

for ii in range(len(ATI_forces)):
    R = SE3.Rz(np.pi)
    transformed_f = -(R*ATI_forces[ii,:].T)
    fingertip_forces[ii,:] = transformed_f.T
#extract raw sensor values, ATI forces values, and pitch, roll
pressure_readings = contact_data[:,4:12]

pitch = contact_data[:,32]
roll = contact_data[:,33] * 180/np.pi

if top:
    pitch = pitch*180/np.pi - 90
elif forty5:
    pitch = pitch*180/np.pi - 45

# network class
# 8 inputs to 5 outputs
class sensorNet(nn.Module):
    def __init__(self):
        super(sensorNet, self).__init__()
        self.l1 = nn.Linear(8,12)
        self.relu1 = nn.ReLU()
        self.l2 = nn.Linear(12,64)
        self.relu2 = nn.ReLU()
        self.l3 = nn.Linear(64,64)
        self.relu3 = nn.ReLU()
        self.l4 = nn.Linear(64,5)

    def forward(self,x):
        output = self.l1(x)
        output = self.relu1(output)
        output = self.l2(output)
        output = self.relu2(output)
        output = self.l3(output)
        output = self.relu3(output)
        output = self.l4(output)
        return output

class sensorDataset(Dataset):
    def __init__(self, X, y):
        # X and y should already be tensors on cuda device
        self.X = X
        self.y = y

    def __len__(self):
        # this should return the size of the dataset
        return len(self.X)

    def __getitem__(self, idx):
        # this should return one sample from the dataset
        features = self.X[idx]
        target = self.y[idx]
        return features, target
    

model = sensorNet().to('cpu')
print('before loading')
model.load_state_dict(torch.load('processed_data/FA6/torch_model_FA6_20240311_102015.pt'))

print('after loading')
model.eval()

# pressure_readings = np.random.randn(10,8)

pressure_readings_32 = pressure_readings.astype(np.float32)
inputs = torch.from_numpy(pressure_readings_32)
# print(inputs)
inputs = inputs.to('cpu')

with torch.no_grad():
    pred = model(inputs)

pred = pred.detach().cpu().numpy()

# for i in range(len(pred)):
for i in range(100000,100010):
     true_vals = np.concatenate([fingertip_forces[i,:],pitch[i].reshape((1,)),roll[i].reshape((1,))])
    #  true_vals = [0,0,0,0,0]
     print('pred')
     print(f"{Color.GREEN}{pred[i]}{Color.END}", end=' ')
     print()
     print('true')
     print(f"{Color.BLUE}{true_vals}{Color.END}", end=' ')
     print()