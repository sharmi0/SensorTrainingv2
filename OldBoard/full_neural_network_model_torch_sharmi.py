import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import numpy as np
from numpy import loadtxt
import matplotlib.pyplot as plt
import time
from mat4py import loadmat
import os


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

def denormalize_model_outputs(predictions, output_max, output_min):
    predicted_outputs = predictions*(output_max - output_min) + output_min
    return predicted_outputs


# load data
sensorname = 'E7_3_4'
extraname = ''
save_model = 1
print(sensorname)
print(extraname)
dataset = loadtxt('processed_data/'+sensorname+'/full_dataset.csv', delimiter=',') # from delta_data_analysis as NN_data array
train_inds = loadtxt('processed_data/'+sensorname+'/train_full_dataset.csv', delimiter=',').astype(int) - 1 # from delta_data_analysis as train array
test_inds = loadtxt('processed_data/'+sensorname+'/test_full_dataset.csv', delimiter=',').astype(int) - 1 # from delta_data_analysis as test array

norm_params = loadmat('processed_data/'+sensorname+'/norm_params.mat')
output_max = np.array(norm_params['force_max']+norm_params['angle_max'])
output_min = np.array(norm_params['force_min']+norm_params['angle_min'])
input_max = np.array(norm_params['pressure_max'])
input_min = np.array(norm_params['pressure_min'])

y = dataset[:,0:5] # ftx, fty, fnz, theta, phi
X = dataset[:,5:13] # s0, s1, s2, s3, s4, s5, s6, s7
y_train = y[train_inds,:]
X_train = X[train_inds,:]
y_test = y[test_inds,:]
X_test = X[test_inds,:]

#filename to output neural net training results
# Get the desktop directory on Windows
desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
filename_rslt = os.path.join(desktop_path, 'train_' + sensorname + 'output.txt')


# send data to cuda
y_train_torch = torch.from_numpy(y_train).to(torch.float32).to('cpu')
X_train_torch = torch.from_numpy(X_train).to(torch.float32).to('cpu')
y_test_torch = torch.from_numpy(y_test).to(torch.float32).to('cpu')
X_test_torch = torch.from_numpy(X_test).to(torch.float32).to('cpu')

# configure and create network
inputs = X_train_torch
outputs = y_train_torch
data = sensorDataset(inputs, outputs)
loader = DataLoader(data, shuffle=True, batch_size=10)
model = sensorNet().to('cpu')
print(model)
loss_fn = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=0.001)

# train network
print("Starting training.")
with open(filename_rslt,'w') as g:
    losses = []
    for epoch in range(20):
        batch_losses = []
        for X_batch, y_batch in loader:
            y_pred = model(X_batch)
            loss = loss_fn(y_pred, y_batch)
            batch_losses.append(loss.item())
            model.zero_grad()
            loss.backward()
            optimizer.step()
        losses.append(np.mean(batch_losses))

        print(f'Finished epoch {epoch}, latest loss {losses[-1]}')
        g.write(f'Finished epoch {epoch}, latest loss {losses[-1]}\n')

    # save model
    save_str = sensorname+extraname
    time_str = time.strftime("%Y%m%d_%H%M%S")
    if save_model:
        torch.save(model.state_dict(), 'processed_data/'+sensorname+'/torch_model_'+save_str+'_'+time_str+'.pt')

    # plot performance
    # plt.figure()
    # plt.plot(losses)
    # plt.ylabel('loss')
    # plt.xlabel('epoch')
    # plt.title("Learning, "+save_str)
    print("Done!")
    # plt.show()

    # Evaluate on the training set
    print('------TRAINING SET------')
    g.write('------TRAINING SET------\n')
    # normalized
    prediction = model(X_train_torch)
    prediction = prediction.detach().cpu().numpy()
    actual= np.array(y_train)
    se= np.square(prediction-actual)
    mse= np.true_divide(np.sum(se, axis=0),len(se))
    rmse= np.sqrt(mse)
    # un-normalized
    pr2 = denormalize_model_outputs(prediction, output_max, output_min)
    ac2 = denormalize_model_outputs(actual, output_max, output_min)
    se2 = np.square(pr2-ac2)
    mse2 = np.true_divide(np.sum(se2, axis=0),len(se2))
    rmse2 = np.sqrt(mse2)
    print('Fx, Fy, Fz, theta, phi')
    print(rmse)
    print('Norm rmse of forces is  '+str(np.sum(rmse[0:3])/3))
    print('Norm rmse of angles is  '+str(np.sum(rmse[3:5])/2))
    print('Norm rmse of all outputs is  '+str(np.sum(rmse)/5))
    print('Fx, Fy, Fz, theta, phi')
    print(rmse2)
    print('Actual rmse of forces is  '+str(np.sum(rmse2[0:3])/3)+' N')
    print('Actual rmse of angles is  '+str(np.sum(rmse2[3:5])/2)+' deg')

    g.write('Fx, Fy, Fz, theta, phi\n')
    g.write(str(rmse) + '\n')
    g.write('Norm rmse of forces is  '+str(np.sum(rmse[0:3])/3) + '\n')
    g.write('Norm rmse of angles is  '+str(np.sum(rmse[3:5])/2) + '\n')
    g.write('Norm rmse of all outputs is  '+str(np.sum(rmse)/5) + '\n')
    g.write('Fx, Fy, Fz, theta, phi\n')
    g.write(str(rmse2) + '\n')
    g.write('Actual rmse of forces is  '+str(np.sum(rmse2[0:3])/3)+' N' + '\n')
    g.write('Actual rmse of angles is  '+str(np.sum(rmse2[3:5])/2)+' deg' + '\n')
    

    # Evaluate on the testing set
    print('------TESTING SET------')
    g.write('------TESTING SET------\n')
    prediction = model(X_test_torch) #an array
    prediction = prediction.detach().cpu().numpy()
    # np.savetxt(filename+'prediction.csv', prediction, delimiter=',') # save prediction for plotting purposes
    actual= np.array(y_test)
    se= np.square(prediction-actual)
    mse= np.true_divide(np.sum(se, axis=0),len(se))
    rmse= np.sqrt(mse)
    # un-normalized
    pr2 = denormalize_model_outputs(prediction, output_max, output_min)
    ac2 = denormalize_model_outputs(actual, output_max, output_min)
    se2 = np.square(pr2-ac2)
    mse2 = np.true_divide(np.sum(se2, axis=0),len(se2))
    rmse2 = np.sqrt(mse2)
    print('Fx, Fy, Fz, theta, phi')
    print(rmse)
    print('Norm rmse of forces is  '+str(np.sum(rmse[0:3])/3))
    print('Norm rmse of angles is  '+str(np.sum(rmse[3:5])/2))
    print('Norm rmse of all outputs is  '+str(np.sum(rmse)/5))
    print('Fx, Fy, Fz, theta, phi')
    print(rmse2)
    print('Actual rmse of forces is  '+str(np.sum(rmse2[0:3])/3)+' N')
    print('Actual rmse of angles is  '+str(np.sum(rmse2[3:5])/2)+' deg')

    g.write('Fx, Fy, Fz, theta, phi\n')
    g.write(str(rmse) + '\n')
    g.write('Norm rmse of forces is  '+str(np.sum(rmse[0:3])/3) + '\n')
    g.write('Norm rmse of angles is  '+str(np.sum(rmse[3:5])/2) + '\n')
    g.write('Norm rmse of all outputs is  '+str(np.sum(rmse)/5) + '\n')
    g.write('Fx, Fy, Fz, theta, phi\n')
    g.write(str(rmse2) + '\n')
    g.write('Actual rmse of forces is  '+str(np.sum(rmse2[0:3])/3)+' N' + '\n')
    g.write('Actual rmse of angles is  '+str(np.sum(rmse2[3:5])/2)+' deg' + '\n')

# pull out weights and biases
params = model.state_dict()
weights = [np.array(params['l1.weight'].detach().cpu()).T, np.array(params['l2.weight'].detach().cpu()).T, np.array(params['l3.weight'].detach().cpu()).T, np.array(params['l4.weight'].detach().cpu()).T]
biases = [np.array(params['l1.bias'].detach().cpu()), np.array(params['l2.bias'].detach().cpu()), np.array(params['l3.bias'].detach().cpu()), np.array(params['l4.bias'].detach().cpu())]


# format and print
# then, copy-paste into struct in mbed
float_formatter = "{:.14f}".format
np.set_printoptions(formatter={'float_kind':float_formatter})
# filename = 'processed_data/'+sensorname+'/'+sensorname+'FULL.txt'


# Specify the filename
filename = os.path.join(desktop_path, sensorname + 'E7_3_4_FULL.txt')
with open(filename,'w') as f:
    # bias 1
    f.write('//Bias 1:\n')
    mat_to_print = biases[0]
    print(np.shape(mat_to_print))
    f.write(np.array2string(mat_to_print, separator='f, ')+'\n')
    # bias 2
    f.write('//Bias 2:\n')
    mat_to_print = biases[1]
    print(np.shape(mat_to_print))
    f.write(np.array2string(mat_to_print, separator='f, ')+'\n')
    # bias 3
    f.write('//Bias 3:\n')
    mat_to_print = biases[2]
    print(np.shape(mat_to_print))
    f.write(np.array2string(mat_to_print, separator='f, ')+'\n')
    # bias 4
    f.write('//Bias 4:\n')
    mat_to_print = biases[3]
    print(np.shape(mat_to_print))
    f.write(np.array2string(mat_to_print, separator=  'f, ')+'\n')
    # weights 1
    f.write('//Weights 1:\n')
    f.write('{')
    mat_to_print = weights[0]
    print(np.shape(mat_to_print))
    for line in mat_to_print:
        f.write(np.array2string(line, separator='f, ')+'\n')
    f.write('},\n')
    # weights 2
    f.write('//Weights 2:\n')
    f.write('{')
    mat_to_print = weights[1]
    print(np.shape(mat_to_print))
    for line in mat_to_print:
        f.write(np.array2string(line, separator='f, ')+'\n')
    f.write('},\n')
    # weights 3
    f.write('//Weights 3:\n')
    f.write('{')
    mat_to_print = weights[2]
    print(np.shape(mat_to_print))
    for line in mat_to_print:
        f.write(np.array2string(line, separator='f, ')+'\n')
    f.write('},\n')
    # weights 4
    f.write('//Weights 4:\n')
    f.write('{')
    mat_to_print = weights[3]
    print(np.shape(mat_to_print))
    for line in mat_to_print:
        f.write(np.array2string(line, separator='f, ')+'\n')
    f.write('},\n')

    # normalization ranges
    float_formatter = "{:.1f}".format
    np.set_printoptions(formatter={'float_kind':float_formatter})

    f.write('//minims:\n')
    mat_to_print = np.concatenate((output_min,input_min)).astype(float)
    print(np.shape(mat_to_print))
    f.write(np.array2string(mat_to_print, separator='f, ')+'\n')

    f.write('//maxims:\n')
    mat_to_print = np.concatenate((output_max,input_max)).astype(float)
    print(np.shape(mat_to_print))
    f.write(np.array2string(mat_to_print, separator='f, ')+'\n')

# post-processing:
# first, replace '[' with '{' and replace ']' with 'f},'
with open(filename,'r') as f:
    data = f.read()
    data = data.replace('[', '{')
    data = data.replace(']', 'f},')
with open(filename,'w') as f:
    f.write(data)
# then remove last comma from each weights array and maxims (lines 87, 353, 1763, 1893, 1900)
comma_lines = [87, 353, 1763, 1893, 1900]
with open(filename,'r') as f:
    data = f.readlines()
    num_lines = 0
    for line in data:
        num_lines+=1
        if num_lines in comma_lines:
            new_line = line[:-2]+line[-1]
            data[num_lines-1] = new_line
with open(filename,'w') as f:
    f.writelines(data)