import torch
import numpy as np
from global_params import *
from torch.autograd import Variable as V

x_values = [i for i in range(11)]
x_train = np.array(x_values, dtype=np.float32)
x_train = x_train.reshape(-1, 1)

y_values = [2 * i + 1 for i in x_values]
y_train = np.array(y_values, dtype=np.float32)
y_train = y_train.reshape(-1, 1)


class linearRegression(torch.nn.Module):
    def __init__(self):
        super(linearRegression, self).__init__()
        self.linear = torch.nn.Linear(lr_inputSize, lr_outputSize)

    def forward(self, x):
        return self.linear(x)


inputDim = 1
outputDim = 1
learningRate = 0.01
epochs = 100

model = linearRegression(inputDim, outputDim)

if torch.cuda.is_available():
    model.cuda()

criterion = torch.nn.MSELoss()
optimizer = torch.optim.SGD(model.parameters(), lr=learningRate)

for epoch in range(epochs):
    if torch.cuda.is_available():
        inputs = V(torch.from_numpy(x_train).cuda())
        targets = V(torch.from_numpy(y_train).cuda())
